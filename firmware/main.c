/* GD32F103 IR camera rewrite -- baseline: image only, no overlays.
 * 32x24 MLX90640 -> bilinear 10x -> 320x240 full screen.
 *
 * PB0-7 = D0-D7   PA2 = RS/DC   PA3 = CS   PC15 = WR   PC14 = RD   PA8 = backlight
 * PA9 = SCL       PA15 = SDA    (software I2C; PB3/PB4/PA15 need AFIO SWJ_CFG=010)
 *
 * dbg[]: 4 = frames/sec, 5 = render cycles, 6 = i2c cycles, 7 = wait cycles
 */
#include <stdint.h>

#define REG(a) (*(volatile uint32_t*)(a))
#define RCC_CTL     REG(0x40021000)
#define RCC_CFG0    REG(0x40021004)
#define RCC_APB2ENR REG(0x40021018)
#define FLASH_ACR   REG(0x40022000)
#define AFIO_MAPR   REG(0x40010004)
#define GPIOA_CRL   REG(0x40010800)
#define GPIOA_CRH   REG(0x40010804)
#define GPIOA_IDR   REG(0x40010808)
#define GPIOA_BSRR  REG(0x40010810)
#define GPIOA_BRR   REG(0x40010814)
#define GPIOB_CRL   REG(0x40010C00)
#define GPIOB_ODR   REG(0x40010C0C)
#define GPIOC_CRH   REG(0x40011004)
#define GPIOC_BSRR  REG(0x40011010)
#define GPIOC_BRR   REG(0x40011014)
#define DEMCR       REG(0xE000EDFC)
#define DWT_CTRL    REG(0xE0001000)
#define CYC         REG(0xE0001004)

#define PA2 (1u<<2)
#define PA3 (1u<<3)
#define PA8 (1u<<8)
#define SCL (1u<<9)
#define SDA (1u<<15)
#define PC14 (1u<<14)
#define PC15 (1u<<15)

#define LCD_W 320
#define LCD_H 240
#define SRC_W 32
#define SRC_H 24
#define MLX_ADDR 0x33
#define NOISE_SHIFT_MAX 4  /* quiet-pixel averaging: 2^4 frames, ~5.6x noise reduction */
#define SPAN_MIN 52        /* minimum displayed span in raw counts (~6 degC) */

volatile uint32_t dbg[24];
static int16_t  frame[834];
static int min_idx=400, max_idx=400;

/* label values, averaged and refreshed at most 3x/sec */
static int32_t disp_c, disp_n, disp_x;
#define CENTER_IDX 400
#define BAR_H  20
#define IMG_Y0 0
#define IMG_H  img_h
static int img_h = LCD_H-BAR_H;
#define BAR_Y0 (LCD_H-BAR_H)
#define GPIOB_CRH   REG(0x40010C04)
#define GPIOC_IDR   REG(0x40011008)
#define GPIOB_IDR   REG(0x40010C08)
#define GPIOB_BSRR  REG(0x40010C10)
#define GPIOC_BSRR2 REG(0x40011010)
static int overlay_on = 1;
/* View mode, cycled by the second button. Lets processing artefacts be told
 * apart from sensor behaviour. Gain/offset calibration stays on in every mode --
 * without it the EEPROM fixed-pattern swamps everything and you learn nothing.
 *   0 = interpolation + temporal filter   (normal; crosshair white)
 *   1 = neither: raw 10x10 blocks         (the sensor as-is; crosshair yellow)
 *   2 = interpolation only, no filter     (isolates the filter; crosshair magenta)
 */
int view_mode = 0;
#define VIEW_INTERP  (view_mode != 1)
#define VIEW_FILTER  (view_mode == 0)
static uint32_t bus_hi = 0xFC00u;
static int cur_pair = 0;

static void delay_us(uint32_t us);

/* The three wheel contacts conduct transitively, so a single driven pin cannot
 * tell the positions apart. Drive each pin in turn instead, then restore the
 * render configuration so the pixel loop keeps a constant upper-bit pattern. */
static int scan_wheel(void){
    uint32_t seen[3];
    GPIOB_CRH=0x88888883u; GPIOB_ODR=0x0100u; delay_us(20); seen[0]=GPIOB_IDR;
    GPIOB_CRH=0x88888838u; GPIOB_ODR=0x0200u; delay_us(20); seen[1]=GPIOB_IDR;
    GPIOB_CRH=0x38888888u; GPIOB_ODR=0x8000u; delay_us(20); seen[2]=GPIOB_IDR;
    GPIOB_CRH=0x88888888u; GPIOB_ODR=0xFC00u;          /* back to render config */
    int pr=0;
    if((seen[0]&(1u<<9))  || (seen[1]&(1u<<8)))  pr|=1;   /* PB8-PB9  */
    if((seen[0]&(1u<<15)) || (seen[2]&(1u<<8)))  pr|=2;   /* PB8-PB15 */
    if((seen[1]&(1u<<15)) || (seen[2]&(1u<<9)))  pr|=4;   /* PB9-PB15 */
    return pr;
}   /* 0=open 1=PB8-PB9 2=PB8-PB15 3=PB9-PB15 */   /* upper GPIOB bits held during bus writes */
#define ADC0 0x40012400u
#define ADC_SR   REG(ADC0+0x00)
#define ADC_CR1  REG(ADC0+0x04)
#define ADC_CR2  REG(ADC0+0x08)
#define ADC_SMPR2 REG(ADC0+0x10)
#define ADC_SQR1 REG(ADC0+0x2C)
#define ADC_SQR3 REG(ADC0+0x34)
#define ADC_DR   REG(ADC0+0x4C)
/* distinct ADC levels seen per channel: {value,count} -- press each control once */
volatile uint32_t adcbuk[2][8][2];
volatile uint32_t adcnow[2];
static uint16_t pal[256];
static uint8_t  gam[256];

static uint32_t adc_read(int ch);
static void delay_us(uint32_t us){ uint32_t t=CYC, n=us*72u; while ((CYC-t)<n){} }
static void delay_ms(uint32_t ms){ while(ms--) delay_us(1000); }

static uint32_t adc_read(int ch){
    ADC_SQR3 = (uint32_t)ch;
    ADC_CR2 |= (1u<<22);                   /* SWSTART */
    uint32_t t=CYC;
    while(!(ADC_SR & (1u<<1))){ if((CYC-t)>72000u) return 0xFFFF; }
    ADC_SR &= ~(1u<<1);
    return ADC_DR & 0xFFFF;
}

static void bucket(int ch, uint32_t v){
    for(int i=0;i<8;i++){
        if(adcbuk[ch][i][1]==0){ adcbuk[ch][i][0]=v; adcbuk[ch][i][1]=1; return; }
        int32_t d=(int32_t)v-(int32_t)adcbuk[ch][i][0]; if(d<0)d=-d;
        if(d<40){ adcbuk[ch][i][1]++; return; }
    }
}

static void clock_init(void){
    RCC_CTL |= (1u<<16); while(!(RCC_CTL&(1u<<17))){}
    FLASH_ACR = 0x32;
    RCC_CFG0  = 0x082b0400;
    RCC_CTL  |= (1u<<24); while(!(RCC_CTL&(1u<<25))){}
    RCC_CFG0  = 0x082b0402;
    while(((RCC_CFG0>>2)&3)!=2){}
}
static void dwt_init(void){ DEMCR |= (1u<<24); CYC = 0; DWT_CTRL |= 1u; }

/* ---------------- ST7789 over bit-banged 8080 ---------------- */
static inline void lcd_cmd(uint8_t c){
    GPIOA_BRR=PA3; GPIOA_BRR=PA2; GPIOC_BSRR=PC14;
    GPIOB_ODR=0xFC00u|c; GPIOC_BRR=PC15; GPIOC_BSRR=PC15; GPIOA_BSRR=PA3;
}
static inline void lcd_dat(uint8_t b){
    GPIOA_BSRR=PA2; GPIOA_BRR=PA3;
    GPIOB_ODR=0xFC00u|b; GPIOC_BRR=PC15; GPIOC_BSRR=PC15; GPIOA_BSRR=PA3;
}
static inline void lcd_px(uint16_t v){ lcd_dat((uint8_t)(v>>8)); lcd_dat((uint8_t)v); }

static const uint8_t init_seq[] = {
    0x11,0, 0x13,0,
    0x36,1, 0xE8,                 /* MY|MX|MV|BGR -> landscape 320x240, correct orientation */
    0xB6,2, 0x0A,0x82,
    0xB0,2, 0x00,0xE0,
    0x3A,1, 0x05,
    0xB2,5, 0x0C,0x0C,0x00,0x33,0x33,
    0xB7,1, 0x35,  0xBB,1, 0x28,  0xC0,1, 0x0C,
    0xC2,2, 0x01,0xFF, 0xC3,1, 0x10, 0xC4,1, 0x20, 0xC6,1, 0x0F,
    0xD0,2, 0xA4,0xA1,
    0xE0,14, 0xD0,0x00,0x02,0x07,0x0A,0x28,0x32,0x44,0x42,0x06,0x0E,0x12,0x14,0x17,
    0xE1,14, 0xD0,0x00,0x02,0x07,0x0A,0x28,0x31,0x54,0x47,0x0E,0x1C,0x17,0x1B,0x1E,
    0x21,0, 0x29,0, 0x00
};
static void lcd_window(uint16_t x0,uint16_t x1,uint16_t y0,uint16_t y1){
    lcd_cmd(0x2A); lcd_px(x0); lcd_px(x1);
    lcd_cmd(0x2B); lcd_px(y0); lcd_px(y1);
    lcd_cmd(0x2C);
}
static void lcd_init(void){
    GPIOA_BRR=PA8; delay_ms(20); GPIOA_BSRR=PA8; delay_ms(150);
    for (const uint8_t*p=init_seq; *p; ){
        uint8_t c=*p++, n=*p++;
        lcd_cmd(c); while(n--) lcd_dat(*p++);
        if (c==0x11) delay_ms(150);
    }
}

/* ---------------- software I2C ---------------- */
#define I2C_HALF 3
static void ihalf(void){ for(volatile int i=0;i<I2C_HALF;i++){ __asm__ volatile("nop"); } }
static inline void scl_hi(void){ GPIOA_BSRR=SCL; }
static inline void scl_lo(void){ GPIOA_BRR =SCL; }
static inline void sda_hi(void){ GPIOA_BSRR=SDA; }
static inline void sda_lo(void){ GPIOA_BRR =SDA; }
static inline int  sda_rd(void){ return (GPIOA_IDR&SDA)?1:0; }

static void i2c_start(void){ sda_hi(); scl_hi(); ihalf(); sda_lo(); ihalf(); scl_lo(); ihalf(); }
static void i2c_stop (void){ sda_lo(); ihalf(); scl_hi(); ihalf(); sda_hi(); ihalf(); }

/* A slave left mid-transfer can hold SDA low forever; clock it out and stop. */
static void i2c_recover(void){
    sda_hi();
    for(int i=0;i<18;i++){ scl_hi(); ihalf(); scl_lo(); ihalf(); }
    scl_hi(); ihalf();
    sda_lo(); ihalf(); scl_hi(); ihalf(); sda_hi(); ihalf();
}
static int i2c_wr(uint8_t b){
    for(int i=0;i<8;i++){ if(b&0x80) sda_hi(); else sda_lo(); b<<=1; ihalf(); scl_hi(); ihalf(); scl_lo(); }
    sda_hi(); ihalf(); scl_hi(); ihalf();
    int nack=sda_rd(); scl_lo(); ihalf();
    return !nack;
}
static uint8_t i2c_rd(int ack){
    uint8_t v=0; sda_hi();
    for(int i=0;i<8;i++){ ihalf(); scl_hi(); ihalf(); v=(uint8_t)((v<<1)|sda_rd()); scl_lo(); }
    if(ack) sda_lo(); else sda_hi();
    ihalf(); scl_hi(); ihalf(); scl_lo(); sda_hi(); ihalf();
    return v;
}
static int mlx_read(uint16_t a, void *dst, uint32_t n){
    uint16_t *d=(uint16_t*)dst;
    i2c_start();
    if(!i2c_wr(MLX_ADDR<<1)||!i2c_wr((uint8_t)(a>>8))||!i2c_wr((uint8_t)a)){ i2c_stop(); return 0; }
    i2c_start();
    if(!i2c_wr((MLX_ADDR<<1)|1)){ i2c_stop(); return 0; }
    for(uint32_t i=0;i<n;i++){
        uint8_t hi=i2c_rd(1), lo=i2c_rd(i<n-1);
        d[i]=(uint16_t)((hi<<8)|lo);
    }
    i2c_stop(); return 1;
}
static int mlx_write(uint16_t a, uint16_t v){
    i2c_start();
    int ok = i2c_wr(MLX_ADDR<<1)&&i2c_wr((uint8_t)(a>>8))&&i2c_wr((uint8_t)a)
          && i2c_wr((uint8_t)(v>>8))&&i2c_wr((uint8_t)v);
    i2c_stop(); return ok;
}

/* ---------------- palette + render ---------------- */
static uint32_t isqrt32(uint32_t n){
    uint32_t res=0, bit=1u<<30;
    while(bit>n) bit>>=2;
    while(bit){
        if(n>=res+bit){ n-=res+bit; res=(res>>1)+bit; }
        else res>>=1;
        bit>>=2;
    }
    return res;
}

static int pal_id = 0;
static int gamma_id = 0;   /* cycles 4.0 -> 3.0 -> 2.0 -> 1.5 -> 4.0 */

static void pal_init(void){
    for(int i=0;i<256;i++){
        int r,g,b;
        if(pal_id==1){                       /* ironbow */
            if      (i< 64){ int u=i;     r=(u*120)/63; g=0;             b=(u*140)/63; }
            else if (i<128){ int u=i-64;  r=120+(u*135)/63; g=0;         b=140-(u*140)/63; }
            else if (i<192){ int u=i-128; r=255;         g=(u*200)/63;   b=0; }
            else           { int u=i-192; r=255;         g=200+(u*55)/63; b=(u*255)/63; }
        } else if(pal_id==2){                /* grayscale */
            r=g=b=i;
        } else {                             /* rainbow (default) */
            if      (i< 48){ int u=i;     r=0;            g=0;            b=(u*255)/47; }
            else if (i<112){ int u=i-48;  r=0;            g=(u*255)/63;   b=255; }
            else if (i<176){ int u=i-112; r=0;            g=255;          b=255-(u*255)/63; }
            else if (i<216){ int u=i-176; r=(u*255)/39;   g=255;          b=0; }
            else           { int u=i-216; r=255;          g=255-(u*255)/39; b=0; }
        }
        pal[i]=(uint16_t)(((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3));
    }
    for(int i=0;i<256;i++){
        uint32_t c2=(uint32_t)i*(uint32_t)i;
        uint32_t c3=c2*(uint32_t)i;
        uint32_t r;
        /* 255*(i/255)^g ; i^4 peaks at 4228250625, which still fits a uint32 */
        if(gamma_id==0)      r=(c2*c2)/16581375u;       /* 4.0 */
        else if(gamma_id==1) r=c3/65025u;               /* 3.0 */
        else if(gamma_id==2) r=c2/255u;                 /* 2.0 */
        else                 r=isqrt32(c3/255u);        /* 1.5 */
        gam[i]=(uint8_t)(r>255?255:r);
    }
}

/* bulk pixel write: CS and RS/DC are hoisted out of the loop by the caller */
static inline void px_fast(uint16_t v){
    GPIOB_ODR=0xFC00u|(uint8_t)(v>>8); GPIOC_BRR=PC15; GPIOC_BSRR=PC15;
    GPIOB_ODR=0xFC00u|(uint8_t)v;      GPIOC_BRR=PC15; GPIOC_BSRR=PC15;
}

static uint16_t ee[832];
static int16_t  poff[768];
static int32_t  gainEE;

/* Melexis offset extraction: per-pixel offset = ref + row/col terms + per-pixel remnant */
static void extract_offsets(void){
    int occRow[24], occCol[32];
    int sRow=(ee[16]&0x0F00)>>8, sCol=(ee[16]&0x00F0)>>4, sRem=(ee[16]&0x000F);
    int32_t offRef=(int16_t)ee[17];
    for(int i=0;i<6;i++){ int p=i*4;
        occRow[p+0]= ee[18+i]&0x000F;
        occRow[p+1]=(ee[18+i]&0x00F0)>>4;
        occRow[p+2]=(ee[18+i]&0x0F00)>>8;
        occRow[p+3]=(ee[18+i]&0xF000)>>12; }
    for(int i=0;i<24;i++) if(occRow[i]>7) occRow[i]-=16;
    for(int i=0;i<8;i++){ int p=i*4;
        occCol[p+0]= ee[24+i]&0x000F;
        occCol[p+1]=(ee[24+i]&0x00F0)>>4;
        occCol[p+2]=(ee[24+i]&0x0F00)>>8;
        occCol[p+3]=(ee[24+i]&0xF000)>>12; }
    for(int i=0;i<32;i++) if(occCol[i]>7) occCol[i]-=16;
    for(int i=0;i<24;i++) for(int j=0;j<32;j++){
        int p=32*i+j;
        int32_t o=(ee[64+p]&0xFC00)>>10; if(o>31) o-=64;
        o = o*(1<<sRem);
        o = offRef + ((int32_t)occRow[i]<<sRow) + ((int32_t)occCol[j]<<sCol) + o;
        poff[p]=(int16_t)o;
    }
    gainEE=(int16_t)ee[48];
}

#ifndef DIAG
/* Motion-adaptive temporal filter.
 *
 * A fixed-strength average forces a straight trade: quiet image or responsive
 * image, pick one. Instead, compare each pixel's change against an estimate of
 * the noise floor and filter hard only where the change looks like noise. Static
 * scenes get ~8 frames of averaging; genuinely moving pixels pass through almost
 * untouched, so there is no added latency on the parts of the image that matter.
 *
 * The noise floor is measured, not assumed: it tracks the mean absolute
 * frame-to-frame change, updated slowly so real motion cannot inflate it much.
 * Runs over the 768 sensor pixels, so the cost is negligible.
 */
static int32_t acc[768];
static int acc_primed;
static int32_t noise_est = 4000;      /* fixed point <<8 */
static int16_t trend[768];            /* EMA of the SIGNED delta, scaled >>4 */
volatile uint32_t dbg_noise;

/* A/B test of the MLX90640 ADC resolution (0x800D bits 11:10).
 * Raw counts scale with resolution, so absolute noise is not comparable --
 * normalise to the scene span. Alternating legs keeps the scene identical.
 * abtest[] = {mean normalised noise 18-bit, same 19-bit, samples, samples} */
volatile uint32_t abtest[4];

static void denoise(void){
    if(!acc_primed){
        for(int i=0;i<768;i++) acc[i]=(int32_t)frame[i]<<8;
        acc_primed=1;
        return;
    }
    /* Thresholds are multiples of the mean ABSOLUTE deviation, and MAD ~ 0.8*sigma.
     * At 2x/4x MAD (1.6/3.2 sigma) noise alone crossed the lower threshold ~11% of
     * the time, so one pixel in nine fell to the barely-filtered path every frame
     * -- the filter was mistaking its own noise for motion, which is what the
     * sparkle was. 3x/6x MAD (2.4/4.8 sigma) is crossed by noise far more rarely. */
    int32_t t_hi=noise_est*6, t_mid=noise_est*3, sumd=0;
    int32_t t_trend=(noise_est>>4)*2;   /* lower = faster response, slightly more noise through */
    for(int i=0;i<768;i++){
        int32_t d=((int32_t)frame[i]<<8) - acc[i];
        int32_t ad = d<0 ? -d : d;
        sumd += ad;
        /* Noise is zero-mean and flips sign frame to frame, so it averages to
         * nothing here; a genuine change keeps the same sign and accumulates.
         * That separates a small persistent change from noise of the same
         * magnitude, which a per-frame threshold cannot do -- and it is why the
         * heavy averaging no longer costs a second of settling. */
        int32_t tr = trend[i];
        tr += ((d>>4) - tr) >> 1;   /* short EMA: detects persistence in fewer frames */
        trend[i] = (int16_t)tr;

        int sh;
        if(ad > t_hi)                        sh = 0;   /* big jump: track at once */
        else if(tr > t_trend || tr < -t_trend) sh = 1; /* persistent drift: track fast */
        else if(ad > t_mid)                  sh = 2;
        else                                 sh = NOISE_SHIFT_MAX;
        acc[i] += d>>sh;
        frame[i] = (int16_t)(acc[i]>>8);
    }
    noise_est += ((sumd/768) - noise_est)>>3;
    if(noise_est<256) noise_est=256;
    dbg_noise=(uint32_t)noise_est;
}
#endif


#include "cal.h"

/* Flat-field / non-uniformity correction.
 *
 * Measured FPN is ~3.3 counts against a ~50-count scene span, which at 10x
 * upscale reads as a grid of blobs. Rather than chase its origin in the
 * calibration maths, measure it: average frames of a uniform surface and store
 * each pixel's deviation. High-passed against a 3x3 local mean so a target that
 * isn't perfectly flat (or lens vignetting) is not baked in. Applied
 * incrementally, so repeated calibrations converge. */

#ifdef DIAG
/* Characterise the sensor with the display pipeline out of the way.
 * Streaming statistics relative to a per-pixel reference, so the squares stay
 * small and no frame buffers are needed:
 *   mean[i]   = ref[i] + d_sum[i]/n           -> averaging leaves FIXED pattern
 *   var[i]    = d_sumsq[i]/n - (d_sum[i]/n)^2 -> per-pixel TEMPORAL noise
 * Host-side maths; the device just accumulates. */
volatile int16_t  d_ref[768];
volatile int32_t  d_sum[768];
volatile int32_t  d_sumsq[768];
volatile uint32_t d_n, d_cfg;
#endif




/* gain-correct then subtract the per-pixel offset -- kills the fixed-pattern dots */
static void correct_frame(void){
    /* use the gain already validated in calc_frame_params -- a second I2C read
     * here was a second chance to catch a mid-update word, and its gr==0
     * fallback multiplied every pixel by ~6200 and saturated the frame */
    int32_t gfp = lg_gfp;
    for(int i=0;i<768;i++){
        int32_t v=(((int32_t)frame[i]*gfp)>>10) - poff[i];
        frame[i]=(int16_t)(v>32767?32767:(v<-32768?-32768:v));
    }
}
static int32_t mn_s, mx_s;
static int scale_primed;

static int32_t r_inv10, r_base10;

static void render_prep(void){
    /* Trimmed range: ignore the 4 most extreme pixels at each end, so a single
     * corrupted word cannot blow out the auto-scale (that was the blue flash). */
    int16_t lo[4]={32767,32767,32767,32767}, hi[4]={-32768,-32768,-32768,-32768};
    for(int i=0;i<768;i++){
        int16_t v=frame[i];
        if(v<lo[3]){ int k=3; while(k>0 && v<lo[k-1]){ lo[k]=lo[k-1]; k--; } lo[k]=v; }
        if(v>hi[3]){ int k=3; while(k>0 && v>hi[k-1]){ hi[k]=hi[k-1]; k--; } hi[k]=v; }
    }
    int16_t mn=lo[3], mx=hi[3];
    for(int i=0;i<768;i++){ if(frame[i]==mn) min_idx=i; else if(frame[i]==mx) max_idx=i; }

    /* measure the checkerboard: mean of each chess-pattern subpage class */
    int32_t s0=0,s1=0;
    for(int r=0;r<SRC_H;r++)
        for(int c=0;c<SRC_W;c++)
            { int16_t v2=frame[r*SRC_W+c]; if(((r+c)&1)==0) s0+=v2; else s1+=v2; }
    dbg[12]=(uint32_t)(s0/384); dbg[13]=(uint32_t)(s1/384);

    /* Smooth the auto-range so one bad word can't wash out a whole frame. */
    if(!scale_primed){ mn_s=mn; mx_s=mx; scale_primed=1; }
    else { mn_s += ((int32_t)mn-mn_s)>>2; mx_s += ((int32_t)mx-mx_s)>>2; }

    /* Don't stretch a span smaller than the thermal detail actually present.
     * Measured: a flat scene gives ~31 counts (~3.6 C) of span, and spreading
     * that over 256 palette entries magnifies ~2 counts of residual noise into
     * a visible 5-20% of the colour range. Below the floor, expand symmetrically
     * about the midpoint: less contrast on genuinely flat scenes, which is
     * honest, and far less amplified noise. ~8.6 counts per degC on this unit. */
    int32_t lo_d=mn_s, hi_d=mx_s;
    int32_t span = hi_d-lo_d;
    if(span < SPAN_MIN){
        int32_t mid=(hi_d+lo_d)/2;
        lo_d = mid - SPAN_MIN/2;
        hi_d = mid + SPAN_MIN/2;
        span = SPAN_MIN;
    }
    if(span<1) span=1;
    r_inv10 = (255*65536)/span;
    r_base10 = lo_d;
}

/* Render one horizontal band, so overlays can be drawn immediately after the
 * rows beneath them are painted. Drawing them all at the end meant the blast
 * erased them for most of each frame -- that was the every-other-frame flicker. */
static void render_band(int Y0,int Y1){
    lcd_window(0,LCD_W-1,(uint16_t)Y0,(uint16_t)Y1);
    GPIOA_BSRR=PA2;
    GPIOA_BRR =PA3;

    const uint32_t vstep = ((uint32_t)SRC_H<<16)/(uint32_t)img_h;
    for(int Y=Y0; Y<=Y1; Y++){
        uint32_t vpos = (uint32_t)(Y-IMG_Y0)*vstep;
        int sy = (int)(vpos>>16);
        if(sy>SRC_H-1) sy=SRC_H-1;
        int wy1 = VIEW_INTERP ? (int)((vpos>>8)&0xFF) : 0, wy0 = 256-wy1;
        const int16_t *r0=&frame[sy*SRC_W];
        const int16_t *r1=&frame[(sy<SRC_H-1?sy+1:sy)*SRC_W];
        uint8_t idxc[SRC_W+1];
        for(int i=0;i<SRC_W;i++){
            int32_t c = ((int32_t)r0[i]*wy0 + (int32_t)r1[i]*wy1)>>8;
            int32_t k = ((c - r_base10) * r_inv10) >> 16;
            idxc[i] = gam[(uint8_t)(k<0 ? 0 : (k>255 ? 255 : k))];
        }
        idxc[SRC_W] = idxc[SRC_W-1];

        for(int sx=0; sx<SRC_W; sx++){
            int a = idxc[sx], b = idxc[sx+1];
            if(!VIEW_INTERP){
                uint16_t c = pal[a];
                for(int fx=0; fx<10; fx++) px_fast(c);
            } else {
                for(int fx=0; fx<10; fx++){
                    int k = (a*(10-fx) + b*fx) * 3277 >> 15;   /* /10 */
                    px_fast(pal[k]);
                }
            }
        }
    }
    GPIOA_BSRR=PA3;            /* release CS */
}

#include "gfx.h"

int main(void){
    clock_init(); dwt_init();
    RCC_APB2ENR |= 1u|(1u<<2)|(1u<<3)|(1u<<4);
    AFIO_MAPR = (AFIO_MAPR & ~(7u<<24)) | (2u<<24);

    GPIOB_CRL = 0x33333333;
    GPIOA_CRL = 0x44443344;
    GPIOC_CRH = 0x33444444;
    GPIOA_CRH = 0x74488873;      /* + PA10,11,12 as pull-up inputs */
    GPIOA_CRL = 0x44443300;      /* PA0,PA1 = analog in (button ladder) */
    GPIOB_CRH = 0x88888888;      /* PB8..PB15 inputs; scan_wheel() drives them */
    GPIOC_CRH = 0x33844444;      /* + PC13     as pull-up input    */
    GPIOA_BSRR = (1u<<10)|(1u<<11)|(1u<<12);

    GPIOC_BSRR2 = (1u<<13);
    sda_hi(); scl_hi();
    GPIOA_BSRR=PA3; GPIOC_BSRR=PC15; GPIOC_BSRR=PC14;

    /* ADC: PCLK2/6 = 12MHz, then calibrate */
    RCC_CFG0 |= (2u<<14);
    RCC_APB2ENR |= (1u<<9);
    ADC_SMPR2 = (7u<<0)|(7u<<3);          /* 239.5 cycles on ch0/ch1 (high-Z ladder) */
    ADC_SQR1  = 0;
    ADC_CR2   = 1u;                        /* ADON */
    delay_us(10);
    ADC_CR2  |= (1u<<3); while(ADC_CR2 & (1u<<3)){}   /* RSTCAL */
    ADC_CR2  |= (1u<<2); while(ADC_CR2 & (1u<<2)){}   /* CAL    */
    ADC_CR2   = 1u | (7u<<17) | (1u<<20);  /* ADON + EXTSEL=SWSTART + EXTTRIG */

    dbg[0]=0xC0FFEE02;
    pal_init();
    lcd_init();

    uint16_t ctrl=0;
    mlx_read(0x800D,&ctrl,1);
    ctrl = (uint16_t)((ctrl & ~(7u<<7)) | (7u<<7));   /* 64 Hz subpage rate */
    ctrl = (uint16_t)((ctrl & ~(3u<<10)) | (2u<<10));  /* 18-bit ADC */
    mlx_write(0x800D, ctrl);
    dbg[1]=ctrl;
    dbg[14]=mlx_read(0x2400, ee, 832);
    extract_offsets();
    extract_cal();
    dbg[15]=(uint32_t)gainEE;
    mlx_write(0x8000,0x0030);

#ifdef DIAG
    /* DIAG_RES: 2 = 18-bit, 3 = 19-bit.  DIAG_RATE: 6 = 32Hz, 7 = 64Hz */
#ifndef DIAG_RES
#define DIAG_RES 3
#endif
#ifndef DIAG_RATE
#define DIAG_RATE 7
#endif
    ctrl = (uint16_t)((ctrl & ~(7u<<7)) | ((uint32_t)DIAG_RATE<<7));
    ctrl = (uint16_t)((ctrl & ~(3u<<10)) | ((uint32_t)DIAG_RES<<10));
    mlx_write(0x800D, ctrl);
    { uint16_t rb=0; mlx_read(0x800D,&rb,1);      /* read back: did it actually take? */
      d_cfg = ((uint32_t)ctrl<<16) | rb; }
    mlx_write(0x8000,0x0030);

    for(int w=0; w<40; w++){                      /* warm up / settle */
        uint16_t st=0; uint32_t tw=CYC;
        while((CYC-tw)<7200000u){ if(mlx_read(0x8000,&st,1) && (st&8)) break; }
        mlx_read(0x0400, frame, 768);
        mlx_write(0x8000,0x0030);
    }
    { uint16_t st=0; uint32_t tw=CYC;
      while((CYC-tw)<7200000u){ if(mlx_read(0x8000,&st,1) && (st&8)) break; }
      mlx_read(0x0400, frame, 768);
      mlx_write(0x8000,0x0030);
      for(int i=0;i<768;i++) d_ref[i]=frame[i]; }

    while(1){
        uint16_t st=0; uint32_t tw=CYC;
        int ok=0;
        while((CYC-tw)<7200000u){ if(mlx_read(0x8000,&st,1) && (st&8)){ ok=1; break; } }
        if(!ok){ i2c_recover(); mlx_write(0x800D,ctrl); mlx_write(0x8000,0x0030); continue; }
        if(!mlx_read(0x0400, frame, 768)){ i2c_recover(); continue; }
        mlx_write(0x8000,0x0030);
        for(int i=0;i<768;i++){
            int32_t d=(int32_t)frame[i]-(int32_t)d_ref[i];
            d_sum[i]+=d; d_sumsq[i]+=d*d;
        }
        d_n++;
    }
#endif
    uint32_t frames=0, t0=CYC, iters=0, fails=0, recov=0;
    int32_t sum_c=0, sum_n=0, sum_x=0; uint32_t nsamp=0, t_lbl=CYC; int primed=0;
    while(1){
        dbg[2]=++iters;
        /* latch any spare input seen low -- press each control and I'll read these */
        adcnow[0]=adc_read(0); adcnow[1]=adc_read(1);
        {   uint32_t lv=adcnow[1];
            int b = (lv<1000)?2 : (lv<3000)?1 : 0;
            /* The ADC's first conversions read 0 before it settles, which decodes
             * identically to the second button being held -- enough of them in a
             * row satisfied the debounce and toggled a mode at every boot. Ignore
             * the button until the ADC is known good. 'stable=-1' also means the
             * first settled reading is adopted rather than treated as a change. */
            static int raw=-1, cnt=0, stable=-1;
            if(iters < 15) b = stable = -1;
            if(b==raw) cnt++; else { raw=b; cnt=0; }
            if(cnt>=2 && b!=stable){
                stable=b;
                if(b==1) overlay_on=!overlay_on;      /* 2045 level toggles overlay */
                else if(b==2) view_mode=(view_mode+1)%3;  /* 0 level: cycle view mode */
            }
        }
        bucket(0,adcnow[0]); bucket(1,adcnow[1]);
        dbg[20] |= (~GPIOA_IDR) & ((1u<<10)|(1u<<11)|(1u<<12));
        {   /* measured settled codes on this unit: 3=left 6=right 5=push */
            cur_pair = scan_wheel();
            int act = (cur_pair==3)?1 : (cur_pair==6)?2 : (cur_pair==5)?3 : 0;
            static int raw=0, cnt=0, stable=0;
            if(act==raw) cnt++; else { raw=act; cnt=0; }
            if(cnt>=1 && act!=stable){
                stable=act;
                if(act==1){ pal_id=(pal_id+2)%3; pal_init(); }
                else if(act==2){ pal_id=(pal_id+1)%3; pal_init(); }
                else if(act==3){ gamma_id=(gamma_id+1)%4; pal_init(); }
            }
            dbg[21]=(uint32_t)cur_pair;
        }
        dbg[22] |= (~GPIOC_IDR) & (1u<<13);

        /* wait for data-ready, but never forever */
        uint32_t tw=CYC; uint16_t st=0; int ok=0;
        while((CYC-tw) < 7200000u){                 /* 100 ms */
            if(mlx_read(0x8000,&st,1) && (st&0x0008)){ ok=1; break; }
        }
        dbg[7]=CYC-tw; dbg[10]=st;
        if(!ok){
            dbg[8]=++fails; dbg[9]=++recov;
            i2c_recover();
            mlx_write(0x800D, ctrl);
            mlx_write(0x8000, 0x0030);
            continue;
        }

        uint32_t t=CYC;
        /* aux first: the sensor has just finished writing, so these are stable */
        if(!mlx_read(0x0700, &frame[768], 64)){
            dbg[8]=++fails; dbg[9]=++recov;
            i2c_recover();
            continue;
        }
        if(!mlx_read(0x0400, frame, 768)){
            dbg[8]=++fails; dbg[9]=++recov;
            i2c_recover();
            continue;
        }
        dbg[6]=CYC-t;
        mlx_write(0x8000,0x0030);

        calc_frame_params(ctrl, st & 1);
        int32_t tc=(int32_t)(mlx_to(CENTER_IDX)*100.f);
        int32_t tn=(int32_t)(mlx_to(min_idx)*100.f);
        int32_t tx=(int32_t)(mlx_to(max_idx)*100.f);
        dbg[16]=(uint32_t)tc; dbg[17]=(uint32_t)tn; dbg[18]=(uint32_t)tx;
        dbg[19]=(uint32_t)(int32_t)(f_ta*100.f);

        /* don't feed the labels until the housekeeping values have validated once,
         * or the first frames after power-on show nonsense temperatures */
        if(lg_valid){ sum_c+=tc; sum_n+=tn; sum_x+=tx; nsamp++; }
        if(!primed && lg_valid){ disp_c=tc; disp_n=tn; disp_x=tx; primed=1; }
        if((CYC-t_lbl) >= 24000000u){          /* 1/3 second */
            if(nsamp){
                disp_c=sum_c/(int32_t)nsamp;
                disp_n=sum_n/(int32_t)nsamp;
                disp_x=sum_x/(int32_t)nsamp;
            }
            sum_c=sum_n=sum_x=0; nsamp=0; t_lbl=CYC;
        }

        correct_frame();

        dbg[14]=aux_rejects;

#ifndef DIAG
        if(VIEW_FILTER) denoise(); else acc_primed=0;
#endif

        t=CYC;
        render_prep();
        img_h = overlay_on ? (LCD_H-BAR_H) : LCD_H;
        render_band(0, img_h/2-1);
        if(overlay_on) draw_crosshair();
        render_band(img_h/2, img_h-1);
        if(overlay_on) draw_bar_if_changed();
        dbg[5]=CYC-t;


        frames++;
        if((CYC-t0) >= 72000000u){ dbg[4]=frames; frames=0; t0=CYC; }
    }
}
