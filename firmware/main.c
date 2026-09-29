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

#define COLOR12 1
#define LCD_W 320
#define LCD_H 240
#define SRC_W 32
#define SRC_H 24
#define MLX_ADDR 0x33
#define NOISE_SHIFT_MAX 4  /* quiet-pixel averaging: 2^4 frames, ~5.6x noise reduction */
/* Sensor subpage rate: 7 = 64 Hz, 6 = 32 Hz.
 * Measured: 64 Hz gives 6.68 counts of temporal noise at 19 fps, 32 Hz gives
 * 4.01 at 16.2 fps. Per unit of wall-clock time that is 1.53 vs 1.00, so 32 Hz
 * is ~1.5x quieter for the same latency -- the 1.67x noise penalty outweighs
 * the 1.17x extra frames. 64 Hz also reinstates the aux-word corruption that
 * 32 Hz eliminated (0 rejects over 442 frames vs a handful per thousand). */
#define REFRESH_SEL 6   /* boot at 32 Hz */

/* Temporal filter: 1 = rolling box window (FIR), 0 = adaptive EMA (IIR).
 * The box is FIR, so a frame leaves the average completely after TFILT_N frames
 * instead of trailing off forever, and every pixel is smoothed by exactly the
 * same amount -- the EMA's per-pixel shift switching smooths neighbours
 * differently, which is itself visible as texture. Noise falls by sqrt(N) and
 * lag is (N-1)/2 frames, both exactly predictable.
 * It has no motion adaptation at all, so movement smears over the window. */
#define FILTER_BOX 1
/* RAM cap: TFILT_MAX * 768 * 2 bytes. Back to 4 now that the flat-field table
 * has been removed -- and worth having, because this sensor's visible noise is
 * essentially ALL temporal (see docs/PERFORMANCE.md), so averaging is the only
 * lever and it scales as sqrt(N). */
#define TFILT_MAX  4

#define NLO 39         /* 5th percentile of 768 = the 38th coldest, plus index 0 */
#define SPAN_MIN 52        /* minimum displayed span in raw counts (~6 degC) */
#if 1   /* smoothstep(fx/10)*256; the #else row is plain linear */
#define W1 7
#define W2 27
#define W3 55
#define W4 90
#define W5 128
#define W6 166
#define W7 201
#define W8 229
#define W9 249
#else
#define W1 26
#define W2 51
#define W3 77
#define W4 102
#define W5 128
#define W6 154
#define W7 179
#define W8 205
#define W9 230
#endif
#define SHARP_INTERP 1     /* 1 = smoothstep interpolation weights, 0 = plain bilinear.
                            * At 10x magnification a linear ramp spreads every edge over
                            * 10 display pixels. Smoothstep (t^2*(3-2t)) concentrates the
                            * transition into the middle ~5, which is roughly the edge
                            * width the 4x-magnified RP2040 camera gets for free, while
                            * staying C1-continuous so it does not look blocky.
                            * Same cost: 2 multiplies per pixel instead of 3. */

volatile uint32_t dbg[32];
static int16_t  frame[834];
static int min_idx=400, max_idx=400;

/* label values, averaged and refreshed at most 3x/sec */
static int32_t disp_c, disp_n, disp_x;
#define CENTER_IDX 400
#define BAR_H  20
#define TOP_H  18                 /* status bar: 7*GSC glyph + 2px box on each side */
static int img_y0 = TOP_H;
#define IMG_Y0 img_y0
#define IMG_H  img_h
static int img_h = LCD_H-BAR_H-TOP_H;
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
int view_mode = 0;   /* interpolation + temporal filter */
#define VIEW_INTERP  (view_mode != 1)
#define VIEW_FILTER  (view_mode == 0)
#define VIEW_DDE     (dde_on)
/* V: 0 = interp+filter, 1 = raw 10x10 blocks, 2 = interp only.
 * DDE and the filter length are separate fields now, so any combination is
 * reachable from the wheel rather than being encoded into the view mode. */
static int dde_on    = 1;
static int tfilt_n   = TFILT_MAX;   /* 3-frame rolling average */
static int refresh64 = (REFRESH_SEL == 7);
static int sel       = 0;   /* which status field the wheel adjusts: P V G B D H R */
#define SEL_N 7

/* R: 0 = our pipeline, 1 = the RP2040 reference pipeline reproduced faithfully.
 * R1 runs the full Melexis CalculateTo on ALL 768 pixels -- the one stage we
 * have never actually run -- so frame[] carries real temperature in tenths of a
 * degC instead of gain/offset-corrected raw counts, and the auto-range then uses
 * his exact formula. It is expensive (6 soft-float 4th-roots per pixel) and that
 * is fine: it exists to answer whether the per-pixel radiometry is what makes
 * his image look different, not to be shipped. */
static int ref_pipe = 0;

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
static uint16_t pal12[256];
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
#define I2C_HALF 2
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

static int pal_id = 3;   /* RP2040 reference palette + range is the default */
/* Counts per degC, measured live. It is NOT a constant: it scales with the ADC
 * resolution (~8.6 at 18-bit, ~5.25 at 19-bit on this unit) and drifts with gain
 * and Ta, so hard-coding it silently mis-sizes anything expressed in degC. */
static int32_t cpd100 = 600;
static int cpd_primed = 0;
/* The trimmed extremes in counts, instantaneous. mn_s/mx_s are EMA-smoothed and
 * must NOT be used to derive cpd100: tn/tx are computed from THIS frame's
 * min_idx/max_idx, so pairing them with smoothed counts mixes two different
 * moments and biases the ratio whenever the scene is changing. */
static int16_t mn_i = 0, mx_i = 0;
/* 4 = linear (1.0), the default: the RP2040 reference ramp has no gamma, so the
 * reference palette must boot linear to match it. The wheel push cycles
 * 1.0 -> 4.0 -> 3.0 -> 2.0 -> 1.5 -> 1.0 and now applies to every palette. */
static int gamma_id = 4;

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
        } else if(pal_id==3){                /* RP2040 reference (see docs/RP2040-COMPARISON.md) */
            /* André Weinand's heatmap: 7 anchors, linearly interpolated.
             * Reproduces heatmap_init() in his main.cpp exactly, including the
             * round() and the exact-anchor special case at i=0 and i=255. */
            static const uint8_t anch[7][3]={
                {  0,  0,  0},   /* black  */
                {  4, 51,255},   /* blue   */
                {  0,253,255},   /* cyan   */
                {  0,249,  0},   /* green  */
                {255,255,  0},   /* yellow */
                {255, 38,  0},   /* red    */
                {255,255,255}    /* white  */
            };
            int num=i*6, k=num/255, rem=num%255;
            if(rem==0){ r=anch[k][0]; g=anch[k][1]; b=anch[k][2]; }
            else{
                /* c1 + round((c2-c1)*rem/255) */
                int r1=anch[k][0], g1=anch[k][1], b1=anch[k][2];
                int r2=anch[k+1][0], g2=anch[k+1][1], b2=anch[k+1][2];
                r = r1 + (2*(r2-r1)*rem + (r2>=r1?255:-255))/510;
                g = g1 + (2*(g2-g1)*rem + (g2>=g1?255:-255))/510;
                b = b1 + (2*(b2-b1)*rem + (b2>=b1?255:-255))/510;
            }
        } else {                             /* rainbow (default) */
            if      (i< 48){ int u=i;     r=0;            g=0;            b=(u*255)/47; }
            else if (i<112){ int u=i-48;  r=0;            g=(u*255)/63;   b=255; }
            else if (i<176){ int u=i-112; r=0;            g=255;          b=255-(u*255)/63; }
            else if (i<216){ int u=i-176; r=(u*255)/39;   g=255;          b=0; }
            else           { int u=i-216; r=255;          g=255-(u*255)/39; b=0; }
        }
        pal[i]=(uint16_t)(((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3));
        /* RGB444 for the image blast: 4096 colours against a 256-entry palette,
         * so nothing is lost, and it is 3 bus bytes per 2 pixels instead of 4 */
        pal12[i]=(uint16_t)(((r&0xF0)<<4)|(g&0xF0)|(b>>4));
    }
    for(int i=0;i<256;i++){
        uint32_t c2=(uint32_t)i*(uint32_t)i;
        uint32_t c3=c2*(uint32_t)i;
        uint32_t r;
        /* 255*(i/255)^g ; i^4 peaks at 4228250625, which still fits a uint32 */
        if(gamma_id==4)      r=(uint32_t)i;             /* 1.0, linear */
        else if(gamma_id==0) r=(c2*c2)/16581375u;       /* 4.0 */
        else if(gamma_id==1) r=c3/65025u;               /* 3.0 */
        else if(gamma_id==2) r=c2/255u;                 /* 2.0 */
        else                 r=isqrt32(c3/255u);        /* 1.5 */
        gam[i]=(uint8_t)(r>255?255:r);
    }
}

/* bulk pixel write: CS and RS/DC are hoisted out of the loop by the caller */
static inline void wr8(uint8_t b){
    GPIOB_ODR=0xFC00u|b; GPIOC_BRR=PC15; GPIOC_BSRR=PC15;
}
/* two RGB444 pixels in three bytes: [R1G1][B1R2][G2B2] */
static inline void px_pair(uint16_t c1,uint16_t c2){
    wr8((uint8_t)(c1>>4));
    wr8((uint8_t)(((c1&0xFu)<<4)|(c2>>8)));
    wr8((uint8_t)c2);
}
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
#if FILTER_BOX
static int16_t hist[TFILT_MAX][768];
static int hist_pos=0, hist_fill=0;
#else
static int32_t acc[768];
#endif
#if !FILTER_BOX
static int acc_primed;
static int32_t noise_est = 4000;      /* fixed point <<8 */
static int16_t trend[768];            /* EMA of the SIGNED delta, scaled >>4 */
#endif

/* A/B test of the MLX90640 ADC resolution (0x800D bits 11:10).
 * Raw counts scale with resolution, so absolute noise is not comparable --
 * normalise to the scene span. Alternating legs keeps the scene identical.
 * abtest[] = {mean normalised noise 18-bit, same 19-bit, samples, samples} */
volatile uint32_t abtest[4];

#if FILTER_BOX
static void denoise(void){
    for(int i=0;i<768;i++) hist[hist_pos][i]=frame[i];
    hist_pos = (hist_pos+1>=tfilt_n) ? 0 : hist_pos+1;
    if(hist_fill<tfilt_n) hist_fill++;
    for(int i=0;i<768;i++){
        int32_t a=0;
        for(int k=0;k<hist_fill;k++) a+=hist[k][i];
        frame[i]=(int16_t)(a/hist_fill);
    }
}
#else
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
}
#endif

/* ---- DDE: digital detail enhancement ---------------------------------------
 * Split the frame into a base and a detail layer, core the noise out of the
 * detail, boost what survives, recombine. This is the standard technique in
 * commercial thermal imagers and it is the only one that gives uniform
 * interiors AND crisp edges at once -- a global tone curve cannot, because it
 * has to pick one slope per brightness.
 *
 * Two details matter:
 *  - the base is an EDGE-AWARE low-pass (neighbours only average in when they
 *    are within ~3 MAD), so edges stay in the base instead of leaking into the
 *    detail layer and getting boosted into halos. A plain blur here is what
 *    made the spatial stage we removed earlier useless.
 *  - the detail layer is CORED: anything below ~1 MAD is set to zero rather
 *    than amplified. Without this, boosting detail boosts the noise we just
 *    spent the temporal filter removing.
 * Cost is ~40k operations on 768 pixels, against ~1.2M cycles per frame
 * currently spent spinning on the sensor's data-ready flag. */
#define DDE_GAIN 28        /* 16 = 1.0x (off), 28 = 1.75x */
static int16_t dde_base[768];

/* Median |difference| between horizontally adjacent pixels.
 * noise_est must NOT be used here: it is the temporal filter's mean
 * frame-to-frame delta, so it tracks SCENE MOTION as much as noise -- measured
 * at 24.21 counts on a moving hand, against a scene span of 20-60. Feeding that
 * in made sim=72 and core=24, so every neighbour counted as "similar" (the
 * edge-aware base degenerating into the plain blur we removed earlier) and
 * essentially all detail was cored away.
 * The median is robust because edges are a small minority of pixel pairs. For
 * gaussian noise, median|delta| ~ 0.954*sigma, so this is sigma to within 5%. */
static int32_t noise_spatial(void){
    uint16_t h[33]; for(int i=0;i<33;i++) h[i]=0;
    int n=0;
    for(int r=0;r<SRC_H;r++)
        for(int c=0;c<SRC_W-1;c++){
            int p=r*SRC_W+c;
            int32_t d=(int32_t)frame[p+1]-(int32_t)frame[p];
            if(d<0) d=-d;
            if(d>32) d=32;
            h[d]++; n++;
        }
    int half=n>>1, acc=0;
    for(int i=0;i<33;i++){ acc+=h[i]; if(acc>=half) return i<1?1:i; }
    return 1;
}

static void dde(void){
    int32_t nz = noise_spatial();
    dbg[11]=(uint32_t)nz;
    const int32_t sim  = nz*3;                 /* "same surface" threshold */
    const int32_t core = nz;                   /* below this, detail is noise */
    /* 4096/n, so sum*rcp>>12 cannot overflow the way a 65536-scaled one would */
    static const uint16_t rcp[10]={0,4096,2048,1365,1024,819,683,585,512,455};

    for(int r=0;r<SRC_H;r++){
        for(int c=0;c<SRC_W;c++){
            int p=r*SRC_W+c;
            int32_t c0=frame[p], sum=c0; int n=1;
            for(int dr=-1;dr<=1;dr++){
                int rr=r+dr; if(rr<0||rr>=SRC_H) continue;
                for(int dc=-1;dc<=1;dc++){
                    if(!dr && !dc) continue;
                    int cc=c+dc; if(cc<0||cc>=SRC_W) continue;
                    int32_t v=frame[rr*SRC_W+cc];
                    int32_t d=v-c0; if(d<0) d=-d;
                    if(d<=sim){ sum+=v; n++; }
                }
            }
            dde_base[p]=(int16_t)((sum*(int32_t)rcp[n])>>12);
        }
    }
    for(int i=0;i<768;i++){
        int32_t d=(int32_t)frame[i]-(int32_t)dde_base[i];
        int32_t ad=d<0?-d:d;
        if(ad<=core) d=0;
        else{ d = (d>0)?(d-core):(d+core); d = (d*DDE_GAIN)>>4; }
        int32_t v=(int32_t)dde_base[i]+d;
        frame[i]=(int16_t)(v>32767?32767:(v<-32768?-32768:v));
    }
}
#endif


#include "cal.h"

/* No flat-field / non-uniformity correction, and this is deliberate -- it has
 * now been tried and rejected TWICE.
 *
 * The second attempt (button-triggered, 32-frame averaged, against surfaces the
 * user chose) was measured with a two-pass test: capture two different uniform
 * surfaces and correlate the tables. Result r = +0.514 overall, and only
 * +0.436 on the high-pass part. That is roughly half real structure and half
 * noise, so applying it removes ~1.4 counts of genuine fixed error while
 * writing in ~1.4 counts of new fixed error -- a wash.
 *
 * The decisive detail: the high-pass correlation is LOWER than the overall one.
 * The reproducible component is SMOOTH, not pixel-to-pixel, so what the capture
 * finds is lens vignetting rather than fixed-pattern noise. Vignetting is
 * multiplicative and would need a gain correction, not this offset one.
 *
 * Confirmed by eye: no stuck or static pixels are visible on this sensor, only
 * random noise. poff[] already removes the fixed pattern, and what is left is
 * temporal -- which is why the temporal filter (B) is the only lever that
 * helps, and why TFILT_MAX is worth spending RAM on. */

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
#define ALPHA_NORM 1   /* per-pixel sensitivity normalisation; 0 to A/B it */

/* 1/alpha[i], normalised to the array mean, in 1/4096 units.
 * Measured on this sensor's EEPROM: alpha has sigma 9.93%, peak-to-peak 42.8%,
 * and it is MULTIPLICATIVE, so the error it causes scales with signal -- a
 * couple of counts on an ambient wall, ~1.7 degC across a hand. Correlation of
 * the uncorrected image against alpha on a flat field measured r = -0.690.
 * The reference RP2040 firmware gets this for free inside CalculateTo; we skip
 * CalculateTo for the image (768 soft-float conversions would cost most of the
 * frame rate), so the division has to happen here instead. One multiply and a
 * shift per pixel. */
static uint16_t arcp[768];

static void build_alpha_rcp(void){
    float sum=0.f;
    for(int i=0;i<768;i++) sum += px_alpha(i);
    float mean = sum/768.f;
    for(int i=0;i<768;i++){
        float a = px_alpha(i);
        int32_t q = (a>0.f) ? (int32_t)((mean/a)*4096.f+0.5f) : 4096;
        if(q<1024) q=1024; else if(q>16383) q=16383;
        arcp[i]=(uint16_t)q;
    }
}

static void correct_frame(void){
    /* use the gain already validated in calc_frame_params -- a second I2C read
     * here was a second chance to catch a mid-update word, and its gr==0
     * fallback multiplied every pixel by ~6200 and saturated the frame */
    int32_t gfp = lg_gfp;
    for(int i=0;i<768;i++){
        int32_t v=(((int32_t)frame[i]*gfp)>>10) - poff[i];
#if ALPHA_NORM
        v = (v * (int32_t)arcp[i]) >> 12;
#endif
        frame[i]=(int16_t)(v>32767?32767:(v<-32768?-32768:v));
    }
}
static int32_t mn_s, mx_s;
static int scale_primed;

static int32_t r_inv10, r_base10;

static void render_prep(void){
    /* Trimmed range: ignore the 4 most extreme pixels at each end, so a single
     * corrupted word cannot blow out the auto-scale (that was the blue flash). */
    /* Keep the NLO coldest in ascending order: lo[3] is the 4th-coldest the OSD
     * and the other palettes use, lo[NLO-1] is the 5th percentile that anchors
     * black for the reference palette. Anchoring on the ABSOLUTE minimum (what
     * the RP2040 firmware does) only yields a black background when the
     * background is itself the noisiest, coldest thing in frame -- true for its
     * unfiltered image, false for ours, which left the background floating at
     * index ~75 (dark blue) with just 2 pixels reaching black.
     * The early-out on lo[NLO-1] rejects ~95% of pixels after warm-up, so the
     * wider array costs little. */
    int16_t lo[NLO]; for(int k=0;k<NLO;k++) lo[k]=32767;
    int16_t hi[4]={-32768,-32768,-32768,-32768};
    for(int i=0;i<768;i++){
        int16_t v=frame[i];
        if(v<lo[NLO-1]){ int k=NLO-1; while(k>0 && v<lo[k-1]){ lo[k]=lo[k-1]; k--; } lo[k]=v; }
        if(v>hi[3]){ int k=3; while(k>0 && v>hi[k-1]){ hi[k]=hi[k-1]; k--; } hi[k]=v; }
    }
    /* Two different jobs, two different answers.
     * The auto-RANGE uses the trimmed extremes, because one spiked pixel really
     * would wreck the image scaling.
     * The LABELS use the true extremes. Reporting the 4th-hottest pixel as "max"
     * meant that aiming the crosshair at a hot spot covering three pixels or
     * fewer showed a centre reading ABOVE the maximum, which is incoherent. The
     * trim was never what protected against the old ~80 degC jumps anyway --
     * those came from corrupted gain/Ta words that scale every pixel, and
     * calc_frame_params' range validation is what fixes those. */
    int16_t mn=lo[3], mx=hi[3];              /* trimmed -> auto-range only */

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
    if(ref_pipe){
        /* step = (ceil(max+1) - floor(min-1)) / 255, index = (v - min)/step.
         * Note he subtracts the UNPADDED min but divides by the PADDED span, so
         * the coldest pixel lands on 0 and the pad is spent entirely at the top.
         * Working in tenths, "whole degrees" means multiples of 10. */
        int32_t mn_dd=lo[0], mx_dd=hi[0];
        int32_t a=mx_dd+10, b=mn_dd-10;
        int32_t hi_p = (a>=0) ? ((a+9)/10)*10 : -(((-a)/10)*10);   /* ceil  to 1 degC */
        int32_t lo_p = (b>=0) ? (b/10)*10 : -((((-b)+9)/10)*10);   /* floor to 1 degC */
        span = hi_p - lo_p;
        if(span<1) span=1;
        lo_d = mn_dd;
        r_inv10 = (255*65536)/span;
        r_base10 = lo_d;
        return;
    }
    if(pal_id==3){
        /* RP2040 reference range: the TRUE frame extremes (no 4th-extreme trim),
         * no inter-frame EMA, and no minimum span -- then padded, which is his
         *   step = (ceil(max+1) - floor(min-1)) / 255
         * That pad averages 3 degC over the two ends; at the measured 8.6
         * counts/degC on this unit that is ~13 counts each side. The exact
         * ceil/floor to integer degC is not reproducible here because the image
         * path carries raw counts with an arbitrary zero, not absolute degC --
         * it is a +/-0.5 degC jitter in the range and nothing else. */
        /* His index is (value - min) / step with step = padded_span/255, i.e. he
         * subtracts the UNPADDED min but divides by the PADDED span. So the
         * coldest pixel lands on index 0 -- pure black -- and the pad is spent
         * entirely at the top, where his hottest pixel reaches only
         * 255*(max-min)/padded_span and never gets to white. Padding both ends
         * symmetrically (what I did first) lifts the coldest pixel to index ~33,
         * a strong blue instead of black, which is visibly wrong at the dark end. */
        lo_d = (int32_t)lo[NLO-1];   /* 5th percentile, not the absolute min */
        int32_t pad = (3*cpd100)/100;            /* his ceil/floor pad averages 3 degC */
        if(pad<4) pad=4; else if(pad>60) pad=60;  /* guard a wild cpd100 */
        /* hi[3], NOT hi[0]. The absolute maximum lets a single corrupted or
         * noise-spiked pixel blow the span up, collapsing every real pixel to
         * index ~0 -- the whole thermal image goes black for one frame while the
         * info bar, drawn separately, survives. That is the same failure as the
         * old "blue flash", which the 4-pixel trim fixed for the other palettes;
         * matching the reference firmware's untrimmed max reintroduced it here. */
        span = ((int32_t)hi[3] - lo_d) + pad;
        hi_d = lo_d + span;
    }
    else if(span < SPAN_MIN){
        int32_t mid=(hi_d+lo_d)/2;
        lo_d = mid - SPAN_MIN/2;
        hi_d = mid + SPAN_MIN/2;
        span = SPAN_MIN;
    }
    if(span<1) span=1;
    r_inv10 = (255*65536)/span;
    r_base10 = lo_d;
    /* A whole-image colour flash has two possible causes: the pixel stream
     * slipping, or the auto-range collapsing so every pixel lands on one
     * palette entry. Count the latter so the two can be told apart rather
     * than guessed at. */
    { static int32_t prev_span = 0;
      if(prev_span > 0 && (span > prev_span*3 || span*3 < prev_span)) dbg[27]++;
      prev_span = span; }
}

/* Render one horizontal band, so overlays can be drawn immediately after the
 * rows beneath them are painted. Drawing them all at the end meant the blast
 * erased them for most of each frame -- that was the every-other-frame flicker. */
static void render_band(int Y0,int Y1){
#if COLOR12
    lcd_cmd(0x3A); lcd_dat(0x03);        /* COLMOD: 12 bit/px for the image */
#endif
    lcd_window(0,LCD_W-1,(uint16_t)Y0,(uint16_t)Y1);
    GPIOA_BSRR=PA2;
    GPIOA_BRR =PA3;

    const uint32_t vstep = ((uint32_t)SRC_H<<16)/(uint32_t)img_h;

    for(int Y=Y0; Y<=Y1; Y++){
        uint32_t vpos = (uint32_t)(Y-IMG_Y0)*vstep;
        int sy = (int)(vpos>>16);
        if(sy>SRC_H-1) sy=SRC_H-1;
        int wy1 = VIEW_INTERP ? (int)((vpos>>8)&0xFF) : 0;
#if SHARP_INTERP
        wy1 = (int)(((uint32_t)wy1*(uint32_t)wy1*(768u-2u*(uint32_t)wy1))>>16);
#endif
        int wy0 = 256-wy1;
        const int16_t *r0=&frame[sy*SRC_W];
        const int16_t *r1=&frame[(sy<SRC_H-1?sy+1:sy)*SRC_W];
        uint8_t idxc[SRC_W+1];
        for(int i=0;i<SRC_W;i++){
            int32_t c = ((int32_t)r0[i]*wy0 + (int32_t)r1[i]*wy1)>>8;
            int32_t k = ((c - r_base10) * r_inv10) >> 16;
            idxc[i] = gam[(uint8_t)(k<0 ? 0 : (k>255 ? 255 : k))];
        }
        idxc[SRC_W] = idxc[SRC_W-1];

        /* a*(256-w) + b*w, all >>8, is algebraically a + ((b-a)*w >> 8): one
         * multiply instead of two. Fully unrolled with the weights as literals,
         * so there is no table load, no index arithmetic and no loop overhead
         * on the hottest loop in the firmware -- 64640 iterations per frame. */
#if COLOR12
#define PL pal12
#define EMIT(c1,c2) px_pair((c1),(c2))
#else
#define PL pal
#define EMIT(c1,c2) do{ px_fast(c1); px_fast(c2); }while(0)
#endif
#define B12(W) PL[a + (((d*(W))>>8))]
        for(int sx=0; sx<SRC_W; sx++){
            int a = idxc[sx];
            if(!VIEW_INTERP){
                uint16_t c = PL[a];
                EMIT(c,c); EMIT(c,c); EMIT(c,c); EMIT(c,c); EMIT(c,c);
            } else {
                int d = (int)idxc[sx+1] - a;
                EMIT(PL[a],     B12(W1));
                EMIT(B12(W2),   B12(W3));
                EMIT(B12(W4),   B12(W5));
                EMIT(B12(W6),   B12(W7));
                EMIT(B12(W8),   B12(W9));
            }
        }
#undef B12
#undef EMIT
#undef PL
    }
    GPIOA_BSRR=PA3;            /* release CS */
#if COLOR12
    lcd_cmd(0x3A); lcd_dat(0x05);        /* back to 16 bit/px for the overlay */
#endif
}

#include "gfx.h"

/* Top status bar: which palette, view mode, gamma and filter are live, so a
 * described setting and an observed image can be matched up. It sits above the
 * thermal image rather than over it, so the render never paints across it and
 * it only redraws when something actually changes.
 *   P<n>  palette   0 rainbow / 1 ironbow / 2 grayscale / 3 RP2040 reference
 *                    (3 is the boot default, so the cycle reads 3,0,1,2)
 *   V<n>  view mode 0 filter+DDE / 1 raw blocks / 2 interp only / 3 filter, DDE off
 *   G<x>  gamma
 *   B<n>  box filter over n frames; 0 = off entirely (1 is skipped, being
 *         the identity), E = adaptive EMA instead
 *   D<n>  DDE on/off */
static int st_field(int x,int idx,int8_t lead,const int8_t *d,int nd){
    int8_t g[8]; int n=0;
    g[n++]=lead;
    for(int i=0;i<nd;i++) g[n++]=d[i];
    g[n]=-1;
    draw_glyphs(x, 2, g, (sel==idx)?C_SEL:C_WHITE, C_BLACK);
    return x + glyphs_w(g) + GW/2;
}

static int fps_disp = 0;
static int fps_dirty = 1;

/* Right-aligned in the status bar. The bar sits above the image, so nothing
 * repaints over it and this only runs when the number actually changes. */
static void draw_fps_if_changed(void){
    static int l_f=-1, l_on=-1;
    if(!fps_dirty && fps_disp==l_f && overlay_on==l_on) return;
    fps_dirty=0; l_f=fps_disp; l_on=overlay_on;
    if(!overlay_on) return;
    int v=fps_disp; if(v>99) v=99; if(v<0) v=0;
    int8_t g[4]; int n=0;
    if(v>=10) g[n++]=(int8_t)(v/10);
    g[n++]=(int8_t)(v%10);
    g[n]=-1;
    draw_glyphs(LCD_W-4-glyphs_w(g), 2, g, C_CYAN, C_BLACK);
}

static void draw_status_if_changed(void){
    static int l_p=-1,l_v=-1,l_g=-1,l_on=-1,l_b=-1,l_d=-1,l_h=-1,l_s=-1,l_r=-1;
    if(pal_id==l_p && view_mode==l_v && gamma_id==l_g && overlay_on==l_on
       && tfilt_n==l_b && dde_on==l_d && refresh64==l_h && sel==l_s
       && ref_pipe==l_r) return;
    l_p=pal_id; l_v=view_mode; l_g=gamma_id; l_on=overlay_on;
    l_b=tfilt_n; l_d=dde_on; l_h=refresh64; l_s=sel; l_r=ref_pipe;

    fill_rect(0,0,LCD_W,TOP_H,C_BLACK);
    fps_dirty=1;              /* the wipe took the fps with it */
    if(!overlay_on) return;

    static const int8_t gdig[5][3] = {{4,GL_DOT,0},{3,GL_DOT,0},{2,GL_DOT,0},
                                      {1,GL_DOT,5},{1,GL_DOT,0}};
    int8_t d[3]; int x=4;
    d[0]=(int8_t)pal_id;                       x=st_field(x,0,GL_P,d,1);
    d[0]=(int8_t)view_mode;                    x=st_field(x,1,GL_V,d,1);
                                               x=st_field(x,2,GL_G,gdig[gamma_id],3);
#if FILTER_BOX
    d[0]=(int8_t)tfilt_n;                      x=st_field(x,3,GL_B,d,1);
#else
    d[0]=-1;                                   x=st_field(x,3,GL_E,d,0);
#endif
    d[0]=(int8_t)(dde_on?1:0);                 x=st_field(x,4,GL_D,d,1);
    d[0]=refresh64?6:3; d[1]=refresh64?4:2;    x=st_field(x,5,GL_H,d,2);
    d[0]=(int8_t)(ref_pipe?1:0);               x=st_field(x,6,GL_R,d,1);
    (void)x;
}

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
    /* 32 Hz subpage rate. Measured (docs/HARDWARE.md): halves temporal noise
     * (6.68 -> 4.01 counts at 19-bit, the sqrt(2) expected from doubled
     * integration time) and leaves FPN unchanged. Costs ~3 fps, landing at ~16,
     * which is exactly where the RP2040 reference camera runs. It also makes a
     * full 832-word read fit inside one 31.2 ms update period instead of
     * straddling it, which is the structural fix for the corrupted aux words. */
    ctrl = (uint16_t)((ctrl & ~(7u<<7)) | ((uint32_t)REFRESH_SEL<<7));
    /* 19-bit ADC. Measured on this unit (docs/HARDWARE.md): 18-bit carries 86.6
     * counts of fixed-pattern noise against 19-bit's 25.2 -- 3.4x worse -- while
     * costing only 3.26 vs 6.68 counts of temporal noise, which poff[] cannot
     * remove and the temporal filter cannot touch. The RP2040 camera uses 19-bit
     * (MLX90640_SetResolution(addr,3)), which is why its uniform areas are clean. */
    ctrl = (uint16_t)((ctrl & ~(3u<<10)) | (3u<<10));  /* 19-bit ADC */
    mlx_write(0x800D, ctrl);
    dbg[1]=ctrl;
    dbg[14]=mlx_read(0x2400, ee, 832);
    extract_offsets();
    extract_cal();
    f4rt_init();
    build_alpha_rcp();
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
        /* Measured, not derived from the stage timers: count whole loop
         * iterations against DWT cycles over a one-second window. */
        { static uint32_t fps_t0=0; static uint32_t fps_n=0;
          fps_n++;
          uint32_t dt=CYC-fps_t0;
          if(dt >= 72000000u){                    /* one second at 72 MHz */
              fps_disp = (int)((fps_n*72000000u)/dt);
              fps_n=0; fps_t0=CYC;
          } }
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
        {   /* measured settled codes on this unit: 3=left 6=right 5=push */
            cur_pair = scan_wheel();
            int act = (cur_pair==3)?1 : (cur_pair==6)?2 : (cur_pair==5)?3 : 0;
            static int raw=0, cnt=0, stable=0;
            if(act==raw) cnt++; else { raw=act; cnt=0; }
            if(cnt>=1 && act!=stable){
                stable=act;
                if(act==3){ sel=(sel+1)%SEL_N; }          /* push: next field */
                else if(act==1 || act==2){                 /* left/right: adjust */
                    int d = (act==2) ? 1 : -1;
                    switch(sel){
                    case 0: pal_id=(pal_id+4+d)%4;      pal_init(); break;
                    case 1: view_mode=(view_mode+3+d)%3;            break;
                    case 2: gamma_id=(gamma_id+5+d)%5;  pal_init(); break;
                    /* B cycles 0, 2, 3 ... TFILT_MAX. 1 is skipped because a
                     * one-frame rolling average returns the frame unchanged --
                     * it is the identity, but still copies 768 pixels into the
                     * history and divides them all by one. 0 is a real bypass:
                     * denoise() is not called at all. */
                    case 3: if(d>0) tfilt_n = (tfilt_n==0) ? 2
                                            : (tfilt_n>=TFILT_MAX) ? 0 : tfilt_n+1;
                            else    tfilt_n = (tfilt_n==0) ? TFILT_MAX
                                            : (tfilt_n<=2) ? 0 : tfilt_n-1;
                            hist_fill=0; hist_pos=0;                break;
                    case 4: dde_on = !dde_on;                       break;
                    case 6: ref_pipe = !ref_pipe;
                            hist_fill=0; hist_pos=0;                break;
                    case 5: refresh64 = !refresh64;
                            { uint16_t c=0;
                              if(mlx_read(0x800D,&c,1)){
                                  c=(uint16_t)((c & ~(7u<<7)) | ((refresh64?7u:6u)<<7));
                                  mlx_write(0x800D,c);
                                  if(mlx_read(0x800D,&c,1)) ctrl=c;  /* read back */
                                  dbg[1]=ctrl;
                              } }
                            hist_fill=0; hist_pos=0;                break;
                    }
                }
            }
        }

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
        /* Locate this frame's hottest and coldest pixels BEFORE correct_frame,
         * while the raw values mlx_to() needs are still present, by computing
         * what the correction will produce without storing it. Taking them from
         * render_prep instead meant the labels used the PREVIOUS frame's
         * indices, so the centre could read hotter than the maximum -- three
         * numbers sampled from three different moments. */
        if(!ref_pipe){
            int32_t gfp2=lg_gfp, mv=2147483647, xv=-2147483647-1;
            for(int p=0;p<768;p++){
                int32_t v=(((int32_t)frame[p]*gfp2)>>10) - poff[p];
                if(v<mv){ mv=v; min_idx=p; }
                if(v>xv){ xv=v; max_idx=p; }
            }
            mn_i=(int16_t)mv; mx_i=(int16_t)xv;
        }

        /* The reference pipeline must convert BEFORE the labels are taken, or
         * they get recomputed from raw against indices belonging to the
         * previous frame -- which is how min ended up reading above max. */
        if(ref_pipe){
            uint32_t t0=CYC;
            ref_convert_frame();
            dbg[24]=CYC-t0;
        }
        int32_t tc,tn,tx;
        if(ref_pipe){
            /* frame[] already IS temperature, so scan it directly. Using the
             * previous frame's min_idx/max_idx instead gave min above max: on a
             * near-uniform scene the extreme pixel moves every frame, so last
             * frame's location is an arbitrary pixel. The reference firmware
             * likewise takes the true min/max of the current frame. */
            int16_t vmn=frame[0], vmx=frame[0];
            for(int p=1;p<768;p++){
                int16_t v=frame[p];
                if(v<vmn) vmn=v; else if(v>vmx) vmx=v;
            }
            tc=(int32_t)frame[CENTER_IDX]*10;
            tn=(int32_t)vmn*10;
            tx=(int32_t)vmx*10;
        }else{
            tc=(int32_t)(mlx_to(CENTER_IDX)*100.f);
            tn=(int32_t)(mlx_to(min_idx)*100.f);
            tx=(int32_t)(mlx_to(max_idx)*100.f);
        }
        dbg[16]=(uint32_t)tc; dbg[17]=(uint32_t)tn; dbg[18]=(uint32_t)tx;
        dbg[19]=(uint32_t)(int32_t)(f_ta*100.f);
        /* mn_i/mx_i are those same two pixels in counts, so their ratio to the
         * temperature difference is the scale factor -- no constant needed. */
        /* Needs real thermal contrast to be well-conditioned: it is a ratio, and
         * on a near-uniform scene the denominator is mostly noise. A 2 degC
         * threshold let a 2.4 degC scene through and produced 10.50 counts/degC
         * where the true value was ~5.7 -- nearly 2x wrong. Require 5 degC, and
         * smooth it so a single bad frame cannot move it far. */
        if(lg_valid && (tx-tn) > 500){
            int32_t c = ((int32_t)(mx_i-mn_i)*10000)/(tx-tn);
            if(c > 100 && c < 3000){
                if(!cpd_primed){ cpd100 = c; cpd_primed = 1; }
                else             cpd100 += (c - cpd100) >> 3;   /* ~8-frame EMA */
            }
        }
        dbg[23]=(uint32_t)cpd100;

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

        if(!ref_pipe) correct_frame();

        dbg[14]=aux_rejects;

#ifndef DIAG
#if FILTER_BOX
        if(VIEW_FILTER && tfilt_n>=2) denoise(); else hist_fill=0;
#else
        if(VIEW_FILTER) denoise(); else acc_primed=0;
#endif
        /* measure post-filter, pre-DDE spatial noise every frame, so the B
         * setting can be A/B'd without eyeballing it */
        dbg[11]=(uint32_t)noise_spatial();
        { uint32_t td=CYC; if(VIEW_DDE) dde(); dbg[3]=CYC-td; }
#endif

        t=CYC;
        { uint32_t tp=CYC; render_prep(); dbg[25]=CYC-tp; }
        img_y0 = overlay_on ? TOP_H : 0;
        img_h  = overlay_on ? (LCD_H-BAR_H-TOP_H) : LCD_H;
        /* One band, then the crosshair on top. This used to be two bands with
         * the crosshair drawn between them, so the second band would not
         * repaint its lower half. Drawing it after the whole image is simpler,
         * and halves the COLMOD switching when COLOR12 is on. */
        { uint32_t tb=CYC;
          render_band(img_y0, img_y0+img_h-1);
          if(overlay_on) draw_crosshair();
          dbg[26]=CYC-tb; }
        draw_status_if_changed();
        draw_fps_if_changed();
        if(overlay_on) draw_bar_if_changed();
        dbg[5]=CYC-t;


        frames++;
        if((CYC-t0) >= 72000000u){ dbg[4]=frames; frames=0; t0=CYC; }
    }
}
