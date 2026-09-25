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
static int gamma_id = 0;   /* 0=1.5  1=2.0  2=3.0 */

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
        uint32_t c3=(uint32_t)i*(uint32_t)i*(uint32_t)i;
        uint32_t r;
        if(gamma_id==0)      r=isqrt32(c3/255u);        /* 1.5 */
        else if(gamma_id==1) r=((uint32_t)i*i)/255u;    /* 2.0 */
        else                 r=c3/65025u;               /* 3.0 */
        gam[i]=(uint8_t)(r>255?255:r);
    }
}

/* bulk pixel write: CS and RS/DC are hoisted out of the loop by the caller */
static inline void px_fast(uint16_t v){
    GPIOB_ODR=0xFC00u|(uint8_t)(v>>8); GPIOC_BRR=PC15; GPIOC_BSRR=PC15;
    GPIOB_ODR=0xFC00u|(uint8_t)v;      GPIOC_BRR=PC15; GPIOC_BSRR=PC15;
}

static int16_t prev[768];
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

/* Per-pixel exponential moving average. NOISE_SHIFT 2 = ~200ms time constant
 * at 20fps; higher = quieter but smearier. Costs 768 ops/frame, i.e. nothing. */
#define NOISE_SHIFT 2
static int32_t acc[768];
static int acc_primed;
static void denoise(void){
    if(!acc_primed){
        for(int i=0;i<768;i++) acc[i]=(int32_t)frame[i]<<8;
        acc_primed=1;
    } else {
        for(int i=0;i<768;i++)
            acc[i] += ((((int32_t)frame[i])<<8) - acc[i]) >> NOISE_SHIFT;
    }
    for(int i=0;i<768;i++) frame[i]=(int16_t)(acc[i]>>8);
}

#include "cal.h"

/* gain-correct then subtract the per-pixel offset -- kills the fixed-pattern dots */
static void correct_frame(void){
    uint16_t g=0;
    mlx_read(0x070A,&g,1);
    int32_t gr=(int16_t)g; if(gr==0) gr=1;
    int32_t gfp=(gainEE*1024)/gr;
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
    uint32_t jump=0;
    for(int i=0;i<768;i++){
        int16_t v=frame[i];
        if(v<lo[3]){ int k=3; while(k>0 && v<lo[k-1]){ lo[k]=lo[k-1]; k--; } lo[k]=v; }
        if(v>hi[3]){ int k=3; while(k>0 && v>hi[k-1]){ hi[k]=hi[k-1]; k--; } hi[k]=v; }
        int32_t d=(int32_t)v-(int32_t)prev[i]; if(d<0)d=-d;
        if(d>2000) jump++;
    }
    int16_t mn=lo[3], mx=hi[3];
    for(int i=0;i<768;i++){ if(frame[i]==mn) min_idx=i; else if(frame[i]==mx) max_idx=i; }

    /* measure the checkerboard: mean of each chess-pattern subpage class */
    int32_t s0=0,s1=0;
    for(int r=0;r<SRC_H;r++)
        for(int c=0;c<SRC_W;c++)
            { int16_t v2=frame[r*SRC_W+c]; if(((r+c)&1)==0) s0+=v2; else s1+=v2; }
    dbg[12]=(uint32_t)(s0/384); dbg[13]=(uint32_t)(s1/384);
    for(int i=0;i<768;i++) prev[i]=frame[i];
    if(jump>dbg[3]) dbg[3]=jump;       /* worst frame seen so far */
    if(jump>50) dbg[11]++;             /* how many frames looked corrupt */

    /* Smooth the auto-range so one bad word can't wash out a whole frame. */
    if(!scale_primed){ mn_s=mn; mx_s=mx; scale_primed=1; }
    else { mn_s += ((int32_t)mn-mn_s)>>2; mx_s += ((int32_t)mx-mx_s)>>2; }

    int32_t span = mx_s-mn_s; if(span<1) span=1;
    r_inv10 = (255*65536)/span;
    r_base10 = mn_s;
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
        int wy1 = (int)((vpos>>8)&0xFF), wy0 = 256-wy1;
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
            for(int fx=0; fx<10; fx++){
                int k = (a*(10-fx) + b*fx) * 3277 >> 15;   /* /10 */
                px_fast(pal[k]);
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
    mlx_write(0x800D, ctrl);
    dbg[1]=ctrl;
    dbg[14]=mlx_read(0x2400, ee, 832);
    extract_offsets();
    extract_cal();
    dbg[15]=(uint32_t)gainEE;
    mlx_write(0x8000,0x0030);

    uint32_t frames=0, t0=CYC, iters=0, fails=0, recov=0;
    int32_t sum_c=0, sum_n=0, sum_x=0; uint32_t nsamp=0, t_lbl=CYC; int primed=0;
    while(1){
        dbg[2]=++iters;
        /* latch any spare input seen low -- press each control and I'll read these */
        adcnow[0]=adc_read(0); adcnow[1]=adc_read(1);
        {   uint32_t lv=adcnow[1];
            int b = (lv<1000)?2 : (lv<3000)?1 : 0;
            static int raw=-1, cnt=0, stable=0;
            if(b==raw) cnt++; else { raw=b; cnt=0; }
            if(cnt>=2 && b!=stable){
                stable=b;
                if(b==1) overlay_on=!overlay_on;   /* 2045 level toggles overlay */
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
                else if(act==3){ gamma_id=(gamma_id+1)%3; pal_init(); }
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
        if(!mlx_read(0x0400, frame, 832)){
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

        sum_c+=tc; sum_n+=tn; sum_x+=tx; nsamp++;
        if(!primed){ disp_c=tc; disp_n=tn; disp_x=tx; primed=1; }
        if((CYC-t_lbl) >= 24000000u){          /* 1/3 second */
            if(nsamp){
                disp_c=sum_c/(int32_t)nsamp;
                disp_n=sum_n/(int32_t)nsamp;
                disp_x=sum_x/(int32_t)nsamp;
            }
            sum_c=sum_n=sum_x=0; nsamp=0; t_lbl=CYC;
        }

        correct_frame();
        denoise();

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
