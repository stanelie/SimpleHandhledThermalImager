/* Overlay drawing: 5x7 font, crosshair, roaming min/max markers.
 * Drawn after the image blast via small windows, so cost tracks overlay area only. */

#define RGB(r,g,b) ((uint16_t)((((r)&0xF8)<<8)|(((g)&0xFC)<<3)|((b)>>3)))
#define C_WHITE RGB(255,255,255)
#define C_BLACK RGB(0,0,0)
#define C_RED   RGB(255,60,60)
#define C_CYAN  RGB(80,220,255)

/* the panel mirrors X, so overlay coords are given in screen space and mapped here */
static inline int mx(int x,int w){ return LCD_W - x - w; }

/* glyphs: 5 columns, bit0 = top row */
static const uint8_t font5x7[][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x60,0x60,0x00,0x00}, /* 10 . */
    {0x08,0x08,0x08,0x08,0x08}, /* 11 - */
    {0x3E,0x41,0x41,0x41,0x22}, /* 12 C */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 13 M */
    {0x00,0x41,0x7F,0x41,0x00}, /* 14 I */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 15 N */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 16 A */
    {0x63,0x14,0x08,0x14,0x63}, /* 17 X */
    {0x00,0x00,0x00,0x00,0x00}, /* 18 space */
};

static void fill_rect(int x,int y,int w,int h,uint16_t c){
    if(x<0||y<0||x+w>LCD_W||y+h>LCD_H||w<=0||h<=0) return;
    x = mx(x,w);
    lcd_window((uint16_t)x,(uint16_t)(x+w-1),(uint16_t)y,(uint16_t)(y+h-1));
    GPIOA_BSRR=PA2; GPIOA_BRR=PA3;
    for(int i=0;i<w*h;i++) px_fast(c);
    GPIOA_BSRR=PA3;
}

#define GSC 2                              /* glyph scale -> 10x14 px */
#define GW  (5*GSC+GSC)                    /* advance incl. gap, gap painted in bg */

static int g_fv=0, g_fh=1;          /* confirmed empirically: rows fine, columns mirrored */


static void draw_glyph(int x,int y,int g,uint16_t fg,uint16_t bg){
    if(x<0||y<0||x+5*GSC>LCD_W||y+7*GSC>LCD_H) return;
    x = mx(x,5*GSC);
    lcd_window((uint16_t)x,(uint16_t)(x+5*GSC-1),(uint16_t)y,(uint16_t)(y+7*GSC-1));
    GPIOA_BSRR=PA2; GPIOA_BRR=PA3;
    for(int r=0;r<7*GSC;r++){
        int sr=g_fv ? 6-(r/GSC) : (r/GSC);
        for(int c=0;c<5*GSC;c++){
            int sc=g_fh ? 4-(c/GSC) : (c/GSC);
            px_fast((font5x7[g][sc]>>sr)&1 ? fg : bg);
        }
    }
    GPIOA_BSRR=PA3;
}

/* text as glyph indices, terminated by -1 */
static int glyphs_w(const int8_t *g){
    int n=0; while(g[n]>=0) n++;
    return n ? (n-1)*GW + 5*GSC : 0;
}

static void draw_glyphs(int x,int y,const int8_t *g,uint16_t fg,uint16_t bg){
    int n=0; while(g[n]>=0) n++;
    if(!n) return;
    /* background box, 1px proud of the text on every side, so characters never
     * touch the thermal image and the inter-character gaps are filled too */
    fill_rect(x-2, y-2, (n-1)*GW + 5*GSC + 4, 7*GSC + 4, bg);
    for(int i=0; i<n; i++) draw_glyph(x+i*GW, y, g[i], fg, bg);
}

/* centi-degrees -> glyph list "-nnC", whole degrees only */
static int fmt_temp(int32_t centi, int8_t *out){
    int n=0;
    if(centi<0){ out[n++]=11; centi=-centi; }
    int ip=(int)((centi+50)/100);
    if(ip>=100) out[n++]=(int8_t)((ip/100)%10);
    if(ip>=10)  out[n++]=(int8_t)((ip/10)%10);
    out[n++]=(int8_t)(ip%10);
    out[n++]=12;
    out[n]=-1;
    return n;
}

static void draw_overlay_test(void){
    static const int8_t s123[]={1,2,3,-1};
    for(int v=0; v<2; v++)
        for(int h=0; h<2; h++){
            int row=v*2+h;
            g_fv=v; g_fh=h;
            /* row marker: that many bars on the far left */
            for(int k=0;k<=row;k++) fill_rect(4+k*5, 40+row*30, 3, 12, C_WHITE);
            draw_glyphs(30, 40+row*30, s123, C_WHITE, C_BLACK);
        }
    g_fv=1; g_fh=1;
}

extern int nuc_busy(void);

static void draw_crosshair(void){
    int cy = img_h/2;
    uint16_t col = nuc_busy() ? C_RED : C_WHITE;   /* red while flat-field calibrating */
    fill_rect(LCD_W/2-10, cy-1, 21, 2, col);
    fill_rect(LCD_W/2-1,  cy-10, 2, 21, col);
}

/* The status bar lives above the image and is never repainted by the blast,
 * so it is redrawn only when a value actually changes -- no tearing. */
static void draw_bar_if_changed(void){
    static int32_t l_n=0x7FFFFFFF, l_c=0, l_x=0; static int l_on=-1, l_p=-1;
    if(disp_n==l_n && disp_c==l_c && disp_x==l_x && overlay_on==l_on && cur_pair==l_p) return;
    l_n=disp_n; l_c=disp_c; l_x=disp_x; l_on=overlay_on; l_p=cur_pair;

    fill_rect(0,BAR_Y0,LCD_W,BAR_H,C_BLACK);
    if(!overlay_on) return;

    int8_t g[10];
    /* values only -- colour identifies them: cyan = min, white = centre, red = max */
    fmt_temp(disp_n, g);  draw_glyphs(4, BAR_Y0+3, g, C_CYAN, C_BLACK);
    fmt_temp(disp_c, g);  draw_glyphs((LCD_W-glyphs_w(g))/2, BAR_Y0+3, g, C_WHITE, C_BLACK);
    fmt_temp(disp_x, g);  draw_glyphs(LCD_W-4-glyphs_w(g), BAR_Y0+3, g, C_RED, C_BLACK);
}
