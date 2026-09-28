/* MLX90640 temperature calibration (Melexis reference math).
 * Textually included by main.c so it can use ee[], poff[], frame[], gainEE.
 * Evaluated only for the few pixels we display, so the float cost is negligible. */

static float fsq(float x){ return x*x; }
/* Newton on the RECIPROCAL square root, so the iteration is multiplies only.
 * The previous version ran 8 Newton steps on sqrt directly, each containing a
 * soft-float DIVIDE -- 16 divides per 4th-root, six 4th-roots per pixel. Verified
 * on host against the C library across x = 1e-12..1e14: max relative error
 * 1.34e-7, against the old version's 1.19e-7, i.e. both at float epsilon. */
static float fsqrtf(float x){
    if(x<=0.f) return 0.f;
    union { float f; uint32_t i; } u; u.f=x;
    u.i = 0x5f3759dfu - (u.i>>1);
    float y=u.f, h=0.5f*x;
    y *= 1.5f - h*y*y;
    y *= 1.5f - h*y*y;
    y *= 1.5f - h*y*y;   /* 3 iterations: 1.65e-7 rel err, vs 1.34e-7 at 4 --
                          * both at float epsilon, measured on host */
    return x*y;
}

/* 2^n is just an exponent field. The old loop multiplied by 2 n times, and
 * px_alpha() calls it with alphaScale_ = 38 for EVERY pixel. Bit-exact against
 * the old version for all n in -126..127. */
static float p2(int n){
    union { float f; uint32_t i; } u;
    if(n>127) n=127; else if(n<-126) n=-126;
    u.i = (uint32_t)(127+n) << 23;
    return u.f;
}

/* Direct 4th root. The Melexis chain needs x^(1/4) three times per pixel, and
 * doing it as fsqrtf(fsqrtf(x)) costs six square roots -- about 90 soft-float
 * operations -- which was the single largest cost in the whole conversion.
 * Split the float into exponent and mantissa instead: interpolate m^(1/4) from
 * a 64-entry table over [1,2), and fold 2^(e/4) back in as an exponent plus one
 * of four fixed factors. About 6 operations instead of 30.
 * Verified on host over the (T_kelvin)^4 domain: 5.68e-6 max relative error,
 * i.e. 0.0017 K on a 300 K target, against a sensor specified to +/-2 degC. */
static float f4rt_tab[65], f4rt_pw[4];

static void f4rt_init(void){
    for(int i=0;i<=64;i++){
        float m = 1.f + (float)i*(1.f/64.f);
        f4rt_tab[i] = fsqrtf(fsqrtf(m));       /* built once; speed irrelevant */
    }
    for(int k=0;k<4;k++) f4rt_pw[k] = fsqrtf(fsqrtf(p2(k)));
}

static float f4rt(float x){
    if(x<=0.f) return 0.f;
    union { float f; uint32_t i; } u; u.f=x;
    int e = (int)((u.i>>23)&0xFFu) - 127;
    uint32_t mb = u.i & 0x7FFFFFu;
    uint32_t idx = mb >> 17;                                   /* 0..63 */
    float fr = (float)(mb & 0x1FFFFu) * (1.f/131072.f);
    float r = f4rt_tab[idx] + (f4rt_tab[idx+1]-f4rt_tab[idx])*fr;
    int q = e >> 2, k = e - (q<<2);                            /* e = 4q + k */
    union { float f; uint32_t i; } pe;
    pe.i = (uint32_t)(127+q) << 23;
    return r * f4rt_pw[k] * pe.f;
}

static float kVdd_, vdd25_, KvPTAT_, KtPTAT_, vPTAT25_, alphaPTAT_;
static float tgc_, cpKta_, cpKv_, KsTa_, ksTo_[5], cpAlpha_[2], cpOffset_[2];
static int   ct_[5], resEE_, ktaScale1_, ktaScale2_, kvScale_;
static int   KtaRC_[4], KvRC_[4], accRow_[24], accCol_[32];
static int   accRemScale_, accColScale_, accRowScale_, alphaScale_;
static float alphaRef_;
static float emiss = 0.95f;

static void extract_cal(void){
    int t;
    t=(ee[51]&0xFF00)>>8; if(t>127)t-=256; kVdd_=32.f*(float)t;
    t=ee[51]&0x00FF;      vdd25_=(float)((((int)t-256)<<5)-8192);

    t=(ee[50]&0xFC00)>>10; if(t>31)t-=64;   KvPTAT_=(float)t/4096.f;
    t=ee[50]&0x03FF;       if(t>511)t-=1024; KtPTAT_=(float)t/8.f;
    vPTAT25_=(float)(int16_t)ee[49];
    alphaPTAT_=((float)(ee[16]&0xF000))/16384.f + 8.f;

    t=ee[60]&0x00FF;       if(t>127)t-=256; tgc_=(float)t/32.f;
    resEE_=(ee[56]&0x3000)>>12;
    t=(ee[60]&0xFF00)>>8;  if(t>127)t-=256; KsTa_=(float)t/8192.f;

    {   int step=((ee[63]&0x3000)>>12)*10;
        ct_[0]=-40; ct_[1]=0;
        ct_[2]=((ee[63]&0x00F0)>>4)*step;
        ct_[3]=ct_[2]+((ee[63]&0x0F00)>>8)*step;
        ct_[4]=400;
        float S=(float)(1u<<((ee[63]&0x000F)+8));
        t=ee[61]&0x00FF;      if(t>127)t-=256; ksTo_[0]=(float)t/S;
        t=(ee[61]&0xFF00)>>8; if(t>127)t-=256; ksTo_[1]=(float)t/S;
        t=ee[62]&0x00FF;      if(t>127)t-=256; ksTo_[2]=(float)t/S;
        t=(ee[62]&0xFF00)>>8; if(t>127)t-=256; ksTo_[3]=(float)t/S;
        ksTo_[4]=-0.0002f;
    }

    {   int as=((ee[32]&0xF000)>>12)+27;
        int o0=ee[58]&0x03FF;        if(o0>511)o0-=1024;
        int o1=(ee[58]&0xFC00)>>10;  if(o1>31)o1-=64; o1+=o0;
        cpOffset_[0]=(float)o0; cpOffset_[1]=(float)o1;
        int a0=ee[57]&0x03FF;        if(a0>511)a0-=1024;
        float A0=(float)a0/p2(as);
        int a1=(ee[57]&0xFC00)>>10;  if(a1>31)a1-=64;
        cpAlpha_[0]=A0; cpAlpha_[1]=(1.f+(float)a1/128.f)*A0;
        ktaScale1_=((ee[56]&0x00F0)>>4)+8;
        kvScale_  =(ee[56]&0x0F00)>>8;
        t=ee[59]&0x00FF;      if(t>127)t-=256; cpKta_=(float)t/p2(ktaScale1_);
        t=(ee[59]&0xFF00)>>8; if(t>127)t-=256; cpKv_ =(float)t/p2(kvScale_);
    }
    ktaScale2_=(ee[56]&0x000F);

    t=(ee[54]&0xFF00)>>8; if(t>127)t-=256; KtaRC_[0]=t;
    t=(ee[54]&0x00FF);    if(t>127)t-=256; KtaRC_[2]=t;
    t=(ee[55]&0xFF00)>>8; if(t>127)t-=256; KtaRC_[1]=t;
    t=(ee[55]&0x00FF);    if(t>127)t-=256; KtaRC_[3]=t;

    t=(ee[52]&0xF000)>>12; if(t>7)t-=16; KvRC_[0]=t;
    t=(ee[52]&0x0F00)>>8;  if(t>7)t-=16; KvRC_[2]=t;
    t=(ee[52]&0x00F0)>>4;  if(t>7)t-=16; KvRC_[1]=t;
    t=(ee[52]&0x000F);     if(t>7)t-=16; KvRC_[3]=t;

    accRemScale_= ee[32]&0x000F;
    accColScale_=(ee[32]&0x00F0)>>4;
    accRowScale_=(ee[32]&0x0F00)>>8;
    alphaScale_ =((ee[32]&0xF000)>>12)+30;
    alphaRef_   =(float)ee[33];
    for(int i=0;i<6;i++){ int p=i*4;
        accRow_[p+0]= ee[34+i]&0x000F;
        accRow_[p+1]=(ee[34+i]&0x00F0)>>4;
        accRow_[p+2]=(ee[34+i]&0x0F00)>>8;
        accRow_[p+3]=(ee[34+i]&0xF000)>>12; }
    for(int i=0;i<24;i++) if(accRow_[i]>7) accRow_[i]-=16;
    for(int i=0;i<8;i++){ int p=i*4;
        accCol_[p+0]= ee[40+i]&0x000F;
        accCol_[p+1]=(ee[40+i]&0x00F0)>>4;
        accCol_[p+2]=(ee[40+i]&0x0F00)>>8;
        accCol_[p+3]=(ee[40+i]&0xF000)>>12; }
    for(int i=0;i<32;i++) if(accCol_[i]>7) accCol_[i]-=16;
}

static float px_alpha(int p){
    int i=p>>5, j=p&31;
    int a=(ee[64+p]&0x03F0)>>4; if(a>31) a-=64;
    a <<= accRemScale_;
    float v = alphaRef_ + (float)(accRow_[i]<<accRowScale_)
                        + (float)(accCol_[j]<<accColScale_) + (float)a;
    return v/p2(alphaScale_);
}
static int px_split(int p){ return 2*(p/32 - (p/64)*2) + p%2; }
static float px_kta(int p){
    int t=(ee[64+p]&0x000E)>>1; if(t>3) t-=8;
    t <<= ktaScale2_;
    return (float)(KtaRC_[px_split(p)] + t)/p2(ktaScale1_);
}
static float px_kv(int p){ return (float)KvRC_[px_split(p)]/p2(kvScale_); }

static float f_vdd, f_ta, f_gain, f_taTr;
static int   f_sub;

volatile uint32_t aux_rejects;
static float lg_vdd=3.3f, lg_ta=25.f, lg_gain=1.f;
static int   lg_valid=0, lg_gfp=1024;

static void calc_frame_params(uint16_t ctrlReg, int subpage){
    int resRAM=(ctrlReg&0x0C00)>>10;
    float vdd=(float)(int16_t)frame[810];
    vdd = (p2(resEE_-resRAM)*vdd - vdd25_)/kVdd_ + 3.3f;
    f_vdd=vdd;

    float ptat   =(float)(int16_t)frame[800];
    float ptatArt=(float)(int16_t)frame[768];
    ptatArt = (ptat/(ptat*alphaPTAT_ + ptatArt))*262144.f;      /* 2^18 */
    float ta = ptatArt/(1.f + KvPTAT_*(vdd-3.3f)) - vPTAT25_;
    f_ta = ta/KtPTAT_ + 25.f;

    float g=(float)(int16_t)frame[778]; if(g==0.f) g=1.f;
    f_gain=(float)gainEE/g;

    /* The aux words share the sensor's RAM with the pixels and can be caught
     * mid-update. Judge the *computed* quantities against physical limits and
     * reuse the previous set when impossible -- Ta and VDD move over seconds,
     * so last frame's values are genuinely the right answer, not a fudge. */
    /* raw counts scale with ADC resolution, so normalise the gain ratio before
     * range-checking it, or the check only ever holds at one resolution */
    float gnorm = f_gain * p2(resRAM - resEE_);
    if(f_vdd>3.0f && f_vdd<3.6f && f_ta>-20.f && f_ta<85.f &&
       gnorm>0.7f && gnorm<1.4f){
        lg_vdd=f_vdd; lg_ta=f_ta; lg_gain=f_gain;
        lg_gfp=(int)(f_gain*1024.f);
        lg_valid=1;
    } else {
        if(lg_valid){ f_vdd=lg_vdd; f_ta=lg_ta; f_gain=lg_gain; }
        aux_rejects++;          /* dbg[14]: how often the sensor's aux words were caught mid-update */
    }

    float ta4=fsq(fsq(f_ta+273.15f));
    float tr4=fsq(fsq((f_ta-8.f)+273.15f));
    f_taTr = tr4 - (tr4-ta4)/emiss;
    f_sub  = subpage;
}

/* The three-stage Melexis chain collapses to a single function of ONE variable.
 * Writing u = ir/ac, the ac factors cancel out of every stage:
 *     Sx  = k1*ac*Tk            with Tk = (u + taTr)^(1/4)
 *     ir/(ac*base + Sx)         = u/(base + k1*Tk)
 *     acc2 = ac*W(u)            so ir/acc2 = u/W(u)
 * so To depends only on u, and u <-> Tk is a bijection. That makes the whole
 * chain -- two of the three 4th roots, both divides after the first, and the
 * band selection -- a table indexed by Tk, rebuilt only when Ta moves.
 * Per pixel this leaves one divide, one 4th root and a lerp. */
#define H_N    64
#define H_LO   180.0f
#define H_STEP (440.0f/(float)(H_N-1))
#define H_INV  ((float)(H_N-1)/440.0f)
static float h_tab[H_N];
static float h_built_ta = -1e30f;

static void build_h_tab(void){
    const float k1=ksTo_[1], base=1.f-k1*273.15f;
    float acr[4];
    acr[0]=1.f/(1.f+ksTo_[0]*40.f);
    acr[1]=1.f;
    acr[2]=1.f+k1*(float)ct_[2];
    acr[3]=acr[2]*(1.f+ksTo_[2]*(float)(ct_[3]-ct_[2]));
    for(int i=0;i<H_N;i++){
        float Tk=H_LO+(float)i*H_STEP;
        float t2=Tk*Tk;
        float u=t2*t2 - f_taTr;                 /* invert Tk = (u+taTr)^(1/4) */
        float To2=f4rt(u/(base+k1*Tk)+f_taTr) - 273.15f;
        int r=(To2<(float)ct_[1])?0:(To2<(float)ct_[2])?1:(To2<(float)ct_[3])?2:3;
        float W=acr[r]*(1.f+ksTo_[r]*(To2-(float)ct_[r]));
        h_tab[i] = (W!=0.f) ? (f4rt(u/W+f_taTr)-273.15f) : To2;
    }
    h_built_ta=f_ta; dbg[28]++;
}

/* Per-pixel constants, as scaled integers. alpha, kta and kv are fixed in the
 * EEPROM and the terms built from them change only when Ta or Vdd move, yet the
 * inner loop was re-decoding EEPROM bitfields and doing three int->float
 * conversions per pixel per frame. Precomputing costs 3 KB:
 *   t_off[p]  poff[p]*(1+kta*dTa)*(1+kv*dVdd), int16 -- poff is already int16
 *             and the correction is within +/-1%, so it cannot overflow.
 *   t_iac[p]  1/ac[p] relative to the array mean, uint16 in 1/16384 units.
 *             That is 3e-5 relative resolution against alpha's 10% spread,
 *             i.e. ~0.002 K through the 4th root -- far below the sensor's
 *             0.7 K noise floor.
 * Rebuilt only when Ta or Vdd actually move. */
static int16_t  t_off[768];
static uint16_t t_iac[768];
static float    t_iac_mul;
static float    t_built_ta = -1e30f, t_built_vdd = -1e30f;

/* Rebuilding all 768 entries at once costs ~18 ms, which lands as a visible
 * stutter on the ~28% of frames where Ta has drifted a quarter degree. The
 * constants are latched at the start of a pass so the table stays
 * self-consistent, then 96 entries are built per frame -- a full refresh takes
 * 8 frames (about half a second), during which Ta cannot have moved far. */
static struct { float ktaTa,kvVdd,ksTaTerm,tgcCpA,invAlphaS,invKtaS1,invKvS,iac_ref; } pxb;
static int px_cursor = -1;                 /* -1 = idle */

static void px_tables_begin(void){
    pxb.ktaTa=f_ta-25.f; pxb.kvVdd=f_vdd-3.3f;
    pxb.ksTaTerm=1.f+KsTa_*pxb.ktaTa;
    pxb.tgcCpA=tgc_*cpAlpha_[f_sub?1:0];
    pxb.invAlphaS=p2(-alphaScale_);
    pxb.invKtaS1=p2(-ktaScale1_);
    pxb.invKvS=p2(-kvScale_);
    float ac_ref=(alphaRef_*pxb.invAlphaS - pxb.tgcCpA)*pxb.ksTaTerm;
    pxb.iac_ref=(ac_ref>0.f)?1.f/ac_ref:1.f;
    t_iac_mul=pxb.iac_ref*(1.f/16384.f);
    px_cursor=0;
}

static void px_tables_step(int n){
    int end=px_cursor+n; if(end>768) end=768;
    for(int p=px_cursor;p<end;p++){
        int a=(ee[64+p]&0x03F0)>>4; if(a>31) a-=64;
        float alpha=(alphaRef_ + (float)(accRow_[p>>5]<<accRowScale_)
                               + (float)(accCol_[p&31]<<accColScale_)
                               + (float)(a<<accRemScale_))*pxb.invAlphaS;
        int sp=px_split(p);
        int t=(ee[64+p]&0x000E)>>1; if(t>3) t-=8;
        float kta=(float)(KtaRC_[sp]+(t<<ktaScale2_))*pxb.invKtaS1;
        float kv =(float)KvRC_[sp]*pxb.invKvS;
        float off=(float)poff[p]*(1.f+kta*pxb.ktaTa)*(1.f+kv*pxb.kvVdd)*16.f;
        int32_t oq=(int32_t)(off<0.f ? off-0.5f : off+0.5f);
        t_off[p]=(int16_t)(oq>32767?32767:(oq<-32768?-32768:oq));
        float ac=(alpha-pxb.tgcCpA)*pxb.ksTaTerm;
        float iac=(ac>0.f)? 1.f/ac : pxb.iac_ref;
        int32_t q=(int32_t)((iac/pxb.iac_ref)*16384.f+0.5f);
        t_iac[p]=(uint16_t)(q<0?0:(q>65535?65535:q));
    }
    px_cursor=end;
    if(px_cursor>=768){ px_cursor=-1; t_built_ta=f_ta; t_built_vdd=f_vdd; dbg[28]++; }
}

static void build_px_tables(void){
    const float ktaTa=(f_ta-25.f), kvVdd=(f_vdd-3.3f);
    const float ksTaTerm=1.f+KsTa_*ktaTa;
    const float tgcCpA=tgc_*cpAlpha_[f_sub?1:0];
    const float invAlphaS=p2(-alphaScale_), invKtaS1=p2(-ktaScale1_), invKvS=p2(-kvScale_);
    /* Scaling reference for the uint16 table. This must NOT be a mean computed
     * into a temporary float[768]: that is 3 KB on a ~2.6 KB stack, and it
     * overflowed into the top of bss on every rebuild. The reference only has
     * to keep the quantised ratios inside uint16, so the EEPROM's nominal
     * alpha serves, is deterministic, and needs no second pass. */
    const float ac_ref  = (alphaRef_*invAlphaS - tgcCpA)*ksTaTerm;
    const float iac_ref = (ac_ref > 0.f) ? 1.f/ac_ref : 1.f;
    t_iac_mul = iac_ref*(1.f/16384.f);
    for(int p=0;p<768;p++){
        int a=(ee[64+p]&0x03F0)>>4; if(a>31) a-=64;
        float alpha=(alphaRef_ + (float)(accRow_[p>>5]<<accRowScale_)
                               + (float)(accCol_[p&31]<<accColScale_)
                               + (float)(a<<accRemScale_))*invAlphaS;
        int sp=px_split(p);
        int t=(ee[64+p]&0x000E)>>1; if(t>3) t-=8;
        float kta=(float)(KtaRC_[sp]+(t<<ktaScale2_))*invKtaS1;
        float kv =(float)KvRC_[sp]*invKvS;

        /* stored in 1/16 count units: poff measures -112..-48 on this sensor,
         * so x16 reaches ~1800 against int16's 32767 -- the precision is free */
        float off=(float)poff[p]*(1.f+kta*ktaTa)*(1.f+kv*kvVdd)*16.f;
        int32_t oq=(int32_t)(off<0.f ? off-0.5f : off+0.5f);
        t_off[p]=(int16_t)(oq>32767?32767:(oq<-32768?-32768:oq));

        float ac=(alpha-tgcCpA)*ksTaTerm;
        float iac=(ac>0.f)? 1.f/ac : iac_ref;
        int32_t q=(int32_t)((iac/iac_ref)*16384.f+0.5f);
        t_iac[p]=(uint16_t)(q<0?0:(q>65535?65535:q));
    }
    t_built_ta=f_ta; t_built_vdd=f_vdd;
}

/* Bulk version of mlx_to() for the reference pipeline. Identical arithmetic;
 * everything that does not depend on the pixel is computed once instead of 768
 * times -- ktaTa, kvVdd, the compensation pixel, the ksTo range coefficients,
 * and the reciprocals that replace per-pixel divides. Writes tenths of a degC
 * back into frame[] in place, which is safe because the compensation pixels at
 * 776/808 sit above the 768 entries being overwritten. */
static void ref_convert_frame(void){
    const float ktaTa=(f_ta-25.f), kvVdd=(f_vdd-3.3f);
    const float cpo = cpOffset_[f_sub?1:0]*(1.f+cpKta_*ktaTa)*(1.f+cpKv_*kvVdd);
    const float irCP = (float)(int16_t)frame[f_sub?808:776]*f_gain - cpo;
    const float inv_emiss = 1.f/emiss;
    const float ksTaTerm  = 1.f+KsTa_*ktaTa;
    const float tgcCpA    = tgc_*cpAlpha_[f_sub?1:0];
    const float tgcIrCP   = tgc_*irCP;
    const float invAlphaS = p2(-alphaScale_);
    const float invKtaS1  = p2(-ktaScale1_);
    const float invKvS    = p2(-kvScale_);
    /* Ta jitters every frame, so an exact float compare here rebuilt the whole
     * 256-entry table on 100% of frames -- measured, 130 rebuilds in 130 frames
     * -- which defeated the point of caching it. The table maps Tk -> To through
     * f_taTr, which moves slowly, so a quarter-degree tolerance is plenty. */
    { float d=f_ta-h_built_ta; if(d<0) d=-d;
      if(d>0.25f) build_h_tab(); }
    if(t_built_ta < -1e29f){ build_px_tables(); }   /* first frame: all at once */
    else{
        if(px_cursor < 0){
            float dt=f_ta-t_built_ta, dv=f_vdd-t_built_vdd;
            if(dt<0)dt=-dt; if(dv<0)dv=-dv;
            if(dt>0.25f || dv>0.01f) px_tables_begin();
        }
        if(px_cursor >= 0) px_tables_step(96);
    }
    const int32_t gfp=(int32_t)(f_gain*16384.f);   /* 1/16 count units */
    const float   iemis=1.f/(emiss*16.f);

    for(int p=0;p<768;p++){
        /* gain and offset in integers, carried at 1/16 count so the
         * quantisation is ~0.005 degC rather than ~0.09 */
        int32_t irq = (((int32_t)frame[p]*gfp)>>10) - (int32_t)t_off[p];
        float ir = (float)irq*iemis - tgcIrCP;
        int32_t dd;
        {
            float u = ir * t_iac_mul * (float)t_iac[p];
            float Tk=f4rt(u + f_taTr);
            float fi=(Tk-H_LO)*H_INV;
            int i=(int)fi;
            if(i<0){ i=0; fi=0.f; }
            else if(i>H_N-2){ i=H_N-2; fi=(float)(H_N-2); }
            float fr=fi-(float)i;
            float To=h_tab[i]+(h_tab[i+1]-h_tab[i])*fr;
            dd=(int32_t)(To*10.f);
        }
        frame[p]=(int16_t)(dd>32767?32767:(dd<-32768?-32768:dd));
    }
}

static float mlx_to(int p){
    float ktaTa=(f_ta-25.f), kvVdd=(f_vdd-3.3f);
    float cpo = cpOffset_[f_sub?1:0]*(1.f+cpKta_*ktaTa)*(1.f+cpKv_*kvVdd);
    float irCP = (float)(int16_t)frame[f_sub?808:776]*f_gain - cpo;

    float ir = (float)(int16_t)frame[p]*f_gain;
    ir -= (float)poff[p]*(1.f+px_kta(p)*ktaTa)*(1.f+px_kv(p)*kvVdd);
    ir /= emiss;
    ir -= tgc_*irCP;

    float ac = (px_alpha(p) - tgc_*cpAlpha_[f_sub?1:0])*(1.f+KsTa_*ktaTa);
    if(ac<=0.f) return -273.15f;

    float ac3=ac*ac*ac;
    float Sx = f4rt(ac3*ir + ac3*ac*f_taTr)*ksTo_[1];
    float To = f4rt(ir/(ac*(1.f-ksTo_[1]*273.15f)+Sx)+f_taTr) - 273.15f;

    int r = (To<(float)ct_[1])?0 : (To<(float)ct_[2])?1 : (To<(float)ct_[3])?2 : 3;
    float acr[4];
    acr[0]=1.f/(1.f+ksTo_[0]*40.f);
    acr[1]=1.f;
    acr[2]=1.f+ksTo_[1]*(float)ct_[2];
    acr[3]=acr[2]*(1.f+ksTo_[2]*(float)(ct_[3]-ct_[2]));
    float acc2 = ac*acr[r]*(1.f + ksTo_[r]*(To-(float)ct_[r]));
    if(acc2==0.f) return To;
    return f4rt(ir/acc2 + f_taTr) - 273.15f;
}
