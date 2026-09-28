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
    const float k1 = ksTo_[1], base = 1.f-k1*273.15f;
    float acr[4];
    acr[0]=1.f/(1.f+ksTo_[0]*40.f);
    acr[1]=1.f;
    acr[2]=1.f+k1*(float)ct_[2];
    acr[3]=acr[2]*(1.f+ksTo_[2]*(float)(ct_[3]-ct_[2]));

    for(int p=0;p<768;p++){
        int a=(ee[64+p]&0x03F0)>>4; if(a>31) a-=64;
        float alpha = (alphaRef_ + (float)(accRow_[p>>5]<<accRowScale_)
                                 + (float)(accCol_[p&31]<<accColScale_)
                                 + (float)(a<<accRemScale_)) * invAlphaS;
        int sp=px_split(p);
        int t=(ee[64+p]&0x000E)>>1; if(t>3) t-=8;
        float kta=(float)(KtaRC_[sp] + (t<<ktaScale2_))*invKtaS1;
        float kv =(float)KvRC_[sp]*invKvS;

        float ir=(float)(int16_t)frame[p]*f_gain;
        ir -= (float)poff[p]*(1.f+kta*ktaTa)*(1.f+kv*kvVdd);
        ir *= inv_emiss;
        ir -= tgcIrCP;

        float ac=(alpha-tgcCpA)*ksTaTerm;
        int32_t dd;
        if(ac<=0.f){ dd=-2731; }
        else{
            float ac3=ac*ac*ac;
            float Sx=fsqrtf(fsqrtf(ac3*ir + ac3*ac*f_taTr))*k1;
            float To=fsqrtf(fsqrtf(ir/(ac*base+Sx)+f_taTr)) - 273.15f;
            int r=(To<(float)ct_[1])?0 : (To<(float)ct_[2])?1 : (To<(float)ct_[3])?2 : 3;
            float acc2=ac*acr[r]*(1.f + ksTo_[r]*(To-(float)ct_[r]));
            if(acc2!=0.f) To=fsqrtf(fsqrtf(ir/acc2 + f_taTr)) - 273.15f;
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
    float Sx = fsqrtf(fsqrtf(ac3*ir + ac3*ac*f_taTr))*ksTo_[1];
    float To = fsqrtf(fsqrtf(ir/(ac*(1.f-ksTo_[1]*273.15f)+Sx)+f_taTr)) - 273.15f;

    int r = (To<(float)ct_[1])?0 : (To<(float)ct_[2])?1 : (To<(float)ct_[3])?2 : 3;
    float acr[4];
    acr[0]=1.f/(1.f+ksTo_[0]*40.f);
    acr[1]=1.f;
    acr[2]=1.f+ksTo_[1]*(float)ct_[2];
    acr[3]=acr[2]*(1.f+ksTo_[2]*(float)(ct_[3]-ct_[2]));
    float acc2 = ac*acr[r]*(1.f + ksTo_[r]*(To-(float)ct_[r]));
    if(acc2==0.f) return To;
    return fsqrtf(fsqrtf(ir/acc2 + f_taTr)) - 273.15f;
}
