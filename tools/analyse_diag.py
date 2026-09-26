import struct, statistics as st, sys
def rd(f,fmt,n): return list(struct.unpack('<%d%s'%(n,fmt), open(f,'rb').read()))
tag=sys.argv[1]; n=int(sys.argv[2],16)
ref=rd(f'{tag}_ref.bin','h',768)
s  =rd(f'{tag}_sum.bin','i',768)
sq =rd(f'{tag}_sumsq.bin','i',768)
mean=[ref[i]+s[i]/n for i in range(768)]
sd  =[]
for i in range(768):
    v=sq[i]/n-(s[i]/n)**2
    sd.append(v**0.5 if v>0 else 0.0)

def hp(v):                       # high-pass against a 3x3 local mean
    o=[]
    for r in range(24):
        for c in range(32):
            nb=[v[rr*32+cc] for rr in range(max(0,r-1),min(24,r+2))
                              for cc in range(max(0,c-1),min(32,c+2))]
            o.append(v[r*32+c]-st.mean(nb))
    return o
res=hp(mean)
colm=[st.mean([mean[r*32+c] for r in range(24)]) for c in range(32)]
rowm=[st.mean([mean[r*32+c] for c in range(32)]) for r in range(24)]
colhp=[colm[c]-st.mean(colm[max(0,c-1):c+2]) for c in range(32)]

print(f"{tag}: n={n} frames")
print(f"  TEMPORAL noise per pixel : median {st.median(sd):5.2f}  mean {st.mean(sd):5.2f} "
      f" p90 {sorted(sd)[int(.9*768)]:5.2f}  worst {max(sd):5.2f}")
print(f"  FIXED pattern (high-pass): stdev {st.pstdev(res):5.2f}  "
      f"p2p {max(res)-min(res):6.2f}")
print(f"  scene span (mean frame)  : {max(mean)-min(mean):6.1f} counts")
print(f"  FPN as % of span         : {100*st.pstdev(res)/max(1e-9,(max(mean)-min(mean))):5.2f}%")
print(f"  column-structure stdev   : {st.pstdev(colhp):5.2f}   row-mean stdev {st.pstdev(rowm):5.2f}")
hot=sorted(range(768), key=lambda i:-abs(res[i]))[:6]
print("  worst fixed pixels (r,c,dev):", ", ".join(f"({i//32},{i%32}){res[i]:+.1f}" for i in hot))
noisy=sorted(range(768), key=lambda i:-sd[i])[:6]
print("  noisiest pixels   (r,c,sd) :", ", ".join(f"({i//32},{i%32}){sd[i]:.1f}" for i in noisy))
