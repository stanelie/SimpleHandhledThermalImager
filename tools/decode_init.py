import os, sys
# The stock firmware dump is NOT in this repo (it is the vendor's binary).
# Dump your own with:
#   openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init" -c "reset halt" \
#     -c "dump_image backup.bin 0x08000000 0x10000" -c "shutdown"
FW_PATH = os.environ.get("STOCK_FW", "backup.bin")
if not os.path.exists(FW_PATH):
    sys.exit(f"stock firmware dump not found at {FW_PATH!r}; set STOCK_FW=/path/to/backup.bin")

import struct
from unicorn import *
from unicorn.arm_const import *

FW=bytearray(open(FW_PATH,'rb').read())
mu=Uc(UC_ARCH_ARM,UC_MODE_THUMB)
mu.mem_map(0x08000000,0x100000)
mu.mem_map(0x20000000,0x10000); mu.mem_map(0x40000000,0x100000)
mu.mem_write(0x08000000,bytes(FW))
# stub out the delay function at 0x8005434 -> "bx lr"
mu.mem_write(0x08005434, b'\x70\x47')

st={'rs':1,'bus':0}
stream=[]
def hook(uc,a,addr,sz,val,u):
    if addr==0x40010810:                      # GPIOA_BSRR (set)
        if val & 4: st['rs']=1
    elif addr==0x40010814:                    # GPIOA_BRR (reset)
        if val & 4: st['rs']=0
    elif addr==0x40010C0C:                    # GPIOB_ODR (data bus)
        st['bus']=val & 0xFF
    elif addr==0x40011010 and (val & 0x8000): # PC15 rising = WR latch
        stream.append((st['rs'], st['bus']))
mu.hook_add(UC_HOOK_MEM_WRITE,hook,begin=0x40000000,end=0x40100000)

sp=0x20000000+0x10000-0x200
mu.reg_write(UC_ARM_REG_SP,sp); mu.reg_write(UC_ARM_REG_LR,0xEEEEEEEF)
try:
    mu.emu_start(0x08006a5c|1, 0xEEEEEEEE, count=5_000_000)
except UcError as e:
    print("stopped:",e,"pc=",hex(mu.reg_read(UC_ARM_REG_PC)))

print(f"=== ORIGINAL PANEL INIT SEQUENCE ({len(stream)} bytes latched) ===")
line=[]
for rs,b in stream:
    if rs==0:
        if line: print("      data:", " ".join(line)); line=[]
        print(f"  CMD  0x{b:02X}")
    else:
        line.append(f"0x{b:02X}")
if line: print("      data:", " ".join(line))
