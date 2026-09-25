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

FW = open(FW_PATH,'rb').read()

FLASH_BASE = 0x08000000
FLASH_SIZE = 0x00100000
RAM_BASE   = 0x20000000
RAM_SIZE   = 0x00010000
PERIPH_BASE = 0x40000000
PERIPH_SIZE = 0x00100000
RETURN_ADDR = 0xEEEEEEEE & ~1  # sentinel, must be even (thumb bit stripped)

REGNAMES = {
    0x40010800:"GPIOA_CRL",0x40010804:"GPIOA_CRH",0x40010808:"GPIOA_IDR",0x4001080C:"GPIOA_ODR",
    0x40010810:"GPIOA_BSRR",0x40010814:"GPIOA_BRR",
    0x40010C00:"GPIOB_CRL",0x40010C04:"GPIOB_CRH",0x40010C08:"GPIOB_IDR",0x40010C0C:"GPIOB_ODR",
    0x40010C10:"GPIOB_BSRR",0x40010C14:"GPIOB_BRR",
    0x40011000:"GPIOC_CRL",0x40011004:"GPIOC_CRH",0x40011008:"GPIOC_IDR",0x4001100C:"GPIOC_ODR",
    0x40011010:"GPIOC_BSRR",0x40011014:"GPIOC_BRR",
}

def bitnames(val):
    names=[]
    for b in range(16):
        if val & (1<<b):
            names.append(f"bit{b}")
    return "+".join(names) if names else "0"

def run_func(entry, args, label):
    mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    mu.mem_map(FLASH_BASE, FLASH_SIZE)
    mu.mem_write(FLASH_BASE, FW)
    mu.mem_map(RAM_BASE, RAM_SIZE)
    mu.mem_map(PERIPH_BASE, PERIPH_SIZE)

    trace = []
    def hook_mem_write(uc, access, address, size, value, user_data):
        pc = uc.reg_read(UC_ARM_REG_PC)
        name = REGNAMES.get(address, hex(address))
        trace.append((pc, name, address, value, size))
    mu.hook_add(UC_HOOK_MEM_WRITE, hook_mem_write, begin=PERIPH_BASE, end=PERIPH_BASE+PERIPH_SIZE)

    sp = RAM_BASE + RAM_SIZE - 0x100
    mu.reg_write(UC_ARM_REG_SP, sp)
    lr = RETURN_ADDR | 1  # thumb bit set in LR value (branch target)
    mu.reg_write(UC_ARM_REG_LR, lr)
    regs = [UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3]
    for r, v in zip(regs, args[:4]):
        mu.reg_write(r, v)
    if len(args) > 4:
        # extra stack args for callee that does ldr r8,[sp,#N] style access -- place after a fake return frame
        for i, extra in enumerate(args[4:]):
            mu.mem_write(sp + i*4, struct.pack('<I', extra))

    try:
        mu.emu_start(entry | 1, RETURN_ADDR, timeout=0, count=200000)
    except UcError as e:
        print(f"[{label}] emu stopped: {e}  pc={hex(mu.reg_read(UC_ARM_REG_PC))}")

    print(f"=== {label} (entry={hex(entry)} args={args}) : {len(trace)} peripheral writes ===")
    for pc, name, addr, value, size in trace:
        extra = f" set={bitnames(value)}" if 'BSRR' in name or 'BRR' in name else (f" val=0x{value:x}" if 'ODR' in name else f" val=0x{value:x}")
        print(f"  pc=0x{pc:08x}  {name:12s} <= 0x{value:08x}{extra}")
    print()

# send_command(cmd=0xAB)
run_func(0x08003980, [0xAB], "send_command")

# window/column-draw function(x, y0, y1) -- distinctive values
run_func(0x08003d4c, [0x0007, 0x000B, 0x0013], "draw_column(x=7,y0=11,y1=19)")
