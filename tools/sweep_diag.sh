#!/bin/bash
# usage: sweep.sh <tag> <RES 2|3> <RATE 6|7> <seconds>
set -e
cd /home/stanelie/Documents/SimpleHandhledThermalImager/firmware
OUT=/tmp/claude-1000/-home-stanelie/4c2b8f99-2a60-4b5d-a1ec-3f62ed1e4bd6/scratchpad
TAG=$1; RES=$2; RATE=$3; SECS=$4
rm -f diag.elf; make diag.elf DIAGFLAGS="-DDIAG_RES=$RES -DDIAG_RATE=$RATE" >/dev/null 2>&1
R=$(arm-none-eabi-nm diag.elf | awk '/ [bBdD] d_ref$/{print "0x"$1}')
S=$(arm-none-eabi-nm diag.elf | awk '/ [bBdD] d_sum$/{print "0x"$1}')
Q=$(arm-none-eabi-nm diag.elf | awk '/ [bBdD] d_sumsq$/{print "0x"$1}')
N=$(arm-none-eabi-nm diag.elf | awk '/ [bBdD] d_n$/{print "0x"$1}')
openocd -f interface/stlink.cfg -c "adapter speed 100" -f target/stm32f1x.cfg \
  -c "init" -c "reset halt" -c "program diag.elf verify" -c "reset run" \
  -c "sleep ${SECS}000" -c "halt" \
  -c "mdw $N 1" \
  -c "dump_image $OUT/${TAG}_ref.bin $R 1536" \
  -c "dump_image $OUT/${TAG}_sum.bin $S 3072" \
  -c "dump_image $OUT/${TAG}_sumsq.bin $Q 3072" \
  -c "shutdown" 2>&1 | grep -oE "0x[0-9a-f]{8}: [0-9a-f]{8}" | tail -1
