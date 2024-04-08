#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)

cd ${lwd}/tools/pitch_smooth

python pitch_smooth.py -f ~/bags/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_4_380to615.db \
    --fitting-pitch-curve \
    --curve-order 1 \
    --polynominal \
    --fitting-window 200 \
    --step-distance 20 \
    --pitch-distance 20



