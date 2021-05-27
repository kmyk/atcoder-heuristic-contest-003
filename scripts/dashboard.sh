#!/bin/bash
k=${1:-50}
mkdir -p log
for D in 100 300 600 900 1200 1500 1800 2000 ; do
    for M in 1 2 ; do
        average=$(python3 scripts/measure.py --count=$k -M $M -D $D 2>&1 | tee log/M${M}D${D}.txt | tail -n 1 | cut -d = -f 2)
        echo M = $M, D = $D: $average
    done
done
