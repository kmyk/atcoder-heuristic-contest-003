#!/bin/bash
k=${1:-50}
mkdir log
for M in 1 2 ; do
    for D in 100 300 600 900 1200 1500 1800 2000 ; do
        average=$(python3 scripts/measure.py -j6 --count=$k -M $M -D $D 2>&1 | tee log/M${M}D${D}.txt | tail -n 1 | cut -d = -f 2)
        echo M = $M, D = $D: $average
    done
done
