#!/bin/bash
mkdir -p log
average=$(python3 scripts/measure.py --count=960 2>&1 | tee log/all.txt | tail -n 1 | cut -d = -f 2)
echo all: $average
for M in 1 2 ; do
    average=$(python3 scripts/measure.py --count=200 -M $M 2>&1 | tee log/M${M}.txt | tail -n 1 | cut -d = -f 2)
    echo M = $M: $average
done
for D in 100 300 600 900 1200 1500 1800 2000 ; do
    for M in 1 2 ; do
        average=$(python3 scripts/measure.py -M $M -D $D 2>&1 | tee log/M${M}D${D}.txt | tail -n 1 | cut -d = -f 2)
        echo M = $M, D = $D: $average
    done
done
