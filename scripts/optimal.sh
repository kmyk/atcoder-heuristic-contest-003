#!/bin/bash
for M in 1 2 ; do
    for D in 100 300 600 900 1200 1500 1800 2000 ; do
        for i in $(seq 50) ; do
            pypy3 scripts/generate.py -M $M -D $D 2>&1 >/dev/null | tail -n 1 | cut -d = -f 2
        done > optimal.tmp
        score=$(awk '{ a += $1 } ; END { print int(a / NR) }' < optimal.tmp)
        rm optimal.tmp
        echo M = $M, D = $D: $score
    done
done
