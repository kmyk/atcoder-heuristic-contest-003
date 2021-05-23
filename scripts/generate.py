#!/usr/bin/env python3
import argparse
import heapq
import math
import random
from logging import DEBUG, basicConfig, getLogger
from typing import *

logger = getLogger(__name__)

H = 30
W = 30
K = 1000


def transpose(f: List[List[int]]) -> List[List[int]]:
    h = len(f)
    w = len(f[0])
    return [[f[y][x] for y in range(h)] for x in range(w)]


def gen_m1(D: int, *, name: str) -> List[List[int]]:
    row = [random.randint(1000 + D, 9000 - D) for _ in range(H)]
    logger.info('%s = %s', name, row)

    hr: List[List[int]] = [[None for x in range(W - 1)] for y in range(H)]  # type: ignore
    for y in range(H):
        for x in range(W - 1):
            hr[y][x] = row[y] + random.randint(-D, D)
    return hr


def gen_m2(D: int, *, name: str) -> List[List[int]]:
    row1 = [random.randint(1000 + D, 9000 - D) for _ in range(H)]
    row2 = [random.randint(1000 + D, 9000 - D) for _ in range(H)]
    sep = [random.randint(1, W - 2) for _ in range(H)]
    logger.info('%s1 = %s', name, row1)
    logger.info('%s2 = %s', name, row2)
    logger.info('sep = %s', sep)

    hr: List[List[int]] = [[None for x in range(W - 1)] for y in range(H)]  # type: ignore
    for y in range(H):
        for x in range(W - 1):
            hr[y][x] = (row1 if x < sep[y] else row2)[y] + random.randint(-D, D)
    return hr


def dijkstra(hr: List[List[int]], vr: List[List[int]], sy: int, sx: int, ty: int, tx: int) -> int:
    INF = 10**18
    dist = [[INF for x in range(W)] for y in range(H)]
    que: List[Tuple[int, int, int]] = []
    dist[sy][sx] = 0
    heapq.heappush(que, (0, sy, sx))
    while que:
        dist_y_x, y, x = heapq.heappop(que)
        if dist[y][x] < dist_y_x:
            continue
        if y == ty and x == tx:
            break
        for i in range(4):
            ny = y + [-1, 1, 0, 0][i]
            nx = x + [0, 0, 1, -1][i]
            if 0 <= ny < H and 0 <= nx < W:
                if i < 2:
                    c = vr[min(y, ny)][x]
                else:
                    c = hr[y][min(x, nx)]
                if dist[y][x] + c < dist[ny][nx]:
                    dist[ny][nx] = dist[y][x] + c
                    heapq.heappush(que, (dist[ny][nx], ny, nx))
    return dist[ty][tx]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int)
    parser.add_argument('-M', choices=(1, 2), type=int, default=random.randint(1, 2))
    parser.add_argument('-D', choices=range(100, 2000 + 1), type=int, default=random.randint(100, 2000))
    args = parser.parse_args()
    basicConfig(level=DEBUG)

    if args.seed is not None:
        random.seed(args.seed * 0xc0ffee)
        logger.info('seed = %s', args.seed)

    logger.info('M = %s', args.M)
    logger.info('D = %s', args.D)
    if args.M == 1:
        hr = gen_m1(D=args.D, name='row')
        vr = transpose(gen_m1(D=args.D, name='col'))
    elif args.M == 2:
        hr = gen_m2(D=args.D, name='row')
        vr = transpose(gen_m2(D=args.D, name='col'))
    for y in range(H):
        print(*hr[y])
    for y in range(H - 1):
        print(*vr[y])

    for _ in range(K):
        while True:
            sy = random.randint(0, H - 1)
            sx = random.randint(0, W - 1)
            ty = random.randint(0, H - 1)
            tx = random.randint(0, W - 1)
            if abs(ty - sy) + abs(tx - sx) >= 10:
                break
        a = dijkstra(hr, vr, sy, sx, ty, tx)
        e = 0.9 + 0.2 * random.random()
        print(sy, sx, ty, tx, a, e)


if __name__ == '__main__':
    main()
