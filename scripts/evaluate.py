import argparse
import concurrent.futures
import math
import os
import pathlib
import subprocess
import sys
from logging import DEBUG, basicConfig, getLogger
from typing import *

logger = getLogger(__name__)


def gen(*, M: Optional[int], D: Optional[int], input_path: pathlib.Path) -> None:
    logger.info('running the generator...')
    with open(input_path, 'w') as fh:
        command = [sys.executable, str(pathlib.Path('scripts', 'generate.py'))]
        if M is not None:
            command.append('-M')
            command.append(str(M))
        if D is not None:
            command.append('-D')
            command.append(str(D))
        subprocess.check_call(command, stdout=fh)


def run(*, command: str, input_path: pathlib.Path, output_path: pathlib.Path) -> None:
    logger.info('running the command for %s...', str(input_path))
    try:
        with open(output_path, 'w') as fh:
            subprocess.check_call([str((pathlib.Path.cwd() / 'tools' / 'target' / 'release' / 'tester').resolve()), str(input_path), command], stdout=fh)
    except subprocess.SubprocessError:
        logger.exception('failed for input = %s', str(input_path))


def vis(*, input_path: pathlib.Path, output_path: pathlib.Path, vis_path: pathlib.Path) -> int:
    logger.info('running the visualizer for %s...', str(input_path))
    try:
        command = [str((pathlib.Path.cwd() / 'tools' / 'target' / 'release' / 'vis').resolve()), str(input_path), str(output_path)]
        score_bytes = subprocess.check_output(command)
    except subprocess.SubprocessError:
        logger.exception('failed for input = %s', str(input_path))
        return 0
    os.rename('out.svg', vis_path)
    if not score_bytes.startswith(b'Score = '):
        raise RuntimeError(score_bytes.decode())
    return int(score_bytes.split()[2])


def main() -> 'NoReturn':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--command', default='./a.out')
    parser.add_argument('-n', '--count', type=int, default=50)
    parser.add_argument('-j', '--jobs', type=int, default=2)
    parser.add_argument('-D', type=int)
    parser.add_argument('-M', type=int)
    args = parser.parse_args()

    basicConfig(level=DEBUG)

    if not pathlib.Path('tools').exists():
        logger.error('tools/ directory is not found')
        sys.exit(1)

    # gen
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as executor:
        for i in range(args.count):
            input_path = pathlib.Path('in', '%04d.txt' % i)
            executor.submit(gen, M=args.M, D=args.D, input_path=input_path)

    # run
    pathlib.Path('out').mkdir(exist_ok=True)
    command = ['cargo', 'build', '--manifest-path', str(pathlib.Path('tools', 'Cargo.toml')), '--release', '--bin', 'tester']
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as executor:
        for i in range(args.count):
            input_path = pathlib.Path('in', '%04d.txt' % i)
            output_path = pathlib.Path('out', '%04d.txt' % i)
            executor.submit(run, command=args.command, input_path=input_path, output_path=output_path)

    # vis
    pathlib.Path('vis').mkdir(exist_ok=True)
    scores: List[int] = []
    command = ['cargo', 'build', '--manifest-path', str(pathlib.Path('tools', 'Cargo.toml')), '--release', '--bin', 'vis']
    subprocess.check_output(command)
    for i in range(args.count):
        input_path = pathlib.Path('in', '%04d.txt' % i)
        output_path = pathlib.Path('out', '%04d.txt' % i)
        vis_path = pathlib.Path('vis', '%04d.svg' % i)
        score = vis(input_path=input_path, output_path=output_path, vis_path=vis_path)
        scores.append(score)
        logger.info('index = {}: score = {}'.format(i, score))
    average = sum(scores) / len(scores)
    logger.info('100 * average = %s', int(100 * average))
    if os.environ.get('GITHUB_ACTIONS') == 'true':
        print('::set-output name=average::{}'.format(int(100 * average)))

    if min(scores) <= 0:
        sys.exit(1)
    sys.exit(0)


if __name__ == '__main__':
    main()
