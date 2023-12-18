'''Dump all ros2 parameters to the command line.
'''

import signal
import subprocess
import os
import argparse


def main(param_substr: str):
    '''Dump all ros2 parameters to the command line.

    Args:
        param_substr (str): substring to filter parameters by
    '''
    print('Dumping all ros2 parameters to the command line.')

    nodes = subprocess.getoutput("ros2 node list").split("\n")
    for node in nodes:
        if node == '':
            continue
        lines = subprocess.getoutput(f"ros2 param dump {node}").split("\n")
        for line in lines:
            if param_substr in line:
                print(f"Node: {node} {line}")
    print('Done.')


if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--param_substr',
                           type=str,
                           default='',
                           help='substring to filter parameters by')

    args = argparser.parse_args()
    main(args.param_substr)
