"""Run correctness checks, the main experiment, and the order study."""
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    root=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--output',default='outputs')
    parser.add_argument('--plot-only',action='store_true')
    options,_=parser.parse_known_args()
    subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-v'],cwd=root,check=True)
    subprocess.run([sys.executable,'run.py',*sys.argv[1:]],cwd=root,check=True)
    if not options.plot_only:
        # Inherits the actual main-run time limit (including --quick) from metadata.
        subprocess.run([sys.executable,'order_study.py','--output',options.output],cwd=root,check=True)

if __name__=='__main__':
    main()
