#!/usr/bin/env python3
"""
Scripts that generate required `#pragma cling` for loading the boost library in the C++ Jupyter Notebook
using the conda env

Usage:
generate_cling_boost.py
"""
import argparse
from pathlib import Path
import os


def main():
    parser = argparse.ArgumentParser("generate_cling_boost")
    parser.add_argument("--target-dir", default=".", help="path to save the file", type=str)
    args = parser.parse_args()

    boost_conda_path = Path(os.environ['CONDA_PREFIX'] + "/include")
    path = boost_conda_path / "boost"
    if not path.exists():
        print("[ERROR] Failed to find boost. Please install via conda install -c conda-forge boost")
        from sys import exit
        exit(1)

    include_dirs = path.glob("**/include")
    filename = "load_boost.h"
    with open(filename, "w") as f:
        f.write("#pragma once\n\n")
        for inc_dir in include_dirs:
            f.write('#pragma cling add_include_path("%s")\n' % str(inc_dir))

    os.makedirs(args.target_dir, exist_ok=True)
    print("Files written to", "%s/%s"%(args.target_dir, filename))


if __name__ == "__main__":
    main()
