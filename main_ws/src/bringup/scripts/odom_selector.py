#!/usr/bin/env python3
"""Compatibility wrapper: run odom_tf_bridge as odom_selector.

Some launch files expect `odom_selector.py`. This wrapper simply runs
`odom_tf_bridge.py` in the same folder so the executable name exists.
"""
import os
import runpy


def main():
    here = os.path.dirname(__file__)
    runpy.run_path(os.path.join(here, "odom_tf_bridge.py"), run_name="__main__")


if __name__ == "__main__":
    main()
