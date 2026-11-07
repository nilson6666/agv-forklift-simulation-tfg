#!/usr/bin/env python3
"""Render the behaviour tree of the unloading mission (pallet_cycle_bt.py)
as PNG and SVG images in this folder.

The tree is built with the same function the mission uses
(`build_default_mission()`), so the images always match the code.

Usage (with ROS 2 sourced; graphviz and python3-pydot installed):
  python3 docs/render_bt_diagram.py

Output:
  bt_cycle_first.{png,svg}  cycle of the first pallet (dropped on the floor)
  bt_cycle_stack.{png,svg}  cycle of a stacked pallet (aligns with the one below)
  bt_mission.{png,svg}      whole mission (3 cycles)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

import py_trees  # noqa: E402
import pallet_cycle_bt  # noqa: E402


def render(tree, name):
    graph = py_trees.display.dot_tree(tree)
    graph.set_rankdir('LR')  # left to right: phases stacked, readable width
    for fmt in ('png', 'svg'):
        path = os.path.join(HERE, f'{name}.{fmt}')
        graph.write(path, format=fmt)
        print(f'wrote {path}')


def main():
    mission = pallet_cycle_bt.build_default_mission()
    render(mission.children[0], 'bt_cycle_first')
    render(mission.children[1], 'bt_cycle_stack')
    render(mission, 'bt_mission')


if __name__ == '__main__':
    main()
