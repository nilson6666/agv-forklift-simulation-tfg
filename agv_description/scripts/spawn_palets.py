#!/usr/bin/env python3
"""Spawn N pallets on the trailer, each with a UNIQUE AprilTag id (tag36h11 0..9).

For each pallet, generates a temporary SDF from
models/palet_export/robot.sdf, replacing the model name and the tag
texture (tag36_11_0000i.png, one per id, in palet_export/assets/), and
creates it through the world's EntityFactory service. Pallets are
dropped from drop-z and physics settles them on the trailer's loading
floor (z=1.7318).

The trailer is already part of the magatzem.sdf world (models/trailer, at (6,3)).

Typical use (sim already running with world:=magatzem.sdf):
    ros2 run agv_description spawn_palets.py --count 4
    ros2 run agv_description spawn_palets.py --count 10        # scale test
    ros2 run agv_description spawn_palets.py --dry-run

Default geometry (trailer at (6,3), bed edge at y=3.245):
pallet i at x = start_x + i*spacing, y=2.645 (tag face EXACTLY at the
edge, facing +y, towards the north aisle), z=1.95.
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_share_directory

MODELS_DIR = os.path.join(get_package_share_directory('agv_description'),
                          'models')
PALET_SDF = os.path.join(MODELS_DIR, 'palet_export', 'robot.sdf')
PALET_ASSETS = os.path.join(MODELS_DIR, 'palet_export', 'assets')


def crear_entitat(world, sdf_filename, name, x, y, z, timeout_ms=10000):
    req = (f'sdf_filename: "{sdf_filename}", '
           f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}, '
           f'name: "{name}"')
    cmd = ['gz', 'service', '-s', f'/world/{world}/create',
           '--reqtype', 'gz.msgs.EntityFactory',
           '--reptype', 'gz.msgs.Boolean',
           '--timeout', str(timeout_ms), '--req', req]
    out = subprocess.run(cmd, capture_output=True, text=True)
    ok = 'data: true' in out.stdout
    if not ok:
        print(f'  ERROR creant {name}: {out.stdout.strip()} {out.stderr.strip()}',
              file=sys.stderr)
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--count', type=int, default=4,
                   help='nombre de palets (ids 0..count-1)')
    p.add_argument('--world', default='magatzem',
                   help='nom del mon SDF (magatzem | mon_sense_ombres)')
    p.add_argument('--trailer-x', type=float, default=6.0)
    p.add_argument('--start-x', type=float, default=7.0,
                   help='x del primer palet')
    p.add_argument('--spacing', type=float, default=1.0,
                   help='pas entre palets en x (palet 0.8 + marge)')
    p.add_argument('--y', type=float, default=2.645,
                   help='y dels palets (2.645 = tag a la vora del llit)')
    p.add_argument('--drop-z', type=float, default=1.95,
                   help='alcada de caiguda (la fisica els assenta)')
    p.add_argument('--dry-run', action='store_true',
                   help='mostra que es faria sense tocar la sim')
    args = p.parse_args()

    # Available tags = actual tag36_11_*.png files
    tags = sorted(re.findall(r'tag36_11_(\d{5})\.png',
                             ' '.join(os.listdir(PALET_ASSETS))))
    if args.count > len(tags):
        sys.exit(f'Nomes hi ha {len(tags)} textures de tag a {PALET_ASSETS} '
                 f'(demanats {args.count}). Baixa mes ids de '
                 'AprilRobotics/apriltag-imgs i escala x51 NEAREST.')

    # Check that the last pallet fits on the bed (x=6..18 with trailer at x=6)
    last_x = args.start_x + (args.count - 1) * args.spacing
    llit_max = args.trailer_x + 12.0 - 0.5  # half-pallet margin
    if last_x > llit_max:
        sys.exit(f'El palet {args.count - 1} cauria a x={last_x:.2f}, fora '
                 f'del llit (max {llit_max:.2f}). Redueix count o spacing.')

    template = open(PALET_SDF).read()

    # CAREFUL: the create service answers "true" when it ACCEPTS the
    # request, but UserCommands reads the SDF file asynchronously on the
    # next sim step. The temporary file must NOT be deleted right after the
    # call (verified race: ALL pallets were lost). They are deleted at the
    # end, after checking with the model list that they exist.
    temporals = []
    demanats = []
    for i in range(args.count):
        x = args.start_x + i * args.spacing
        nom = f'palet_{i}'
        print(f'{nom}: tag id {i} a ({x:.2f}, {args.y}, {args.drop_z})')
        if args.dry_run:
            continue
        sdf = template.replace('tag36_11_00000.png', f'tag36_11_{i:05d}.png')
        sdf = sdf.replace('<model name="pallet">', f'<model name="{nom}">')
        with tempfile.NamedTemporaryFile('w', suffix=f'_{nom}.sdf',
                                         delete=False) as f:
            f.write(sdf)
            temporals.append(f.name)
        if crear_entitat(args.world, temporals[-1], nom, x, args.y,
                         args.drop_z):
            demanats.append(nom)
        time.sleep(0.7)  # let physics settle before the next one

    if args.dry_run:
        return

    time.sleep(2.0)  # margin for UserCommands to consume the files
    out = subprocess.run(['gz', 'model', '--list'],
                         capture_output=True, text=True)
    vius = [n for n in demanats if n in out.stdout]
    for t in temporals:
        os.unlink(t)

    print(f'\n{len(vius)}/{args.count} palets creats al mon "{args.world}".')
    perduts = [n for n in demanats if n not in vius]
    if perduts:
        print(f'PERDUTS (no apareixen a la sim): {", ".join(perduts)}',
              file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
