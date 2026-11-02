#!/usr/bin/env python3
"""Open the current arena geometry in Gazebo Classic 11 without ROS."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def make_preview(source, destination):
    """Adapt only world configuration; keep all arena models unchanged."""
    tree = ET.parse(source)
    tree.getroot().set('version', '1.7')
    world = tree.getroot().find('world')
    world.set('name', 'arena_preview_classic')
    for plugin in world.findall('plugin'):
        world.remove(plugin)
    world.find('physics').set('type', 'ode')
    # The temporary world lives outside the asset directory. Absolute file URIs
    # allow Classic to load the same meshes from any working directory.
    for uri in world.findall('.//mesh/uri'):
        if '://' not in uri.text:
            uri.text = (source.parent / uri.text).resolve().as_uri()
    old_gui = world.find('gui')
    if old_gui is not None:
        world.remove(old_gui)
    camera = ET.SubElement(ET.SubElement(world, 'gui'), 'camera', name='user_camera')
    ET.SubElement(camera, 'pose').text = '-28 -32 30 0 0.615 0.852'
    ET.SubElement(camera, 'view_controller').text = 'orbit'
    ET.SubElement(camera, 'projection_type').text = 'perspective'
    ET.indent(tree, space='  ')
    tree.write(destination, encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true',
                        help='Validate the converted world with Classic gz sdf; do not open the GUI.')
    args = parser.parse_args()
    executable = 'gz' if args.check else 'gazebo'
    if shutil.which(executable) is None:
        parser.exit(1, f'{executable} tidak ditemukan. Preview ini membutuhkan Gazebo Classic 11.\n')
    workspace = Path(__file__).resolve().parents[1]
    source = workspace / 'src/sim_bringup/worlds/dummy_arena.sdf'
    environment = os.environ.copy()
    environment['GAZEBO_MODEL_DATABASE_URI'] = ''
    with tempfile.TemporaryDirectory(prefix='ros2_fp_classic_preview_') as directory:
        preview = Path(directory) / 'arena.world'
        make_preview(source, preview)
        command = (['gz', 'sdf', '-k', str(preview)] if args.check
                   else ['gazebo', '--verbose', str(preview)])
        print('Preview arena: platform, empat ramp, dan empat papan ArUco.', flush=True)
        if not args.check:
            print('Tutup Gazebo atau tekan Ctrl+C di terminal untuk keluar.', flush=True)
        try:
            return subprocess.call(command, env=environment)
        except KeyboardInterrupt:
            return 130


if __name__ == '__main__':
    raise SystemExit(main())
