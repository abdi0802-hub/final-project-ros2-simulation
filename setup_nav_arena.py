#!/usr/bin/env python3
"""Bangun konfigurasi Nav2 siap pakai untuk arena rover (Humble).

Pemakaian (di terminal yang sudah `source scripts/setup_env.bash`):
    python3 setup_nav_arena.py [folder_output]        # default: ~/arena_nav

Yang dibuat di folder_output:
  arena.pgm + arena.yaml   peta 2D: ramp & platform BEBAS, pagar/tembok/papan terisi
  nav2_arena.yaml          parameter Nav2 = parameter bawaan nav2_bringup yang sudah diubah
  bt_no_recovery.xml       behavior tree tanpa spin/backup (tanpa putar-putar pemulihan)

Perubahan terhadap parameter bawaan:
  * use_sim_time True di semua node
  * costmap hanya static_layer + inflation_layer (tanpa voxel/obstacle layer dari LiDAR,
    yang menandai ramp sebagai penghalang dan 'sensor origin out of map bounds')
  * velocity_smoother tidak lagi membatasi kecepatan 0,26 m/s
  * toleransi yaw tujuan 3,14 (orientasi akhir bebas), toleransi xy 0,3 m
  * AMCL: pose awal = spawn (-11, 0, 0), noise odometri lebih besar
  * planner A*, robot_radius 0,65 m
"""
import copy
import math
import os
import sys
from pathlib import Path

import yaml

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else '~/arena_nav').expanduser()
SPAWN = (-11.0, 0.0, 0.0)

# ------------------------------------------------------------------ peta 2D
RES, HALF = 0.05, 15.0
N = int(round(2 * HALF / RES))
FREE, OCC = 254, 0


def build_map():
    img = bytearray([FREE]) * (N * N)

    def col(x):
        return min(N - 1, max(0, int((x + HALF) / RES)))

    def row(y):
        return min(N - 1, max(0, N - 1 - int((y + HALF) / RES)))

    def rect(x0, x1, y0, y1):
        c0, c1 = col(min(x0, x1)), col(max(x0, x1) - 1e-9)
        r0, r1 = row(max(y0, y1)), row(min(y0, y1) + 1e-9)
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                img[r * N + c] = OCC

    def disc(cx, cy, rad):
        for r in range(N):
            y = HALF - (r + 0.5) * RES
            for c in range(N):
                x = -HALF + (c + 0.5) * RES
                if (x - cx) ** 2 + (y - cy) ** 2 <= rad * rad:
                    img[r * N + c] = OCC

    rect(14.75, 15.0, -15, 15)
    rect(-15.0, -14.75, -15, 15)
    rect(-15, 15, 14.75, 15.0)
    rect(-15, 15, -15.0, -14.75)
    for s in (-1, 1):                                   # pagar platform (4 bukaan ramp 3 m)
        rect(s * 4.0, s * 4.2, 1.5, 4.2)
        rect(s * 4.0, s * 4.2, -4.2, -1.5)
        rect(1.5, 4.2, s * 4.0, s * 4.2)
        rect(-4.2, -1.5, s * 4.0, s * 4.2)
        for side in (-1, 1):                            # pagar ramp
            a, b = side * 1.5, side * 1.7
            rect(s * 4.0, s * 10.0, a, b)
            rect(a, b, s * 4.0, s * 10.0)
    for sx in (-1, 1):                                  # papan ArUco di sudut
        for sy in (-1, 1):
            disc(sx * 13.0, sy * 13.0, 0.6)

    (OUT / 'arena.pgm').write_bytes(f'P5\n{N} {N}\n255\n'.encode() + bytes(img))
    (OUT / 'arena.yaml').write_text(
        'image: arena.pgm\nmode: trinary\n'
        f'resolution: {RES}\norigin: [{-HALF}, {-HALF}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


# ------------------------------------------------------------ parameter Nav2
def load_default_params():
    try:
        from ament_index_python.packages import get_package_share_directory
        base = Path(get_package_share_directory('nav2_bringup'))
    except Exception:
        base = Path('/opt/ros/humble/share/nav2_bringup')
    path = base / 'params' / 'nav2_params.yaml'
    if not path.exists():
        sys.exit(f'Tidak menemukan {path}. Jalankan setelah `source scripts/setup_env.bash` '
                 'dan pastikan ros-humble-nav2-bringup terpasang.')
    return yaml.safe_load(path.read_text())


def rp(cfg, *keys):
    """Ambil dict ros__parameters di cfg[k1][k2]..., dibuat jika belum ada."""
    d = cfg
    for k in keys:
        d = d.setdefault(k, {})
    return d.setdefault('ros__parameters', {})


def tune(cfg, bt_xml):
    # use_sim_time di semua node
    for name, node in cfg.items():
        if isinstance(node, dict):
            if 'ros__parameters' in node:
                node['ros__parameters']['use_sim_time'] = True
            else:
                for sub in node.values():
                    if isinstance(sub, dict) and 'ros__parameters' in sub:
                        sub['ros__parameters']['use_sim_time'] = True

    # --- AMCL
    a = rp(cfg, 'amcl')
    a.update(base_frame_id='base_footprint', odom_frame_id='odom', global_frame_id='map',
             scan_topic='scan', laser_max_range=30.0, max_beams=120,
             update_min_d=0.1, update_min_a=0.1, transform_tolerance=1.0,
             alpha1=0.3, alpha2=0.3, alpha3=0.3, alpha4=0.3,
             set_initial_pose=True, always_reset_initial_pose=False,
             initial_pose={'x': SPAWN[0], 'y': SPAWN[1], 'z': 0.0, 'yaw': SPAWN[2]})

    # --- costmap (tanpa layer berbasis LiDAR)
    inflation = {'plugin': 'nav2_costmap_2d::InflationLayer',
                 'cost_scaling_factor': 3.0, 'inflation_radius': 0.9}
    static = {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_subscribe_transient_local': True}
    for which in ('global_costmap', 'local_costmap'):
        c = rp(cfg, which, which)
        c.pop('footprint', None)
        c.update(robot_radius=0.65, robot_base_frame='base_footprint', global_frame='map',
                 plugins=['static_layer', 'inflation_layer'], resolution=0.05,
                 update_frequency=5.0 if which == 'local_costmap' else 1.0,
                 publish_frequency=2.0 if which == 'local_costmap' else 1.0)
        c['static_layer'] = copy.deepcopy(static)
        c['inflation_layer'] = copy.deepcopy(inflation)
        c.pop('voxel_layer', None)
        c.pop('obstacle_layer', None)
        if which == 'local_costmap':
            c.update(rolling_window=True, width=6, height=6)

    # --- planner
    p = rp(cfg, 'planner_server')
    g = p.setdefault('GridBased', {})
    g.setdefault('plugin', 'nav2_navfn_planner/NavfnPlanner')
    g.update(tolerance=0.5, use_astar=True, allow_unknown=True)

    # --- controller (DWB bawaan, kecepatan lebih tinggi, orientasi akhir bebas)
    ctl = rp(cfg, 'controller_server')
    ctl['failure_tolerance'] = 1.0
    ctl.setdefault('progress_checker', {}).update(required_movement_radius=0.3,
                                                  movement_time_allowance=30.0)
    ctl.setdefault('general_goal_checker', {}).update(xy_goal_tolerance=0.3,
                                                      yaw_goal_tolerance=3.14, stateful=True)
    ctl.setdefault('FollowPath', {}).update(
        max_vel_x=0.6, max_speed_xy=0.6, min_vel_x=0.0, min_speed_xy=0.0, max_vel_theta=0.8,
        acc_lim_x=1.0, decel_lim_x=-1.0, acc_lim_theta=1.0, decel_lim_theta=-1.0,
        transform_tolerance=0.5, xy_goal_tolerance=0.3)

    # --- velocity smoother (bawaan membatasi 0,26 m/s)
    vs = rp(cfg, 'velocity_smoother')
    vs.update(max_velocity=[0.6, 0.0, 0.8], min_velocity=[-0.3, 0.0, -0.8],
              max_accel=[1.0, 0.0, 1.0], max_decel=[-1.0, 0.0, -1.0])

    # --- BT navigator: tanpa recovery spin/backup
    b = rp(cfg, 'bt_navigator')
    b.update(robot_base_frame='base_footprint', default_nav_to_pose_bt_xml=str(bt_xml))
    return cfg


BT_XML = """<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateNoRecovery">
      <RateController hz="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </RateController>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
"""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    build_map()
    bt_path = (OUT / 'bt_no_recovery.xml').resolve()
    bt_path.write_text(BT_XML)
    cfg = tune(load_default_params(), bt_path)
    (OUT / 'nav2_arena.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    print(f'Selesai. File ada di {OUT}\n')
    print('Jalankan Nav2 (setelah simulasi hidup):')
    print(f'  ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true autostart:=true \\\n'
          f'      map:={OUT}/arena.yaml params_file:={OUT}/nav2_arena.yaml')


if __name__ == '__main__':
    main()
