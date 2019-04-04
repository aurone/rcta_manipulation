#!/usr/bin/env python3

import sys

if __name__ == '__main__':
    canonical_x = 0.85
    canonical_y = 0.0
    canonical_z = 0.0
    canonical_qz = 90.0

    x_samples = 3
    y_samples = 3
    z_samples = 3
    qz_samples = 3
    sample_res_x = 0.05
    sample_res_y = 0.05
    sample_res_z = 0.05
    sample_res_qz = 5.0

    num = 0
    for x in range(x_samples):
        for y in range(y_samples):
            for z in range(z_samples):
                for qz in range(qz_samples):
                    xx = canonical_x - (x_samples // 2 + x) * sample_res_x
                    yy = canonical_y - (y_samples // 2 + y) * sample_res_y
                    zz = canonical_z - (z_samples // 2 + z) * sample_res_z
                    qzz = canonical_qz - (qz_samples // 2 + qz) * sample_res_qz
                    # fname = f'test-{num}.yaml'
                    # fname = f'test.yaml'
                    fname = 'test-{}.yaml'.format(num)
                    with open(fname, 'w') as f:
                        f.write('start_state:\n')
                        f.write('  world_joint\\\\x: 0.0\n')
                        f.write('  world_joint\\\\y: 0.0\n')
                        f.write('  world_joint\\\\theta: 0.0\n')
                        f.write('  limb_right_joint1: 135.0\n')
                        f.write('  limb_right_joint2: 0.0\n')
                        f.write('  limb_right_joint3: 180.0\n')
                        f.write('  limb_right_joint4: 45.0\n')
                        f.write('  limb_right_joint5: 30.0\n')
                        f.write('  limb_right_joint6: 90.0\n')
                        f.write('  limb_right_joint7: -135.0\n')
                        f.write('  limb_left_joint1: -90.0\n')
                        f.write('  limb_left_joint2: 90.0\n')
                        f.write('  limb_left_joint3: 90.0\n')
                        f.write('  limb_left_joint4: 180.0\n')
                        f.write('  limb_left_joint5: -90.0\n')
                        f.write('  limb_left_joint6: 0.0\n')
                        f.write('  limb_left_joint7: 0.0\n')
                        f.write('object_start_position: 1.0\n')
                        f.write('object_goal_position: 0.45\n')
                        f.write('object_pose: [{}, {}, {}, {}, 0, 0 ]\n'.format(xx, yy, zz, qzz))
                    num = num + 1


