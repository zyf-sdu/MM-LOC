#!/usr/bin/env python3
"""Convert NCLT ground-truth CSV to body or project IMU-origin TUM."""
import argparse
import math


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (cy*cp*sr-sy*sp*cr, sy*cp*sr+cy*sp*cr,
            sy*cp*cr-cy*sp*sr, cy*cp*cr+sy*sp*sr)


def rotated_offset(r, p, y, v=(-0.11, -0.18, -0.71)):
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    x, yy, z = v
    return (cy*cp*x+(cy*sp*sr-sy*cr)*yy+(cy*sp*cr+sy*sr)*z,
            sy*cp*x+(sy*sp*sr+cy*cr)*yy+(sy*sp*cr-cy*sr)*z,
            -sp*x+cp*sr*yy+cp*cr*z)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv")
    parser.add_argument("output_tum")
    parser.add_argument('--frame', choices=['imu','body'], default='imu',
                        help='Reference pose origin; legacy default is imu')
    args = parser.parse_args()
    written = 0
    with open(args.input_csv) as src, open(args.output_tum, "w") as dst:
        for line in src:
            try:
                t, x, y, z, r, p, yw = (float(v.strip()) for v in line.split(","))
            except (ValueError, TypeError):
                continue
            dx, dy, dz = rotated_offset(r, p, yw) if args.frame == 'imu' else (0,0,0)
            qx, qy, qz, qw = quaternion(r, p, yw)
            dst.write(f"{t/1e6:.9f} {x+dx:.9f} {y+dy:.9f} {z+dz:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
            written += 1
    print(f"written_poses={written}")


if __name__ == "__main__":
    main()
