#!/usr/bin/env python3
"""Record nav_msgs/Odometry as a TUM trajectory."""
import argparse
import os
import rospy
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/roll/mapping/odometry")
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    output = open(args.output, "w", buffering=1)
    count = [0]

    def callback(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        output.write("{:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f}\n".format(
            msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        count[0] += 1

    rospy.init_node("odom_tum_recorder", anonymous=True)
    rospy.Subscriber(args.topic, Odometry, callback, queue_size=1000)
    rospy.on_shutdown(output.close)
    rospy.spin()
    print("recorded_poses={}".format(count[0]))


if __name__ == "__main__":
    main()
