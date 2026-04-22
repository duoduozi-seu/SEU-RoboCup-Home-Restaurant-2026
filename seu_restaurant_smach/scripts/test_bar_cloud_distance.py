#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import heapq
import math
import time

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as point_cloud2


def estimate_distance(cloud_msgs, cfg):
    filtered_x_values = []
    raw_count = 0
    finite_count = 0

    for cloud_msg in cloud_msgs:
        for point in point_cloud2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            raw_count += 1
            try:
                x, y, z = float(point[0]), float(point[1]), float(point[2])
            except (TypeError, ValueError, IndexError):
                continue

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            finite_count += 1

            if x < cfg["x_min"] or x > cfg["x_max"]:
                continue
            if y < cfg["y_min"] or y > cfg["y_max"]:
                continue
            if z < cfg["z_min"] or z > cfg["z_max"]:
                continue
            filtered_x_values.append(x)

    nearest_count = int(cfg["nearest_count"])
    stats = {
        "raw_count": raw_count,
        "finite_count": finite_count,
        "filtered_count": len(filtered_x_values),
        "nearest_count": nearest_count,
        "frame_count": len(cloud_msgs),
    }
    if len(filtered_x_values) < nearest_count:
        return None, stats

    nearest_x_values = heapq.nsmallest(nearest_count, filtered_x_values)
    distance_m = sum(nearest_x_values) / float(nearest_count)
    stats["nearest_min"] = min(nearest_x_values)
    stats["nearest_max"] = max(nearest_x_values)
    return distance_m, stats


def main():
    rospy.init_node("test_bar_cloud_distance")

    cfg = {
        "topic": rospy.get_param("~topic", "/cloud_registered_body"),
        "target_distance": float(rospy.get_param("~target_distance", 0.35)),
        "x_min": float(rospy.get_param("~x_min", 0.26)),
        "x_max": float(rospy.get_param("~x_max", 1.00)),
        "y_min": float(rospy.get_param("~y_min", -0.10)),
        "y_max": float(rospy.get_param("~y_max", 0.10)),
        "z_min": float(rospy.get_param("~z_min", 0.2)),
        "z_max": float(rospy.get_param("~z_max", 0.7)),
        "nearest_count": int(rospy.get_param("~nearest_count", 50)),
        "frame_count": int(rospy.get_param("~frame_count", 10)),
        "timeout_sec": float(rospy.get_param("~timeout_sec", 1.0)),
        "rate_hz": float(rospy.get_param("~rate_hz", 2.0)),
    }

    print(
        "cloud distance test | topic={topic} box=x[{x_min:.2f},{x_max:.2f}] "
        "y[{y_min:.2f},{y_max:.2f}] z[{z_min:.2f},{z_max:.2f}] "
        "nearest={nearest_count} frames={frame_count} target={target_distance:.2f}m".format(**cfg),
        flush=True,
    )

    rate = rospy.Rate(cfg["rate_hz"])
    while not rospy.is_shutdown():
        cloud_msgs = []
        try:
            for _ in range(cfg["frame_count"]):
                cloud_msgs.append(
                    rospy.wait_for_message(
                        cfg["topic"],
                        PointCloud2,
                        timeout=cfg["timeout_sec"],
                    )
                )
        except Exception as exc:
            print(
                "{} timeout/no cloud | topic={} got_frames={}/{} error={}".format(
                    time.strftime("%H:%M:%S"),
                    cfg["topic"],
                    len(cloud_msgs),
                    cfg["frame_count"],
                    exc,
                ),
                flush=True,
            )
            rate.sleep()
            continue

        distance_m, stats = estimate_distance(cloud_msgs, cfg)
        frame_id = getattr(getattr(cloud_msgs[-1], "header", None), "frame_id", "")
        stamp = time.strftime("%H:%M:%S")
        if distance_m is None:
            print(
                "{} no distance | frame={} frames={} raw={} finite={} filtered={} need={}".format(
                    stamp,
                    frame_id,
                    stats["frame_count"],
                    stats["raw_count"],
                    stats["finite_count"],
                    stats["filtered_count"],
                    stats["nearest_count"],
                ),
                flush=True,
            )
            rate.sleep()
            continue

        move_m = distance_m - cfg["target_distance"]
        print(
            "{} distance={:.3f}m move_to_target={:+.3f}m frame={} frames={} filtered={} "
            "nearest_x=[{:.3f},{:.3f}]".format(
                stamp,
                distance_m,
                move_m,
                frame_id,
                stats["frame_count"],
                stats["filtered_count"],
                stats["nearest_min"],
                stats["nearest_max"],
            ),
            flush=True,
        )
        rate.sleep()


if __name__ == "__main__":
    main()
