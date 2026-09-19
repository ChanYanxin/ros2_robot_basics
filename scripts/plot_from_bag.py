#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_topics(bag_path: str, storage_id: str):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id=storage_id,
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }

    target_t, target_q = [], []
    actual_t, actual_q = [], []

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in ("/target_joint_state", "/joint_states"):
            continue

        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)
        if not msg.position:
            continue

        t = timestamp_ns * 1e-9
        if topic == "/target_joint_state":
            target_t.append(t)
            target_q.append(float(msg.position[0]))
        else:
            actual_t.append(t)
            actual_q.append(float(msg.position[0]))

    if not target_t or not actual_t:
        raise RuntimeError(
            "Bag does not contain usable /target_joint_state and /joint_states data."
        )

    t0 = min(target_t[0], actual_t[0])
    target_t = [t - t0 for t in target_t]
    actual_t = [t - t0 for t in actual_t]

    return target_t, target_q, actual_t, actual_q


def main():
    parser = argparse.ArgumentParser(
        description="Plot desired and actual joint angle from a ROS2 bag."
    )
    parser.add_argument("bag", help="Path to rosbag2 directory.")
    parser.add_argument(
        "--storage-id",
        default="sqlite3",
        help="rosbag2 storage plugin used when recording (default: sqlite3).",
    )
    parser.add_argument(
        "--output",
        default="docs/joint_tracking.png",
        help="Output image path.",
    )
    args = parser.parse_args()

    target_t, target_q, actual_t, actual_q = read_topics(
        args.bag, args.storage_id
    )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(9, 5))
    plt.plot(target_t, target_q, label="Target angle")
    plt.plot(actual_t, actual_q, label="Actual angle")
    plt.xlabel("Time [s]")
    plt.ylabel("Joint angle [rad]")
    plt.title("ROS2 Joint Tracking with PD Control")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output, dpi=160)
    print(f"Saved plot to: {output}")


if __name__ == "__main__":
    main()
