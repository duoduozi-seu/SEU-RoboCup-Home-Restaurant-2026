#!/usr/bin/env python3
import argparse

import rospy

from seu_voice.srv import seu_voice


def parse_args():
    parser = argparse.ArgumentParser(description="Demo client for seu_voice service.")
    parser.add_argument("text", nargs="+", help="text to speak")
    parser.add_argument(
        "--service",
        default="/voice_service",
        help="service name, default: /voice_service",
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("seu_voice_client_demo", anonymous=True)
    args = parse_args()

    text = " ".join(args.text).strip()
    if not text:
        rospy.logerr("text is empty")
        return

    rospy.wait_for_service(args.service)
    client = rospy.ServiceProxy(args.service, seu_voice)

    try:
        resp = client(text=text)
    except rospy.ServiceException as exc:
        rospy.logerr("service call failed: %s", exc)
        return

    rospy.loginfo(
        "state=%s errorcode=%d synth=%.3fs play=%.3fs total=%.3fs errormsg=%s",
        resp.state,
        resp.errorcode,
        resp.synth_time_sec,
        resp.play_time_sec,
        resp.total_time_sec,
        resp.errormsg,
    )


if __name__ == "__main__":
    main()
