#!/usr/bin/env python3
"""Send evolo a move_path goal written in scene metres.

python3 send_evolo_path.py <scenario.yaml> --ros-args -p use_sim_time:=true

Metres are Unity world coordinates: x = east, y = north (Unity z). In
AskoBearingSetups the origin is evolo's start pose. Obstacle boats use the
same numbers in WaypointNPCController.
"""

import json
import sys

import rclpy
import yaml
from geodesy.utm import UTMPoint
from rclpy.action import ActionClient
from rclpy.node import Node
from smarc_msgs.action import BaseAction

# ponytail: AskoWorld "GLOBALREF - AskoPierTip" only; read these from the
# scene's GlobalReferencePoint if another world prefab is used.
REF_UNITY_X, REF_UNITY_Z = -871.0, -1934.0
REF_EASTING, REF_NORTHING = 652149.8149329375, 6523374.627482311
UTM_ZONE, UTM_BAND = 33, "V"


def metres_to_latlon(east, north):
    """Unity world metres -> (lat, lon), matching GlobalReferencePoint."""
    utm = UTMPoint(
        REF_EASTING + east - REF_UNITY_X,
        REF_NORTHING + north - REF_UNITY_Z,
        zone=UTM_ZONE,
        band=UTM_BAND,
    )
    geo = utm.toMsg()
    return geo.latitude, geo.longitude


def move_path_goal(evolo):
    """move_path JSON from the scenario's evolo section."""
    tolerance = float(evolo.get("tolerance", 10.0))
    waypoints = []
    for east, north in evolo["waypoints"]:
        lat, lon = metres_to_latlon(east, north)
        waypoints.append(
            {"latitude": lat, "longitude": lon, "tolerance": tolerance}
        )
    return {"speed": evolo.get("speed", "standard"), "waypoints": waypoints}


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    if len(args) != 2:
        sys.exit(__doc__)
    with open(args[1]) as file:
        goal_json = move_path_goal(yaml.safe_load(file)["evolo"])

    rclpy.init()
    node = Node("send_evolo_path")
    client = ActionClient(node, BaseAction, "/evolo/move_path")
    if not client.wait_for_server(timeout_sec=10.0):
        sys.exit("/evolo/move_path server not available")

    goal = BaseAction.Goal()
    goal.goal.data = json.dumps(goal_json)
    handle_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, handle_future)
    handle = handle_future.result()
    if not handle.accepted:
        sys.exit("move_path rejected the goal")
    node.get_logger().info(f"Sent {len(goal_json['waypoints'])} waypoints")

    result_future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future)
        node.get_logger().info(f"Done, success={result_future.result().result.success}")
    except KeyboardInterrupt:
        handle.cancel_goal_async()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
