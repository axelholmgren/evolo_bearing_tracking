"""python3 test_send_evolo_path.py (or pytest) after sourcing ROS."""

import math

from send_evolo_path import metres_to_latlon, move_path_goal


def test_globalref_maps_to_its_latlon():
    lat, lon = metres_to_latlon(-871.0, -1934.0)
    assert abs(lat - 58.823332) < 1e-6 and abs(lon - 17.635227) < 1e-6


def test_100m_north_is_100m():
    lat0, lon0 = metres_to_latlon(0.0, 0.0)
    lat1, lon1 = metres_to_latlon(0.0, 100.0)
    north_m = (lat1 - lat0) * 111_320.0  # metres per degree latitude (approx.)
    assert abs(north_m - 100.0) < 1.0 and abs(lon1 - lon0) < 1e-4


def test_goal_shape():
    goal = move_path_goal({"waypoints": [[0, 100], [50, 200]], "tolerance": 5})
    assert goal["speed"] == "standard"
    assert [wp["tolerance"] for wp in goal["waypoints"]] == [5.0, 5.0]
    assert all(math.isfinite(wp["latitude"]) for wp in goal["waypoints"])


if __name__ == "__main__":
    test_globalref_maps_to_its_latlon()
    test_100m_north_is_100m()
    test_goal_shape()
    print("ok")
