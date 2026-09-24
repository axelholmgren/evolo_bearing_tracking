from types import SimpleNamespace

from geometry_msgs.msg import QuaternionStamped, TransformStamped

from evolo_bearing.bearing_marker_node import BearingRayNode


def test_ray_history_keeps_each_marker():
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    ray = QuaternionStamped()
    ray.quaternion.w = 1.0
    published = []
    node = SimpleNamespace(
        tf_buffer=SimpleNamespace(lookup_transform=lambda **_: transform),
        camera_frame="evolo/z1_camera_link",
        correction_valid=False,
        keep_ray_history=True,
        next_marker_id=0,
        marker_publisher=SimpleNamespace(publish=published.append),
    )

    BearingRayNode.poi_callback(node, ray)
    BearingRayNode.poi_callback(node, ray)

    assert [marker.id for marker in published] == [0, 1]
    assert all(marker.lifetime.sec == 0 for marker in published)

    node.keep_ray_history = False
    BearingRayNode.poi_callback(node, ray)
    assert published[-1].id == 0
    assert published[-1].lifetime.sec == 1
