from types import SimpleNamespace

import pytest
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

import evolo_gimbal_calibration.corrected_camera_tf_node as correction_node
from evolo_gimbal_calibration.corrected_camera_tf_node import CalibratedCameraTFNode


def test_camera_trigger_preserves_local_behavior_and_uses_global_camera_stamp(monkeypatch):
    monkeypatch.setattr(correction_node, "correct_yaw", lambda *args, **kwargs: SimpleNamespace(yaw_deg=90.0, valid=True))
    yaw = TransformStamped()
    yaw.header.frame_id = "base"
    yaw.child_frame_id = "yaw"
    yaw.header.stamp.sec = 12
    yaw.transform.rotation.w = 1.0

    camera = TransformStamped()
    camera.header.frame_id = "map"
    camera.child_frame_id = "global_camera"
    camera.header.stamp.sec = 11
    camera.transform.translation.x = 10.0
    camera.transform.rotation.w = 1.0

    lookups = []
    sent = []
    node = SimpleNamespace(
        camera_tf_driven=True,
        base_frame="base",
        yaw_frame="yaw",
        camera_frame="global_camera",
        corrected_camera_frame="corrected_camera",
        correction_mode="shape",
        negate_correction=False,
        extend_correction=False,
        tf_buffer=SimpleNamespace(lookup_transform=lambda parent, child, stamp: (
            lookups.append((parent, child, stamp.nanoseconds)) or yaw
        )),
        tf_broadcaster=SimpleNamespace(sendTransform=sent.append),
        valid_publisher=SimpleNamespace(publish=lambda msg: None),
    )
    node.publish_corrected_transform = lambda base, raw=None: (
        CalibratedCameraTFNode.publish_corrected_transform(node, base, raw)
    )

    CalibratedCameraTFNode.tf_callback(node, TFMessage(transforms=[camera]))

    assert lookups == [("base", "yaw", 11_000_000_000)]
    assert len(sent) == 1
    assert sent[0].header.stamp == camera.header.stamp
    assert sent[0].header.frame_id == "map"
    assert sent[0].child_frame_id == "corrected_camera"
    assert sent[0].transform.translation.x == 10.0
    assert sent[0].transform.rotation.z == pytest.approx(2**-0.5)

    local_camera = TransformStamped()
    local_camera.header.frame_id = "yaw"
    local_camera.child_frame_id = "local_camera"
    local_camera.transform.translation.x = 10.0
    local_camera.transform.rotation.w = 1.0
    node.camera_tf_driven = False
    node.camera_frame = "local_camera"
    node.tf_buffer.lookup_transform = lambda parent, child, stamp: (
        lookups.append((parent, child, stamp.nanoseconds)) or local_camera
    )

    CalibratedCameraTFNode.tf_callback(node, TFMessage(transforms=[yaw]))

    assert lookups[-1] == ("yaw", "local_camera", 12_000_000_000)
    assert sent[-1].header.frame_id == "yaw"
    assert sent[-1].header.stamp == yaw.header.stamp
    assert sent[-1].transform.translation.x == pytest.approx(0.0, abs=1e-12)
    assert sent[-1].transform.translation.y == pytest.approx(10.0)
