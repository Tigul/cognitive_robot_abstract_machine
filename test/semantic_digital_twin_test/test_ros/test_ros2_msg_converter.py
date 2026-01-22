from geometry_msgs.msg import TransformStamped

from semantic_digital_twin.adapters.ros.msg_converter import (
    HomogeneousTransformationMatrixROS2Converter,
)


def test_convert_transform(cylinder_bot_world):
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "bot"
    transform.transform.translation.x = 1.0
    transform.transform.rotation.x = 1.0
    transform.transform.rotation.w = 0.0
    transformation_matrix = (
        HomogeneousTransformationMatrixROS2Converter.from_ros2_message(
            transform, world=cylinder_bot_world
        )
    )
    position = transformation_matrix.to_position().evaluate()
    rotation = transformation_matrix.to_quaternion().evaluate()
    assert position[0] == transform.transform.translation.x
    assert position[1] == transform.transform.translation.y
    assert position[2] == transform.transform.translation.z

    assert rotation[0] == transform.transform.rotation.x
    assert rotation[1] == transform.transform.rotation.y
    assert rotation[2] == transform.transform.rotation.z
    assert rotation[3] == transform.transform.rotation.w
    assert transformation_matrix.child_frame == cylinder_bot_world.get_body_by_name(
        "bot"
    )
    assert transformation_matrix.reference_frame == cylinder_bot_world.get_body_by_name(
        "map"
    )

    transform2 = HomogeneousTransformationMatrixROS2Converter.to_ros2_message(
        transformation_matrix
    )
    assert transform == transform2
