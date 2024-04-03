from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qx, qy, qz, qw]

rot_vect_imu = np.array([np.deg2rad(180.0), np.deg2rad(0.0), np.deg2rad(180.0)])
rot_vect_gps = np.array([np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0)])
q_imu = get_quaternion_from_euler(rot_vect_imu[0], rot_vect_imu[1], rot_vect_imu[2])
q_gps = get_quaternion_from_euler(rot_vect_gps[0], rot_vect_gps[1], rot_vect_gps[2])

def generate_launch_description():

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='',
        name='spatial_imu_transform',
        output='screen',
        arguments=["0.469", "0", "0.462", str(q_imu[0]), str(q_imu[1]), str(q_imu[2]), str(q_imu[3]), "base_link", "imu2"]
    )

    gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='',
        name='spatial_gps_transform',
        output='screen',
        arguments=["0.8131", "0", "-0.01935", str(q_gps[0]), str(q_gps[1]), str(q_gps[2]), str(q_gps[3]), "imu2", "gps"]
    )

    spatial_node = Node(
        package='hal_an_spatial',
        executable='HALSpatial',
        name='hal_an_spatial',
        output={
                "stdout": "screen",
                "stderr": "screen",
        },
        parameters=[os.path.join(get_package_share_directory("hal_an_spatial"), 'params', 'params.yaml')],
    )

    return LaunchDescription([imu_tf, gps_tf, spatial_node])
