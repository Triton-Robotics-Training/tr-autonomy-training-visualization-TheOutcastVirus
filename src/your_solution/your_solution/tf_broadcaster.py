import math

from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster

import numpy as np

import rclpy
from rclpy.node import Node

from tr_messages.msg import DetWithImg, SimGroundTruth


class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.detection_sub = self.create_subscription(
            DetWithImg,
            '/detections',
            self.detection_callback,
            10
        )
        
        self.ground_truth_sub = self.create_subscription(
            SimGroundTruth,
            '/simulation/ground_truth',
            self.ground_truth_callback,
            10
        )
    
    def detection_callback(self, msg):
        for detection in msg.detection_info.detections:
            for result in detection.results:
                t = TransformStamped()
                # TODO this is the wrong timestamp
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_frame'
                t.child_frame_id = 'detected_panel'
                
                pose = result.pose.pose
                t.transform.translation.x = pose.position.x
                t.transform.translation.y = pose.position.y
                t.transform.translation.z = pose.position.z
                t.transform.rotation.x = pose.orientation.x
                t.transform.rotation.y = pose.orientation.y
                t.transform.rotation.z = pose.orientation.z
                t.transform.rotation.w = pose.orientation.w
                
                self.tf_broadcaster.sendTransform(t)
    
    def ground_truth_callback(self, msg):
        t = TransformStamped()
        # TODO wrong timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_frame'
        
        camera_pose = msg.primary_robot.camera_pose
        t.transform.translation.x = camera_pose.position.x
        t.transform.translation.y = camera_pose.position.y
        t.transform.translation.z = camera_pose.position.z
        t.transform.rotation.x = camera_pose.orientation.x
        t.transform.rotation.y = camera_pose.orientation.y
        t.transform.rotation.z = camera_pose.orientation.z
        t.transform.rotation.w = camera_pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
        
        for i, panel_pose in enumerate(msg.secondary_robot.armor_panel_poses):
            t = TransformStamped()
            # TODO wrong timestamp
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f'panel_{i}'
            
            t.transform.translation.x = panel_pose.position.x
            t.transform.translation.y = panel_pose.position.y
            t.transform.translation.z = panel_pose.position.z
            t.transform.rotation.x = panel_pose.orientation.x
            t.transform.rotation.y = panel_pose.orientation.y
            t.transform.rotation.z = panel_pose.orientation.z
            t.transform.rotation.w = panel_pose.orientation.w
            
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
