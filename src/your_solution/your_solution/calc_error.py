import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import math


class CalcError(Node):
    def __init__(self):
        super().__init__('calc_error')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.x_err_pub = self.create_publisher(Float64, 'x_err', 10)
        self.y_err_pub = self.create_publisher(Float64, 'y_err', 10)
        self.z_err_pub = self.create_publisher(Float64, 'z_err', 10)
        
        self.timer = self.create_timer(0.1, self.calculate_error)
    
    def calculate_error(self):
        try:
            detected_transform = self.tf_buffer.lookup_transform(
                'map', 'detected_panel', rclpy.time.Time())
            
            panel_transforms = []
            for i in range(4):
                try:
                    panel_transform = self.tf_buffer.lookup_transform(
                        'map', f'panel_{i}', rclpy.time.Time())
                    panel_transforms.append(panel_transform)
                except tf2_ros.TransformException:
                    continue
            
            if not panel_transforms:
                return
            
            detected_pos = detected_transform.transform.translation
            
            closest_distance = float('inf')
            closest_panel_pos = None
            
            for panel_transform in panel_transforms:
                panel_pos = panel_transform.transform.translation
                distance = math.sqrt(
                    (detected_pos.x - panel_pos.x)**2 +
                    (detected_pos.y - panel_pos.y)**2 +
                    (detected_pos.z - panel_pos.z)**2
                )
                
                if distance < closest_distance:
                    closest_distance = distance
                    closest_panel_pos = panel_pos
            
            if closest_panel_pos is not None:
                x_error = detected_pos.x - closest_panel_pos.x
                y_error = detected_pos.y - closest_panel_pos.y
                z_error = detected_pos.z - closest_panel_pos.z
                
                x_err_msg = Float64()
                x_err_msg.data = x_error
                self.x_err_pub.publish(x_err_msg)
                
                y_err_msg = Float64()
                y_err_msg.data = y_error
                self.y_err_pub.publish(y_err_msg)
                
                z_err_msg = Float64()
                z_err_msg.data = z_error
                self.z_err_pub.publish(z_err_msg)
                
        except tf2_ros.TransformException:
            pass


def main():
    rclpy.init()
    node = CalcError()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()