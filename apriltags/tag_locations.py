import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class tag_locations(Node):

    def __init__(self):
        super().__init__("tag_locations")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.camera_frame = "camera_color_optical_frame"
        self.tag1 = "global"
        self.tag2 = "odom"

        self.cam_tag1 = None
        self.cam_tag2 = None
        self.tag1_tag2 = None
    def timer_callback(self):
        try:
            t1 = self.tf_buffer.lookup_transform(
                self.camera_frame, self.tag1, rclpy.time.Time()
            )
            t2 = self.tf_buffer.lookup_transform(
                self.camera_frame, self.tag2, rclpy.time.Time()
            )

            t3 = self.tf_buffer.lookup_transform(
                self.tag1, self.tag2, rclpy.time.Time()
            )
            
            self.cam_tag1 = [
                            t1.transform.translation.x,
                            t1.transform.translation.y,
                            t1.transform.translation.z
                            ]
            self.cam_tag2 = [
                            t2.transform.translation.x,
                            t2.transform.translation.y,
                            t2.transform.translation.z
                            ]
            self.tag1_tag2 = [
                t3.transform.translation.x,
                t3.transform.translation.y,
                t3.transform.translation.z
            ]
        except TransformException as ex:
            self.get_logger().info(
                    f"Could not transform {ex}"
                )
        
        if (self.cam_tag1 and self.cam_tag2 is not None):
            # self.get_logger().info("Got transform from camera to robot base")
            x = self.cam_tag1[0]
            y = self.cam_tag1[1]
            z = self.cam_tag1[2]
            # self.get_logger().info(f"T1: x={x}, y={y}, z={z}")
            x1 = self.cam_tag2[0]
            y1 = self.cam_tag2[1]
            z1 = self.cam_tag2[2]
            # self.get_logger().info(f"T2: x={x1}, y={y1}, z={z1}")
            x2 = self.tag1_tag2[0]
            y2 = self.tag1_tag2[1]
            z2 = self.tag1_tag2[2]
            self.get_logger().info(f"T2: x={x2}, y={y2}, z={z2}")

            # self.get_logger().info(f"diff: x={x-x1}, y={y-y1}, z={z-z1}")
            # self.get_logger().info(f"dist: {((x-x1)**2+(y-y1)**2+(z-z1)**2)**0.5}")
            # self.get_logger().info(tcg)
            # self.get_logger().info(tco)


def main(args=None):
    rclpy.init(args=args)
    node=tag_locations()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
