import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
# from nav2_msgs.msg import Odometry


class tag_locations(Node):

    def __init__(self):
        super().__init__("tag_locations")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.timer_callback)

        # self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)

        # self.camera_frame = "camera_color_optical_frame"
        self.camera_frame = "zed_camera_center"
        self.tag1 = "global"
        self.tag2 = "odom"

        self.cam_tag1 = None
        self.cam_tag2 = None
        self.tag1_tag2 = None

        # self.odom = Odometry()

    def odom_pub(self,posx ,posy, posz, orx, ory, orz, orw):
        """_summary_

        Args:
            posx (_type_): _description_
            posy (_type_): _description_
            posz (_type_): _description_
            orx (_type_): _description_
            ory (_type_): _description_
            orz (_type_): _description_
            orw (_type_): _description_
        """
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.odom.pose.pose.position.x = posx
        self.odom.pose.pose.position.y = posy
        self.odom.pose.pose.position.z = posz
        self.odom.pose.pose.orientation.x = orx
        self.odom.pose.pose.orientation.y = ory
        self.odom.pose.pose.orientation.z = orz
        self.odom.pose.pose.orientation.w = orw
        self.odom_publisher.publish(self.odom)

    def timer_callback(self):
        """_summary_
        """
        try:
            # t1 = self.tf_buffer.lookup_transform(
            #     self.camera_frame, self.tag1, rclpy.time.Time()
            # )
            t2 = self.tf_buffer.lookup_transform(
                self.camera_frame, self.tag2, rclpy.time.Time()
            )

            t3 = self.tf_buffer.lookup_transform(
                self.tag1, self.tag2, rclpy.time.Time()
            )

            # self.cam_tag1 = [
            #     t1.transform.translation.x,
            #     t1.transform.translation.y,
            #     t1.transform.translation.z,
            #     t1.transform.rotation.x,
            #     t1.transform.rotation.y,
            #     t1.transform.rotation.z,
            #     t1.transform.rotation.w
            # ]
            self.cam_tag2 = [
                t2.transform.translation.x,
                t2.transform.translation.y,
                t2.transform.translation.z,
            ]
            self.tag1_tag2 = [
                t3.transform.translation.x,
                t3.transform.translation.y,
                t3.transform.translation.z,
                t3.transform.rotation.x,
                t3.transform.rotation.y,
                t3.transform.rotation.z,
                t3.transform.rotation.w
            ]
        except TransformException as ex:
            self.get_logger().info(f"Could not transform {ex}")

        if self.cam_tag1 and self.cam_tag2 is not None:
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
            orx = self.tag1_tag2[3]
            ory = self.tag1_tag2[4]
            orz = self.tag1_tag2[5]
            orw = self.tag1_tag2[6]
            self.get_logger().info(f"T2: x={x2}, y={y2}, z={z2}")
            self.odom_pub(x2, y2, z2, orx, ory, orz, orw)

            # self.get_logger().info(f"diff: x={x-x1}, y={y-y1}, z={z-z1}")
            # self.get_logger().info(f"dist: {((x-x1)**2+(y-y1)**2+(z-z1)**2)**0.5}")
            # self.get_logger().info(tcg)
            # self.get_logger().info(tco)


def main(args=None):
    rclpy.init(args=args)
    node = tag_locations()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
