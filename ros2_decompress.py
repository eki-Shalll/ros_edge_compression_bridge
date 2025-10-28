# (ROS2 Python Node)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_compression_msgs_ros2.msg import CompressedJpegXL # ROS2 版本的自定义消息
from cv_bridge import CvBridge
import imagecodecs
import numpy as np

class JpegXLDecompressor(Node):
    def __init__(self):
        super().__init__('jxl_decompressor_node')
        self.bridge = CvBridge()

        # Publisher for the decompressed raw image
        self.image_pub = self.create_publisher(Image, '/camera/image_raw/decompressed', 10)
        
        # Subscriber for our custom JXL message
        self.create_subscription(
            CompressedJpegXL,
            '/camera/image_raw/jxl', # This will be the topic name after bridging
            self.jxl_callback,
            10
        )
        self.get_logger().info('JPEG XL Decompressor Node is running...')

    def jxl_callback(self, jxl_msg):
        try:
            # 1. Decode JXL bytes to NumPy array
            #    The 'out' parameter can be used for pre-allocating memory.
            decoded_image = imagecodecs.jpegxl_decode(jxl_msg.data)
        except Exception as e:
            self.get_logger().error(f"JPEG XL Decoding Error: {e}")
            return

        # Sanity check if decoded image has the expected shape
        expected_shape = (jxl_msg.height, jxl_msg.width)
        if decoded_image.ndim == 3:
            expected_shape += (decoded_image.shape[2],) # Add channel if it exists
        
        if decoded_image.shape != expected_shape:
            self.get_logger().warn(f"Decoded shape {decoded_image.shape} differs from expected {expected_shape}")
            # Potentially reshape if it's a simple flatten/unflatten issue
            # decoded_image = decoded_image.reshape(expected_shape)

        try:
            # 2. Convert NumPy array back to ROS Image message
            #    We use the encoding stored in our custom message
            ros_image = self.bridge.cv2_to_imgmsg(decoded_image, encoding=jxl_msg.encoding)
            ros_image.header = jxl_msg.header # Restore header
            
            # 3. Publish the standard Image message
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    decompressor = JpegXLDecompressor()
    rclpy.spin(decompressor)
    decompressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
