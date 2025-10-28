import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from custom_compression_msgs.msg import CompressedJpegXL
from cv_bridge import CvBridge, CvBridgeError
import imagecodecs
import numpy as np

def format_size(size_in_bytes):
    """
    将字节大小转换为人类可读的格式 (KB, MB)。
    """
    if size_in_bytes < 1024:
        return f"{size_in_bytes} Bytes"
    elif size_in_bytes < 1024 * 1024:
        return f"{size_in_bytes / 1024.0:.2f} KB"
    else:
        return f"{size_in_bytes / (1024.0 * 1024.0):.2f} MB"

class JpegXLToJpegNode(Node):
    def __init__(self):
        super().__init__('jxl_transcoder_and_decompressor_node')
        self.get_logger().info("JXL to JPEG/Raw Transcoder Node (ROS2) starting...")
        
        self.bridge = CvBridge()

        visual_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # --- 发布者 ---
        
        # KEY CHANGE 2: 使用 image_transport 来创建发布者
        # 注意看话题名的变化：我们现在提供的是“基础话题名”，
        # image_transport 会自动创建 /left_camera/image/final/compressed 话题。


        self.left_jpeg_pub = self.create_publisher(
            CompressedImage, 
            '/left_camera/image/final_jpeg', 
            visual_qos_profile
        )
        self.right_jpeg_pub = self.create_publisher(
            CompressedImage, 
            '/right_camera/image/final_jpeg', 
            visual_qos_profile
        )

        # 发布原始图像的这部分可以保留，用于调试，它本身是正确的
        self.left_raw_pub = self.create_publisher(
            Image, 
            '/left_camera/image/final_raw', 
            visual_qos_profile
        )
        self.right_raw_pub = self.create_publisher(
            Image, 
            '/right_camera/image/final_raw', 
            visual_qos_profile
        )

        # --- 订阅者 ---
        self.create_subscription(
            CompressedJpegXL, 
            '/left_camera/image/jxl_compressed', 
            self.left_callback, 
            visual_qos_profile
        )
        self.create_subscription(
            CompressedJpegXL, 
            '/right_camera/image/jxl_compressed', 
            self.right_callback, 
            visual_qos_profile
        )
        
        self.get_logger().info("Node is ready. Waiting for JXL messages...")

    def process_and_publish(self, jxl_msg, jpeg_publisher, raw_publisher, camera_side):
        try:
            decoded_from_jxl = imagecodecs.jpegxl_decode(jxl_msg.data)
            jpeg_encoded_data = imagecodecs.jpeg_encode(decoded_from_jxl, level=85)
            
            # --- 发布 CompressedImage (JPEG) ---
            jpeg_msg = CompressedImage()
            jpeg_msg.header = jxl_msg.header
            jpeg_msg.format = "jpeg"
            jpeg_msg.data = jpeg_encoded_data# .tobytes() 是正确的
            
            # KEY CHANGE 3: 发布方法不变，直接使用 image_transport 的发布者
            jpeg_publisher.publish(jpeg_msg)
            
            # --- 发布 Raw Image (用于对比) ---
            raw_image_msg = self.bridge.cv2_to_imgmsg(decoded_from_jxl, encoding=jxl_msg.encoding)
            raw_image_msg.header = jxl_msg.header
            raw_publisher.publish(raw_image_msg)

            self.get_logger().info(
                f"[{camera_side.upper()}] Processed frame. Final JPEG size: {format_size(len(jpeg_encoded_data))}"
            )

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error on {camera_side} side: {e}")
        except Exception as e:
            self.get_logger().error(f"An error occurred during processing on {camera_side} side: {str(e)}")

    def left_callback(self, jxl_msg): 
        self.process_and_publish(jxl_msg, self.left_jpeg_pub, self.left_raw_pub, "left")

    def right_callback(self, jxl_msg): 
        self.process_and_publish(jxl_msg, self.right_jpeg_pub, self.right_raw_pub, "right")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = JpegXLToJpegNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()