#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from custom_compression_msgs.msg import CompressedJpegXL 
from cv_bridge import CvBridge, CvBridgeError
import imagecodecs
import numpy as np

def format_size(size_in_bytes):
    """
    Converts a size in bytes to a human-readable format (KB, MB).
    """
    if size_in_bytes < 1024:
        return f"{size_in_bytes} Bytes"
    elif size_in_bytes < 1024 * 1024:
        return f"{size_in_bytes / 1024.0:.2f} KB"
    else:
        return f"{size_in_bytes / (1024.0 * 1024.0):.2f} MB"

class JxlCompressorNode:
    def __init__(self):
        rospy.init_node('jxl_compressor_node')
        rospy.loginfo("JXL Compressor Node starting...")
        self.bridge = CvBridge()
        
        self.left_pub = rospy.Publisher('/left_camera/image/jxl_compressed', CompressedJpegXL, queue_size=2)
        self.right_pub = rospy.Publisher('/right_camera/image/jxl_compressed', CompressedJpegXL, queue_size=2)
        
        rospy.Subscriber('/left_camera/image', Image, self.left_callback, queue_size=2, buff_size=2**24)
        rospy.Subscriber('/right_camera/image', Image, self.right_callback, queue_size=2, buff_size=2**24)
        rospy.loginfo("JXL Compressor Node is ready.")

    def process_image(self, ros_image_msg, publisher, camera_side):
        if not ros_image_msg.data:
            rospy.logerr("Received an empty image message on %s side. Skipping.", camera_side)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='passthrough')
            jxl_encoded_data = imagecodecs.jpegxl_encode(np.ascontiguousarray(cv_image), level=90)
            
            raw_size = len(ros_image_msg.data)
            compressed_size = len(jxl_encoded_data)

            if raw_size > 0:
                compression_ratio = float(raw_size) / compressed_size
                # 使用 format_size 函数转换大小单位
                rospy.loginfo(
                    "[%s] Raw->JXL Ratio: %.2f : 1 (%s -> %s)",
                    camera_side.upper(), 
                    compression_ratio, 
                    format_size(raw_size), 
                    format_size(compressed_size)
                )
            
            jxl_msg = CompressedJpegXL()
            jxl_msg.header = ros_image_msg.header
            jxl_msg.header.frame_id = (
                ros_image_msg.header.frame_id 
                or f"{camera_side}_camera_frame"
            )
            jxl_msg.height = ros_image_msg.height
            jxl_msg.width = ros_image_msg.width
            jxl_msg.encoding = ros_image_msg.encoding
            jxl_msg.data = jxl_encoded_data
            
            publisher.publish(jxl_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error on %s side: %s", camera_side, str(e))
        except Exception as e:
            rospy.logerr("An unexpected error occurred on %s side: %s", camera_side, str(e))

    def left_callback(self, ros_image_msg):
        self.process_image(ros_image_msg, self.left_pub, "left")

    def right_callback(self, ros_image_msg):
        self.process_image(ros_image_msg, self.right_pub, "right")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = JxlCompressorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
