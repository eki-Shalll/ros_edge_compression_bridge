#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from custom_compression_msgs_ros1.msg import CompressedJpegXL  # 导入你的自定义消息
from cv_bridge import CvBridge
import imagecodecs  # 使用 imagecodecs 库
import numpy as np

class JpegXLCompressor:
    def __init__(self):
        rospy.init_node('jxl_compressor_node')
        self.bridge = CvBridge()

        # Publisher for our custom JXL message
        self.jxl_pub = rospy.Publisher('/camera/image_raw/jxl', CompressedJpegXL, queue_size=1)
        
        # Subscriber for the raw image
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1, buff_size=2**24)
        
        rospy.loginfo("JPEG XL Compressor Node is running...")

    def image_callback(self, ros_image):
        try:
            # 1. ROS Image -> OpenCV/NumPy array
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        try:
            # 2. Encode NumPy array to JXL bytes
            #    - 'jxl' is the codec name
            #    - level=90 sets the quality/effort. For JXL, this is often a "distance" parameter.
            #      You need to experiment with what works best (e.g., lossless=False, distance=1.0).
            #      A simple quality parameter might also be available.
            jxl_encoded_data = imagecodecs.jpegxl_encode(cv_image, level=90)
        except Exception as e:
            rospy.logerr(f"JPEG XL Encoding Error: {e}")
            return
            
        # 3. Create and populate a custom message
        jxl_msg = CompressedJpegXL()
        jxl_msg.header = ros_image.header  # Preserve timestamp and frame_id
        jxl_msg.height = ros_image.height
        jxl_msg.width = ros_image.width
        jxl_msg.encoding = ros_image.encoding
        jxl_msg.data = jxl_encoded_data

        # 4. Publish the custom message
        self.jxl_pub.publish(jxl_msg)

if __name__ == '__main__':
    try:
        compressor = JpegXLCompressor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
