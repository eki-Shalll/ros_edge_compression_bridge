#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from custom_compression_msgs.msg import CompressedJpegXL
from cv_bridge import CvBridge
import imagecodecs
import numpy as np
import sys # 导入sys模块来检查Python版本

class JpegXLCompressor:
    def __init__(self):
        # 打印正在使用的Python版本，用于诊断
        rospy.loginfo("Running with Python version: %s", sys.version)

        self.bridge = CvBridge()
        self.jxl_pub = rospy.Publisher('/camera/image/jxl_compressed', CompressedJpegXL, queue_size=1)
        
        # 关键修改：增加了 buff_size 参数，这对于大消息很重要
        self.image_sub = rospy.Subscriber(
            '/camera/image_raw', 
            Image, 
            self.image_callback, 
            queue_size=1, 
            buff_size=2**24 # 16MB buffer
        )
        
        rospy.loginfo("JXL Compressor Node: Ready to compress images. Waiting for messages...")

    def image_callback(self, ros_image_msg):
        # =================================================================
        # 诊断点 1: 在函数入口处添加一个明确的打印，确认回调被触发
        # =================================================================
        rospy.logwarn(">>> image_callback triggered for frame_id: %s", ros_image_msg.header.frame_id)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='passthrough')
            
            original_size = len(ros_image_msg.data)
            
            jxl_encoded_data = imagecodecs.jpegxl_encode(cv_image, level=90)
            
            compressed_size = len(jxl_encoded_data)

            if original_size > 0:
                compression_ratio = float(original_size) / compressed_size
                rospy.loginfo(
                    "%s - Original: %d, Compressed: %d, Ratio: %.1f:1",
                    rospy.get_name(),
                    original_size,
                    compressed_size,
                    compression_ratio
                )

            jxl_msg = CompressedJpegXL()
            jxl_msg.header = ros_image_msg.header
            jxl_msg.height = ros_image_msg.height
            jxl_msg.width = ros_image_msg.width
            jxl_msg.encoding = ros_image_msg.encoding
            jxl_msg.data = jxl_encoded_data
            
            self.jxl_pub.publish(jxl_msg)

        except Exception as e:
            # =================================================================
            # 诊断点 2: 在except块中打印捕获到的具体错误信息
            # =================================================================
            rospy.logerr("!!! An error occurred in image_callback: %s", str(e))

# 主函数部分保持不变
if __name__ == '__main__':
    rospy.init_node('jxl_compressor_node')
    try:
        compressor = JpegXLCompressor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass