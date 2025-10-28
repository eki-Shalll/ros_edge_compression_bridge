#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from custom_compression_msgs.msg import CompressedJpegXL 
from cv_bridge import CvBridge
import imagecodecs

class JpegXLCompressor:
    def __init__(self):
        rospy.loginfo("Starting JXL Compressor Node for stereo cameras...")
        self.bridge = CvBridge()
        
        self.left_pub = rospy.Publisher('/left_camera/image/jxl_compressed', CompressedJpegXL, queue_size=2)
        self.right_pub = rospy.Publisher('/right_camera/image/jxl_compressed', CompressedJpegXL, queue_size=2)
        
        # ==================== 关键修正 ====================
        # 为两个订阅者都添加 buff_size 参数
        # ================================================
        rospy.Subscriber(
            '/left_camera/image', 
            Image, 
            self.left_callback, 
            queue_size=2, 
            buff_size=2**24 # 16MB buffer
        )
        rospy.Subscriber(
            '/right_camera/image', 
            Image, 
            self.right_callback, 
            queue_size=2, 
            buff_size=2**24 # 16MB buffer
        )
        rospy.loginfo("JXL Compressor Node is ready and waiting for images.")

    def compress_and_publish(self, ros_image_msg, publisher):
        rospy.logwarn(">>> Compressing frame from %s", ros_image_msg.header.frame_id)
        
        try:
            # 获取原始数据大小
            original_size = len(ros_image_msg.data)
            if original_size == 0: return # 如果是空图像则跳过

            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, "bgr8")
            jxl_encoded_data = imagecodecs.jpegxl_encode(cv_image, level=90)
            
            jxl_compressed_size = len(jxl_encoded_data)
            
            # ==================== 新增打印信息 ====================
            jxl_ratio = float(original_size) / jxl_compressed_size
            rospy.loginfo(
                "JXL Compression | Topic: %s | Original: %d B -> JXL: %d B | Ratio: %.1f:1",
                ros_image_msg.header.frame_id,
                original_size,
                jxl_compressed_size,
                jxl_ratio
            )
            # ====================================================

            jxl_msg = CompressedJpegXL()
            # ... 填充 jxl_msg ...
            publisher.publish(jxl_msg)
            
        except Exception as e:
            rospy.logerr("Compression Error on %s: %s", ros_image_msg.header.frame_id, str(e))

    def left_callback(self, ros_image_msg):
        self.compress_and_publish(ros_image_msg, self.left_pub)

    def right_callback(self, ros_image_msg):
        self.compress_and_publish(ros_image_msg, self.right_pub)

if __name__ == '__main__':
    rospy.init_node('jxl_compressor_node')
    try:
        compressor = JpegXLCompressor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
