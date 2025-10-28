#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from custom_compression_msgs.msg import CompressedJpegXL 
from cv_bridge import CvBridge, CvBridgeError
import imagecodecs
import numpy as np

class FinalTestCompressor:
    def __init__(self):
        rospy.loginfo("Starting FINAL TEST Compressor Node...")
        self.bridge = CvBridge()
        
        self.left_pub = rospy.Publisher('/left_camera/image/jxl_compressed', CompressedJpegXL, queue_size=10, latch=True)
        self.right_pub = rospy.Publisher('/right_camera/image/jxl_compressed', CompressedJpegXL, queue_size=10, latch=True)
        
        rospy.Subscriber('/left_camera/image', Image, self.left_callback, queue_size=2, buff_size=2**24)
        rospy.Subscriber('/right_camera/image', Image, self.right_callback, queue_size=2, buff_size=2**24)
        rospy.loginfo("FINAL TEST Compressor Node is ready.")

    def final_test_publish(self, ros_image_msg, publisher):
        rospy.logwarn(">>> FINAL TEST callback triggered for %s", publisher.name)
        
        if not ros_image_msg.data:
            rospy.logerr("!!! Received an empty image message. Skipping.")
            return

        try:
            # --- 步骤 1: 测试 cv_bridge ---
            rospy.loginfo("Step 1: Converting with cv_bridge...")
            # 你的原始图像编码是 rgb8，我们最好直接用它，而不是强制转成 bgr8
            # passthrough 会尽量保持原始编码
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='passthrough')
            rospy.loginfo(" -> cv_bridge SUCCESS. Shape: %s, Dtype: %s", cv_image.shape, cv_image.dtype)

            # --- 步骤 2: 测试 imagecodecs ---
            rospy.loginfo("Step 2: Encoding with imagecodecs...")
            # 确保输入数组的内存是连续的，这可以避免一些底层库的bug
            cv_image_contiguous = np.ascontiguousarray(cv_image)
            jxl_encoded_data = imagecodecs.jpegxl_encode(cv_image_contiguous, level=90)
            
            if not jxl_encoded_data:
                rospy.logerr("!!! imagecodecs.jpegxl_encode returned an empty result!")
                return
            rospy.loginfo(" -> imagecodecs SUCCESS. Compressed size: %d bytes", len(jxl_encoded_data))
            
            # --- 步骤 3: 准备并发布消息 ---
            jxl_msg = CompressedJpegXL()
            jxl_msg.header = ros_image_msg.header
            jxl_msg.height = ros_image_msg.height
            jxl_msg.width = ros_image_msg.width
            jxl_msg.encoding = ros_image_msg.encoding # 使用原始的编码 'rgb8'
            jxl_msg.data = jxl_encoded_data
            
            rospy.loginfo("Step 3: Publishing REAL compressed message...")
            publisher.publish(jxl_msg)
            rospy.loginfo(" -> REAL message published successfully for %s", publisher.name)

        except CvBridgeError as e:
            rospy.logerr("!!! FINAL TEST FAILED at cv_bridge step: %s", str(e))
        except Exception as e:
            rospy.logerr("!!! FINAL TEST FAILED at imagecodecs or other step: %s", str(e))

    def left_callback(self, ros_image_msg):
        self.final_test_publish(ros_image_msg, self.left_pub)

    def right_callback(self, ros_image_msg):
        self.final_test_publish(ros_image_msg, self.right_pub)

if __name__ == '__main__':
    rospy.init_node('jxl_compressor_node')
    try:
        compressor = FinalTestCompressor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass