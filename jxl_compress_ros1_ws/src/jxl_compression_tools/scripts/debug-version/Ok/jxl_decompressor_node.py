#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from custom_compression_msgs.msg import CompressedJpegXL 
from cv_bridge import CvBridge, CvBridgeError
import imagecodecs
import numpy as np

class JpegXLTranscoder:
    def __init__(self):
        rospy.loginfo("Starting JXL to JPEG Transcoder Node for stereo cameras...")
        self.bridge = CvBridge()
        
        self.left_pub = rospy.Publisher('/left_camera/image/decompressed/compressed', CompressedImage, queue_size=2)
        self.right_pub = rospy.Publisher('/right_camera/image/decompressed/compressed', CompressedImage, queue_size=2)
        
        rospy.Subscriber('/left_camera/image/jxl_compressed', CompressedJpegXL, self.left_callback, queue_size=2, buff_size=2**24)
        rospy.Subscriber('/right_camera/image/jxl_compressed', CompressedJpegXL, self.right_callback, queue_size=2, buff_size=2**24)
        rospy.loginfo("JXL to JPEG Transcoder Node is ready.")

    def transcode_and_publish(self, jxl_msg, publisher):
        rospy.logwarn(">>> Transcoder Callback for %s", publisher.name)
        
        if not jxl_msg.data:
            rospy.logerr("!!! Received an empty JXL message. Skipping.")
            return

        try:
            # --- 步骤 1: JXL 解码 ---
            decoded_image = imagecodecs.jpegxl_decode(jxl_msg.data)
            
            # --- 步骤 2: JPEG 编码 ---
            jpeg_encoded_data = imagecodecs.jpeg_encode(decoded_image, level=85)

            # ==================== 关键修正 ====================
            # 检查 jpeg_encoded_data 的类型
            # 根据不同的 Python 和库版本，它可能返回 bytes 或 numpy.ndarray
            # 我们需要统一处理，确保最终赋值给 jpeg_msg.data 的是 bytes 类型
            
            final_jpeg_bytes = None
            if isinstance(jpeg_encoded_data, np.ndarray):
                # 如果是 Numpy 数组，调用 .tobytes()
                final_jpeg_bytes = jpeg_encoded_data.tobytes()
            elif isinstance(jpeg_encoded_data, bytes):
                # 如果它本身就是 bytes，直接使用
                final_jpeg_bytes = jpeg_encoded_data
            else:
                rospy.logerr("!!! jpeg_encode returned an unknown type: %s", type(jpeg_encoded_data))
                return
            # =================================================

            # --- 步骤 3: 创建并发布 CompressedImage ---
            jpeg_msg = CompressedImage()
            jpeg_msg.header = jxl_msg.header
            jpeg_msg.format = "jpeg"
            jpeg_msg.data = final_jpeg_bytes # 使用我们处理好的 bytes
            
            publisher.publish(jpeg_msg)
            
            # --- 步骤 4: 打印统计信息 ---
            rospy.loginfo(
                "Transcode SUCCESS for %s", publisher.name
            )

        except Exception as e:
            rospy.logerr("!!! An unexpected error occurred in transcode_and_publish: %s", str(e))

    def left_callback(self, jxl_msg):
        self.transcode_and_publish(jxl_msg, self.left_pub)

    def right_callback(self, jxl_msg):
        self.transcode_and_publish(jxl_msg, self.right_pub)

if __name__ == '__main__':
    rospy.init_node('jxl_transcoder_node')
    try:
        transcoder = JpegXLTranscoder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass