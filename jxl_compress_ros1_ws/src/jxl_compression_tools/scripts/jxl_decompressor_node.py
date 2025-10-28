#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from custom_compression_msgs.msg import CompressedJpegXL 
from cv_bridge import CvBridge
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

class JpegXLToJpegNode:
    def __init__(self):
        rospy.init_node('jxl_to_jpeg_node')
        rospy.loginfo("JXL to JPEG Transcoder Node starting...")
        self.bridge = CvBridge()
        
        self.left_pub = rospy.Publisher('/left_camera/image/decompressed/compressed', CompressedImage, queue_size=2)
        self.right_pub = rospy.Publisher('/right_camera/image/decompressed/compressed', CompressedImage, queue_size=2)
        
        rospy.Subscriber('/left_camera/image/jxl_compressed', CompressedJpegXL, self.left_callback, queue_size=2, buff_size=2**24)
        rospy.Subscriber('/right_camera/image/jxl_compressed', CompressedJpegXL, self.right_callback, queue_size=2, buff_size=2**24)
        rospy.loginfo("JXL to JPEG Transcoder Node is ready.")

    def transcode_image(self, jxl_msg, publisher, camera_side):
        if not jxl_msg.data:
            rospy.logerr("Received an empty JXL message on %s side. Skipping.", camera_side)
            return

        try:
            decoded_image = imagecodecs.jpegxl_decode(jxl_msg.data)
            jpeg_encoded_data = imagecodecs.jpeg_encode(decoded_image, level=85)
            
            final_jpeg_size = len(jpeg_encoded_data)
            # 使用 format_size 函数转换大小单位
            rospy.loginfo(
                "[%s] Final JPEG size: %s",
                camera_side.upper(), 
                format_size(final_jpeg_size)
            )

            jpeg_msg = CompressedImage()
            jpeg_msg.header = jxl_msg.header
            jpeg_msg.format = "jpeg"
            jpeg_msg.data = jpeg_encoded_data.tobytes() if hasattr(jpeg_encoded_data, 'tobytes') else jpeg_encoded_data
            
            publisher.publish(jpeg_msg)

        except Exception as e:
            rospy.logerr("An error occurred during transcoding on %s side: %s", camera_side, str(e))

    def left_callback(self, jxl_msg):
        self.transcode_image(jxl_msg, self.left_pub, "left")

    def right_callback(self, jxl_msg):
        self.transcode_image(jxl_msg, self.right_pub, "right")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = JpegXLToJpegNode()
        node.run()
    except rospy.ROSInterruptException:
        pass