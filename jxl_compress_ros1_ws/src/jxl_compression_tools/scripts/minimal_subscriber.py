#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def minimal_callback(msg):
    """这是一个最简单的回调函数，只打印一句话。"""
    rospy.logwarn(">>> Minimal callback triggered! seq: %d", msg.header.seq)

def main():
    """主函数"""
    rospy.init_node('minimal_subscriber_node')
    
    # 直接订阅 /camera/image_raw，我们会用 launch 文件重映射它
    rospy.Subscriber('/camera/image_raw', Image, minimal_callback)
    
    rospy.loginfo("Minimal subscriber is running. Waiting for messages...")
    
    rospy.spin()

if __name__ == '__main__':
    main()