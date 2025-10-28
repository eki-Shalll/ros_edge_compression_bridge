# CompressedJpegXL.msg
std_msgs/Header header

# 原始图像的尺寸，解压缩时需要
uint32 height
uint32 width

# 像素格式，例如 "rgb8", "bgr8", "mono8"
string encoding

# 存放JXL编码后的字节流
uint8[] data
