import pyrealsense2 as rs

context = rs.context()
devices = context.query_devices()

print(len(rs.context().query_devices()))
print(rs.context().query_devices()[0].get_info(rs.camera_info.serial_number))
for i, device in enumerate(devices):
    print(f"相机 {i}: 序列号 = {device.get_info(rs.camera_info.serial_number)}")

# 相机 0: 序列号 = 135122074163
# 相机 1: 序列号 = 135122077457