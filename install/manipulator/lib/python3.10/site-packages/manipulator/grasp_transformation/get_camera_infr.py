import pyrealsense2 as rs

# 创建管道（pipeline）
pipeline = rs.pipeline()
# 创建配置（config）
config = rs.config()
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

# 启动管道
pipeline.start(config)

# 获取一帧数据
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()

# 获取 RGB 相机内参
color_intr = color_frame.profile.as_video_stream_profile().get_intrinsics()
# 获取 深度相机内参
depth_intr = depth_frame.profile.as_video_stream_profile().get_intrinsics()

# 输出 RGB 和 深度相机的内参
print("RGB 相机内参：")
print(f"ppx (cx) = {color_intr.ppx}, ppy (cy) = {color_intr.ppy}")
print(f"fx = {color_intr.fx}, fy = {color_intr.fy}")
print(f"宽度 = {color_intr.width}, 高度 = {color_intr.height}")
print(f"畸变参数 = {color_intr.coeffs}")

print("\n深度相机内参：")
print(f"ppx (cx) = {depth_intr.ppx}, ppy (cy) = {depth_intr.ppy}")
print(f"fx = {depth_intr.fx}, fy = {depth_intr.fy}")
print(f"宽度 = {depth_intr.width}, 高度 = {depth_intr.height}")
print(f"畸变参数 = {depth_intr.coeffs}")

# 停止管道
pipeline.stop()
