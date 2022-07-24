import pyrealsense2 as rs
import time
import numpy as np
import matplotlib.pyplot as plt

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
profile = pipeline.start(config)

time.sleep(1)
frames = pipeline.wait_for_frames()

align_to_color = rs.stream.color;
align = rs.align(align_to_color)
aligned_frames = align.process(frames)
aligned_depth_frame = aligned_frames.get_depth_frame()
intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
print(intrinsics)
# depth_stream = profile.get_stream(rs.stream.depth)
# depth_stream = depth_stream.as('video_stream_profile')
# intrinsics1 = depth_stream.get_intrinsics()
# print("intrinsics: ", intrinsics1)

depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
    # Convert images to numpy arrays
depth_image = np.asanyarray(depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())
fig, axes = plt.subplots(1, 2)
for ax, im in zip(axes, [color_image, depth_image]):
    ax.imshow(im)
    ax.axis('off')
plt.show()
pipeline.stop()
