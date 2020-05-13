# -*- coding: utf-8 -*-
"""
@File    : post_processing.py
@Author  : forestjyli
"""
# 依次顺序应用这些过滤器时效果最佳。
# 在更长的范围内，它还有助于使用disparity_transform从深度表示转换为视差形式：

import numpy as np  # fundamental package for scientific computing 科学计算的基本软件包
import matplotlib.pyplot as plt  # 2D plotting library producing publication quality figures 2D绘图库产生出版物质量数据
import pyrealsense2 as rs  # Intel RealSense cross-platform open-source API 英特尔实感跨平台开源API

print("Environment Ready")

# 【Setup: 配置】
pipe = rs.pipeline()
cfg = rs.config()
# cfg.enable_device_from_file("stairs.bag")
cfg.enable_device('839112061668')
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(cfg)

# 【Skip 5 first frames to give the Auto-Exposure time to adjust 跳过前5帧以设置自动曝光时间】
for x in range(5):
    pipe.wait_for_frames()

frames = []
for x in range(10):
    frameset = pipe.wait_for_frames()
    frames.append(frameset.get_depth_frame())

# 【Cleanup: 清理：】
pipe.stop()
print("Frames Captured")

# 【Visualising the Data 可视化数据】
# 创建着色器(其实这个可以替代opencv的convertScaleAbs()和applyColorMap()函数了,但是是在多少米范围内map呢?)
colorizer = rs.colorizer()
# colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
# print(colorized_depth.shape)  # (480, 640, 3)
# cv2.imshow('win', colorized_depth)
# cv2.waitKey(0)

# 绘图不显示网格
plt.rcParams["axes.grid"] = False
# 图形尺寸,单位英尺
plt.rcParams['figure.figsize'] = [8, 4]
# plt.imshow(colorized_depth)

depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)

# 创建抽取过滤器
decimation = rs.decimation_filter()
# decimated_depth = decimation.process(depth_frame)
# print(type(decimation)) # <class 'pyrealsense2.pyrealsense2.decimation_filter'>
# print(type(decimated_depth))  # <class 'pyrealsense2.pyrealsense2.frame'>
# # 您可以通过滤波器幅度选项来控制抽取量（线性比例因子）。
# # 注意不断变化的图像分辨率
decimation.set_option(rs.option.filter_magnitude, 4)

# [2、空间过滤器]
# Spatial Filter
# Spatial Filter is a fast implementation of Domain-Transform Edge Preserving Smoothing
# 空间滤波器是域转换边缘保留平滑的快速实现
spatial = rs.spatial_filter()
# filtered_depth = spatial.process(depth_frame)
# We can emphesize the effect of the filter by cranking-up smooth_alpha and smooth_delta options:
# 我们可以通过增加smooth_alpha和smooth_delta选项来强调滤镜的效果：
spatial.set_option(rs.option.filter_magnitude, 5)
spatial.set_option(rs.option.filter_smooth_alpha, 1)
spatial.set_option(rs.option.filter_smooth_delta, 50)
# The filter also offers some basic spatial hole filling capabilities:
# 该过滤器还提供一些基本的空间孔填充功能：
spatial.set_option(rs.option.holes_fill, 3)

# Next, we need to "feed" the frames to the filter one by one:
# 接下来，我们需要将帧逐一“馈入”到过滤器：
temporal = rs.temporal_filter()

# 孔填充过滤器提供了附加的深度外推层：
hole_filling = rs.hole_filling_filter()

for x in range(10):
    frame = frames[x]
    frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = temporal.process(frame)
    frame = disparity_to_depth.process(frame)
    frame = hole_filling.process(frame)

    colorized_depth = np.asanyarray(colorizer.colorize(frame).get_data())
    plt.imshow(colorized_depth)
    plt.show()