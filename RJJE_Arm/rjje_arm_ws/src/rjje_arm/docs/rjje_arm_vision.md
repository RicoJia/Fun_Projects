# RJJE Arm Vision
## 3D Object Detection
1. Visualize point cloud
    1. Examples
        - [Real Sense Tracker](https://github.com/spkywood/realsense-tracker)
        - [Robomaster board detection](https://blog.csdn.net/weixin_39298885/article/details/120207053)
    2. We need image and depth map aligned. Since we have images coming from 2 cameras, we need to "align them" by finding reference points.
        - /camera/aligned_depth_to_color: aligned info
        - /camera/color - RGB info
        - /camera/depth - non-aligned depth information
    3. Also, we can utilize realsense2_camera package to output "ordered" point clouds to us. Therefore have ```ordered_pc:=true```
        - [doc](http://docs.ros.org/en/api/sensor_msgs/html/point__cloud2_8py_source.html#l00060)

2.  1. YOLO v5 + pick and place.
        - YOLO v5 docker, being able to run
        - get images -> yolo, test

