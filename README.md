# cloud_3dpc_extractor
ROS Package containing a cloud-based dynamically scalable stereo vision platform based on stereo_image_proc.

## Overview
The aim of this ROS Package is to provide a 3D Point Cloud (3DPC) extraction platform using Cloud Computing's main 
features, especially that of dynamic scalability. This implies that, should the user require faster stereo image 
processing times, then it is possible to launch more 3D Point Cloud Extractors at runtime to satisfy the demand. On the other
side, if the user required less computing power, then it is possible to shut down the extractors at runtime without
having to restart the platform. 

This platform uses the stereo_image_proc library (http://wiki.ros.org/stereo_image_proc) for the 3D Point Cloud Extraction,
and consists of two nodes: the front-end buffer (stereo_cam_buffer) and the 3DPC Extractors (extractor_node). In order
to exploit the parallelism, a pipeline is created
