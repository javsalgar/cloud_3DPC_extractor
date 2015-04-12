# cloud_3dpc_extractor
ROS Package containing a cloud-based dynamically scalable stereo vision platform based on stereo_image_proc.

## Overview
The aim of this ROS Package is to provide a 3D Point Cloud (3DPC) extraction platform using Cloud Computing's main 
features, especially that of dynamic scalability. This implies that, should the user require faster stereo image 
processing times, then it is possible to launch more 3D Point Cloud Extractors at runtime to satisfy the demand. On the other
side, if the user required less computing power, then it is possible to shut down the extractors at runtime without
having to restart the platform. 

This platform uses the stereo_image_proc library (http://wiki.ros.org/stereo_image_proc) for the 3D Point Cloud Extraction,
and consists of two nodes: the front-end buffer (`stereo_cam_buffer`) and the 3DPC Extractors (extractor_node). In order
to exploit the parallelism, a pipeline is created. The front-end node receives the stereo stream and scatters it to the 3DPC Extractors in a pipeline fashion. 

## Quick Start
To begin with, you need to have a calibrated stereo camera running with (at least) the following topics:
```
[/NAMESPACE]/left/image_raw
[/NAMESPACE]/right/image_raw
[/NAMESPACE]/left/camera_info
[/NAMESPACE]/right/camera_info
```
A minimal architecture would consist of one `stereo_cam_buffer` instance and one `extractor_node` instance. To launch them (in the same machine) use the following commands:
```
$ rosrun stereo_cam_buffer buffer
$ rosrun extractor_node extractor_node.py
```
To check that the system is working, you can check if there is activity in the `/points2` topic, where the generated 3DPCs are available:

`$ rostopic hz /points2`

You can also use visualization tools like `rviz`. Should you require more computing power, you can launch more `extractor_node` instances at runtime:

`$ rosrun extractor_node extractor_node.py`

## Cloud execution
If you have a cloud (or a cluster) available, you can execute the architecture using multiple nodes. In order to do so, one of the nodes must be the master. The following example will suppose four machines: `camera.example.com` ,`frontend.example.com`, `extractor1.example.com` and `extractor2.example.com`. camera.example.com will have the stereo camera running, and will be the ROS Master. `frontend.example.com` will have a `stereo_cam_buffer` instance, extractor1.example.com will have one `extractor_node` instance and `extractor2.example.com` will have two `extractor_node` instances. In order to do so, the following commands are necessary:

```
user@camera:~$ roscore &
user@camera:~$ <command to launch the camera node>

user@frontend:~$ ROS_MASTER_URI=http://camera.example.com:11311 rosrun stereo_cam_buffer buffer

user@extractor1:~$ ROS_MASTER_URI=http://camera.example.com:11311 rosrun extractor_node extractor_node.py

user@extractor2:~$ ROS_MASTER_URI=http://camera.example.com:11311 rosrun extractor_node extractor_node.py
```

This example assumes that you have a DNS server that can resolve all the domain names. Otherwise, you will need to
edit the /etc/hosts file in all the computers. 

## Nodes

### Stereo Cam Buffer (`stereo_cam_buffer`)
This node receives the stereo stream from the camera and scatters it in a round-robin fashion. In other words, it buffers and forwards the stereo frame pairs to have them processed by the 3DPC Extractors. It allows the use of either tcp or udp, together with different compression techniques that the image_transport package offers (raw, theora, compressed). 

#### Subscribed topics
##### Left Camera
* `left/image_raw` (sensor_msgs/Image)

 Image stream from the left camera
* `left/camera_info` (sensor_msgs/CameraInfo)

 Metadata from the left camera
#### Published topics


### 3DPC Extractor (`extractor_node`)

#### Published topics

#### Subscribed topics

## Docker images
