#!/usr/bin/env python

import bondpy.bondpy as bondpy
import rospy
import roslib
import roslaunch
import uuid
import time
import rosparam

roslib.load_manifest('cloud_3dpc_extractor_msgs')

import cloud_3dpc_extractor_msgs
import cloud_3dpc_extractor_msgs.msg._NewProcNode as NewProcNode

# Callback

def restore_callback(event):
    nodeInfo = NewProcNode.NewProcNode()
    nodeInfo.id = id
    nodeInfo.topic = id + "/bond"
    pub.publish(nodeInfo)

# Get parameters

id = "id" + str(uuid.uuid4()).replace("-","")

rospy.init_node(id + "node")

delay = rospy.get_param('~delay', 10)
prefilter_size = rospy.get_param('~prefilter_size', '23')
prefilter_cap  = rospy.get_param('~prefilter_cap', '33')
correlation_window_size = rospy.get_param('~correlation_window_size', '41')
min_disparity = rospy.get_param('~min_disparity', '44')
disparity_range = rospy.get_param('~disparity_range', '64')
uniqueness_ratio = rospy.get_param('~uniqueness_ratio', '15.0')
texture_threshold = rospy.get_param('~texture_threshold', '10')
speckle_size = rospy.get_param('~speckle_size', '356')
speckle_range = rospy.get_param('~speckle_range', '7')
measure_time = rospy.get_param('~measure_time', 'true')
restore_time = rospy.get_param('~restore_time', 5)
heartbeat_period = rospy.get_param('~heartbeat_period', 5.0)
heartbeat_timeout = rospy.get_param('~heartbeat_timeout', 10.0)

#output_filename_comm = rospy.get_param('~output_filename_comm', 'time_sip_')
#output_filename_proc = rospy.get_param('~output_filename_proc', 'time_sip_')

rospy.loginfo("\n"
    "=================================================================\n"
    "Started Stereo Image Proc Wrapper with the following paramterers:\n"
    "=================================================================\n"
    "ID                      : " + id                       + "\n"
    "Delay (s)               : " + str(delay)               + "\n"
    "Prefilter size          : " + prefilter_size           + "\n"
    "Prefilter cap           : " + prefilter_cap            + "\n"
    "Correlation Window Size : " + correlation_window_size  + "\n"
    "Min. Disparity          : " + min_disparity            + "\n"
    "Disparity Range         : " + disparity_range          + "\n"
    "Uniqueness Ratio        : " + uniqueness_ratio         + "\n"
    "Texture Threshold       : " + texture_threshold        + "\n"
    "Speckle Size            : " + speckle_size             + "\n"
    "Speckle Range           : " + speckle_range            + "\n"
    "Measure Time            : " + measure_time             + "\n"
    "Bond Restore Time (s)   : " + str(restore_time)        + "\n"
    "Heartbeat Period (s)    : " + str(heartbeat_period)    + "\n"
    "Heartbeat Timeout (s)   : " + str(heartbeat_timeout)   + "\n"
#    "Output Filename (Proc)  : " + output_filename_proc     + "\n"
#    "Output Filename (Comm)  : " + output_filename_comm     + "\n"
    "=================================================================\n")

time.sleep(delay)

pub = rospy.Publisher("/new_proc_node", NewProcNode.NewProcNode)

node = roslaunch.core.Node(package="stereo_image_proc", node_type="stereo_image_proc",
                           name="stereo_proc" + id +  "_node", namespace=id , machine_name="localhost", args="",
respawn=True, remap_args=[("/" + id + "/points2", "/points2")], env_args=None, output=None,
cwd=None, launch_prefix=None, required=False)

roslaunchInstance = roslaunch.scriptapi.ROSLaunch()
roslaunchInstance.start()

rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "prefilter_size", prefilter_size)
rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "prefilter_cap", prefilter_cap)
rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "correlation_window_size", correlation_window_size)
rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "min_disparity", min_disparity)
rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "disparity_range", disparity_range)
rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "uniqueness_ratio", uniqueness_ratio)
rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "texture_threshold", texture_threshold)
rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "speckle_size", speckle_size)
rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "speckle_range", speckle_range)
rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "measure_time", measure_time)
#rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "output_filename_proc", output_filename_proc)
#rosparam.set_param("/" + id + "/" + "stereo_proc" + id +  "_node" + "/" + "output_filename_comm", output_filename_comm)

roslaunchInstance.launch(node)

nodeInfo = NewProcNode.NewProcNode()
nodeInfo.id = id
nodeInfo.topic = id + "/bond"

pub.publish(nodeInfo)

bond = bondpy.Bond(id + "/bond", id)

bond.set_heartbeat_period(heartbeat_period)
bond.set_heartbeat_timeout(heartbeat_timeout)

rospy.Timer(rospy.Duration(restore_time), restore_callback)

bond.start()
rospy.spin()
