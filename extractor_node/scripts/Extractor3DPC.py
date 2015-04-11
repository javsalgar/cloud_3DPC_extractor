# -*- coding: utf-8 -*-
"""
@package Extractor3DPC

  Cloud-based 3DPC Extraction platform Extractor

  @author Javier J. Salmeron-Garcia (jsalmeron2@us.es)

  University of Seville

  This file is part of Cloud-based 3DPC Extraction platform

  Cloud-based 3DPC Extraction platform is free software:
  you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Cloud-based 3DPC Extraction platform is distributed in the hope
  that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Cloud-based 3DPC Extraction platform.  If not, see <http://www.gnu.org/licenses/>.
"""

import bondpy.bondpy as bondpy
import time
import rosparam
import roslib
roslib.load_manifest('cloud_3dpc_extractor_msgs')
import cloud_3dpc_extractor_msgs
import cloud_3dpc_extractor_msgs.msg._New3DPCExtractor as New3DPCExtractor
import rospy
import roslaunch


class Extractor3DPC:
    """ 3DPC Extractor, wrapping the stereo_image_proc library"""

    ## Stereo image proc attributes
    delay = 10
    prefilter_size = 23
    prefilter_cap = 33
    correlation_window_size = 41
    min_disparity = 44
    disparity_range = 64
    uniqueness_ratio = 15.0
    texture_threshold = 10
    speckle_size = 356
    speckle_range = 7
    restore_time = 5

    ## Bond library attributes
    heartbeat_period = 5.0
    heartbeat_timeout = 10.0
    new_3dpc_msg_queue_size = 10

    ## Extractor id
    id = "extractor_3dpc_"

    ## Node information publisher
    node_publisher = []

    ## Roslaunch instance
    roslaunch_instance = []

    def __init__(self, name):
        self.id = self.id + name

    ## Launch the stereo_image_proc node and inform the front-end (stereo_cam_buffer) its availability
    def start(self):
        self.delay = rospy.get_param('~delay', 10)
        self.prefilter_size = rospy.get_param('~prefilter_size', '23')
        self.prefilter_cap  = rospy.get_param('~prefilter_cap', '33')
        self.correlation_window_size = rospy.get_param('~correlation_window_size', '41')
        self.min_disparity = rospy.get_param('~min_disparity', '44')
        self.disparity_range = rospy.get_param('~disparity_range', '64')
        self.uniqueness_ratio = rospy.get_param('~uniqueness_ratio', '15.0')
        self.texture_threshold = rospy.get_param('~texture_threshold', '10')
        self.speckle_size = rospy.get_param('~speckle_size', '356')
        self.speckle_range = rospy.get_param('~speckle_range', '7')
        self.restore_time = rospy.get_param('~restore_time', 5)
        self.heartbeat_period = rospy.get_param('~heartbeat_period', 5.0)
        self.heartbeat_timeout = rospy.get_param('~heartbeat_timeout', 10.0)
        rospy.loginfo("\n"
            "=================================================================\n"
            "Started 3DPC Extractor with the following paramterers:\n"
            "=================================================================\n"
            "ID                      : " + self.id                       + "\n"
            "Delay (s)               : " + str(self.delay)               + "\n"
            "Prefilter size          : " + self.prefilter_size           + "\n"
            "Prefilter cap           : " + self.prefilter_cap            + "\n"
            "Correlation Window Size : " + self.correlation_window_size  + "\n"
            "Min. Disparity          : " + self.min_disparity            + "\n"
            "Disparity Range         : " + self.disparity_range          + "\n"
            "Uniqueness Ratio        : " + self.uniqueness_ratio         + "\n"
            "Texture Threshold       : " + self.texture_threshold        + "\n"
            "Speckle Size            : " + self.speckle_size             + "\n"
            "Speckle Range           : " + self.speckle_range            + "\n"
            "Bond Restore Time (s)   : " + str(self.restore_time)        + "\n"
            "Heartbeat Period (s)    : " + str(self.heartbeat_period)    + "\n"
            "Heartbeat Timeout (s)   : " + str(self.heartbeat_timeout)   + "\n"
            "New 3DPC Msg Queue size : " + str(self.new_3dpc_msg_queue_size) + "\n"
            "=================================================================\n")

        time.sleep(self.delay)

        self.node_publisher = rospy.Publisher("/new_3dpc_extractor", New3DPCExtractor.New3DPCExtractor,
                                                queue_size=self.new_3dpc_msg_queue_size)

        node = roslaunch.core.Node(package="stereo_image_proc", node_type="stereo_image_proc",
                                   name=self.id, machine_name="localhost", args="",
                                   respawn=True, remap_args=[("/left/image_color", self.id + "/left/image_color"),
                                                             ("/left/image_mono", self.id + "/left/image_mono"),
                                                             ("/left/image_rect", self.id + "/left/image_rect"),
                                                             ("/left/image_rect_color", self.id + "/left/image_rect_color"),
                                                             ("/right/image_color", self.id + "/right/image_color"),
                                                             ("/right/image_mono", self.id + "/right/image_mono"),
                                                             ("/right/image_rect", self.id + "/right/image_rect"),
                                                             ("/right/image_rect_color", self.id + "/right/image_rect_color")],
                                   env_args=None, output=None,
                                   cwd=None, launch_prefix=None, required=False)

        self.roslaunch_instance = roslaunch.scriptapi.ROSLaunch()
        self.roslaunch_instance.start()

        param_path = "/" + self.id + "/"

        rosparam.set_param(param_path + "prefilter_size", self.prefilter_size)
        rosparam.set_param(param_path + "prefilter_cap", self.prefilter_cap)
        rosparam.set_param(param_path + "correlation_window_size", self.correlation_window_size)
        rosparam.set_param(param_path + "min_disparity", self.min_disparity)
        rosparam.set_param(param_path + "disparity_range", self.disparity_range)
        rosparam.set_param(param_path + "uniqueness_ratio", self.uniqueness_ratio)
        rosparam.set_param(param_path + "texture_threshold", self.texture_threshold)
        rosparam.set_param(param_path + "speckle_size", self.speckle_size)
        rosparam.set_param(param_path + "speckle_range", self.speckle_range)

        self.roslaunch_instance.launch(node)

        new_extractor_msg = New3DPCExtractor.New3DPCExtractor()
        new_extractor_msg.id = self.id
        new_extractor_msg.topic = self.id + "/bond"

        self.node_publisher.publish(new_extractor_msg)

        bond = bondpy.Bond(self.id + "/bond", self.id)

        bond.set_heartbeat_period(self.heartbeat_period)
        bond.set_heartbeat_timeout(self.heartbeat_timeout)

        rospy.Timer(rospy.Duration(self.restore_time), self.restore_bond)

        bond.start()

    ## Attempt to recreate the link for the buffer, in case it is lost
    def restore_bond(self, event):
        new_extractor_msg = New3DPCExtractor.New3DPCExtractor()
        new_extractor_msg.id = self.id
        new_extractor_msg.topic = self.id + "/bond"
        self.node_publisher.publish(new_extractor_msg)
