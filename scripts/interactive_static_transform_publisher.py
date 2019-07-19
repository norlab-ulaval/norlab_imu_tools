#!/usr/bin/env python

"""

The original node (licence below) modified by Vladimir Kubelka, kubelvla@gmail.com for the purpose of setting-up a static transform)
Licence: BSD
Laval University, NORLAB, 2019

////////////////////////////
////////////////////////////

Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import tf
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion



server = None
menu_handler = MenuHandler()
br = None

tf_position = None
tf_orientation = None
tf_parent_frame = "parent_frame"
tf_child_frame = "child_frame"
marker_position = None
marker_x_offset = 2.0     # For drawing the marker slightly offset from the child frame origin
tf_update_period = 0.01




def frameCallback( msg ):
    global br
    time = rospy.Time.now()
    br.sendTransform( (tf_position.x, tf_position.y, tf_position.z),
                      (tf_orientation.x, tf_orientation.y, tf_orientation.z, tf_orientation.w),
                      time, tf_child_frame, tf_parent_frame )

def processFeedback( feedback ):
    global tf_position, tf_orientation
    # s = "Feedback from marker '" + feedback.marker_name
    # s += "' / control '" + feedback.control_name + "'"
    #
    # mp = ""
    # if feedback.mouse_point_valid:
    #     mp = " at " + str(feedback.mouse_point.x)
    #     mp += ", " + str(feedback.mouse_point.y)
    #     mp += ", " + str(feedback.mouse_point.z)
    #     mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        pass

    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo("The equivalent static transform command:")
        message = 'rosrun tf static_transform_publisher {0} {1} {2} {3} {4} {5} {6} {7} {8} {9:d}'.format(
            tf_position.x,
            tf_position.y,
            tf_position.z,
            tf_orientation.x,
            tf_orientation.y,
            tf_orientation.z,
            tf_orientation.w,
            tf_parent_frame,
            tf_child_frame,
            int(round(tf_update_period * 1000)))  # in milliseconds

        rospy.loginfo(message)
        pass

    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        # rospy.loginfo( s + ": pose changed")
        new_tf_position = copy.deepcopy(feedback.pose.position)
        new_tf_position.x = new_tf_position.x - marker_x_offset  # The marker is offset for nice visualisation purpose
        tf_position = new_tf_position
        tf_orientation = feedback.pose.orientation

    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        #rospy.loginfo( s + ": mouse down" + mp + "." )
        pass
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        #rospy.loginfo( s + ": mouse up" + mp + "." )
        pass

    server.applyChanges()


def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control



#####################################################################
# Marker Creation

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s

def make6DofMarker( fixed, interaction_mode, position, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = tf_parent_frame
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "transform_marker"
    int_marker.description = "Drag to modify the transform"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )



if __name__=="__main__":
    rospy.init_node("interactive_static_transform_publisher")
    br = tf.TransformBroadcaster()


    tf_position = Point(0.0, 0.0, 0.0)
    tf_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    marker_position = Point(tf_position.x + marker_x_offset, tf_position.y, tf_position.z)

    if rospy.has_param('~child_frame'):
        tf_child_frame = rospy.get_param('~child_frame')

    if rospy.has_param('~parent_frame'):
        tf_parent_frame = rospy.get_param('~parent_frame')

    if rospy.has_param('~tf_publish_period_in_sec'):
        tf_update_period = rospy.get_param('~tf_publish_period_in_sec')


    server = InteractiveMarkerServer(rospy.get_name()+"/transform_control")

    menu_handler.insert( "Generate the static transform command", callback=processFeedback )

    make6DofMarker( True, InteractiveMarkerControl.MENU, marker_position, True)

    server.applyChanges()

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(tf_update_period), frameCallback)

    rospy.spin()
