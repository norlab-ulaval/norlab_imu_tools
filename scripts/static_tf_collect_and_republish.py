#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import time


all_static_transforms = []


def tf_msg_callback(msg):
    global all_static_transforms
    print("Got TF message with " + str(len(msg.transforms))) + " transforms."
    all_static_transforms += msg.transforms


def tf_listener():
    global all_static_transforms
    rospy.init_node('static_tf2_listener', anonymous=True)
    sub = rospy.Subscriber("tf_static", TFMessage, tf_msg_callback)
    pub = rospy.Publisher('tf_static', TFMessage, latch=True, queue_size=1)

    # check what transforms we've heart
    while not rospy.is_shutdown():
        print("Num. of TFs received so far: " + str(len(all_static_transforms)))
        control_char = raw_input("c: clear, p: publish latched (and un-suscribe), q: quit: ")

        if control_char=="c":
            all_static_transforms = []

        elif control_char=="p":
            sub.unregister()
            time.sleep(2)
            newTFMessage = TFMessage()
            newTFMessage.transforms = all_static_transforms
            pub.publish(newTFMessage)
            print("Publishing the latched tf_static thing and waiting for the final CTRL+C...")
            break

        elif control_char=="q":
            return

        else:
            print("Input c, p, or q. Not " + control_char + ". Facepalm.")

    rospy.spin()

if __name__ == '__main__':
    tf_listener()