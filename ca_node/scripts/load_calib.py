#!/usr/bin/python

package_name = 'create_node'

import roslib
roslib.load_manifest(package_name)
import rospy
from create_node.srv import SetTurtlebotMode
import yaml
import os
import rospkg
import dynamic_reconfigure.client
import subprocess

service = 'turtlebot_node/set_operation_mode'
cmd_vel_received = False


def cmd_vel_callback(msg):
    global cmd_vel_received
    cmd_vel_received = True

def get_usb_to_serial_id():
    usbpath = subprocess.check_output("readlink -f /sys/class/tty/ttyUSB0", shell=True)
    usbpath = usbpath.strip()
    if len(usbpath) == 0:
        return None
    serialid = ""
    try:
        f = open(usbpath + "/../../../../serial", "r")
        serialid = f.read().strip()
        f.close()
    except:
        pass
    try:
        f = open(usbpath + "/../../../../idVendor", "r")
        serialid += f.read().strip()
        f.close()
        f = open(usbpath + "/../../../../idProduct", "r")
        serialid += f.read().strip()
        f.close()
    except:
        pass
    if len(serialid.strip()) == 0:
        return None
    return serialid

def get_kinect_serial():
    ret = subprocess.check_output("lsusb -v -d 045e:02ae | grep Serial | awk '{print $3}'", shell=True)
    if len(ret.strip()) > 0:
        return ret.strip()
    return None

def load_yaml(path):
    try:
        f = open(path, "r")
        docs = yaml.load_all(f)
        return docs

    except:
        return None

if __name__ == '__main__':
    rospy.init_node('calibration_loader')

    cmd_vel_sub = rospy.Subscriber('cmd_vel', rospy.AnyMsg, cmd_vel_callback)
    
    # wait for the main create node to come up (no point in using dynamic reconfigure until then)
    rospy.wait_for_service(service)

    # wait for a kinect to appear

    kinect_serial = None
    while not rospy.is_shutdown() and kinect_serial is None and not cmd_vel_received:
        try:
            kinect_serial = get_kinect_serial()
        except:
            rospy.sleep(2)

    if rospy.is_shutdown():
        exit(0)

    if kinect_serial is None:
        rospy.loginfo("Velocity command received, assuming the robot is operational.  Attempting to calibration based on the usb to serial convert's id")
        kinect_serial = get_usb_to_serial_id()
        if kinect_serial is None:
            rospy.logwarn("Calibration failed, no unique identifier for the robot could be found.")
            exit(0) # well, at least we tried

    rospack = rospkg.RosPack()

    # search for a configuration file matching he kinect's serial number
    # first try the default directory
    whichfile1 = str(kinect_serial) + ".yaml"
    docs = load_yaml(whichfile1)
    whichfile = whichfile1
    if docs is None:
         # ok, try using os's expandpath and ros_home
         ros_home = os.environ.get('ROS_HOME')
         if ros_home is None:
            ros_home = "~/.ros"

         whichfile2 = os.path.expanduser(ros_home +"/turtlebot_create/" +str(kinect_serial) + ".yaml")
         docs = load_yaml(whichfile2)
         whichfile = whichfile2
         if docs is None:
             # now try the package path

             whichfile3 = rospack.get_path(package_name) + "/" + str(kinect_serial) + ".yaml"
             docs = load_yaml(whichfile3)
             whichfile = whichfile3

    if docs is None:
        rospy.logwarn("Could not load a calibration file for this turtlebot, it is recommended to run turtlebot_calibration to generate a new file.")
        rospy.logwarn("Search paths: %s , %s , %s", whichfile1, whichfile2, whichfile3)
        exit(0)

    # use dynamic_reconfigure to set the params of turtlebot_node
    for doc in docs:
        client = dynamic_reconfigure.client.Client("turtlebot_node")

        resp = client.update_configuration(doc)

    # we're done
    rospy.loginfo("Turtlebot calibration successfully loaded from %s", whichfile)
