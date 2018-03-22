#!/usr/bin/env python

import math, copy, random

import rospy, rospkg

from std_msgs.msg import Bool, Empty, String, Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped

import dynamic_reconfigure.server as dr

import tf2_ros
import tf2_geometry_msgs
import tf_conversions as tfc
import PyKDL as kdl

from bioloid.serial_bus import SerialPort, SerialBus
from bioloid.device_type_parser import DeviceTypeParser
from bioloid.device_type import DeviceTypes
from bioloid.device import Device
from bioloid.bus import BusError

class PtLaser:
    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 5)

        self.initialized = False
        self.sub_target_pose = rospy.Subscriber('~target_pose', PoseStamped, self.target_pose_cb)

        self.initialized = self.init_devices()

        self.sub_joint_pose = rospy.Subscriber('~joint_pose', Float32MultiArray, self.controller_cb)

        self.pub_joint_pose = rospy.Publisher('~current_joint_pose', Float32MultiArray, queue_size=10)

    def init_devices(self):
        baud_rate = rospy.get_param('~baud_rate', 1000000)
        port = rospy.get_param('~port', '/dev/ttyUSB0')

        pkg = rospkg.RosPack()
        dev_conf_path = pkg.get_path('hmri_pt_laser') + '/devcfg'

        self.joint_ids = rospy.get_param('~joint_ids', [1, 2])
        self.zeros = {}
        self.zeros[self.joint_ids[0]] = 150.0
        self.zeros[self.joint_ids[1]] = 60.0

        try:
            serial_port = SerialPort(port = port, baudrate = baud_rate)
        except serial.serialutil.SerialException:
            rospy.logfatal('Unable to open \'{}\''.format(port))
            return False

        self.bus = SerialBus(serial_port, show_packets = True)

        dev_types = DeviceTypes()
        parser = DeviceTypeParser(dev_types)
        parser.parse_dev_type_files(dev_conf_path)
        servo_type = dev_types.get('servo')
        if not servo_type:
            rospy.logfatal('Unable to find device type \'servo\'')
            return False

        self.dev = {}
        self.led = {}
        self.vel = {}
        self.pos = {}
        self.ccw_lim = {}
        self.cw_lim = {}
        self.cur_pos = {}

        for i in self.joint_ids:
            self.dev[i] = Device(self.bus, i, servo_type)

            ping = {}
            while not self.dev[i].ping():
                rospy.logerr('Device {} is not responding. Will try again later'.format(i))
                rospy.sleep(0.5)

            self.led[i] = self.dev[i].get_dev_reg('led')
            self.vel[i] = self.dev[i].get_dev_reg('moving-speed')
            self.pos[i] = self.dev[i].get_dev_reg('goal-position')
            self.cur_pos[i] = self.dev[i].get_dev_reg('present-position')

            self.ccw_lim[i] = self.dev[i].get_dev_reg('ccw-angle-limit')
            self.cw_lim[i] = self.dev[i].get_dev_reg('cw-angle-limit')
            rospy.loginfo('J{} pos: {} lim: ({}, {})'.format(i, self.pos[i].get() - self.zeros[i], self.cw_lim[i].get() - self.zeros[i], self.ccw_lim[i].get() - self.zeros[i]))

        return True

    def fk(self):
        return Pose()

    def ik(self, pose):
        j1 = yaw = 150.0
        j2 = pitch = 150.0

        return j1, j2

    def target_pose_cb(self, pose):
        if not self.initialized:
            return

        j1, j2 = self.ik(pose)

    def controller_cb(self, j):
        if not self.initialized:
            return

        if len(j.data) != 2:
            rospy.logerr_throttle(0.5, 'Array size should be {}', len(self.joint_ids))
            return

        new_pos = {}
        new_pos_print = {}
        vel = {}

        # > 360
        max_dist = 0 # deg

        max_vel = 114.0 # RPM
        max_t = 0 # sec

        for idx, i in enumerate(self.joint_ids):
            rospy.loginfo('cur_pos[{}]: {}'.format(i, self.cur_pos[i].get() - self.zeros[i]))

        for idx, i in enumerate(self.joint_ids):
            new_pos[i] = j.data[idx] + self.zeros[i]
            new_pos_print[i] = j.data[idx]
            dist = math.fabs(self.cur_pos[i].get() - new_pos[i])
            if dist > max_dist:
                max_dist = dist

        max_t = max_dist / (114.0)

        rospy.loginfo('max_t: {}'.format(max_t))

        for i in self.joint_ids:
            if max_t:
                vel[i] = math.fabs(self.cur_pos[i].get() - new_pos[i]) / max_t
            else:
                vel[i] = 0.0

        rospy.loginfo('new_pos: {}'.format(new_pos_print))
        rospy.loginfo('Vel: {}'.format(vel))

        for i in self.joint_ids:
            self.led[i].set(0)
            is_error = False
            try:
                self.vel[i].set(vel[i])
                self.pos[i].set(new_pos[i], deferred = True)

            except BusError as e:
                # Set LED
                self.led[i].set(1)
                # Don't move
                self.pos[i].set(self.cur_pos[i].get())
                rospy.logerr_throttle(0.5, e)
                is_error = True

        if not is_error:
            self.bus.send_action()

    def cur_joint_pos(self):
        joint_pos = Float32MultiArray()

        for i in self.joint_ids:
            joint_pos.data.append(self.cur_pos[i].get() - self.zeros[i])

        return joint_pos

    def run(self):
        loop_rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            try:
                self.pub_joint_pose.publish(self.cur_joint_pos())
                loop_rate.sleep()
            except rospy.ROSException, e:
                if e.message == 'ROS time moved backwards':
                    rospy.logwarn("Saw a negative time change, resetting.")

if __name__ == '__main__':
    rospy.init_node('pt_laser', anonymous = False)

    pt_laser = PtLaser()

    try:
        pt_laser.run()
    except rospy.ROSInterruptException:
        rospy.logdebug('Exiting')
        pass
