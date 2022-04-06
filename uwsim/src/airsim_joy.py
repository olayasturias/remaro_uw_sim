#!/usr/bin/env python
import os
import rospy
import airsim
import numpy as np
from std_msgs.msg import Bool
# from geometry_msgs.msg import Twist, Accel, Vector3
from sensor_msgs.msg import Joy

class VehicleTeleop:
    def __init__(self):
        # Load the mapping for each input
        self._axes = dict(x=4, y=3, z=1,
                          roll=2, pitch=5, yaw=0,
                          xfast=-1, yfast=-1, zfast=-1,
                          rollfast=-1, pitchfast=-1, yawfast=-1)
        # Load the gain for each joystick axis input
        # (default values for the XBox 360 controller)
        self._axes_gain = dict(x=3, y=3, z=0.5,
                               roll=0.5, pitch=0.5, yaw=0.5,
                               xfast=6, yfast=6, zfast=1,
                               rollfast=2, pitchfast=2, yawfast=2)

        if rospy.has_param('~mapping'):
            mapping = rospy.get_param('~mapping')
            for tag in self._axes:
                if tag not in mapping:
                    rospy.loginfo('Tag not found in axes mapping, '
                                  'tag=%s' % tag)
                else:
                    if 'axis' in mapping[tag]:
                        self._axes[tag] = mapping[tag]['axis']
                    if 'gain' in mapping[tag]:
                        self._axes_gain[tag] = mapping[tag]['gain']

        # Dead zone: Force values close to 0 to 0
        # (Recommended for imprecise controllers)
        self._deadzone = 0.5
        if rospy.has_param('~deadzone'):
            self._deadzone = float(rospy.get_param('~deadzone'))

        # Default for the RB button of the XBox 360 controller
        self._deadman_button = -1
        if rospy.has_param('~deadman_button'):
            self._deadman_button = int(rospy.get_param('~deadman_button'))

        # If these buttons are pressed, the arm will not move
        if rospy.has_param('~exclusion_buttons'):
            self._exclusion_buttons = rospy.get_param('~exclusion_buttons')
            if type(self._exclusion_buttons) in [float, int]:
                self._exclusion_buttons = [int(self._exclusion_buttons)]
            elif type(self._exclusion_buttons) == list:
                for n in self._exclusion_buttons:
                    if type(n) not in [float, int]:
                        raise rospy.ROSException(
                            'Exclusion buttons must be an integer index to '
                            'the joystick button')
        else:
            self._exclusion_buttons = list()

        # Default for the start button of the XBox 360 controller
        self._home_button = 7
        if rospy.has_param('~home_button'):
            self._home_button = int(rospy.get_param('~home_button'))

        # Joystick topic subscriber
        self._joy_sub = rospy.Subscriber('/joy', Joy, self._joy_callback)

        # Connect to airsim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection() 
        self.client.enableApiControl(True)     


        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()
        

    def _parse_joy(self, joy=None):
        lv = np.zeros(3) # linear velocities x y z
        av = np.zeros(3) # angular velocities roll pitch yaw
        if joy is not None:
            # Linear velocities:
            if self._axes['x'] > -1 and abs(joy.axes[self._axes['x']]) > self._deadzone:
                lv[0] = self._axes_gain['x'] * joy.axes[self._axes['x']]

            if self._axes['y'] > -1 and abs(joy.axes[self._axes['y']]) > self._deadzone:
                lv[1] = self._axes_gain['y'] * joy.axes[self._axes['y']]

            if self._axes['z'] > -1 and abs(joy.axes[self._axes['z']]) > self._deadzone:
                lv[2] = self._axes_gain['z'] * joy.axes[self._axes['z']]

            if self._axes['xfast'] > -1 and abs(joy.axes[self._axes['xfast']]) > self._deadzone:
                lv[0] = self._axes_gain['xfast'] * joy.axes[self._axes['xfast']]

            if self._axes['yfast'] > -1 and abs(joy.axes[self._axes['yfast']]) > self._deadzone:
                lv[1] = self._axes_gain['yfast'] * joy.axes[self._axes['yfast']]

            if self._axes['zfast'] > -1 and abs(joy.axes[self._axes['zfast']]) > self._deadzone:
                lv[2] = self._axes_gain['zfast'] * joy.axes[self._axes['zfast']]

            # Angular velocities:
            if self._axes['roll'] > -1 and abs(joy.axes[self._axes['roll']]) > self._deadzone:
                av[0] = self._axes_gain['roll'] * joy.axes[self._axes['roll']]

            if self._axes['rollfast'] > -1 and abs(joy.axes[self._axes['rollfast']]) > self._deadzone:
                av[0] = self._axes_gain['rollfast'] * joy.axes[self._axes['rollfast']]

            if self._axes['pitch'] > -1 and abs(joy.axes[self._axes['pitch']]) > self._deadzone:
                av[1] = self._axes_gain['pitch'] * joy.axes[self._axes['pitch']]

            if self._axes['pitchfast'] > -1 and abs(joy.axes[self._axes['pitchfast']]) > self._deadzone:
                av[1] = self._axes_gain['pitchfast'] * joy.axes[self._axes['pitchfast']]

            if self._axes['yaw'] > -1 and abs(joy.axes[self._axes['yaw']]) > self._deadzone:
                av[2] = self._axes_gain['yaw'] * joy.axes[self._axes['yaw']]

            if self._axes['yawfast'] > -1 and abs(joy.axes[self._axes['yawfast']]) > self._deadzone:
                av[2] = self._axes_gain['yawfast'] * joy.axes[self._axes['yawfast']]

        else:
            lv = np.zeros(3)
            la = np.zeros(3)
        print('lv',lv)
        return lv,av

    def _joy_callback(self, joy):
        rospy.loginfo('joycallback')
        # If any exclusion buttons are pressed, do nothing
        self.client.cancelLastTask()
        self.client.enableApiControl(True)
        try:
            for n in self._exclusion_buttons:
                if joy.buttons[n] == 1:
                    lv,av = self._parse_joy()
                    self.client.moveByVelocityBodyFrameAsync(lv[0], lv[1], lv[2], 0.1, airsim.DrivetrainType.ForwardOnly,
                                            airsim.YawMode(False, 0)).join()
                    # self._output_pub.publish(cmd)
                    return

            if self._deadman_button != -1:
                if joy.buttons[self._deadman_button] == 1:
                    lv,la = self._parse_joy(joy)
                    self.client.moveByVelocityBodyFrameAsync(lv[0], lv[1], lv[2], 0.1, airsim.DrivetrainType.ForwardOnly,
                                            airsim.YawMode(False, 0)).join()
                else:
                    lv,la = self._parse_joy()
                    self.client.moveByVelocityBodyFrameAsync(lv[0], lv[1], lv[2], 0.1, airsim.DrivetrainType.ForwardOnly,
                                            airsim.YawMode(False, 0)).join()
            else:
                lv,la = self._parse_joy(joy)

            self.client.moveByVelocityBodyFrameAsync(lv[0], lv[1], lv[2], 0.1, airsim.DrivetrainType.ForwardOnly,
                                            airsim.YawMode(False, 0)).join()
            # self._output_pub.publish(cmd)
            # self._home_pressed_pub.publish(
            #     Bool(bool(joy.buttons[self._home_button])))
        except Exception as e:
            print('Error occurred while parsing joystick input,'
                  ' check if the joy_id corresponds to the joystick ' 
                  'being used. message={}'.format(e))
                  

if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    teleop = VehicleTeleop()

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
