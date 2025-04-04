#!/usr/bin/env python3

# Command line arguments
from optparse import OptionParser

# ROS imports
import roslib, rospy, actionlib
import rospkg

from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16

# Crazyflie imports
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig as cfLogConfig
from rospy_crazyflie.msg import *
import rospy_crazyflie.client
from rospy_crazyflie.srv import *

# standard libraries
import numpy as np
import yaml, json
import pandas
import atexit
import sys
import os
import time

# Special ROS imports
from rospy_message_converter import message_converter

try:
    from collections.abc import MutableMapping
except:
    from collections import MutableMapping

################################################################################

def load_configuration(config_file):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('braitenfly')
    config_path = os.path.join(package_path, 'config', 'braitenfly_config.yaml')
    with open(config_path) as stream:
        config = yaml.safe_load(stream)
    return config

def flatten(dictionary, parent_key='', separator='_'):
    items = []
    for key, value in dictionary.items():
        new_key = parent_key + separator + key if parent_key else key
        if isinstance(value, MutableMapping):
            items.extend(flatten(value, new_key, separator=separator).items())
        else:
            items.append((new_key, value))
    return dict(items)

################################################################################

class Braiten_Fly(object):
    def __init__(self, config_file, crazyflie_name, crazyflie_number, takeoff, buzzer):
        
        rospy.init_node("braiten_fly")
        print("Initialize braiten-fly")

        self.config = load_configuration(config_file)
        self.crazyflie_name = crazyflie_name
        self.takeoff = takeoff

        # Connect to a Crazyflie on the server
        crazyflies = rospy_crazyflie.client.get_crazyflies(server='/crazyflie_server')
        self.cfclient = rospy_crazyflie.client.Client(crazyflies[crazyflie_number])
        self.current_blocking_action = None
        print('Client initialized.')

        # Play startup sound
        self.buzzer = buzzer
        if self.buzzer:
            print('Playing sound...', end='')
            # self.cfclient.play_buzzer(number=11, frequency=500, duration=3.0, stop=True)
            command = ('play_buzzer', [11, 500, 3.0, True])
            self.execute_command(command)
            print('done.')

        atexit.register(self.land_and_shutdown)

        self.history_length = 100
        self.module_rate = rospy.Rate(20) # check modules at X Hz

        self.home = None 
        #self.safety_net = np.array(self.config[safety_net])

        # Set up sensor subscribers
        ###########################
        self.sensor_topics = [  'KalmanPositionEst', # (float) kalman.stateX, kalman.stateY, kalman.stateZ
                                'MotorPower', # (int32_t) motor.m4, motor.m1, motor.m2, motor.m3
                                'Stabilizer', # (float) stabilizer.roll, stabilizer.pitch, stabilizer.yaw, (unit16_t) stabilizer.thrust 
                                'Acceleration', # (float) acc.x, acc.y, acc.z
                                'Gyro', # (float) gyro.x, gyro.y, gyro.z
                                'StateEst', # (float) stateEstimate.ax, stateEstimate.ay, stateEstimate.az, stateEstimate.vx, stateEstimate.vy, stateEstimate.vz
                                'Range', # (unit16_t) range.back, range.front, range.left, range.right, range.up, range.zrange
                                ]

        ############################

        self.sensor_history = {topic_name: None for topic_name in self.sensor_topics}
        self.module_history = []
        self.module_history_timestamps = []

        self.action_stack = [] # list of lists, each sublist should be ['cfclient action', parameters]
        self.action_stack_length = 60

        self.buzzer_stack = [] # list of lists, each sublist should be [timestamp, 'cfclient action', parameters], where timestamp says the time at which to execute
        self.buzzer_stack_length = 100

        ############################

        self.sensor_subscribers = {}
        for topic_name in self.sensor_topics:
            self.sensor_subscribers[topic_name] = rospy.Subscriber(self.crazyflie_name + '/' + topic_name, 
                                                                   eval(topic_name), # topic names are also message types.. yes, eval() is bad coding practice 
                                                                   self.sensor_callback,
                                                                   topic_name, # send topic name to call back
                                                                   queue_size = 10,
                                                                   ) 
        

    def wait(self):
        while self.cfclient.action_in_progress():
            rospy.sleep(.01)

    def land_and_shutdown(self):
        if self.takeoff:
            self.cfclient.land()
            self.wait()
            rospy.sleep(0.5)
            if self.buzzer:
                self.buzzer_stack = []
                command = ('play_buzzer', [0, 0, 0, 0])
                self.execute_command(command)
                rospy.sleep(0.1)
            self.takeoff = 0
            del self.cfclient
            sys.exit(0)

    def sensor_callback(self, msg, topic_name):
        # save current sensor info
        msg_dictionary = message_converter.convert_ros_message_to_dictionary(msg)
        flat_msg_dictionary = flatten(msg_dictionary)

        if self.sensor_history[topic_name] is None:
            self.sensor_history[topic_name] = pandas.DataFrame(flat_msg_dictionary, index=[0])
        else:
            self.sensor_history[topic_name] = pandas.concat([self.sensor_history[topic_name], pandas.DataFrame(flat_msg_dictionary, index=[self.sensor_history[topic_name].index.max()+1])])

        # remove old history
        while len(self.sensor_history[topic_name]) > self.history_length:
            self.sensor_history[topic_name] = self.sensor_history[topic_name].drop(self.sensor_history[topic_name].index.min())

    def execute_command(self, command):
        command_name, parameters = command

        if command_name == 'shutdown':
            self.takeoff = 0
            del self.cfclient
            sys.exit(0)

        if parameters is None:
            self.cfclient.__getattribute__(command_name)()
        elif type(parameters) is list:
            self.cfclient.__getattribute__(command_name)(*parameters)
        else:
            self.cfclient.__getattribute__(command_name)(parameters)

        self.wait()

    def fix_priority(self, priority):
        if priority == -1: # insert at bottom of stack
            priority = len(self.action_stack)
        if priority > len(self.action_stack):
            priority = len(self.action_stack)
        return priority

    def main(self):

        print('Modules loaded:')
        print(self.config['modules'])

        rospy.sleep(0.1)

        if self.takeoff:
            print('Taking off!!')
            self.cfclient.take_off(.75)
            self.wait()

        else:
            print('Not taking off!!')

        self.start_position = self.sensor_history['KalmanPositionEst'][['stateX', 'stateY', 'stateZ']].values[-1]

        while not rospy.is_shutdown():
            self.timenow = rospy.get_time()

            # clear out old module history (to prevent memory leak)
            while len(self.module_history) > self.history_length:
                _ = self.module_history.pop(0)
                _ = self.module_history_timestamps.pop(0)

            # remove actions from the bottom of the stack if there are too many
            if len(self.action_stack) > self.action_stack_length:
                ix = len(self.action_stack) - self.action_stack_length
                del(self.action_stack[ix:])
            if len(self.buzzer_stack) > self.buzzer_stack_length:
                ix = len(self.buzzer_stack) - self.buzzer_stack_length
                del(self.buzzer_stack[ix:])

            # do the buzzer modules
            for module in self.config['modules']:
                if module is not None:
                    if 'buzzer' in module:
                        priority, command = self.__getattribute__('module_' + module)(module)
                        if command is not None:
                            for i, c in enumerate(command[::-1]):
                                self.buzzer_stack.insert(0, c)
                            self.module_history_timestamps.append(self.buzzer_stack[-1][0])             
            # execute command from the top of the buzzer stack
            if len(self.buzzer_stack) > 0:
                #print(self.buzzer_stack)
                if time.time() > self.buzzer_stack[0][0]:
                    buzzer_command = self.buzzer_stack.pop(0)
                    self.execute_command(buzzer_command[1:])

            # check each module
            # if a module with very high priority is encountered, execute immediately
            for module in self.config['modules']:
                if module is not None:
                    if 'buzzer' not in module:
                        priority, command = self.__getattribute__('module_' + module)(module)

                        # add the commands to the stack
                        if command is not None:
                            self.module_history.append(module)
                            self.module_history_timestamps.append(self.timenow)
                            if type(command[0]) is not list: # we have only 1 command
                                command = [command,]

                            for i, c in enumerate(command):
                                p = self.fix_priority(priority)
                                p = self.fix_priority(p+i)
                                self.action_stack.insert(p, c)
                        
                        # go straight to execution when presented with an immediate priority command
                        if priority == 0:
                            continue

            if len(self.action_stack) > 0:
                pass
                # print(self.action_stack)

            # execute command from the top of the stack
            if self.takeoff and len(self.action_stack) > 0:
                command = self.action_stack.pop(0)
                self.execute_command(command)
            
            self.module_rate.sleep()

    def module_buzzer_frontrangefinder(self, module_name):
        """
        If an object is nearby to the forward facing range finder, play a sound for 0.5 second

        :return:
        commands    : (list) of four signed command actions
        """

        parameters = self.config[module_name]
        distance_threshold = parameters

        ranges = self.sensor_history['Range'][['front', 'left', 'back', 'right']].values[-1]

        if ranges[0] < distance_threshold:
            self.buzzer_stack = []
            return 1, [ [0, 'play_buzzer', [12, 2000, 0, 0]],
                        [time.time()+0.05, 'play_buzzer', [12, 3000, 0, 0]],
                        [time.time()+0.1, 'play_buzzer', [12, 4000, 0, 0]],
                        [time.time()+0.15, 'play_buzzer', [12, 5000, 0, 0]],
                        [time.time()+0.2, 'play_buzzer', [12, 6000, 0, 0]],
                        [time.time()+0.25, 'play_buzzer', [12, 7000, 0, 0]],
                        [time.time()+0.3, 'play_buzzer', [12, 8000, 0, 0]],
                        [time.time()+0.35, 'play_buzzer', [0, 500, 0, 0]]]
        else:
            return None, None

    def module_land_toprangefinder(self, module_name):
        """
        If an object is above, land, and shutdown.

        ending action

        :return: None x3
        """

        parameters = self.config[module_name]
        land_threshold, = parameters

        if self.sensor_history['Range']['up'].values[-1] < land_threshold:
            return 0, [['land',None], ['shutdown',None]]
        else:
            return None, None

    def module_land_bottomrangefinder(self, module_name):
        """
        If an object is below, land, and shutdown.

        ending action

        :return: None x3
        """

        parameters = self.config[module_name]
        land_threshold, = parameters

        if self.takeoff:
            return 0, [['land',None], ['shutdown',None]]
        else:
            return None, None

    def module_approach_frontrangefinder(self, module_name):
        """
        If an object is nearby to the forward facing range finder, move forwards by a specified amount.

        Low priority, open loop command.

        :return:
        priority    : (int) 1 or 0 indicating high or low priority, respectively
        action      : None
        commands    : (list) of four signed command actions
        """

        parameters = self.config[module_name]
        high_distance_threshold, low_distance_threshold, approach_distance = parameters

        ranges = self.sensor_history['Range'][['front', 'left', 'back', 'right']].values[-1]

        if ranges[0] < high_distance_threshold and ranges[0] > low_distance_threshold:
            return [-1, ['forward', approach_distance]]
        else:
            return None, None

    def module_land_bottomrangefinder(self, module_name):
        """
        If an object is below, land, and shutdown.

        ending action

        :return: None x3
        """

        parameters = self.config[module_name]
        land_threshold, = parameters

        if self.takeoff:
            return 0, [['land',None], ['shutdown',None]]
        else:
            return None, None

    def module_approach_frontrangefinder(self, module_name):
        """
        If an object is nearby to the forward facing range finder, move forwards by a specified amount.

        Low priority, open loop command.

        :return:
        priority    : (int) 1 or 0 indicating high or low priority, respectively
        action      : None
        commands    : (list) of four signed command actions
        """

        parameters = self.config[module_name]
        high_distance_threshold, low_distance_threshold, approach_distance = parameters

        ranges = self.sensor_history['Range'][['front', 'left', 'back', 'right']].values[-1]

        if ranges[0] < high_distance_threshold and ranges[0] > low_distance_threshold:
            return [-1, ['forward', approach_distance]]
        else:
            return None, None

    def module_retreat_toprangefinder(self, module_name):
        """
        If an object is nearby to the top range finder, move down by a specified amount.

        High priority, open loop command

        :return:
        priority    : (int) 1 or 0 indicating high or low priority, respectively
        action      : None
        commands    : (list) of four signed command actions
        """

        parameters = self.config[module_name]
        distance_threshold, retreat_distance = parameters

        toprange = self.sensor_history['Range'][['up']].values[-1]

        if toprange < distance_threshold:
            return [1, ['down', retreat_distance]]
        else:
            return None, None

    def module_retreat_allrangefinders(self, module_name):
        """
        If an object is nearby to any sideways range finder, move away by a specified amount.

        High priority, open loop command

        :return:
        priority    : (int) 1 or 0 indicating high or low priority, respectively
        action      : None
        commands    : (list) of four signed command actions
        """

        parameters = self.config[module_name]
        distance_threshold, retreat_distance = parameters

        ranges = self.sensor_history['Range'][['front', 'left', 'back', 'right']].values[-1]


        # check sensor values give sensor history
        commands_numerical = [0, 0, 0]
        if ranges[0] < distance_threshold:
            commands_numerical[0] += -1*retreat_distance
        if ranges[1] < distance_threshold:
            commands_numerical[1] += -1*retreat_distance
        if ranges[2] < distance_threshold:
            commands_numerical[0] += retreat_distance
        if ranges[3] < distance_threshold:
            commands_numerical[1] += retreat_distance

        commands_action = []

        if commands_numerical[0] < 0:
            commands_action.append(['back', float(np.abs(commands_numerical[0])) ])
        elif commands_numerical[0] > 0:
            commands_action.append(['forward', float(np.abs(commands_numerical[0])) ])

        if commands_numerical[1] < 0:
            commands_action.append(['right', float(np.abs(commands_numerical[1])) ])
        elif commands_numerical[1] > 0:
            commands_action.append(['left', float(np.abs(commands_numerical[1])) ])

        if commands_numerical[2] < 0:
            commands_action.append(['down', float(np.abs(commands_numerical[2])) ])
        elif commands_numerical[2] > 0:
            commands_action.append(['up', float(np.abs(commands_numerical[2])) ])

        if len(commands_action) > 0:
            return 0, commands_action
        else:
            return None, None


    def module_orient_rangefinders(self, module_name):
        """
        If there is an object within a given threshold distance from the four lateral facing rangefinders,
        perform a yaw rotation to face the object.

        Low priority, open loop command

        :return:
        priority    : (int) 1 or 0 indicating high or low priority, respectively
        action      : None
        commands    : (list) of four signed command actions
        """

        parameters = self.config[module_name]
        high_distance_threshold, low_distance_threshold, turnangle = parameters

        ranges = self.sensor_history['Range'][['front', 'left', 'back', 'right']].values[-1]


        command_turnangle = 0

        if np.abs(self.sensor_history['Gyro']['z'].values[-1]) < 30:
            if np.min(ranges) < high_distance_threshold and np.min(ranges) > low_distance_threshold: # this is a little funny
                if np.argmin(ranges) == 1:
                    command_turnangle = turnangle
                if np.argmin(ranges) == 2:
                    command_turnangle = -1*turnangle
                if np.argmin(ranges) == 3:
                    command_turnangle = -1*turnangle

        turnangle = int(command_turnangle)
        if turnangle > 0:
            return 1, ['turn_left', int(np.abs(turnangle))]
        elif turnangle < 0:
            return 1, ['turn_right', int(np.abs(turnangle))]
        else:
            return None, None

    def module_circleright_topfronttrangefinder_delayed(self, module_name):
        """
        If there is an object within the threshold distance from the top (upward) facing range finder,
        and if in the last 3 seconds there was an object in front, but there isn't any more, then
        fly in circles.

        low priority, overriding action

        :return:
        priority    : (int) 1 or 0 indicating high or low priority, respectively
        action      : action name (circle right)
        commands    : (list) action args
        """

        parameters = self.config[module_name]
        sensor_time_history, distance_threshold, circle_radius = parameters

        # check sensor values give sensor history
        ranges_now = self.sensor_history['Range'][['up', 'front']].values[-1]

        q = 'stamp_secs >= ' + str( float(self.timenow) - float(3))

        circle = False
        if ranges_now[0] < distance_threshold:
            if self.sensor_history['Range'].query(q)['front'].min() < distance_threshold:
                if ranges_now[1] > distance_threshold:
                    circle = True

        if circle:
            commands = []
            for i in range(20):
                commands.append(['forward', 0.1])
                commands.append(['turn_right', 18])
            return -1, commands

        else:
            return None, None



# ------------------------------------ jaleesa's modules -----------------------------------------------------------------------------------#

            
    def module_buzzer_inactivity(self, module_name):
        """
        If no actions have been performed in the last X seconds, trigger the buzzer.

        :return:
        priority    : low 
        commands    : (list) of buzzer commands
        """

        parameters = self.config[module_name]
        inactivity_threshold, buzzer_frequency, buzzer_duration = parameters
 
        current_time = rospy.get_time()


       # If no actions have been performed in the last X seconds, trigger the buzzer
        if (self.module_history_timestamps != [])  and (current_time - self.module_history_timestamps[-1]) > inactivity_threshold:
            #print('triggered inactivity buzzer, time since last action is', (current_time - self.module_history_timestamps[-1]) )
            return -1, [ [time.time(), 'play_buzzer', [12, buzzer_frequency, buzzer_duration, 0]],
                        [time.time()+0.05, 'play_buzzer', [0, 500, 0, 0]],]
        else:
            return None, None




    def module_siderangefinders_changealtitude(self, module_name):
        """
        If an object is on one side of range finder, move up by a specified amount. If an object is on other side of range finder, move down by another specified amount


        """

        parameters = self.config[module_name]
        distance_threshold, sensor_up, up_distance, sensor_down, down_distance = parameters

        ranges = self.sensor_history['Range'][['front', 'left', 'back', 'right']].values[-1]


        # check sensor values give sensor history
        commands_numerical = [0, 0,]
        commands_action = []

        if ranges[sensor_up] < distance_threshold:
            commands_numerical[0] += up_distance
            commands_action.append(['up', float(np.abs(commands_numerical[0])) ])
        if ranges[sensor_down] < distance_threshold:
            commands_numerical[1] += down_distance
            commands_action.append(['down', float(np.abs(commands_numerical[1])) ])

        if len(commands_action) > 0:
            return 1, commands_action
        else:
            return None, None


    def module_buzzer_changealtitude(self, module_name):
        """
        If increasing or decreasing in altitude, play a sound - student chooses whether up or downward motion causes the noise


        """

        parameters = self.config[module_name]
        buzzer_frequency, buzzer_duration, triggering_direction = parameters

        position1 = self.sensor_history['KalmanPositionEst'][['stateX', 'stateY', 'stateZ']].values[-1]
        position2 = self.sensor_history['KalmanPositionEst'][['stateX', 'stateY', 'stateZ']].values[-10]
   
        
        
        if (triggering_direction=="up") and (position1-position2)[2] > 0.03: # if crazyflie went up
            #print('z position change:', (position1-position2)[2])
            self.buzzer_stack = []
            return 1, [ [time.time(), 'play_buzzer', [12, buzzer_frequency, buzzer_duration, 0]],
                        [time.time()+0.05, 'play_buzzer', [0, 500, 0, 0]]]

                                          
        if (triggering_direction=="down") and (position1-position2)[2] < -0.03: # if crazyflie went down
            #print('z position change:', (position1-position2)[2])
            self.buzzer_stack = []
            return 1, [ [time.time(), 'play_buzzer', [12, buzzer_frequency, buzzer_duration, 0]],
                        [time.time()+0.05, 'play_buzzer', [0, 500, 0, 0]]]

        else:
            return None, None

    def module_buzzer_movelaterally(self, module_name):
        """
        If going a predefined direction, play a sound

        """

        if len(self.sensor_history['KalmanPositionEst'][['stateX', 'stateY', 'stateZ']].values) < 11:
            return None, None

        parameters = self.config[module_name]
        buzzer_frequency, buzzer_duration, triggering_direction = parameters

        position1 = self.sensor_history['KalmanPositionEst'][['stateX', 'stateY', 'stateZ']].values[-1]
        position2 = self.sensor_history['KalmanPositionEst'][['stateX', 'stateY', 'stateZ']].values[-10]

        
        if (triggering_direction=="right") and (position1-position2)[1] > 0.1: 
            #print(triggering_direction, 'position change:', (position1-position2)[1])
            self.buzzer_stack = []
            return 1, [ [time.time(), 'play_buzzer', [12, buzzer_frequency+2000, buzzer_duration, 0]],
                        [time.time()+0.05, 'play_buzzer', [12, buzzer_frequency+1000, 0, 0]],
                        [time.time()+0.1, 'play_buzzer', [12, buzzer_frequency, 0, 0]],                        
                        [time.time()+0.15, 'play_buzzer', [0, 500, 0, 0]]]

                                         
        if (triggering_direction=="left") and (position1-position2)[1] < -0.1:
            #print(triggering_direction, 'position change:', (position1-position2)[1])
            self.buzzer_stack = []
            return 1, [ [time.time(), 'play_buzzer', [12, buzzer_frequency+2000, buzzer_duration, 0]],
                        [time.time()+0.05, 'play_buzzer', [12, buzzer_frequency+1000, 0, 0]],
                        [time.time()+0.1, 'play_buzzer', [12, buzzer_frequency, 0, 0]],                        
                        [time.time()+0.15, 'play_buzzer', [0, 500, 0, 0]]]
        
        if (triggering_direction=="forward") and (position1-position2)[0] > 0.1: 
            #print(triggering_direction, 'position change:', (position1-position2)[0])
            self.buzzer_stack = []
            return 1, [ [time.time(), 'play_buzzer', [12, buzzer_frequency+2000, buzzer_duration, 0]],
                        [time.time()+0.05, 'play_buzzer', [12, buzzer_frequency+1000, 0, 0]],
                        [time.time()+0.1, 'play_buzzer', [12, buzzer_frequency, 0, 0]],                        
                        [time.time()+0.15, 'play_buzzer', [0, 500, 0, 0]]]
                                          
        if (triggering_direction=="backward") and (position1-position2)[0] < -0.1:
            #print(triggering_direction, 'position change:',(position1-position2)[0])
            self.buzzer_stack = []
            return 1, [ [time.time(), 'play_buzzer', [12, buzzer_frequency+2000, 0, 0]],
                        [time.time()+0.05, 'play_buzzer', [12, buzzer_frequency+1000, 0, 0]],
                        [time.time()+0.1, 'play_buzzer', [12, buzzer_frequency, 0, 0]],                        
                        [time.time()+0.15, 'play_buzzer', [0, 500, 0, 0]]]
        else:
            return None, None

    def module_buzzer_roll(self, module_name):
        """
        Module buzzer frequency based on roll, but only when not flying.

        :return:
        commands    : (list) of four signed command actions
        """

        parameters = self.config[module_name]
        self.buzzer_stack = []
        if self.buzzer and (not self.takeoff):
            return 0, [ [1, 'play_buzzer', [14, 2000, 0, 0]] ]
        else:
            return None, None

    def module_buzzer_ramp(self, module_name):
        """
        Module buzzer ramp speed based on left vs right sensor distance, but only when not flying.

        :return:
        commands    : (list) of four signed command actions
        """

        parameters = self.config[module_name]
        distance_sum_thresh = parameters[0]
        front, left, back, right = self.sensor_history['Range'][['front', 'left', 'back', 'right']].values[-1]

        self.buzzer_stack = []
        if self.buzzer and (not self.takeoff):
            if (right + left) < distance_sum_thresh:
                if left > right:
                    command = [[0, 'play_buzzer', [8, 2000, 0, 0]]]  # slow ramp
                else:
                    command = [[0, 'play_buzzer', [9, 2000, 0, 0]]]  # fast ramp
            else:
                command = [[0, 'play_buzzer', [0, 2000, 0, 0]]]

            return 0, command
        else:
            return None, None

    def module_buzzer_homing(self, module_name):
        """
        Module buzzer ramp speed based on left vs right sensor distance, but only when not flying.

        :return:
        commands    : (list) of four signed command actions
        """

        parameters = self.config[module_name]


        x0, y0, z0 = self.start_position

        x, y, z = self.sensor_history['KalmanPositionEst'][['stateX', 'stateY', 'stateZ']].values[-1]
        distance = np.sqrt((x - x0)**2 + (y - y0)**2)

        low_frequency = 20
        high_frequency = 5000

        low_distance = 0
        high_distance = 2

        frequency = int(map_range(distance, low_distance, high_distance, low_frequency, high_frequency))
        if frequency > high_frequency:
            frequency = high_frequency

        if self.buzzer:
            command = [[time.time(), 'play_buzzer', [12, frequency, 0, 0]]]
            print(np.round(distance, 3), frequency)

            return 1, command
        else:
            return None, None


def map_range(x, old_min, old_max, new_min, new_max):
    return (x - old_min) / (old_max - old_min) * (new_max - new_min) + new_min



# -----------------------------------------------------------------------------------------------------------------------#            
            
################################################################################

if __name__ == '__main__':    
    parser = OptionParser()
    parser.add_option("--takeoff", type="int", dest="takeoff", default=0,
                        help="take off (0 or 1), default 0 = will not take off")
    parser.add_option("--crazyflie", type="str", dest="crazyflie_name", default='crazyflie1',
                        help="crazyflie name (base topic)")
    parser.add_option("--config", type="str", dest="config", default='braitenfly_config.yaml',
                        help="crazyflie config file")
    parser.add_option("--crazyflie_number", type="int", dest="crazyflie_number", default=0,
                        help="which crazyflie to connect to on server, default 0")
    parser.add_option("--buzzer", type="int", dest="buzzer", default=1,
                        help="buzzer (0 or 1), default 1 = will use buzzer")

    (options, args) = parser.parse_args()

    braiten_fly = Braiten_Fly(options.config,
                              options.crazyflie_name,
                              options.crazyflie_number,
                              options.takeoff,
                              options.buzzer)
    braiten_fly.main()
