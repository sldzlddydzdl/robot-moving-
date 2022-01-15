#!/usr/bin/env python2

# rospy for use ros with python
import rospy

# thread module of python
import threading

# time module of python (refer to unix time)
import time

import math

# keyboard module of pynput python package for keyboard input
from pynput import keyboard

# actionlib for Connect with ROS Action Server
import actionlib

# ur dashboard messages to use with ros service or ros action
from ur_dashboard_msgs.msg import SetModeAction, \
                                    SetModeGoal, \
                                    RobotMode

# ur dashboard service messages to require service to ur controller
from ur_dashboard_msgs.srv import GetRobotMode, \
                                    GetProgramState, \
                                    GetLoadedProgram, \
                                    GetSafetyMode, \
                                    Load

# controller manager service messages to choose controller for ur robot
from controller_manager_msgs.srv import SwitchControllerRequest, \
                                        SwitchController, \
                                        LoadControllerRequest, \
                                        LoadController

# Trigger Module for standard service
from std_srvs.srv import Trigger

# standard messages for various purpose (e.g. String)
import std_msgs.msg

# cartesian control messages to control ur robot in cartesian coordinate
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, \
                                        FollowCartesianTrajectoryGoal, \
                                        FollowCartesianTrajectoryResult, \
                                        CartesianTrajectoryPoint

# TFMessage module of tf2 messages, tf means transformation
# It used to get information of end-effector pose 
from tf2_msgs.msg import TFMessage

# Rotation module of scipy package
# It used for convert quaternion to euler or euler to quaternion
from scipy.spatial.transform import Rotation

ALL_CONTROLLERS = [
        "scaled_pos_joint_traj_controller",
        "pos_joint_traj_controller",
        "scaled_vel_joint_traj_controller",
        "vel_joint_traj_controller",
        "joint_group_vel_controller",
        "forward_joint_traj_controller",
        "forward_cartesian_traj_controller",
        "twist_controller",
        "pose_based_cartesian_traj_controller",
        "joint_based_cartesian_traj_controller",
        ]

class UR():
    def __init__(self):

        # Initialize UR Class
        print("UR Class is Initializing")



        # Class Variables Initialization
        # ========================================
        # timeout for wait any server it connect
        timeout = rospy.Duration(30)
        # quaternion pose variable, we get pose of end-effector to this variable.
        self.pose_quat = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # time for unit control of ur robot
        self.time_from_start=0.01

        # set flag for control ur robot
        self.move_flag = False
        
        # ROS Service Initialization
        # ========================================
        # Get Current Robot Mode
        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        # Get Current Program State
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        # Get Current Loaded Program
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        # Get Current Safety Mode State
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)
        
        # Connect to Dashboard Server
        self.s_connectToDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)
        # Quit from Dashboard Server
        self.s_quitFromDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)
        # Load Program
        self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        # Play Program
        self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        # Stop Program
        self.s_stopProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        


        # ROS Action Initialization
        # ========================================
        # Connect to Set Mode Action Server from Client
        self.set_mode_client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        try:
            self.set_mode_client.wait_for_server(timeout)
            print("set mode action client is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))
            
        # Connect to Load Controller Action Server from Client
        self.load_controllers_client = rospy.ServiceProxy('/controller_manager/load_controller',
                LoadController)
        try:
            self.load_controllers_client.wait_for_service(timeout)
            print("controller load service is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach controller load service. Make sure that the driver is actually running."
                " Msg: {}".format(err))
            
        # Connect to Switch Controller Action Server from Client
        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',
                SwitchController)
        try:
            self.switch_controllers_client.wait_for_service(timeout)
            print("controller switch service is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach controller switch service. Make sure that the driver is actually running."
                " Msg: {}".format(err))
            
        # Connect to Cartesian Controller Action Server from Client
        self.cartesian_passthrough_trajectory_client = actionlib.SimpleActionClient(
            '/forward_cartesian_traj_controller/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        if not self.cartesian_passthrough_trajectory_client.wait_for_server(timeout):
            self.fail(
                "Could not reach cartesian passthrough controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))



        # ROS Publisher & Subscriber Initialization
        # ========================================
        # script publisher for specific purpose
        #self.script_publisher = rospy.Publisher("/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)



        #Running up the Manipulator
        # ========================================
        # Connect to Dashboard Server
        print("try to connect to server")
        resp = self.s_connectToDashboardServer()
        print("server is connected")
        # Power Off the Robot (for the situation the robot is powered on)
        print("try to run the robot")
        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)
        # Set Robot Mode to Running
        # RUNNING : Execute Power ON & Break Release
        self.set_robot_to_mode(RobotMode.RUNNING)
        rospy.sleep(5)
        print("robot is running")
        # Load Program that we made already
        # It works with Real Robot but URSim
        self.s_loadProgram("/programs/ros.urp")
        # Play the Program that we Load
        rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
        resp = self.s_playProgram()
        rospy.sleep(0.5)
        # If you wanna observe the movement of UR Robot in RViz, you should turn on Joint State Controller.
        # It'd publish TF Information of UR Joint.
        # If you don'd turn on Joint State Controller, you'd loose Joint TF Information and can't observe movement of UR Robot.
        self.load_controller("joint_state_controller")
        rospy.sleep(0.5)
        self.switch_on_controller("joint_state_controller")
        rospy.sleep(0.5)
        # Switch the Controller for UR Robot to Forward Cartesian Trajectory Controller
        self.switch_on_controller("forward_cartesian_traj_controller")
        rospy.sleep(0.5)

        # Event handler for Keyboard Input 
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    # Function for Pressed Keyboard Event
    def on_press(self, key):
        #print('Key %s pressed' % key)
        # check robot is moving, if robot is moving, then do nothing.
        if self.move_flag == False:
            threading.Thread(target=self.move, args=(key,)).start()

    # Function for Released Keyboard Event
    def on_release(self, key):
        #print('Key %s released' %key)
        if key == keyboard.Key.esc:
            self.finalize()
            return False

    # Function for take care of Keyboard Input
    # Convert Quaternion to Euler
    # Change value of euler pose and call the control function(cartesian_traj)
    def move(self, key):

        # get keyboard input and add number to specific value 
        if key == keyboard.KeyCode(char='r'):
            self.square()
        elif key == keyboard.KeyCode(char='1'):
            self.pick_and_place()
        elif key == keyboard.KeyCode(char='2'):
            self.circle()
        elif key == keyboard.KeyCode(char='3'):
            self.object_observation()
            

    # Function for Control UR Robot in the Cartesian Coordinate
    def cartesian_traj(self, pose_list):     
        goal = FollowCartesianTrajectoryGoal()

        for i, l in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose.position.x = l[0]
            point.pose.position.y = l[1]
            point.pose.position.z = l[2]

            rot = Rotation.from_euler('xyz', [l[3], l[4], l[5]], degrees=True)
            rot_quat = rot.as_quat()
            #print(rot_quat)

            point.pose.orientation.x = -rot_quat[0]
            point.pose.orientation.y = -rot_quat[1]
            point.pose.orientation.z = -rot_quat[2]
            point.pose.orientation.w = -rot_quat[3]
            print(point.pose)
            
            time_from_start = l[6]
            point.time_from_start = rospy.Duration(time_from_start)
            goal.trajectory.points.append(point)
        
        goal.goal_time_tolerance = rospy.Duration(0.6)
        
        self.cartesian_passthrough_trajectory_client.send_goal(goal)
        wait = self.cartesian_passthrough_trajectory_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.cartesian_passthrough_trajectory_client.get_result()
        print(self.cartesian_trajectory_client.get_result())

        #rospy.loginfo("Received result SUCCESSFUL")

    # if the robot trying to reach point which is not stable,
    # there would be error from the robot controller
    # to prevent that situation, we set the limit for the controller
    def check_cartesian_limit(self, x, y, z, roll, pitch, yaw):
        if (math.sqrt(pow(x,2)+pow(y,2)+pow(z,2)) > 1.2) \
                or (math.sqrt(pow(x,2)+pow(y,2)+pow(z,2)) < 0.05) \
                or x < 0.0 or z < 0.0 \
                or (roll > -135.0 and roll < 135.0) \
                or (pitch < -45.0 or pitch > 45.0) \
                or (yaw > 180.0 and yaw < 0.0): 
            return False
        else: 
            return True
        

    # Function to Set Robot Mode
    def set_robot_to_mode(self, target_mode):
        goal = SetModeGoal()
        goal.target_robot_mode = target_mode
        goal.play_program = True # we use headless mode during tests
        # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
        goal.stop_program = False

        self.set_mode_client.send_goal(goal)
        self.set_mode_client.wait_for_result()
        return self.set_mode_client.get_result().success

    # Function to Load the Controller for UR Robot
    def load_controller(self, controller_name):
        srv = LoadControllerRequest()
        srv.name = controller_name
        result = self.load_controllers_client(srv)
        print(result)
      
    # Function to Switch the Controller for UR Robot
    def switch_on_controller(self, controller_name):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""
        srv = SwitchControllerRequest()
        srv.stop_controllers = ALL_CONTROLLERS
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controllers_client(srv)
        print(result)

    # Function to Power OFF of UR Robot When the Program is over
    def finalize(self):
        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)

    def square(self):
        # pose list
        pose_list = [[0.8, 0.0, 0.5, 180.0, 0.0, 90.0, 5.0],
                [0.8, 0.3, 0.5, 180.0, 0.0, 90.0, 10.0],
                [0.8, 0.0, 0.8, 180.0, 0.0, 90.0, 15.0],
                [0.8, -0.3, 0.5, 180.0, 0.0, 90.0, 20.0],
                [0.8, 0.0, 0.2, 180.0, 0.0, 90.0, 25.0],
                [0.8, 0.0, 0.5, 180.0, 0.0, 90.0, 30.0]]

        self.run(pose_list)

    def pick_and_place(self):
        # pose list
        pose_list=[[0.9, 0.0, 0.5, 180.0, 0.0, 90.0,5.0],
            [0.1, 1.0, 0.5, 180.0, 0.0, 90.0,15.0],[0.1, 1.0, 0.1, 180.0, 0.0, 90.0,18.0],
            [0.1, 1.0, 0.5, 180.0, 0.0, 90.0,23.0],[0.9, 0.0, 0.5, 180.0, 0.0, 90.0,31.0],
            [0.1, -1.0, 0.5, 180.0, 0.0, 90.0,40.0],[0.1, -1.0, 0.1, 180.0, 0.0, 90.0,43.0],
            [0.1, -1.0, 0.5, 180.0, 0.0, 90.0,46.0],[0.9, 0.0, 0.5, 180.0, 0.0, 90.0,53.0]]
        

        self.run(pose_list)

    def circle(self):
        # pose list
        # pose_list=[[0 for i in range(7)]for j in range(38)]
        # pose_list[0]=[0.6, 0.0, 0.5, 180.0, 0.0, 90.0,5.0]
        # for x in range(1,37,1):
        #     deg=x-1
        #     rad=deg*10
        22#     pose_list[x][0]=0.6+0.2*math.cos(math.radians(rad*180/3.14))
        #     pose_list[x][1]=

        
        pose_list = []
        pose_list.append([])
        pose_list[0].append(0.6)
        pose_list[0].append(0.0)
        pose_list[0].append(0.5)
        pose_list[0].append(180.0)
        pose_list[0].append(0.0)
        pose_list[0].append(90.0)
        pose_list[0].append(2.0)
        for i in range(1, 38, 1):
            pose_list.append([])
            pose_list[i].append(0.6+0.2*math.cos(math.radians(i*10)))
            pose_list[i].append(0.2*math.sin(math.radians(i*10)))
            pose_list[i].append(0.5)
            pose_list[i].append(180.0)
            pose_list[i].append(0.0)
            pose_list[i].append(90.0)
            pose_list[i].append(2+i*2)
        pose_list.append([])
        pose_list[38].append(0.6)
        pose_list[38].append(0.0)
        pose_list[38].append(0.5)
        pose_list[38].append(180.0)
        pose_list[38].append(0.0)
        pose_list[38].append(90.0)
        pose_list[38].append(80.0)
        
        #pose_list=[[0.6, 0.0, 0.5, 180.0, 0.0, 90.0,3.0],[0.8, 0.0, 0.5, 180.0, 0.0, 90.0,6.0],[0.796, 0.0347, 0.5, 180.0, 0.0, 90.0,9.0],
        # [0.787, 0.0684, 0.5, 180.0, 0.0, 90.0,12.0],[0.773, 0.1, 0.5, 180.0, 0.0, 90.0,15.0],[0.753, 0.1285, 0.5, 180.0, 0.0, 90.0,18.0],
        # [0.728, 0.1532, 0.5, 180.0, 0.0, 90.0,21.0],[0.7, 0.1732, 0.5, 180.0, 0.0, 90.0,24.0],[0.6684, 0.17879, 0.5, 180.0, 0.0, 90.0,27.0],
        # [0.63, 0.19696, 0.5, 180.0, 0.0, 90.0,30.0],[0.6, 0.2, 0.5, 180.0, 0.0, 90.0,33.0],[0.565, 0.19696, 0.5, 180.0, 0.0, 90.0,45.0],
        # [0.5315, 0.17879, 0.5, 180.0, 0.0, 90.0,39.0],[0.5, 0.1732, 0.5, 180.0, 0.0, 90.0,42.0],[0.471, 0.1532, 0.5, 180.0, 0.0, 90.0,26.0],
        # [0.4467, 0.1285, 0.5, 180.0, 0.0, 90.0,48.0],[0.4267, 0.1, 0.5, 180.0, 0.0, 90.0,51.0],[0.412, 0.0684, 0.5, 180.0, 0.0, 90.0,54.0],
        # [0.403, 0.0347, 0.5, 180.0, 0.0, 90.0,57.0],[0.4, 0.0, 0.5, 180.0, 0.0, 90.0,60.0],[0.403, -0.0347, 0.5, 180.0, 0.0, 90.0,63.0],
        # [0.412, -0.0684, 0.5, 180.0, 0.0, 90.0,66.0],[0.4267, -0.1, 0.5, 180.0, 0.0, 90.0,69.0],[0.4467, -0.1285, 0.5, 180.0, 0.0, 90.0,72.0]
        # ,[0.471, -0.1532, 0.5, 180.0, 0.0, 90.0,75.0],[0.5, -0.1732, 0.5, 180.0, 0.0, 90.0,78.0],[0.5315, -0.17879, 0.5, 180.0, 0.0, 90.0,81.0]
        # ,[0.565, -0.19696, 0.5, 180.0, 0.0, 90.0,83.0],[0.6, -0.2, 0.5, 180.0, 0.0, 90.0,86.0],[0.63, -0.19696, 0.5, 180.0, 0.0, 90.0,89.0]
        # ,[0.6684, -0.17879, 0.5, 180.0, 0.0, 90.0,92.0],[0.7, -0.1732, 0.5, 180.0, 0.0, 90.0,95.0],[0.728, -0.1532, 0.5, 180.0, 0.0, 90.0,98.0]
        # ,[0.753, -0.1285, 0.5, 180.0, 0.0, 90.0,101.0],[0.773, -0.1, 0.5, 180.0, 0.0, 90.0,104.0],[0.787, -0.0684, 0.5, 180.0, 0.0, 90.0,107.0]
        # ,[0.796, -0.0347, 0.5, 180.0, 0.0, 90.0,110.0],[0.8, 0.0, 0.5, 180.0, 0.0, 90.0,113.0],[0.6, 0.0, 0.5, 180.0, 0.0, 90.0,117.0]]
        self.run(pose_list)

        #print(pose_list)2
            
    def object_observation(self):
        # pose list

        self.run(pose_list)

    def run(self, pose_list):    
        self.move_flag = True
        for l in pose_list:
            # check cartesian limit to 1.0 m of the 3d circle
            check = self.check_cartesian_limit(l[0], 
                                        l[1], 
                                        l[2],
                                        l[3],
                                        l[4],
                                        l[5])
            if check == False: 
                print("cartesian_limit error")
                print("the error pose is")
                print(l)
                return

        # Call Cartesian Trajectory Function with Changed Pose Parameters
        self.cartesian_traj(pose_list)
        self.move_flag = False

# Main, We name the node's name here and Create UR Instance
if __name__ == '__main__':
    rospy.init_node('ur_ros_cartesian_control_node') 
    try:
        ur = UR()
    except rospy.ROSInterruptException: pass
