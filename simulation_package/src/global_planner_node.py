#!/usr/bin/python3

import numpy as np
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,PoseStamped
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import Odometry, OccupancyGrid,Path
from main_functions_planner import StateValidityChecker,compute_path,move_to_point


class global_planner:

    # Global_planner Constructor
    def __init__(self, gridmap_topic, odom_topic,cmd_vel_topic, dominion, distance_threshold):

        # ATTRIBUTES
        # List of points which define the plan. None if there is no plan
        self.path = []
        # State Validity Checker object                                                 
        self.svc = StateValidityChecker(distance_threshold)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None
        # Goal where the robot has to move, None if it is not set                                                                   
        self.goal = None
        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                           
        self.dominion = dominion    

        self.check = False                                   

        # CONTROLLER PARAMETERS
        # Proportional linear velocity controller gain
        self.Kv = 0.5
        # Proportional angular velocity controller gain                   
        self.Kw = 0.5
        # Maximum linear velocity control action                   
        self.v_max = 0.15
        # Maximum angular velocity control action               
        self.w_max = 0.3      
        # Number of iterations
        self.k=50000
        #  Step between points   
        self.step_size=.5
        # Turning radius
        self.turning_radius=10
        # Prbability of growing towards the goal
        self.p=0.7
        # # search distance
        # self.r=30
        self.delta_q=70


        # PUBLISHERS
        # Publisher for sending velocity commands to the robot
        self.cmd_pub =rospy.Publisher(cmd_vel_topic,Twist, queue_size=1)#: publisher to cmd_vel_topic
        # Publisher for visualizing the path to with rviz
        self.marker_pub = rospy.Publisher("/path_marker", Marker, queue_size=1)
        self.path_pub= rospy.Publisher('/path', Path, queue_size=1)

        
        # SUBSCRIBERS
        self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid,self.get_gridmap)#: subscriber to gridmap_topic from Octomap Server  
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry,self.get_odom) # TODO: subscriber to odom_topic  
        self.move_goal_sub =rospy.Subscriber('/move_base_simple/goal', PoseStamped,self.get_goal) # : subscriber to /move_base_simple/goal published by rviz    
        
        # TIMERS
        # Timer for velocity controller
        # rospy.Timer(rospy.Duration(0.1), self.controller)
        print('waiting for a goal...')


    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    

    # Goal callback: Get new goal from /move_base_simple/goal topic published by rviz 
    # and computes a plan to it using self.plan() method
    def get_goal(self, goal):
        if self.svc.there_is_map:
            print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
             # to send zero velocity while planning
            self.path = None                                                   
            self.path = self.plan()
    

    ## Map callback:  Gets the latest occupancy map published by Octomap server and update 
    # the state validity checker
    def get_gridmap(self, gridmap):
        # to avoid map update too often (change value if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 3:            
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)
            # If the robot is following a path, check if it is still valid
            if self.path is not None and len(self.path) > 0:
                # create total_path adding the current position to the rest of waypoints in the path
                total_path = [np.array([self.current_pose[0],self.current_pose[1]])] + self.path
                # TODO: check total_path validity. If total_path is not valid make self.path = None and replan
                if self.svc.check_path(total_path)==False:
                    self.path=None
                    self.path=self.plan()  
                    if not (math.isclose(self.path[-1][0],self.goal[0])) and not (math.isclose(self.path[-1][1],self.goal[1])):
                        self.path=None
                        self.path=self.plan()  
               


    # Solve plan from current position to self.goal. 
    def plan(self):
        # List of waypoints [x, y] that compose the plan
        path = []
        trial = 0
        # If planning fials, allow replanning for several trials
        while len(path) == 0 and trial < 5:
            print("Compute new path")
            # plan a path from self.current_pose to self.goal
            path = compute_path(self.current_pose,self.goal,self.k,self.step_size,self.turning_radius, self.delta_q,self.p,self.svc)
            # path = compute_path(self.current_pose,self.goal,self.svc,self.dominion,0.5)
            trial += 1  
        if trial == 5:
            # If planning fails, consider increasing the planning time
            print("Path not found!")
        else:
            print("Path found")
            # Publish plan marker to visualize in rviz
            self.publish_path(path)
            self.publish_path_to_controller(path)
            # remove initial waypoint in the path (current pose is already reached)
            del path[0]                 
        return path

    
    # This method is called every 0.1s. It computes the velocity comands in order to reach the 
    # next waypoint in the path. It also sends zero velocity commands if there is no active path.
    # def controller(self, event):
    #     v = 0
    #     w = 0
    #     if self.path is not None and len(self.path) > 0:
    #         # If current wait point reached with some tolerance move to next way point, otherwise move to current point
    #         if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < 2*self.svc.resolution:
    #             print("Position {} reached".format(self.path[0]))
    #             del self.path[0]
    #             if len(self.path) == 0:
    #                 err=np.linalg.norm(self.goal- self.current_pose[0:2]) 
    #                 if err > 2*self.svc.resolution: 
    #                     print('not yet, be patient')
    #                     self.path = None
    #                     self.path = self.plan()
    #                 else:
    #                     self.goal = None
    #                     print("Final position reached! ,ready for a new goal ^_^")
    #         else:
    #             # TODO: Compute velocities using controller function in utils_lib
    #             v,w=move_to_point(self.current_pose,self.path[0],self.Kv,self.Kw)
               
        
    #     # Publish velocity commands
    #     self.__send_commnd__(v, w)

    # PUBLISHER HELPERS

    # Transform linear and angular velocity (v, w) into a Twist message and publish it
    # def __send_commnd__(self, v, w):
    #     cmd = Twist()
    #     cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
    #     cmd.linear.y = 0
    #     cmd.linear.z = 0
    #     cmd.angular.x = 0
    #     cmd.angular.y = 0
    #     cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
    #     self.cmd_pub.publish(cmd)

    # Publish a path as a series of line markers
    def publish_path(self, path):
        if len(path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 0
            color_red.g = 1
            color_red.b = 0
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)

    def publish_path_to_controller(self,path):
        """
        Publish the ROS message containing the path
        """
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()
        
        for p in path:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.orientation.w=1
            msg.poses.append(pose)


        self.path_pub.publish(msg)


# MAIN FUNCTION
if __name__ == '__main__':
    rospy.init_node('global_planner_node')   
    node = global_planner('/projected_map', '/odom','/cmd_vel', np.array([-15.0, 15.0]), .2)
    # Run forever
    rospy.spin()
