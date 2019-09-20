#!/usr/bin/env python 
import sys
import rospy
import numpy as np
import tf
import math
import actionlib  

from actionlib_msgs.msg import * 
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetPlan, GetPlanRequest
from geometry_msgs.msg import TransformStamped, Point, Quaternion, Twist 
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal  

class NavTest():
    def __init__(self):
        rospy.init_node('auto_explore', anonymous=True)
        self.get_map = rospy.ServiceProxy('/dynamic_map', GetMap)
        # self.get_costmap = rospy.ServiceProxy('/map_service', GetMap)
        self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.pub_visual = rospy.Publisher('visualize_map', OccupancyGrid, queue_size=10)
        self.listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  

        
        
        initial_pose = PoseWithCovarianceStamped()
        while initial_pose.header.stamp == "":  
            rospy.sleep(1) 
        print(initial_pose.header.stamp)
        
        self.unavalable_target = {}
        self.abortFlag = False
        # get the initial map
        self.update_map()
        # while not rospy.is_shutdown():
        #     self.update_map()
        #     self.visualize_map(0,0)
        # rospy.spin() 

        # self.visualize_map(self.grid_x, self.grid_y, 30, "search field")
        # self.excute_plan()
        # print(self.grid2pose(500,6000,90))

        # thita = tf.transformations.euler_from_quaternion(self.get_position()[1])[2]
        # print(self.get_position())
        # print(self.get_position()[1])
        # print(type(tf.transformations.euler_from_quaternion(self.get_position()[1])))
        # print(tf.transformations.euler_from_quaternion(self.get_position()[1])[2])
        # print(Quaternion(list(tf.transformations.quaternion_from_euler(0,0,thita).reshape(-1,4)[0])))
        # rospy.spin()
        r = 0
        (last_trans,rot) = self.get_position()
        while not rospy.is_shutdown():

            # To check if the robot move or not
            (trans,rot) = self.get_position()
            if self.isclose(trans[0], last_trans[0]) and self.isclose(trans[1], last_trans[1]):
                pass
            else:
                self.update_map()
                r = 0
                last_trans = trans
                self.unavalable_target = {}

            # if not self.abortFlag:
            #     self.update_map()
            #     self.unavalable_target = {}
            
            goals = []
            bestplan = float("inf")
            while bestplan == float("inf"):
                while goals == [] or noBestGoalFlag:
                    r += 10
                    self.visualize_map(self.grid_x, self.grid_y, r, "search field")
                    goals = self.search_boundary(self.grid_x, self.grid_y, r)
                    noBestGoalFlag = False

                bestgoal = goals[0]
                print('Trying to find the best goal...')
                for goal in goals:
                    new_plan = self.get_plan(goal[0],goal[1])
                    # print new_plan
                    if bestplan > new_plan and new_plan > 0:
                        bestgoal = goal
                        bestplan = new_plan
                print(bestplan)
                if bestplan == float("inf"):
                    print('No best goal exist, search field increasing...')
                    noBestGoalFlag = True
                else:
                    print('Best goal finded.')
            
            self.visualize_map(bestgoal[0], bestgoal[1])
            # print(self.get_position()[1])
            # print(thita)
            # print(self.grid2pose(bestgoal[0],bestgoal[1],thita))
            self.excute_plan(bestgoal[0],bestgoal[1])
        # print("Path Length:",self.get_plan(308,244))

        rospy.spin()

    def excute_plan(self,target_x, target_y):
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']
        thita = tf.transformations.euler_from_quaternion(self.get_position()[1])[2]
        goal_pose =  self.grid2pose(target_x,target_y,thita)
  
        self.goal = MoveBaseGoal()  
        self.goal.target_pose.pose = goal_pose
        self.goal.target_pose.header.frame_id = 'map'  
        self.goal.target_pose.header.stamp = rospy.Time.now()  

        rospy.loginfo("Going to: " + str(goal_pose.position.x) + ',' + str(goal_pose.position.y))  

        self.move_base.send_goal(self.goal)
        
        # finished_within_time = self.move_base.wait_for_result(rospy.Duration(50))   
        start_time = rospy.Time.now()
        running_time = rospy.Time.now() - start_time
        
        finish_flag = False
        while running_time.secs <= 20 and self.move_base.get_state() in [0,1,6,7,9]:
            running_time = rospy.Time.now() - start_time
            (trans,rot) = self.get_position()
            if self.isclose(trans[0], goal_pose.position.x) and self.isclose(trans[1], goal_pose.position.y):
                finish_flag = True
                self.move_base.cancel_goal()  

        if running_time.secs > 20:  
            self.move_base.cancel_goal()
            self.abortFlag = False  
            rospy.loginfo("Timed out achieving goal")  
        else:  
            state = self.move_base.get_state()  
            if state == GoalStatus.SUCCEEDED or finish_flag:  
                self.abortFlag = False  
                rospy.loginfo("Goal succeeded!") 
            else: 
                if state ==  GoalStatus.ABORTED:
                    self.abortFlag = True
                    self.unavalable_target[(target_x,target_y)] = 1
                rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
    
    def isclose(self, a, b, rel_tol= 0.10):
        return abs(a-b) <= rel_tol

    def get_plan(self, x, y):
        planner = GetPlanRequest()
        mbot_position = self.get_position()
        
        # Set planner start pose
        planner.start.pose.position.x = mbot_position[0][0]
        planner.start.pose.position.y = mbot_position[0][1]
        planner.start.pose.position.z = mbot_position[0][2]
        planner.start.pose.orientation.x = mbot_position[1][0]
        planner.start.pose.orientation.y = mbot_position[1][1]
        planner.start.pose.orientation.z = mbot_position[1][2]
        planner.start.pose.orientation.w = mbot_position[1][3]
        planner.start.header.frame_id = 'map'
        planner.tolerance = 0
        
        # Set planner goal pose
        planner.goal.pose.orientation = planner.start.pose.orientation
        planner.goal.pose.position = self.grid2pose(x,y,0).position
        planner.goal.header.frame_id = 'map'
       
        # Get the plan to the goal
        path = self.make_plan(planner)
        return len(path.plan.poses)

    def grid2pose(self, x, y, thita):
        target = Pose()
        target.position.x = x * self.resolution + self.origin_x
        target.position.y = y * self.resolution + self.origin_y
        target.orientation.x = tf.transformations.quaternion_from_euler(0,0,thita)[0]
        target.orientation.y = tf.transformations.quaternion_from_euler(0,0,thita)[1]
        target.orientation.z = tf.transformations.quaternion_from_euler(0,0,thita)[2]
        target.orientation.w = tf.transformations.quaternion_from_euler(0,0,thita)[3]
        return target

    def search_boundary(self, x, y, r):
        search_points = []
        for i in range(x-r, x+r+1):
            for j in range(y-r, y+r+1):
                if ( max(r-10,5) <= ((i-x)**2+(j-y)**2)**0.5 <= r and self.map_data[j,i] == -1 
                and i <= self.width and j<= self.height) and self.bundary_map[j,i]<50 and (i,j) not in self.unavalable_target:
                    search_points.append([i,j])
        return search_points

    def visualize_map(self, x, y, r = 10, mode = 'position'):
        
        new_map = OccupancyGrid()
        new_map.info = self.map.info
        data = np.copy(self.map_data)
        
        if mode == 'position':
            print("Visuanlize the target position...")
            for i in range(x-5,x+5):
                for j in range(y-5,y+5):
                    data[j,i] = 100
        elif mode == 'search field':
            print("Visuanlize the search field: R =", r)
            for i in range(x-r, x+r+1):
                for j in range(y-r, y+r+1):
                    if ( (r-10) <= ((i-x)**2+(j-y)**2)**0.5 <= r
                     and i <= self.width and j< self.height):
                        data[j,i] = 100
        
        new_map.data = tuple(data.reshape(1,-1)[0])
        rate = rospy.Rate(10) 
        i = 0
        while not rospy.is_shutdown():
            self.pub_visual.publish(new_map)
            rate.sleep()
            i = i+1 
            if i > 3:
                break

    def update_map(self):
        (trans,rot) = self.get_position()
        pos_x, pos_y = trans[:-1]
        self.map = self.get_map().map
        # self.costmap = self.get_costmap().map
        self.width = self.map.info.width
        self.height = self.map.info.height
        self.resolution = self.map.info.resolution
        self.target_map = np.ones((self.height,self.width), dtype=bool)
        self.map_data = np.array(self.map.data).reshape(self.height,self.width)
        # self.costmap_data = np.array(self.costmap.data).reshape(self.height,self.width)
        self.origin_x = self.map.info.origin.position.x
        self.origin_y = self.map.info.origin.position.y
        self.grid_x = int(round((pos_x - self.origin_x)/self.resolution))
        self.grid_y = int(round((pos_y - self.origin_y)/self.resolution))
        self.bundary_map = np.zeros((self.height,self.width))
        
        for i in range(-3,4):
            self.bundary_map += np.roll(self.map_data,i,axis=0) + np.roll(self.map_data,i,axis=1)

    def get_position(self):
        while not rospy.is_shutdown():
            try:
                pose = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return pose

    def callback(self, data):
        rospy.loginfo("Cost map undated!")
        self.waitFlag = True
        self.costmap = data
  

if __name__ == '__main__':  
    try:  
        NavTest()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Auto-explore finished.")