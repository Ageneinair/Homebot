#!/usr/bin/env python 

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetMapResponse
from map_msgs.msg import OccupancyGridUpdate

class MapService(object):

    def __init__(self):
        self.data = GetMapResponse()
        a = OccupancyGridUpdate()
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.callback)
        self.waitFlag = False
        while not self.waitFlag:
            pass
        self.waitFlag = False
        rospy.Subscriber("/move_base/global_costmap/costmap_update", OccupancyGridUpdate, self.callback2)
        s = rospy.Service('/map_service', GetMap, self.handleMapService)
        rospy.loginfo("Costmap service have been ready!")
        rospy.spin()

    def callback(self, data):
        rospy.loginfo("Cost map updated!")
        self.waitFlag = True
        self.data.map = data

    def callback2(self,data):
        print(1)
        temp_up = np.array(data.data).reshape(data.height,data.width)
        temp_data = np.array(self.data.map.data).reshape(self.data.map.height,self.data.map.width)
        temp_data[data.y:data.y+data.height,data.x,data.x+data.width] = temp_up
        self.data.map.data = tuple(temp_data.reshape(1,-1)[0])

    def handleMapService(self,a):
        rospy.loginfo("Cost map service has been called!")
        return self.data

if __name__ == "__main__":
    try:
        rospy.init_node('map_service', anonymous=True)
        map_service = MapService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
