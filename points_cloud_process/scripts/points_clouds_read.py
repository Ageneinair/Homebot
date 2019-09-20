#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np 
# from scipy import stats 
import tf
import tf2_ros
import tf2_py as tf2
from roslib import message
import sensor_msgs.point_cloud2 as pc2
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
#from transform_point_cloud.cfg import LookupTransformConfig

class PointCloud(object):
    def __init__(self):
        self.xmin=[]
        self.xmax=[]
        self.ymin=[]
        self.ymax=[]
        #self.pcs=[]
        self.outs=[]
        self.newimage=False
        self.epsilon=0.5
        self.firstPoint=[]
        self.listener = tf.TransformListener()
        self.pc = PointCloud2()
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bbcallback)
        rospy.Subscriber("/kinect/depth/points", PointCloud2, self.callback)
        
        self.pub = rospy.Publisher('/pointcloud_recorder', PointCloud2, queue_size=10)


    def bbcallback(self, bboxes):
        self.xmin=[]
        self.xmax=[]
        self.ymin=[]
        self.ymax=[]
        
        bs=bboxes.bounding_boxes
        for bbox in bs:
            if bbox.Class!="person":
                continue
            self.xmin.append(bbox.xmin)
            self.xmax.append(bbox.xmax)
            self.ymin.append(bbox.ymin)
            self.ymax.append(bbox.ymax)
        self.newimage=True
    def callback(self, data):
        self.pc = data
        #print(data.header)
        # print("-------------------")
        # print(type(data), dir(data))
        # print(type(data.data), dir(data.data))
        # msg = data.data
        # print(len(msg))
        # print(data.width)
        # print(data.height)
        # print('!!')
        # print(data.point_step)
        # arr = np.fromstring(msg, dtype=float)
        # print(stats.describe(arr))
        # Xmin=162
        # Xmax= 457
        # Ymin=0
        # Ymax=480
        """tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        try:
            # trans = tf_buffer.lookup_transform('map', data.header.frame_id,
            #                                 rospy.Time())
            trans = tf_buffer.lookup_transform("map","kinect_frame_optical",
                                            rospy.Time())
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(data, trans)
        self.pub.publish(cloud_out)"""
        (trans,rot) = self.get_position()
        theta = tf.transformations.euler_from_quaternion(self.get_position()[1])[2]
        # self.xmin=[0]
        # self.xmax=[639]
        # self.ymin=[0]
        # self.ymax=[479]
        # for pointcloud in self.pcs:
        #     self.pub.publish(pointcloud)
        if not self.newimage: 
            pc=pc2.create_cloud(data.header,data.fields,self.outs)
            pc.header.frame_id='map'
            self.pub.publish(pc)
            return 
        for it in range(len(self.xmin)):
            uvs=[(i,j) for i in range(self.xmin[it],self.xmax[it],1) for j in range(self.ymin[it],self.ymax[it],1)]
            out = list(pc2.read_points(data,uvs=uvs))
            if self.checkobjectalreadyexists(out[0]):
                break
            self.firstPoint.append(out[0])
            for k in range(len(out)):
                
                #val=list(nn-trans[ii] for ii,nn in enumerate(out[k][:3]))
                # val=[out[k][0]+trans[0],-out[k][2]+trans[1],-out[k][1]+trans[2]]
                val=[out[k][0],-out[k][2],-out[k][1]]
                val=[val[0]*np.cos(theta)+val[1]*np.sin(theta),val[1]*np.cos(theta)-val[0]*np.sin(theta),val[2]]
                val=[val[0]+trans[0],-val[1]+trans[1],val[2]+trans[2]]
                val=val+[out[k][3]]
                val=tuple(val)
                out[k]=val
            self.outs+=out
            pc=pc2.create_cloud(data.header,data.fields,self.outs)
            pc.header.frame_id='map'
            self.pub.publish(pc)
        self.newimage=False
    def checkobjectalreadyexists(self,l):
        for pt in self.firstPoint:
            tmp=False
            for i in range(3):
                if abs(pt[i]-l[i])>self.epsilon:
                    tmp=True
            if not tmp:
                return True
            
    def rotate(self,a,b,c,x,y,z):
        v=np.array((x,y,z,1))
        X=np.array(((1, 0, 0, 0),
                    (0,np.cos(a),-np.sin(a),0),
                    (0,np.sin(a),np.cos(a),0),
                    (0,0,0,1)))
        Y=np.array(((np.cos(b), 0, np.sin(b), 0),
                    (0,1, 0,0),
                    (-np.sin(b),0,np.cos(b),0),
                    (0,0,0,1)))
        Z=np.array(((np.cos(c),-np.sin(c),0,0),
                    (np.sin(c),np.cos(c),0,0),
                    (0,0,1,0),
                    (0,0,0,1)))
        v=np.matmul(Z,v)
        v=np.matmul(Y,v)
        v=np.matmul(X,v)
        return v
    def get_position(self):
        while not rospy.is_shutdown():
            try:
                pose = self.listener.lookupTransform('/map', '/kinect_frame_optical', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return pose



if __name__ == '__main__':  
    try:
        rospy.init_node('pointcloud_recorder', anonymous=True)  
        p = PointCloud()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")
