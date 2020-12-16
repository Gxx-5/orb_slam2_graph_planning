#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion,Point,Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8,Empty
import os
import threading
import time 
import numpy as np
from orb_slam2_ros.srv import GetPath
from Quateration import Quaternion as qua
from Quateration import rotate,pose2transform

#import tf_conversions.posemath as pm
# def callback(data):
#     print(data.data)

# rospy.init_node('tracking')

# rospy.Subscriber('/status_orb', Int8, callback)
# rospy.spin()


class HybirdLocalization:
    def __init__(self):
        rospy.init_node('pathtracking')
        rospy.Subscriber('/briq/status_orb', Int8, self.StatusRec)
        rospy.Subscriber('/mobile_base/odom', Odometry, self.OdometryRec)
        rospy.Subscriber('/orb_slam2_rgbd/pose', PoseStamped, self.ORBRec)
        self.lossTrack=False


        self.rest_pub = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty,queue_size = 1)
        self.controlpose_pub = rospy.Publisher('/ControlPose',PoseStamped,queue_size = 1)
        self.odom_msg=Odometry()
        self.status_msg=Int8()

        self.orb_pose=PoseStamped()

        self.finalPose=Pose()#丢失后的位姿
        self.controlPose=Pose()#最终控制位姿
        self.controlPoseStamped=PoseStamped()
    
    def getState(self):
        th=self.quat_to_angle(self.controlPose.orientation)
        print('qua',self.controlPose.orientation)
        state = [self.controlPose.position.x,self.controlPose.position.y,th]
        return np.array(state)


    def getPose(self):
        return self.controlPose

    def OdometryRec(self,data):
        self.odom_msg=data

    def StatusRec(self,data):
        self.status_msg=data
        if self.lossTrack==False and self.status_msg.data==3:#丢失切换
            self.finalPose=self.orb_pose.pose
            self.lossTrack=True
            self.rest_pub.publish()

        if self.lossTrack==True and self.status_msg.data==2:#恢复切换
            self.finalPose=self.orb_pose.pose
            self.lossTrack=False
            self.rest_pub.publish()

        if self.status_msg.data==2:#正常跟踪时
            self.controlPose=self.orb_pose.pose
            self.finalPose=self.orb_pose.pose

        if self.status_msg.data==3:#跟踪持续丢失时
            self.controlPose=self.addPose(self.finalPose,self.odom_msg.pose.pose)

        self.controlPoseStamped.pose=self.controlPose
        self.controlPoseStamped.header.frame_id='map'
        self.controlPoseStamped.header.stamp = rospy.get_rostime()
        self.controlpose_pub.publish(self.controlPoseStamped)

    def ORBRec(self,data):
        self.orb_pose=data

    def quat_to_angle(self,quat):
        w = float(quat.w)
        x = float(quat.x)
        y = float(quat.y)
        z = float(quat.z)
        alpha = np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y))
        angle = np.arctan2(2*(w*z+x*y),1-2*(z*z+y*y))
        angle = angle*180/np.pi
        alpha = alpha*180/np.pi
        angle= np.deg2rad(angle+180)
        return angle

    def _mulQ(self,a,b):
        r1=a.orientation.w
        r2=b.orientation.w
        v1=np.array([a.orientation.x,a.orientation.y,a.orientation.z])
        v2=np.array([b.orientation.x,b.orientation.y,b.orientation.z])
        r=r1*r2-np.dot(v1,v2)
        v=r1 * v2 + r2 * v1 + np.cross(v1, v2)
        res=np.array([r,v[0],v[1],v[2]])
        return res

    def addPose(self,a,b):
        c=Pose()
        try:
            res = self._mulQ(a,b)
            qr=qua(a.orientation.w,a.orientation.x,a.orientation.y,a.orientation.z)
            c.orientation.w = res[0]
            c.orientation.x = res[1]
            c.orientation.y = res[2]
            c.orientation.z = res[3]
            t=rotate(qr,b.position.x,b.position.y,b.position.z)
            c.position.x = a.position.x+t[0]
            c.position.y = a.position.y+t[1]
            c.position.z = a.position.z+t[2]
            return c
        except:
            return a

class Tracking(HybirdLocalization):
    def __init__(self):
        super(Tracking, self).__init__()
        self.pathservice = rospy.ServiceProxy('/briq/get_path', GetPath)
        self.path=[]
        self.tracking=False
        self.tracking_idx=-1
        self.movepub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    def getPath(self):
        goal=Point()
        res=self.pathservice(goal,True)
        path=[]
        for r in res.path:
            path.append([r.x,r.y,r.z])
        return path

    # def getPath(self,goal):
    #     goal=Point()
    #     res=self.pathservice(goal,False)
    #     path=[]
    #     for r in res:
    #         path.append([r.x,r.y,r.z])#goal->...->init
    #     return path
    
    def checkReach(self,target_idx,status,path,TH=0.17):

        dist=np.linalg.norm(path[target_idx][0:2] - status[0:2],ord=2)
        if dist<TH :
            return True
        else:
            return False

    def TrackingInit(self,path):
        self.path=path
        self.tracking=True
        self.tracking_idx=len(self.path)-1
        
    def Tracking(self,status):
        if self.tracking_idx==-1:
            return False
        if self.tracking_idx==0:
            reach=self.checkReach(self.tracking_idx,status,self.path)
            if reach==True :
                self.control(0,stop=True)
                # self.tracking=False
                self.path.reverse()
                self.tracking_idx=len(self.path)-1
                return True
        else:
            reach=self.checkReach(self.tracking_idx,status,self.path)

            
            target_angle=self.get_angle(self.path[self.tracking_idx],status)
            now_angle=np.rad2deg(status[2])
            error=self.angle_diff(now_angle,target_angle)
            print('err',now_angle,target_angle,error,
            self.tracking_idx,
            len(self.path),
            self.path[self.tracking_idx]
            )
            self.control(-error,stop=False)
        if reach==True :
            self.tracking_idx=self.tracking_idx-1
        return False

    def control(self,diff_angle,stop):
        RP=0.02
        LP=0.025
        target_speed=0.21
        if stop==True:
            self.base_control(line=0,ang=0)
        else:
            if abs(diff_angle)>70:
                self.base_control(line=0,ang=RP*diff_angle)
            else:
                self.base_control(line=target_speed,ang=LP*diff_angle)

    def base_control(self,line=0,ang=0):
        twist = Twist()
        if ang>1:ang=1.3
        if ang<-1:ang=-1.3
        twist.linear.x = line; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = ang
        self.movepub.publish(twist)
        print(twist)
    # def tracking(self,path,status,reset=False):
    #     if self.nowTarget is None:
    #         self.nowTarget=len(path)-1
    def angle_diff(self,a,b):
        # a=np.rad2deg(a)
        # b=np.rad2deg(b)
        d1=a-b
        d2=360-abs(d1)
        if d1>0:
            d2*=-1.0
        a=[d1,d2]
        return min(a,key=lambda o: abs(o))

    def get_angle(self,me,goal):
        tmp = np.rad2deg(np.arctan2(goal[1]-me[1],goal[0]-me[0]))
        if tmp<0:
            tmp=360-abs(tmp)
        return tmp


    # def track()
    # # def projection2camera(self,point,pose,K):
        


            
        
    # def deltamodel(self,control,state,dT=0.1):
    #     v,w=control[0],control[1]
    #     x1,y1,th1=state[0],state[1],state[2]
    #     dth=w*dT
    #     dx=v*np.cos(th1+(dth/2))*dT
    #     dy=v*np.sin(th1+(dth/2))*dT
    #     th=th1+dth
    #     x=x1+dx
    #     y=y1+dy
    #     return np.array([x,y,th])

    # def modelpredict(self,controlList,state):
    #     stateList=[]
    #     for control in controlList:
    #         state=self.deltamodel(control,state)
    #         stateList.append(state)
    #     return stateList

    # def costFunction(self,nowState,states,controlList,path,target_speed=[0.2,1.0]):
    #     nearpath=[]
        
    #     for p in path:
    #         dist=np.linalg.norm(p[0:2]-nowState[0:2],ord=2)
    #         nearpath.append([dist,p[0],p[1]])
    #     neastpath = sorted(range(len(nearpath)), key=lambda d:d[0], reverse = False)
    #     if len(nearpath)>3
    #         refpath=neastpath[0:2]
    #     else:
    #         refpath=[neastpath[0]]
    #     traj_cost=0
    #     for s in states:
    #         for rp in refpath:
    #             traj_cost+=np.linalg.norm(rp[1:3]-s[0:2],ord=2)
    #     traj_cost=traj_cost/float(len(refpath))
        
    #     target_cost = np.linalg.norm(states[-1][0:2] - path[0][0:2],ord=2)
        
    #     speed_cost=0
    #     for c in controlList:
    #         speed_cost+=np.linalg.norm(target_speed - c,ord=2)
        
    #     speed_cost=speed_cost/float(len(controlList))

    #     slim_cost=(np.var(controlList[:,0])+np.var(controlList[:,1]))/2.0
    #     w=[1,1,0.5,0.3]
    #     cost= w[0]*traj_cost + w[1]*target_cost + w[2]*speed_cost + w[3]*slim_cost
    #     return cost


    # def PSO_OptimalControl(self,init=True,path,lastControl=[0,0],v_sigma=0.2,w_sigma=1.0):
    #     nowstate=self.getState()
    #     controlSample=[]
        
    #     for s in range(50):
    #         controlList=[]
    #         for _ in range(3):
    #             if init==True:
    #                 v=np.random.normal(0.0,v_sigma)
    #                 w=np.random.normal(0.0,w_sigma)
    #             else:
    #                 v=np.random.normal(lastControl[0],v_sigma)
    #                 w=np.random.normal(lastControl[1],w_sigma)
    #             controlList.append([v,w])
    #         controlSample.append(controlList)
        
    #     for cl in controlSample:
    #         statelist=self.modelpredict(cl,nowstate)
    #         cost=self.costFunction(nowstate,statelist,cl,path)

# class Tracking(HybirdLocalization):
#     def __init__(self):
#         super.__init__()
#         self.getpath = rospy.ServiceProxy('/briq/get_path', GetPath)
    
#     def PathPlanning(self,goal):
        

import sys, select, termios, tty
        
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

import signal

def sigint_handler(signum, frame):
    print('exit')
    exit()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGHUP, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)
    tracking=Tracking()
    time.sleep(2)
    tracking.rest_pub.publish()
    settings = termios.tcgetattr(sys.stdin)
    while 	rospy.is_shutdown()==False:
        
        key = getKey()
        os.system("clear")
        if key=='p':
            path=tracking.getPath()
            tracking.TrackingInit(path)
        else:
            if tracking.tracking==True:
                tracking.Tracking(tracking.getState())
        if key=='q':
            tracking.control(0,stop=True)
            exit()
            break
        if key=='s':
            tracking.tracking=False
            tracking.control(0,stop=True)
        print('state',tracking.getState())
       
        # time.sleep(0.01)
