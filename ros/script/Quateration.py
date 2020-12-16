# -*- coding: utf-8 -*
"""
Created on March 2019
@author: yukino
@description：Create a Quaternion class
"""
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion,Point
 
class Quaternion:
    def __init__(self, s, x, y, z):
        """构造函数"""
        self.s = s
        self.x = x
        self.y = y
        self.z = z
        self.vector = [x, y, z]
        self.all = [s, x, y, z]
 
    def __str__(self):
        """输出操作重载"""
        op = [" ", "i ", "j ", "k"]
        q = self.all.copy()
        result = ""
        for i in range(4):
            if q[i] < -1e-8 or q[i] > 1e-8:
                result = result + str(round(q[i], 4)) + op[i]
        if result == "":
            return "0"
        else:
            return result
 
    def __add__(self, quater):
        """加法运算符重载"""
        q = self.all.copy()
        for i in range(4):
            q[i] += quater.all[i]
        return Quaternion(q[0], q[1], q[2], q[3])
 
    def __sub__(self, quater):
        """减法运算符重载"""
        q = self.all.copy()
        for i in range(4):
            q[i] -= quater.all[i]
        return Quaternion(q[0], q[1], q[2], q[3])
 
    def __mul__(self, quater):
        """乘法运算符重载"""
        q = self.all.copy()
        p = quater.all.copy()
        s = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3]
        x = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2]
        y = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1]
        z = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0]
        return Quaternion(s, x, y, z)
 
    def divide(self, quaternion):
        """右除"""
        result = self * quaternion.inverse()
        return result
 
    def modpow(self):
        """模的平方"""
        q = self.all.copy()
        return sum([i**2 for i in q])
 
    def mod(self):
        """求模"""
        return pow(self.modpow(), 1/2)
 
    def conj(self):
        """转置"""
        q = self.all.copy()
        for i in range(1, 4):
            q[i] = -q[i]
        return Quaternion(q[0], q[1], q[2], q[3])
 
    def inverse(self):
        """求逆"""
        q = self.all.copy()
        mod = self.modpow()
        for i in range(4):
            q[i] /= mod
        return Quaternion(q[0], -q[1], -q[2], -q[3])

def rotate(q,x,y,z):
    p=Quaternion(0,x,y,z)
    res=q*p*q.inverse()
    return res.vector

def pose2transform(pose):
    tx,ty,tz = pose.position.x,pose.position.y,pose.position.z

    w,rx,ry,rz = pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z

    transform=[
        [1-2*ry*ry-2*rz*rz,   2*rx*ry+2*w*z,       2*rx*rz-2*rw*ry,                   tx],
        [2*rx*ry - 2*rw*rz, 1-2*rx*rx-2*rz*rz,  2*ry*rz+2*rw*rx,                ty],
        [2*rx*rz+2*rw*ry,   2*ry*rz - 2*rw*rx,     1-2*rx*rx-2*ry*ry,   tz],
        [0,0,0,1]
    ]
    rotate =  [ [1-2*ry*ry-2*rz*rz,   2*rx*ry+2*rw*rz,       2*rx*rz-2*rw*ry],
                        [2*rx*ry - 2*rw*rz, 1-2*rx*rx-2*rz*rz,  2*ry*rz+2*rw*rx],
                        [2*rx*rz+2*rw*ry,   2*ry*rz - 2*rw*rx,     1-2*rx*rx-2*ry*ry]]
    return np.array(transform)
