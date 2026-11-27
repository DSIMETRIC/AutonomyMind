"""Independent stationary reference sensor producer, no control output."""
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from autonomymind_runtime.dds_protocol import envelope,encode
from autonomymind_runtime.sensor_evidence import project_scan

def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

class Producer(Node):
    def __init__(self,a):
        super().__init__("sensor_"+a.id)
        self.a=a;self.data={};self.last=-1.
        self.pub=self.create_publisher(String,"/autonomymind_agents/events",QoSProfile(depth=128,reliability=ReliabilityPolicy.RELIABLE))
        self.odom=self.create_subscription(Odometry,a.odom,lambda m:self.save("odom",m),qos_profile_sensor_data)
        self.scan=self.create_subscription(LaserScan,a.scan,lambda m:self.save("scan",m),qos_profile_sensor_data)
        self.timer=self.create_timer(.5,self.tick)
    def save(self,k,m):self.data[k]=(m,time.monotonic())
    def tick(self):
        if len(self.data)!=2 or any(time.monotonic()-v[1]>.6 for v in self.data.values()):return
        o=self.data["odom"][0];s=self.data["scan"][0]
        if stamp(s)<=self.last or abs(stamp(o)-stamp(s))>.35 or o.header.frame_id!="world_enu":return
        p=o.pose.pose.position;q=o.pose.pose.orientation;v=o.twist.twist.linear;w=o.twist.twist.angular
        if not all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z)):return
        if abs(q.x)>.05 or abs(q.y)>.05 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.001 or math.hypot(v.x,v.y,v.z)>.05 or math.hypot(w.x,w.y,w.z)>.05:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        hits=project_scan(s.ranges,s.angle_min,s.angle_increment,s.range_min,s.range_max,(p.x,p.y,p.z,yaw),tuple(self.a.offset))
        # One representative return per 0.25 m cell, capped to 48 nearest cells.
        cells={}
        for i,point in hits:
            cell=tuple(math.floor(x/.25) for x in point)
            distance=math.dist(point,(p.x,p.y,p.z))
            if cell not in cells or distance<cells[cell][0]:cells[cell]=(distance,point)
        now=time.time()
        for cell,(_,point) in sorted(cells.items(),key=lambda x:x[1][0])[:48]:
            payload={"observation_id":":".join(map(str,cell)),"timestamp_s":now,"frame_id":"world_enu","position_m":point,"velocity_mps":[0.,0.,0.],"velocity_known":False,"confidence":1.,"uncertainty_m":.2,"label":"lidar_surface_return","simulation_stamp_s":stamp(s),"source":"gazebo_gpu_lidar"}
            self.pub.publish(String(data=encode(envelope(self.a.id,"*","world.observation",payload,now,2))))
        self.last=stamp(s)

def main():
    p=argparse.ArgumentParser();p.add_argument("--id",required=True);p.add_argument("--odom",required=True);p.add_argument("--scan",required=True);p.add_argument("--offset",type=float,nargs=3,required=True);a=p.parse_args()
    rclpy.init();n=Producer(a)
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
