"""Independent simulator relay; no hardware transport."""
import time
import math
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

def main():
    rclpy.init()
    n=rclpy.create_node("evasion_watchdog")
    last=None
    scan=None
    out=n.create_publisher(Twist,"/autonomymind_gazebo_only/cmd_vel",10)
    def command(m):
        nonlocal last
        last=(m,time.monotonic())
    def observe(m):
        nonlocal scan
        scan=(m,time.monotonic())
    def tick():
        now=time.monotonic()
        result=Twist()
        if last and scan and now-last[1]<0.30 and now-scan[1]<0.60:
            m=last[0];s=scan[0]
            vals=[min(r,s.range_max) for i,r in enumerate(s.ranges) if abs(s.angle_min+i*s.angle_increment)<0.8 and (r==math.inf or s.range_min<=r<=s.range_max)]
            if vals and min(vals)>0.65 and 0<=m.linear.x<=0.25 and abs(m.angular.z)<=0.55:
                result.linear.x=m.linear.x;result.angular.z=m.angular.z
        out.publish(result)
    subs=[n.create_subscription(Twist,"/evasion/approved_pulse",command,10),
          n.create_subscription(LaserScan,"/mobile/scan",observe,qos_profile_sensor_data)]
    timer=n.create_timer(0.05,tick)
    try:rclpy.spin(n)
    finally:
        out.publish(Twist());n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
