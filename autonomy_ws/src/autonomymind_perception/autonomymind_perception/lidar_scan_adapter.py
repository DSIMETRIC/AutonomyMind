import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from autonomymind_interfaces.msg import Observation
from .common import make_observation

class LidarScanAdapter(Node):
    def __init__(self):
        super().__init__("autonomymind_lidar_scan_adapter")
        self.declare_parameter("input_topic","/scan")
        self.declare_parameter("output_topic","/autonomymind/observations/lidar")
        self.declare_parameter("source","lidar")
        self.declare_parameter("min_range_m",0.05)
        input_topic=self.get_parameter("input_topic").value
        output_topic=self.get_parameter("output_topic").value
        self.source=str(self.get_parameter("source").value)
        self.min_range=float(self.get_parameter("min_range_m").value)
        self.pub=self.create_publisher(Observation,output_topic,20)
        self.sub=self.create_subscription(LaserScan,input_topic,self.on_scan,20)

    def on_scan(self,msg: LaserScan):
        valid=[]
        for i,r in enumerate(msg.ranges):
            rv=float(r)
            if math.isfinite(rv) and rv >= max(self.min_range,float(msg.range_min)) and rv <= float(msg.range_max):
                valid.append((rv,i))
        if not valid:
            return
        distance,index=min(valid,key=lambda x:x[0])
        angle=float(msg.angle_min)+float(index)*float(msg.angle_increment)
        x=distance*math.cos(angle)
        y=distance*math.sin(angle)
        stamp=float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
        obs=make_observation(
            prefix="lidar",agent_id="perception_lidar_adapter",timestamp_s=stamp,
            frame_id=msg.header.frame_id or "lidar",source=self.source,kind="detection",
            confidence=1.0,position=[x,y,0.0],velocity=[0.0,0.0,0.0],label="nearest_obstacle",
            provenance={"producer":"perception_lidar_adapter","source_type":"sensor"},
            attributes={"distance_m":distance,"bearing_rad":angle,"beam_index":index},
        )
        self.pub.publish(obs)

def main():
    rclpy.init()
    node=LidarScanAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
