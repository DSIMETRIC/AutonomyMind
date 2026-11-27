"""Read-only smoke test of actual simulator sensor streams."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry

def main():
    rclpy.init()
    node=rclpy.create_node("mobile_sensor_probe")
    latest={}
    counts={}
    subs=[]
    def receive(name,msg):
        latest[name]=msg
        counts[name]=counts.get(name,0)+1
    try:
        for name,typ,topic in [
            ("rgb",Image,"/mobile/image"),("lidar",LaserScan,"/mobile/scan"),
            ("imu",Imu,"/mobile/imu"),("odometry",Odometry,"/gazebo/rover/odometry")]:
            subs.append(node.create_subscription(typ,topic,lambda m,n=name:receive(n,m),qos_profile_sensor_data))
        end=time.monotonic()+90
        while not all(counts.get(n,0)>=3 for n in ("rgb","lidar","imu","odometry")):
            if time.monotonic()>end:
                raise RuntimeError("Missing sensor messages: "+str(counts))
            rclpy.spin_once(node,timeout_sec=0.2)
        image=latest["rgb"]; scan=latest["lidar"]; imu=latest["imu"]
        valid=[x for x in scan.ranges if math.isfinite(x) and scan.range_min<=x<=scan.range_max]
        if image.width!=320 or image.height!=240 or len(image.data)<image.height*image.step or not valid:
            raise RuntimeError("Invalid image or empty valid scan")
        if not all(math.isfinite(x) for x in (imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z)):
            raise RuntimeError("Invalid IMU")
        result={"counts":counts,"rgb_dimensions":[image.width,image.height],
                "nearest_lidar_m":min(valid),"imu_acceleration":[imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z],
                "source":"gazebo_simulated_sensors","movement_tested":False}
        out=Path("/gazebo/outbox");out.mkdir(parents=True,exist_ok=True)
        (out/"sensors.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps(result,indent=2))
        print("PASS: Gazebo RGB / LiDAR / IMU / odometry streams")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__=="__main__":
    main()
