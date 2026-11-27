"""Bounded, simulator-only straight-line experiment. No physical robot routing."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from autonomymind_interfaces.srv import AssessSimulationTrajectory

def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec*1e-9

def main():
    rclpy.init()
    node=rclpy.create_node("gazebo_closed_loop_probe")
    latest={}
    subs=[]
    def receive(name,m):
        if name not in latest or stamp(m)>stamp(latest[name][0]):
            latest[name]=(m,time.monotonic())
    for name in ("rover","drone","obstacle"):
        subs.append(node.create_subscription(Odometry,"/gazebo/"+name+"/odometry",
                    lambda m,n=name:receive(n,m),qos_profile_sensor_data))
    subs.append(node.create_subscription(LaserScan,"/mobile/scan",lambda m:receive("scan",m),qos_profile_sensor_data))
    cmd=node.create_publisher(Twist,"/autonomymind_gazebo_only/cmd_vel",10)
    odom=node.create_publisher(Odometry,"/autonomymind_sim/simulation/odometry",qos_profile_sensor_data)
    client=node.create_client(AssessSimulationTrajectory,"/autonomymind_sim/simulation/assess_trajectory")
    trace=[]
    def send(speed=0.0):
        m=Twist();m.linear.x=float(speed);cmd.publish(m)
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.02)
    def ready():
        return len(latest)==4 and all(time.monotonic()-v[1]<0.6 for v in latest.values())
    def clearance():
        s=latest["scan"][0]
        vals=[r for i,r in enumerate(s.ranges) if abs(s.angle_min+i*s.angle_increment)<0.35
              and math.isfinite(r) and s.range_min<=r<=s.range_max]
        return min(vals) if vals else None
    try:
        if not client.wait_for_service(timeout_sec=30): raise RuntimeError("RTA unavailable")
        end=time.monotonic()+90
        while not (ready() and cmd.get_subscription_count() and odom.get_subscription_count()):
            if time.monotonic()>end: raise RuntimeError("Missing simulator data or bridge")
            send();spin(0.1)
        send();spin(2.0)
        if not ready(): raise RuntimeError("Stale sensor stream")
        initial=latest["rover"][0].pose.pose.position.x
        deadline=time.monotonic()+120
        reason="timeout"
        seq=time.time_ns()
        while time.monotonic()<deadline:
            send();spin(0.2)
            if not ready(): reason="stale_data";break
            distance=clearance()
            if distance is None: reason="invalid_scan";break
            ego=latest["rover"][0];p=ego.pose.pose.position
            o=ego.pose.pose.orientation
            # This experiment assumes a straight, level, stationary-start rover.
            if abs(o.x)>0.05 or abs(o.y)>0.05 or abs(o.z)>0.05 or abs(p.z)>0.09:
                reason="unstable_pose";break
            if any(latest[n][0].header.frame_id!="world_enu" for n in ("rover","drone","obstacle")):
                reason="wrong_frame";break
            if max(abs(stamp(latest[n][0])-stamp(ego)) for n in ("drone","obstacle","scan"))>0.35:
                reason="unsynchronized";break
            if distance<=1.4: reason="lidar_stop";break
            q=AssessSimulationTrajectory.Request()
            q.vehicle_id="sim-rover";q.frame_id="world_enu";q.sequence=seq;seq+=1
            q.odom_stamp_sec=ego.header.stamp.sec;q.odom_stamp_nanosec=ego.header.stamp.nanosec
            q.sample_dt_s=1.0;q.x_m=[0.0,0.2,0.4];q.y_m=[0.0]*3;q.z_m=[0.0]*3
            q.peak_speed_mps=0.2;q.peak_accel_mps2=0.2;q.peak_jerk_mps3=0.2
            for n in ("drone","obstacle"):
                m=latest[n][0];other=m.pose.pose.position
                q.threat_position_m.extend([other.x-p.x,other.y-p.y,other.z-p.z])
                q.threat_velocity_mps.extend([-ego.twist.twist.linear.x,-ego.twist.twist.linear.y,-ego.twist.twist.linear.z])
            q.environment_valid=True;q.communication_valid=True
            odom.publish(ego);spin(0.06)
            f=client.call_async(q)
            rclpy.spin_until_future_complete(node,f,timeout_sec=0.4)
            if not f.done() or f.result() is None: reason="rta_timeout";break
            a=f.result()
            trace.append({"x_m":p.x,"lidar_m":distance,"rta_recorded":a.recorded,"reason":a.reason,"sequence":q.sequence})
            if not a.recorded: reason="rta_rejected";break
            if not ready() or clearance() is None or clearance()<=1.4: reason="sensor_stop";break
            # Short pulses only, separated by zero commands. Not a real-time watchdog.
            send(0.2);spin(0.2);send()
        send();spin(1.0)
        final=latest["rover"][0].pose.pose.position.x
        result={"stop_reason":reason,"initial_x_m":initial,"final_x_m":final,
                "displacement_m":final-initial,"last_lidar_m":clearance(),"assessments":trace,
                "source":"gazebo_reference_closed_loop","physical_hardware":False}
        out=Path("/gazebo/outbox");out.mkdir(parents=True,exist_ok=True)
        (out/"closed_loop.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps({k:v for k,v in result.items() if k!="assessments"},indent=2))
        if final-initial<0.1 or reason not in ("lidar_stop","rta_rejected","sensor_stop"):
            raise RuntimeError("Reference experiment did not demonstrate advance and obstacle stop")
        if not ready() or clearance() is None or clearance()<0.5 or abs(latest["rover"][0].twist.twist.linear.x)>0.05:
            raise RuntimeError("Final clearance or stationary check failed")
        print("PASS: bounded Gazebo rover advance and obstacle stop")
    finally:
        for _ in range(5):
            send();spin(0.05)
        node.destroy_node();rclpy.shutdown()
if __name__=="__main__":
    main()
