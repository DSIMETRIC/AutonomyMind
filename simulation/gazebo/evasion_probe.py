"""Simulator-only reference route around a known static obstacle."""
import copy
import json
import math
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from autonomymind_interfaces.srv import AssessSimulationTrajectory
from evasion_geometry import candidates, offsets, yaw_of

def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

def main():
    rclpy.init()
    n=rclpy.create_node("evasion_probe")
    latest={};trace=[];subs=[]
    def receive(k,m):
        if k not in latest or stamp(m)>stamp(latest[k][0]):latest[k]=(m,time.monotonic())
    for k in ("rover","drone","obstacle"):
        subs.append(n.create_subscription(Odometry,"/gazebo/"+k+"/odometry",lambda m,k=k:receive(k,m),qos_profile_sensor_data))
    subs.append(n.create_subscription(LaserScan,"/mobile/scan",lambda m:receive("scan",m),qos_profile_sensor_data))
    pub=n.create_publisher(Twist,"/evasion/approved_pulse",10)
    odom=n.create_publisher(Odometry,"/autonomymind_sim/simulation/odometry",qos_profile_sensor_data)
    client=n.create_client(AssessSimulationTrajectory,"/autonomymind_sim/simulation/assess_trajectory")
    out=Path("/gazebo/outbox");out.mkdir(parents=True,exist_ok=True)
    def send(v=0.,w=0.):
        m=Twist();m.linear.x=v;m.angular.z=w;pub.publish(m)
    def spin(t):
        end=time.monotonic()+t
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    def ready():
        return len(latest)==4 and all(time.monotonic()-v[1]<.6 for v in latest.values())
    def save(status):
        d={"status":status,"source":"gazebo_reference_evasion","events":trace}
        tmp=out/"evasion.tmp";tmp.write_text(json.dumps(d,allow_nan=False));tmp.replace(out/"evasion.json")
    try:
        if not client.wait_for_service(timeout_sec=30):raise RuntimeError("RTA unavailable")
        end=time.monotonic()+90
        while not (ready() and pub.get_subscription_count() and odom.get_subscription_count()):
            if time.monotonic()>end:raise RuntimeError("Missing simulator or watchdog")
            send();spin(.1)
        seq=time.time_ns();deadline=time.monotonic()+360
        # Known reference waypoints, not a general-purpose obstacle planner.
        route=[(1.7,1.8),(4.5,1.8),(6.0,0.0)]
        stage=0;status="timeout"
        while time.monotonic()<deadline:
            send();spin(.18)
            if not ready():status="stale";break
            ego=copy.deepcopy(latest["rover"][0]);p=ego.pose.pose.position;q=ego.pose.pose.orientation
            if abs(q.x)>.05 or abs(q.y)>.05 or abs(p.z)>.09:status="unstable";break
            if any(latest[k][0].header.frame_id!="world_enu" for k in ("rover","drone","obstacle")):status="frame";break
            if max(abs(stamp(latest[k][0])-stamp(ego)) for k in ("drone","obstacle","scan"))>.35:status="sync";break
            raw_z=p.z
            if -1e-6<=p.z<0:p.z=0.
            yaw=yaw_of(q)
            tx,ty=route[stage]
            if math.hypot(tx-p.x,ty-p.y)<.35:
                stage+=1
                if stage==len(route):status="goal_reached";break
                tx,ty=route[stage]
            scan=latest["scan"][0]
            vals=[min(r,scan.range_max) for i,r in enumerate(scan.ranges) if abs(scan.angle_min+i*scan.angle_increment)<.8 and (r==math.inf or scan.range_min<=r<=scan.range_max)]
            if not vals or min(vals)<=.65:status="lidar_stop";break
            threat_pos=[];threat_vel=[]
            vx=math.cos(yaw)*ego.twist.twist.linear.x-math.sin(yaw)*ego.twist.twist.linear.y
            vy=math.sin(yaw)*ego.twist.twist.linear.x+math.cos(yaw)*ego.twist.twist.linear.y
            for k in ("drone","obstacle"):
                m=latest[k][0];o=m.pose.pose.position
                threat_pos.extend([o.x-p.x,o.y-p.y,o.z-p.z])
                a=yaw_of(m.pose.pose.orientation);v=m.twist.twist.linear
                threat_vel.extend([math.cos(a)*v.x-math.sin(a)*v.y-vx,math.sin(a)*v.x+math.cos(a)*v.y-vy,v.z-ego.twist.twist.linear.z])
            options=[]
            # Reuse the exact snapshot for the four independent reference assessments.
            for name,v,w in candidates(yaw):
                xs,ys=offsets(yaw,v,w)
                req=AssessSimulationTrajectory.Request()
                req.vehicle_id="sim-rover";req.frame_id="world_enu"
                req.odom_stamp_sec=ego.header.stamp.sec;req.odom_stamp_nanosec=ego.header.stamp.nanosec
                req.sequence=seq;seq+=1;req.sample_dt_s=.25
                req.x_m=xs;req.y_m=ys;req.z_m=[0.]*len(xs)
                req.peak_speed_mps=v;req.peak_yaw_rate_dps=abs(math.degrees(w))
                req.peak_accel_mps2=2.;req.peak_jerk_mps3=8.
                req.threat_position_m=threat_pos;req.threat_velocity_mps=threat_vel
                req.environment_valid=True;req.communication_valid=True
                odom.publish(ego);spin(.04)
                f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=.3)
                a=f.result() if f.done() else None
                score=math.hypot(tx-p.x-xs[-1],ty-p.y-ys[-1])
                options.append({"name":name,"v":v,"w":w,"accepted":bool(a and a.recorded),"reason":a.reason if a else "timeout","score":score,"x":xs,"y":ys})
                if not ready():break
            accepted=[a for a in options if a["accepted"] and a["name"]!="BRAKE"]
            chosen=min(accepted,key=lambda a:a["score"]) if accepted else None
            trace.append({"x_m":p.x,"y_m":p.y,"raw_z_m":raw_z,"forwarded_z_m":p.z,"yaw_rad":yaw,"waypoint":stage,"lidar_m":min(vals),"options":options,"selected":chosen["name"] if chosen else "BRAKE"})
            save("running")
            if len(options)!=4 or not ready() or not chosen:status="no_authorized_motion";break
            send(chosen["v"],chosen["w"]);spin(.24);send()
        send();spin(1.)
        save(status)
        print(json.dumps({"status":status,"cycles":len(trace),"last":trace[-1] if trace else None},indent=2))
        if status!="goal_reached":raise RuntimeError("Reference route incomplete")
        print("PASS: Gazebo reference waypoint evasion")
    finally:
        send();spin(.1);n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
