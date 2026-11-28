"""Simulator-only bounded rover pulses; static drone, no hardware adapter."""
import copy
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from autonomymind_interfaces.srv import AssessSimulationTrajectory as S
from autonomymind_runtime.dds_protocol import envelope,encode,decode,Receiver,PeerDirectory

def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def pos(m):
    p=m.pose.pose.position;return (p.x,p.y,p.z)

def main():
    config=json.loads(Path("/gazebo/execution_gate.json").read_text())
    rclpy.init();n=rclpy.create_node("execution_gate_probe")
    latest={};replies={};directory=PeerDirectory();receiver=Receiver("execution_gate",{"reservation.reply"})
    def observe(k,m):
        if k not in latest or stamp(m)>stamp(latest[k][0]):latest[k]=(m,time.monotonic())
    subs=[n.create_subscription(Odometry,"/gazebo/"+k+"/odometry",lambda m,k=k:observe(k,m),qos_profile_sensor_data) for k in ("rover","drone","obstacle")]
    subs.append(n.create_subscription(LaserScan,"/mobile/scan",lambda m:observe("scan",m),qos_profile_sensor_data))
    qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
    pub=n.create_publisher(String,"/autonomymind_agents/events",qos)
    def receive(raw):
        try:m=decode(raw.data)
        except ValueError:return
        if m["kind"]=="heartbeat":directory.observe(m,time.time(),time.monotonic())
        elif m["sender"]=="reservations" and receiver.check(m,time.time())=="accepted":replies[m["correlation"]]=m["payload"]
    subs.append(n.create_subscription(String,"/autonomymind_agents/events",receive,qos))
    cmd=n.create_publisher(Twist,"/evasion/approved_pulse",10)
    odom=n.create_publisher(Odometry,"/fleet/rover/simulation/odometry",qos_profile_sensor_data)
    client=n.create_client(S,"/fleet/rover/simulation/assess_trajectory")
    def send(v=0.):
        m=Twist();m.linear.x=float(v);cmd.publish(m)
    def wait(fn,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.01)
            if fn():return True
        return False
    def ready():
        if len(latest)!=4 or any(time.monotonic()-v[1]>.6 for v in latest.values()):return False
        ego=latest["rover"][0]
        if any(latest[k][0].header.frame_id!="world_enu" for k in ("rover","drone","obstacle")):return False
        if any(abs(stamp(m)-stamp(ego))>.35 for m,_ in latest.values()):return False
        s=latest["scan"][0]
        vals=[r for i,r in enumerate(s.ranges) if abs(s.angle_min+i*s.angle_increment)<.35 and math.isfinite(r) and s.range_min<=r<=s.range_max]
        return bool(vals) and min(vals)>1.4
    def ask(kind,payload):
        m=envelope("execution_gate","reservations",kind,payload,time.time(),2)
        pub.publish(String(data=encode(m)))
        if not wait(lambda:m["correlation"] in replies,1):raise RuntimeError("reservation timeout")
        return replies.pop(m["correlation"])
    trace=[];reason="not_started";initial=None
    try:
        if not wait(lambda:ready() and cmd.get_subscription_count()>0 and odom.get_subscription_count()>0 and client.service_is_ready() and "reservations" in directory.discover("trajectory_reservations",time.monotonic()),90):raise RuntimeError("inputs unavailable")
        initial=pos(latest["rover"][0]);deadline=time.monotonic()+60
        reason="timeout"
        while time.monotonic()<deadline:
            send();wait(lambda:False,.25)
            if not ready():reason="sensor_stop";break
            ego=copy.deepcopy(latest["rover"][0]);p=pos(ego);q0=ego.pose.pose.orientation
            if p[0]-initial[0]>=config["target_distance_m"]:reason="target_reached";break
            if abs(p[1]-initial[1])>.1 or abs(q0.x)>.05 or abs(q0.y)>.05 or abs(q0.z)>.05 or abs(p[2])>.09:reason="pose_stop";break
            if -1e-6<=p[2]<0:ego.pose.pose.position.z=0.
            p=pos(ego)
            speed=float(config["speed_mps"]);dt=1.;offset=speed*dt
            rows=[]
            for spec in config["requests"]:
                k=spec["member"];start=p if k=="rover" else pos(latest[k][0]);end=[start[0]+offset,start[1],start[2]] if k=="rover" else list(start)
                rows.append({**spec,"points":[[0.,*start],[dt,*end]]})
            state=ask("reservation.query",{})
            plan=ask("reservation.submit",{"frame_id":"world_enu","expected_revision":state["revision"],"requests":rows})
            if plan.get("status")!="scheduled" or plan.get("motion_authorized") is not False:reason="reservation_rejected";break
            reserved={r["member"]:r for r in plan["reservations"]}
            if set(reserved)!={r["member"] for r in rows} or any(reserved[r["member"]]["delay_s"]!=0 or any(reserved[r["member"]][k]!=v for k,v in r.items()) for r in rows):reason="reservation_mismatch";break
            if not ready() or math.dist(pos(latest["rover"][0]),p)>.025:reason="state_changed";break
            q=S.Request();q.vehicle_id="sim-rover";q.frame_id="world_enu";q.sequence=time.time_ns()
            q.odom_stamp_sec=ego.header.stamp.sec;q.odom_stamp_nanosec=ego.header.stamp.nanosec
            q.sample_dt_s=dt;q.x_m=[0.,offset];q.y_m=[0.,0.];q.z_m=[0.,0.]
            q.peak_speed_mps=speed;q.peak_accel_mps2=speed;q.peak_jerk_mps3=speed
            q.environment_valid=True;q.communication_valid=True
            for k in ("drone","obstacle"):
                q.threat_position_m.extend([b-a for a,b in zip(p,pos(latest[k][0]))]);q.threat_velocity_mps.extend([0.,0.,0.])
            odom.publish(ego);wait(lambda:False,.06)
            f=client.call_async(q)
            if not wait(f.done,.4) or f.result() is None:reason="rta_timeout";break
            a=f.result()
            if not a.recorded or a.sequence!=q.sequence or a.profile_id!="reference-rover-v1":reason="rta_rejected";break
            final=ask("reservation.query",{})
            if not final["active"] or final["revision"]!=plan["revision"] or final["plan"]["expires_at"]<time.time()+.5:reason="reservation_expired";break
            if not ready() or math.dist(pos(latest["rover"][0]),p)>.025:reason="state_changed";break
            send(speed)
            wait(lambda:not ready(),.2)
            send()
            trace.append({"sequence":q.sequence,"revision":plan["revision"],"rta_reason":a.reason,"start":p,"pulse_s":.2,"speed_mps":speed})
        send();wait(lambda:False,1.)
        final=pos(latest["rover"][0]);v=latest["rover"][0].twist.twist.linear
        result={"source":"gazebo_rover_only_reference","stop_reason":reason,"displacement_m":final[0]-initial[0],"pulses":len(trace),"stationary_final":math.hypot(v.x,v.y,v.z)<.05,"physical_hardware":False,"trace":trace}
        out=Path("/gazebo/outbox/execution_gate.json");out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps({k:v for k,v in result.items() if k!="trace"},indent=2))
        if reason!="target_reached" or not result["stationary_final"] or not trace:raise RuntimeError("reference movement acceptance failed")
        print("PASS: reservation and RTA gated rover pulses in Gazebo")
    finally:
        for _ in range(5):send();wait(lambda:False,.05)
        n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
