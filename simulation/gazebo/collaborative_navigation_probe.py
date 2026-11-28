"""Live shared evidence -> A* -> local proposal -> reservation -> rover RTA. Read only."""
import copy
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from autonomymind_interfaces.srv import AssessSimulationTrajectory as S
from autonomymind_runtime.dds_protocol import envelope,encode,decode,PeerDirectory,Receiver
from autonomymind_runtime.navigation import Pose2
from autonomymind_runtime.collaborative_navigation import propose

def main():
    cfg=json.loads(Path("/gazebo/collaborative_navigation.json").read_text())
    rclpy.init();n=rclpy.create_node("collaborative_navigation_probe")
    latest={};replies={};directory=PeerDirectory();receiver=Receiver("navigation_probe",{"world.snapshot","reservation.reply"})
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def observe(k,m):
        if k not in latest or stamp(m)>stamp(latest[k][0]):latest[k]=(m,time.monotonic())
    subs=[n.create_subscription(Odometry,"/gazebo/"+k+"/odometry",lambda m,k=k:observe(k,m),qos_profile_sensor_data) for k in ("rover","drone")]
    qos=QoSProfile(depth=128,reliability=ReliabilityPolicy.RELIABLE)
    pub=n.create_publisher(String,"/autonomymind_agents/events",qos)
    def receive(raw):
        try:m=decode(raw.data)
        except ValueError:return
        if m["kind"]=="heartbeat":directory.observe(m,time.time(),time.monotonic())
        elif receiver.check(m,time.time())=="accepted":replies[m["correlation"]]=m
    subs.append(n.create_subscription(String,"/autonomymind_agents/events",receive,qos))
    odom=n.create_publisher(Odometry,"/fleet/rover/simulation/odometry",qos_profile_sensor_data)
    client=n.create_client(S,"/fleet/rover/simulation/assess_trajectory")
    def wait(fn,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.02)
            if fn():return True
        return False
    def ask(peer,kind,payload):
        m=envelope("navigation_probe",peer,kind,payload,time.time(),3)
        pub.publish(String(data=encode(m)))
        if not wait(lambda:m["correlation"] in replies,2):raise RuntimeError("DDS response missing")
        reply=replies.pop(m["correlation"])
        if reply["sender"]!=peer:raise RuntimeError("unexpected reply source")
        return reply["payload"]
    def pos(m):
        p=m.pose.pose.position;return (p.x,p.y,p.z)
    def ready():
        return len(latest)==2 and all(time.monotonic()-v[1]<.6 for v in latest.values()) and abs(stamp(latest["rover"][0])-stamp(latest["drone"][0]))<.35
    def capture():
        if not ready():raise RuntimeError("stale odometry")
        result={}
        for k,(m,_) in latest.items():
            q=m.pose.pose.orientation;v=m.twist.twist.linear;w=m.twist.twist.angular
            if m.header.frame_id!="world_enu" or not all(math.isfinite(x) for x in (*pos(m),q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z)) or abs(q.x)>.05 or abs(q.y)>.05 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.001 or math.hypot(v.x,v.y,v.z)>.05 or math.hypot(w.x,w.y,w.z)>.05:raise RuntimeError("stationary level reference required")
            m=copy.deepcopy(m)
            if k=="rover" and -1e-4<=m.pose.pose.position.z<0:m.pose.pose.position.z=0.
            result[k]=m
        return result
    try:
        if not wait(lambda:ready() and directory.discover("shared_world",time.monotonic()) and "reservations" in directory.discover("trajectory_reservations",time.monotonic()) and client.service_is_ready() and odom.get_subscription_count()>0,90):raise RuntimeError("inputs unavailable")
        world=directory.discover("shared_world",time.monotonic())[0]
        deadline=time.monotonic()+30
        result=None
        while time.monotonic()<deadline:
            initial=capture();ego=initial["rover"];p=pos(ego);q0=ego.pose.pose.orientation
            pose=Pose2(p[0],p[1],math.atan2(2*(q0.w*q0.z+q0.x*q0.y),1-2*(q0.y*q0.y+q0.z*q0.z)))
            snap=ask(world,"world.query",{});evaluated=time.time()
            try:result=propose(snap,evaluated,pose,cfg)
            except ValueError:
                wait(lambda:False,.2);continue
            if result["remote_additional_cells"] and result["candidates"]:break
            wait(lambda:False,.2)
        if not result or not result["remote_additional_cells"] or not result["candidates"]:raise RuntimeError("shared route or local candidate unavailable")
        candidate=next((c for c in result.pop("candidates") if c.velocity>0 or abs(c.yaw_rate)>0),None)
        if candidate is None:raise RuntimeError("no nonstationary candidate")
        points=[[float(t),float(x),float(y),float(p[2])] for t,x,y,_ in candidate.points]
        drone=pos(initial["drone"]);duration=points[-1][0]
        rows=[{**r,"points":points if r["member"]=="rover" else [[0.,*drone],[duration,*drone]]} for r in cfg["requests"]]
        state=ask("reservations","reservation.query",{})
        plan=ask("reservations","reservation.submit",{"frame_id":"world_enu","expected_revision":state["revision"],"requests":rows})
        if plan.get("status")!="scheduled" or plan.get("motion_authorized") is not False:raise RuntimeError("reservation rejected: "+json.dumps(plan,sort_keys=True))
        reserved={r["member"]:r for r in plan["reservations"]}
        if set(reserved)!={r["member"] for r in rows} or any(reserved[r["member"]]["delay_s"]!=0 or any(reserved[r["member"]][k]!=v for k,v in r.items()) for r in rows):raise RuntimeError("reservation mismatch")
        current=capture()
        if any(math.dist(pos(current[k]),pos(initial[k]))>.025 for k in initial):raise RuntimeError("scene changed")
        if time.time()-evaluated>=2:raise RuntimeError("evidence too old")
        # Assess the exact sampled candidate relative to its original odometry.
        q=S.Request();q.vehicle_id="sim-rover";q.frame_id="world_enu";q.sequence=time.time_ns()
        q.odom_stamp_sec=ego.header.stamp.sec;q.odom_stamp_nanosec=ego.header.stamp.nanosec
        q.sample_dt_s=float(points[1][0]-points[0][0])
        q.x_m=[float(v[1]-p[0]) for v in points];q.y_m=[float(v[2]-p[1]) for v in points];q.z_m=[0.]*len(points)
        q.peak_speed_mps=float(candidate.velocity);q.peak_yaw_rate_dps=math.degrees(abs(candidate.yaw_rate))
        q.peak_accel_mps2=float(candidate.velocity/q.sample_dt_s);q.peak_jerk_mps3=q.peak_accel_mps2/q.sample_dt_s
        q.environment_valid=True;q.communication_valid=True
        threats=[(c["center"][0],c["center"][1],p[2]) for c in result["occupied_cells"]]+[drone]
        if len(threats)>100:raise RuntimeError("RTA threat capacity exceeded")
        for point in threats:
            q.threat_position_m.extend([float(b-a) for a,b in zip(p,point)]);q.threat_velocity_mps.extend([0.,0.,0.])
        odom.publish(ego);wait(lambda:False,.06)
        f=client.call_async(q)
        if not wait(f.done,2) or f.result() is None:raise RuntimeError("RTA timeout")
        a=f.result()
        final=ask("reservations","reservation.query",{})
        if not final["active"] or final["revision"]!=plan["revision"]:raise RuntimeError("reservation expired or superseded")
        result.update(source="stationary_gazebo_shared_navigation",reservation_revision=plan["revision"],candidate={"velocity":candidate.velocity,"yaw_rate":candidate.yaw_rate,"points":points},rta_recorded=a.recorded,rta_reason=a.reason,profile_id=a.profile_id,cpp_rta_assessed=True)
        out=Path("/gazebo/outbox/collaborative_navigation.json");out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps({k:v for k,v in result.items() if k not in ("local_route","shared_route","occupied_cells","candidate")},indent=2))
        if a.sequence!=q.sequence or a.profile_id!="reference-rover-v1" or not a.recorded or a.reason!="reference_checks_passed":raise RuntimeError("candidate not accepted")
        print("PASS: shared Gazebo evidence -> A* -> local candidate -> DDS reservation -> C++ RTA")
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
