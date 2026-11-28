"""Stationary Gazebo odometry -> DDS reservations -> namespaced RTA, no motion."""
import copy
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from autonomymind_interfaces.srv import AssessSimulationTrajectory as S
from autonomymind_runtime.dds_protocol import envelope,encode,decode,PeerDirectory,Receiver
from autonomymind_runtime.spatiotemporal import Request,conflict

def main():
    config=json.loads(Path("/gazebo/integrated_preview.json").read_text())
    ids=[r["member"] for r in config["requests"]]
    all_ids=ids+["obstacle"]
    rclpy.init();n=Node("integrated_preview")
    latest={};replies={};directory=PeerDirectory();receiver=Receiver("integrated_preview",{"reservation.reply"})
    subs=[n.create_subscription(Odometry,"/gazebo/"+k+"/odometry",lambda m,k=k:latest.__setitem__(k,(m,time.monotonic())),qos_profile_sensor_data) for k in all_ids]
    pubs={k:n.create_publisher(Odometry,"/fleet/"+k+"/simulation/odometry",qos_profile_sensor_data) for k in ids}
    clients={k:n.create_client(S,"/fleet/"+k+"/simulation/assess_trajectory") for k in ids}
    qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
    pub=n.create_publisher(String,"/autonomymind_agents/events",qos)
    def receive(raw):
        try:m=decode(raw.data)
        except ValueError:return
        if m["kind"]=="heartbeat":directory.observe(m,time.time(),time.monotonic())
        elif m["sender"]=="reservations" and receiver.check(m,time.time())=="accepted":replies[m["correlation"]]=m["payload"]
    subs.append(n.create_subscription(String,"/autonomymind_agents/events",receive,qos))
    def wait(fn,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.02)
            if fn():return True
        return False
    def pos(m):
        p=m.pose.pose.position;return (p.x,p.y,p.z)
    def ready():
        if set(latest)!=set(all_ids) or any(time.monotonic()-v[1]>.6 for v in latest.values()):return False
        stamps=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m,_ in latest.values()]
        return max(stamps)-min(stamps)<.35
    def capture():
        if not ready():raise RuntimeError("stale or unsynchronized odometry")
        result={}
        for k,(m,_) in latest.items():
            v=m.twist.twist.linear;w=m.twist.twist.angular
            if m.header.frame_id!="world_enu" or not all(math.isfinite(x) for x in (*pos(m),v.x,v.y,v.z,w.x,w.y,w.z)) or math.hypot(v.x,v.y,v.z)>.05 or math.hypot(w.x,w.y,w.z)>.05:
                raise RuntimeError("stationary world_enu reference required")
            m=copy.deepcopy(m)
            if k=="rover" and -1e-4<=m.pose.pose.position.z<0:m.pose.pose.position.z=0.
            result[k]=m
        return result
    def ask(kind,payload):
        m=envelope("integrated_preview","reservations",kind,payload,time.time(),5)
        pub.publish(String(data=encode(m)))
        if not wait(lambda:m["correlation"] in replies,3):raise RuntimeError("DDS response missing")
        return replies.pop(m["correlation"])
    try:
        if not wait(lambda:ready() and "reservations" in directory.discover("trajectory_reservations",time.monotonic()) and all(c.service_is_ready() for c in clients.values()) and all(p.get_subscription_count()>0 for p in pubs.values()),90):
            raise RuntimeError("inputs unavailable")
        initial=capture()
        rows=[]
        for r in config["requests"]:
            rows.append({**{k:v for k,v in r.items() if k not in ("goal","duration_s")},"points":[[0.,*pos(initial[r["member"]])],[r["duration_s"],*r["goal"]]]})
        state=ask("reservation.query",{})
        plan=ask("reservation.submit",{"frame_id":"world_enu","expected_revision":state["revision"],"requests":rows})
        if plan["status"]!="scheduled" or plan["motion_authorized"] or len(plan["reservations"])!=len(ids):
            raise RuntimeError("reservation batch not scheduled")
        reservations={r["member"]:r for r in plan["reservations"]}
        if set(reservations)!=set(ids):raise RuntimeError("membership mismatch")
        for r in rows:
            if any(reservations[r["member"]][k]!=v for k,v in r.items()):raise RuntimeError("proposal changed")
        for i,k in enumerate(ids):
            for j in ids[i+1:]:
                a,b=(Request(**{x:y for x,y in reservations[v].items() if x!="delay_s"}) for v in (k,j))
                if conflict(a,reservations[k]["delay_s"],b,reservations[j]["delay_s"],config["settings"]["horizon_s"],config["settings"]["margin_m"]):raise RuntimeError("remaining reservation conflict")
        records=[]
        for k in ids:
            current=capture()
            if any(math.dist(pos(current[j]),pos(initial[j]))>.03 for j in all_ids):raise RuntimeError("scene moved")
            r=reservations[k]
            if r["delay_s"]!=0:raise RuntimeError("preview supports zero-delay segments only")
            own=current[k];start=pos(own);goal=[float(v) for v in r["points"][-1][1:]];dt=float(r["points"][-1][0])
            if not math.isfinite(dt) or dt<=0 or not all(math.isfinite(v) for v in goal):raise RuntimeError("invalid reservation segment")
            delta=[b-a for a,b in zip(start,goal)]
            q=S.Request();q.vehicle_id="sim-"+k;q.frame_id="world_enu"
            q.sequence=time.time_ns();q.odom_stamp_sec=own.header.stamp.sec;q.odom_stamp_nanosec=own.header.stamp.nanosec
            q.sample_dt_s=dt;q.x_m=[0.,delta[0]];q.y_m=[0.,delta[1]];q.z_m=[0.,delta[2]]
            q.peak_speed_mps=math.hypot(*delta)/dt;q.peak_vertical_speed_mps=abs(delta[2])/dt
            q.peak_accel_mps2=q.peak_speed_mps/dt;q.peak_jerk_mps3=q.peak_accel_mps2/dt
            q.peak_yaw_rate_dps=0.0;q.peak_bank_angle_deg=0.0
            q.environment_valid=True;q.communication_valid=True
            for j in all_ids:
                if j==k:continue
                q.threat_position_m.extend([b-a for a,b in zip(start,pos(current[j]))])
                q.threat_velocity_mps.extend([0.,0.,0.])
            pubs[k].publish(own);wait(lambda:False,.1)
            future=clients[k].call_async(q)
            if not wait(future.done,2) or future.result() is None:raise RuntimeError("RTA timeout")
            a=future.result()
            records.append({"member":k,"sequence":q.sequence,"profile_id":a.profile_id,"recorded":a.recorded,"reason":a.reason,"start":start,"goal":goal,"sample_dt_s":dt})
        final=ask("reservation.query",{})
        if not final["active"] or final["revision"]!=plan["revision"]:raise RuntimeError("reservation changed or expired")
        result={"source":"stationary_gazebo_odometry_live_dds","reservation_revision":plan["revision"],"reservations":plan["reservations"],"assessments":records,"coverage_complete":False,"motion_authorized":False,"cpp_rta_assessed":True}
        out=Path("/gazebo/outbox/integrated_preview.json");out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps(result,indent=2))
        if not all(r["recorded"] and r["reason"]=="reference_checks_passed" for r in records) or len({r["profile_id"] for r in records})!=len(ids):raise RuntimeError("reference assessment not accepted")
        print("PASS: live Gazebo odometry -> DDS reservations -> independent C++ assessments")
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
