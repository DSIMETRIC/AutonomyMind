"""Read-only Gazebo ground truth -> ROS 2 -> C++ reference assessment."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from autonomymind_interfaces.srv import AssessSimulationTrajectory

def main():
    rclpy.init()
    node = rclpy.create_node("gazebo_reference_probe")
    latest = {}
    subscriptions = []
    def receive(name, msg):
        latest[name] = (msg, time.monotonic())
    for name in ("rover", "drone", "obstacle"):
        subscriptions.append(node.create_subscription(
            Odometry, "/gazebo/"+name+"/odometry",
            lambda m, n=name: receive(n,m), qos_profile_sensor_data))
    pub = node.create_publisher(Odometry, "/autonomymind_sim/simulation/odometry", qos_profile_sensor_data)
    client = node.create_client(AssessSimulationTrajectory, "/autonomymind_sim/simulation/assess_trajectory")
    try:
        if not client.wait_for_service(timeout_sec=60):
            raise RuntimeError("RTA unavailable")
        deadline = time.monotonic()+60
        while len(latest)<3 or any(time.monotonic()-v[1]>0.4 for v in latest.values()):
            if time.monotonic()>deadline:
                raise RuntimeError("Gazebo odometry unavailable")
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count()==0:
            while pub.get_subscription_count()==0 and time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        def assess(seq, displacement):
            # All three models are stationary reference geometry.
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(time.monotonic()-v[1]>0.4 for v in latest.values()):
                raise RuntimeError("stale Gazebo receipt")
            ego=latest["rover"][0]
            if any(m.header.frame_id!="world_enu" for m,_ in latest.values()):
                raise RuntimeError("unexpected frame")
            p=ego.pose.pose.position
            q=AssessSimulationTrajectory.Request()
            q.vehicle_id="sim-rover"; q.frame_id="world_enu"
            q.sequence=seq
            q.odom_stamp_sec=ego.header.stamp.sec
            q.odom_stamp_nanosec=ego.header.stamp.nanosec
            q.sample_dt_s=1.0
            q.x_m=[0.0, displacement]; q.y_m=[0.0,0.0]; q.z_m=[0.0,0.0]
            q.peak_speed_mps=abs(displacement)
            for name in ("drone","obstacle"):
                other=latest[name][0].pose.pose.position
                q.threat_position_m.extend([other.x-p.x,other.y-p.y,other.z-p.z])
                q.threat_velocity_mps.extend([0.0,0.0,0.0])
            q.environment_valid=True; q.communication_valid=True
            # Forward the exact snapshot for the request; this is not a continuous watchdog.
            for _ in range(3):
                pub.publish(ego)
                time.sleep(0.03)
            future=client.call_async(q)
            rclpy.spin_until_future_complete(node,future,timeout_sec=5)
            if not future.done() or future.result() is None:
                raise RuntimeError("RTA timeout")
            a=future.result()
            return {"sequence":seq,"candidate_displacement_m":displacement,
                    "recorded":a.recorded,"reason":a.reason,
                    "profile_id":a.profile_id,"state_receipt_age_s":a.state_receipt_age_s,
                    "relative_threat_positions_m":list(q.threat_position_m)}
        results=[assess(1,0.0),assess(2,4.0)]
        out=Path("/gazebo/outbox"); out.mkdir(parents=True,exist_ok=True)
        (out/"assessment.json").write_text(json.dumps(results,indent=2,allow_nan=False)+"\n")
        print(json.dumps(results,indent=2))
        if not results[0]["recorded"] or results[1]["recorded"] or results[1]["reason"]!="constraint_rejected":
            raise RuntimeError("unexpected RTA assessment")
        print("PASS: Gazebo ground truth -> ROS 2 -> C++ RTA")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__=="__main__":
    main()
