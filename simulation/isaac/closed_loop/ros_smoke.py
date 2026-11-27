"""Synthetic DDS integration test, not an Isaac Sim run."""
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from autonomymind_interfaces.srv import AssessSimulationTrajectory

def main():
    rclpy.init()
    node=rclpy.create_node("simulation_rta_smoke")
    try:
        pub=node.create_publisher(Odometry,"/autonomymind_sim/simulation/odometry",qos_profile_sensor_data)
        client=node.create_client(AssessSimulationTrajectory,"/autonomymind_sim/simulation/assess_trajectory")
        assert client.wait_for_service(timeout_sec=15)
        odom=Odometry()
        odom.header.frame_id="world_enu"
        odom.child_frame_id="base_link"
        odom.header.stamp.sec=1
        odom.pose.pose.orientation.w=1.0
        def call(seq, distance, refresh=True):
            if refresh:
                for _ in range(5):
                    pub.publish(odom)
                    rclpy.spin_once(node,timeout_sec=0.02)
                    time.sleep(0.03)
            q=AssessSimulationTrajectory.Request()
            q.vehicle_id="sim-rover"; q.frame_id="world_enu"; q.odom_stamp_sec=1
            q.sequence=seq; q.sample_dt_s=1.0
            q.x_m=[0.0,0.0]; q.y_m=[0.0,0.0]; q.z_m=[0.0,0.0]
            q.threat_position_m=[float(distance),0.0,0.0]
            q.threat_velocity_mps=[0.0,0.0,0.0]
            q.environment_valid=True; q.communication_valid=True
            f=client.call_async(q)
            rclpy.spin_until_future_complete(node,f,timeout_sec=5)
            assert f.done() and f.result() is not None
            return f.result()
        assert call(1,50).recorded
        assert not call(1,50).recorded
        assert not call(2,0).recorded
        time.sleep(0.7)
        assert not call(3,50,False).recorded
        print("PASS: synthetic ROS 2 RTA assessment, replay, separation, stale receipt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__=="__main__":
    main()
