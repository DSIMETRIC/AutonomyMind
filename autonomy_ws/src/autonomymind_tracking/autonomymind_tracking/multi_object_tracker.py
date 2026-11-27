import json
import rclpy
from rclpy.node import Node
from autonomymind_interfaces.msg import Observation
from .tracker_core import Detection3D, MultiObjectTracker

class MultiObjectTrackerNode(Node):
    def __init__(self):
        super().__init__("autonomymind_multi_object_tracker")
        self.declare_parameter("input_topic","/autonomymind/observations/fused")
        self.declare_parameter("output_topic","/autonomymind/observations/tracks")
        self.declare_parameter("gate_m",2.0)
        self.declare_parameter("max_missed",5)
        self.declare_parameter("prediction_horizon_s",2.0)
        self.tracker=MultiObjectTracker(
            gate_m=float(self.get_parameter("gate_m").value),
            max_missed=int(self.get_parameter("max_missed").value),
        )
        self.horizon=float(self.get_parameter("prediction_horizon_s").value)
        self.pub=self.create_publisher(Observation,str(self.get_parameter("output_topic").value),50)
        self.create_subscription(Observation,str(self.get_parameter("input_topic").value),self.on_obs,50)

    def on_obs(self,msg: Observation):
        try:
            attrs=json.loads(msg.attributes_json or "{}")
        except Exception:
            attrs={}
        cov=attrs.get("covariance_diag_m2",[0.25,0.25,0.25])
        d=Detection3D(
            observation_id=msg.observation_id,
            timestamp_s=float(msg.timestamp_s),
            label=msg.label,
            confidence=float(msg.confidence),
            position_m=tuple(float(x) for x in msg.position_m),
            covariance_diag_m2=tuple(float(x) for x in cov),
        )
        tracks=self.tracker.update([d],d.timestamp_s)
        for t in tracks:
            if d.observation_id not in t.source_ids:
                continue
            out=Observation()
            out.schema_version="1.0"
            out.observation_id=t.track_id
            out.agent_id="tracking_agent"
            out.agent_version="0.1.0"
            out.timestamp_s=t.timestamp_s
            out.frame_id=msg.frame_id
            out.source="multi_object_tracker"
            out.kind="track"
            out.confidence=t.confidence
            out.position_m=[float(x) for x in t.position_m]
            out.velocity_mps=[float(x) for x in t.velocity_mps]
            out.label=t.label
            pred=t.predict(t.timestamp_s+self.horizon)
            out.provenance_json=json.dumps({
                "producer":"tracking_agent",
                "source_type":"fusion",
                "sources":t.source_ids,
            },sort_keys=True)
            out.attributes_json=json.dumps({
                "track_id":t.track_id,
                "age":t.age,
                "missed":t.missed,
                "covariance_diag_m2":list(t.covariance_diag_m2),
                "prediction_horizon_s":self.horizon,
                "predicted_position_m":list(pred),
            },sort_keys=True)
            self.pub.publish(out)

def main():
    rclpy.init()
    node=MultiObjectTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
