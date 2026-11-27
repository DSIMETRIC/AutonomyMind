import json
import rclpy
from rclpy.node import Node
from autonomymind_interfaces.msg import Observation
from .geometry import load_calibration
from .fusion_core import associate

class CameraLidarFusion(Node):
    def __init__(self):
        super().__init__("autonomymind_camera_lidar_fusion")
        self.declare_parameter("camera_topic","/autonomymind/observations/camera")
        self.declare_parameter("lidar_topic","/autonomymind/observations/lidar")
        self.declare_parameter("output_topic","/autonomymind/observations/fused")
        self.declare_parameter("calibration_file","")
        self.declare_parameter("max_dt_s",0.10)
        self.declare_parameter("bbox_margin_px",8.0)
        calibration=str(self.get_parameter("calibration_file").value)
        if not calibration:
            raise RuntimeError("calibration_file parameter is required")
        self.cam,self.ex=load_calibration(calibration)
        self.max_dt=float(self.get_parameter("max_dt_s").value)
        self.margin=float(self.get_parameter("bbox_margin_px").value)
        self.camera=[]
        self.lidar=[]
        self.pub=self.create_publisher(Observation,str(self.get_parameter("output_topic").value),20)
        self.create_subscription(Observation,str(self.get_parameter("camera_topic").value),self.on_camera,20)
        self.create_subscription(Observation,str(self.get_parameter("lidar_topic").value),self.on_lidar,50)

    def on_camera(self,msg):
        self.camera.append(msg)
        self._trim(msg.timestamp_s)
        self._fuse()

    def on_lidar(self,msg):
        self.lidar.append(msg)
        self._trim(msg.timestamp_s)
        self._fuse()

    def _trim(self,now):
        cutoff=float(now)-max(1.0,10.0*self.max_dt)
        self.camera=[x for x in self.camera if float(x.timestamp_s)>=cutoff]
        self.lidar=[x for x in self.lidar if float(x.timestamp_s)>=cutoff]

    def _fuse(self):
        for a in associate(self.camera,self.lidar,self.cam,self.ex,self.max_dt,self.margin):
            msg=Observation()
            msg.schema_version="1.0"
            msg.observation_id=f"fused-{a.camera_id}-{a.lidar_id}"
            msg.agent_id="sensor_fusion_agent"
            msg.agent_version="0.1.0"
            cam=next(x for x in self.camera if x.observation_id==a.camera_id)
            lidar=next(x for x in self.lidar if x.observation_id==a.lidar_id)
            msg.timestamp_s=max(float(cam.timestamp_s),float(lidar.timestamp_s))
            msg.frame_id=lidar.frame_id
            msg.source="camera_lidar_fusion"
            msg.kind="track"
            msg.confidence=float(a.confidence)
            msg.position_m=[float(v) for v in a.position_m]
            msg.velocity_mps=[0.0,0.0,0.0]
            msg.label=a.label
            msg.provenance_json=json.dumps({
                "producer":"sensor_fusion_agent",
                "source_type":"fusion",
                "sources":[a.camera_id,a.lidar_id],
                "calibration":"external_file",
            },sort_keys=True)
            msg.attributes_json=json.dumps({
                "association_score":a.score,
                "covariance_diag_m2":list(a.covariance_diag),
                "camera_observation_id":a.camera_id,
                "lidar_observation_id":a.lidar_id,
            },sort_keys=True)
            self.pub.publish(msg)
            self.camera=[x for x in self.camera if x.observation_id!=a.camera_id]
            self.lidar=[x for x in self.lidar if x.observation_id!=a.lidar_id]

def main():
    rclpy.init()
    node=CameraLidarFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
