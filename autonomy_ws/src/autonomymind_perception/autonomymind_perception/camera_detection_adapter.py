import json
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from autonomymind_interfaces.msg import Observation
from .common import make_observation

class CameraDetectionAdapter(Node):
    def __init__(self):
        super().__init__("autonomymind_camera_detection_adapter")
        self.declare_parameter("input_topic","/detections")
        self.declare_parameter("output_topic","/autonomymind/observations/camera")
        self.declare_parameter("source","camera_detector")
        self.declare_parameter("frame_id_fallback","camera")
        self.declare_parameter("min_confidence",0.25)
        input_topic=self.get_parameter("input_topic").value
        output_topic=self.get_parameter("output_topic").value
        self.source=str(self.get_parameter("source").value)
        self.frame_fallback=str(self.get_parameter("frame_id_fallback").value)
        self.min_conf=float(self.get_parameter("min_confidence").value)
        self.pub=self.create_publisher(Observation,output_topic,20)
        self.sub=self.create_subscription(Detection2DArray,input_topic,self.on_detections,20)

    def on_detections(self,msg: Detection2DArray):
        stamp=float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
        frame=msg.header.frame_id or self.frame_fallback
        for det in msg.detections:
            if not det.results:
                continue
            best=max(det.results,key=lambda r: float(r.hypothesis.score))
            confidence=float(best.hypothesis.score)
            if confidence < self.min_conf:
                continue
            label=str(best.hypothesis.class_id)
            center=det.bbox.center.position
            attrs={
                "bbox_center_x_px":float(center.x),
                "bbox_center_y_px":float(center.y),
                "bbox_size_x_px":float(det.bbox.size_x),
                "bbox_size_y_px":float(det.bbox.size_y),
                "bbox_xyxy":[float(center.x-det.bbox.size_x/2),float(center.y-det.bbox.size_y/2),
                             float(center.x+det.bbox.size_x/2),float(center.y+det.bbox.size_y/2)],
                "detection_source":"vision_msgs/Detection2DArray",
            }
            obs=make_observation(
                prefix="cam",agent_id="perception_camera_adapter",timestamp_s=stamp,
                frame_id=frame,source=self.source,kind="detection",confidence=confidence,
                position=[0.0,0.0,0.0],velocity=[0.0,0.0,0.0],label=label,
                provenance={"producer":"perception_camera_adapter","source_type":"model"},
                attributes=attrs,
            )
            self.pub.publish(obs)

def main():
    rclpy.init()
    node=CameraDetectionAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
