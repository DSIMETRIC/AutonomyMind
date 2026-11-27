import argparse
import json
from pathlib import Path

def _deps():
    import numpy as np
    from PIL import Image
    import onnxruntime as ort
    return np,Image,ort

def iou_xyxy(a,b):
    x1=max(a[0],b[0]); y1=max(a[1],b[1]); x2=min(a[2],b[2]); y2=min(a[3],b[3])
    inter=max(0.0,x2-x1)*max(0.0,y2-y1)
    aa=max(0.0,a[2]-a[0])*max(0.0,a[3]-a[1])
    bb=max(0.0,b[2]-b[0])*max(0.0,b[3]-b[1])
    den=aa+bb-inter
    return inter/den if den>0 else 0.0

def nms(detections,iou_threshold):
    kept=[]
    for d in sorted(detections,key=lambda x:x["confidence"],reverse=True):
        if all(d["class_id"]!=k["class_id"] or iou_xyxy(d["bbox_xyxy"],k["bbox_xyxy"])<iou_threshold for k in kept):
            kept.append(d)
    return kept

def decode_yolo(output, image_w, image_h, input_w, input_h, conf_threshold=0.25, iou_threshold=0.45):
    np,_,_=_deps()
    arr=np.asarray(output)
    if arr.ndim==3:
        arr=arr[0]
    if arr.ndim!=2:
        raise ValueError(f"expected 2D/3D YOLO output, got shape {arr.shape}")
    if arr.shape[0] < arr.shape[1] and arr.shape[0] <= 512:
        arr=arr.T
    if arr.shape[1] < 5:
        raise ValueError("YOLO rows must contain box + class scores")
    sx=float(image_w)/float(input_w)
    sy=float(image_h)/float(input_h)
    detections=[]
    for row in arr:
        cx,cy,w,h=[float(v) for v in row[:4]]
        scores=row[4:]
        cls=int(np.argmax(scores))
        conf=float(scores[cls])
        if conf < conf_threshold:
            continue
        x1=max(0.0,(cx-w/2.0)*sx); y1=max(0.0,(cy-h/2.0)*sy)
        x2=min(float(image_w),(cx+w/2.0)*sx); y2=min(float(image_h),(cy+h/2.0)*sy)
        detections.append({"class_id":cls,"confidence":conf,"bbox_xyxy":[x1,y1,x2,y2]})
    return nms(detections,iou_threshold)

def run(model_path,image_path,labels_path=None,conf=0.25,iou=0.45):
    np,Image,ort=_deps()
    image=Image.open(image_path).convert("RGB")
    original_w,original_h=image.size
    session=ort.InferenceSession(str(model_path),providers=["CPUExecutionProvider"])
    inp=session.get_inputs()[0]
    shape=inp.shape
    input_h=int(shape[2]) if isinstance(shape[2],int) else 640
    input_w=int(shape[3]) if isinstance(shape[3],int) else 640
    resized=image.resize((input_w,input_h))
    tensor=np.asarray(resized,dtype=np.float32)/255.0
    tensor=np.transpose(tensor,(2,0,1))[None,...]
    outputs=session.run(None,{inp.name:tensor})
    detections=decode_yolo(outputs[0],original_w,original_h,input_w,input_h,conf,iou)
    labels=[]
    if labels_path:
        labels=[x.strip() for x in Path(labels_path).read_text(encoding="utf-8").splitlines() if x.strip()]
    observations=[]
    for i,d in enumerate(detections):
        label=labels[d["class_id"]] if d["class_id"] < len(labels) else str(d["class_id"])
        observations.append({
            "schema_version":"1.0",
            "observation_id":f"onnx-{i}",
            "agent_id":"perception_onnx_agent",
            "agent_version":"0.1.0",
            "timestamp_s":0.0,
            "frame_id":"camera",
            "source":"onnxruntime",
            "kind":"detection",
            "confidence":d["confidence"],
            "position_m":[0.0,0.0,0.0],
            "velocity_mps":[0.0,0.0,0.0],
            "label":label,
            "attributes":{"bbox_xyxy":d["bbox_xyxy"],"image_width":original_w,"image_height":original_h},
            "provenance":{"producer":"perception_onnx_agent","source_type":"model","model_path":Path(model_path).name}
        })
    return observations

def main():
    p=argparse.ArgumentParser(description="Run a YOLO-style ONNX detector and emit AutonomyMind Observation JSON")
    p.add_argument("--model",required=True)
    p.add_argument("--image",required=True)
    p.add_argument("--labels")
    p.add_argument("--confidence",type=float,default=0.25)
    p.add_argument("--iou",type=float,default=0.45)
    p.add_argument("--output")
    a=p.parse_args()
    result=run(a.model,a.image,a.labels,a.confidence,a.iou)
    payload=json.dumps(result,indent=2,sort_keys=True)
    if a.output:
        Path(a.output).write_text(payload+"\n",encoding="utf-8")
    else:
        print(payload)

if __name__=="__main__":
    main()
