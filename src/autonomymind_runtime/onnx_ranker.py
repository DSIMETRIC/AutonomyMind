"""Local ONNX advisory ranking, with verified artifact bytes and explicit feature contract."""
import hashlib
import math
from pathlib import Path
from .model_backend import ModelRegistry, ModelSpec

FEATURES=("distance_m","telemetry_age_s","uncertainty_m","task_priority")
class OnnxRanker:
    def __init__(self,path,sha256):
        import numpy as np
        import onnxruntime as ort
        artifact=Path(path).read_bytes()
        if hashlib.sha256(artifact).hexdigest()!=sha256:
            raise ValueError("model digest mismatch")
        options=ort.SessionOptions()
        options.intra_op_num_threads=1
        options.inter_op_num_threads=1
        self.session=ort.InferenceSession(artifact,sess_options=options,providers=["CPUExecutionProvider"])
        self.np=np
        inputs=self.session.get_inputs();outputs=self.session.get_outputs()
        if len(inputs)!=1 or len(outputs)!=1 or inputs[0].name!="features" or outputs[0].name!="scores" or inputs[0].type!="tensor(float)" or outputs[0].type!="tensor(float)":
            raise ValueError("incompatible model interface")
        if len(inputs[0].shape)!=2 or inputs[0].shape[1]!=len(FEATURES):
            raise ValueError("incompatible feature shape")
    def infer(self,request):
        ids=request["candidate_ids"];features=request["features"]
        if not 1<=len(ids)<=256 or set(features)!=set(ids):raise ValueError("invalid candidate features")
        rows=[]
        for id in ids:
            row=features[id]
            if set(row)!=set(FEATURES) or not all(isinstance(row[k],(int,float)) and not isinstance(row[k],bool) and math.isfinite(row[k]) and abs(row[k])<=1e6 for k in FEATURES):
                raise ValueError("invalid feature values")
            rows.append([row[k] for k in FEATURES])
        scores=self.session.run(["scores"],{"features":self.np.asarray(rows,dtype=self.np.float32)})[0]
        if scores.shape not in ((len(ids),),(len(ids),1)) or not self.np.isfinite(scores).all():
            raise ValueError("invalid model scores")
        ranked=sorted(zip(ids,scores.reshape(-1)),key=lambda item:(-float(item[1]),item[0]))
        return {"ranked_ids":[x[0] for x in ranked],"explanation":"ONNX candidate utility ranking; not a safety assessment."}

def load_ranker(config):
    spec=ModelSpec(config["id"],config["version"],"onnx-cpu",config["sha256"])
    registry=ModelRegistry()
    registry.register(spec,OnnxRanker(config["path"],spec.artifact_sha256))
    def rank(request):return registry.advise(spec.id,spec.version,request)
    return rank
