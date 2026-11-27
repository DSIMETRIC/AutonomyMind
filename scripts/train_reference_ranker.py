"""Train a small reproducible utility model on synthetic examples, not real fleet data."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import onnx
from onnx import helper,TensorProto,numpy_helper

def main():
    p=argparse.ArgumentParser();p.add_argument("--output",required=True);a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(42)
    x=rng.uniform(0,1,(2048,4))*np.array([100,3,2,100])
    # Synthetic supervision: prefer distance, then lower age and uncertainty.
    y=-x[:,0]-.2*x[:,1]-.1*x[:,2]+.01*x[:,3]
    weights=np.linalg.lstsq(x,y,rcond=None)[0].astype(np.float32).reshape(4,1)
    graph=helper.make_graph([helper.make_node("MatMul",["features","weights"],["scores"])],"synthetic_reference_ranker",
        [helper.make_tensor_value_info("features",TensorProto.FLOAT,["N",4])],
        [helper.make_tensor_value_info("scores",TensorProto.FLOAT,["N",1])],
        [numpy_helper.from_array(weights,"weights")])
    model=helper.make_model(graph,opset_imports=[helper.make_opsetid("",17)],ir_version=9)
    onnx.checker.check_model(model)
    artifact=out/"ranker.onnx";onnx.save(model,artifact)
    config={"id":"synthetic-utility-ranker","version":"1","path":"/models/ranker.onnx","sha256":hashlib.sha256(artifact.read_bytes()).hexdigest()}
    (out/"ranker.json").write_text(json.dumps(config,indent=2)+"\n")
    print(json.dumps(config,indent=2))
if __name__=="__main__":main()
