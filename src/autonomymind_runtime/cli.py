import argparse, json, pathlib
from .isaac_adapter import IsaacScenarioAdapter
from .pipeline import AutonomyPipeline

def contract_check():
    root=pathlib.Path("/workspace/orchestration/contracts")
    required=["observation.schema.json","world_state.schema.json","agent_proposal.schema.json","safety_decision.schema.json"]
    missing=[x for x in required if not (root/x).exists()]
    if missing: raise SystemExit("missing contracts: "+",".join(missing))
    print("contract-check: PASS")

def demo(scenario: str, outbox: str):
    frames=IsaacScenarioAdapter().load(scenario)
    pipeline=AutonomyPipeline(outbox)
    results=[pipeline.step(f) for f in frames]
    print(json.dumps(results,indent=2,sort_keys=True))
    if not results[-1]["safety_decision"]["deterministic"]: raise SystemExit(2)

def main():
    p=argparse.ArgumentParser()
    sub=p.add_subparsers(dest="cmd",required=True)
    d=sub.add_parser("demo"); d.add_argument("--scenario",required=True); d.add_argument("--outbox",default="/tmp/autonomymind-outbox")
    sub.add_parser("contract-check")
    a=p.parse_args()
    if a.cmd=="demo": demo(a.scenario,a.outbox)
    else: contract_check()

if __name__=="__main__": main()
