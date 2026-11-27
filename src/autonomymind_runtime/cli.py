import argparse, json, pathlib
from .isaac_adapter import IsaacScenarioAdapter
from .pipeline import AutonomyPipeline
from .dynamic_pipeline import DynamicAvoidancePipeline
from .cooperative_pipeline import CooperativePipeline
from .mission_demo import run_mission
from .replay import summarize_trace

def contract_check():
    root=pathlib.Path("/workspace/orchestration/contracts")
    required=["observation.schema.json","world_state.schema.json","agent_proposal.schema.json","safety_decision.schema.json"]
    missing=[x for x in required if not (root/x).exists()]
    if missing:
        raise SystemExit("missing contracts: "+",".join(missing))
    for name in required:
        data=json.loads((root/name).read_text(encoding="utf-8"))
        if data.get("$schema")!="https://json-schema.org/draft/2020-12/schema":
            raise SystemExit(f"{name}: unsupported or missing JSON Schema dialect")
        if data.get("type")!="object":
            raise SystemExit(f"{name}: root type must be object")
        if "schema_version" not in data.get("properties",{}):
            raise SystemExit(f"{name}: schema_version property missing")
    print("contract-check: PASS")

def demo(scenario: str, outbox: str):
    frames=IsaacScenarioAdapter().load(scenario)
    pipeline=AutonomyPipeline(outbox)
    results=[pipeline.step(f) for f in frames]
    print(json.dumps(results,indent=2,sort_keys=True))
    if not results[-1]["safety_decision"]["deterministic"]:
        raise SystemExit(2)

def dynamic_demo(scenario: str, trace: str):
    data=json.loads(pathlib.Path(scenario).read_text(encoding="utf-8"))
    frames=data.get("frames",[])
    if not frames:
        raise SystemExit("dynamic scenario contains no frames")
    pipe=DynamicAvoidancePipeline(trace)
    results=[pipe.step(f) for f in frames]
    print(json.dumps(results,indent=2,sort_keys=True))
    final=results[-1]
    if not (final["selected_safety"] or {}).get("accepted",False):
        raise SystemExit(3)

def cooperative_demo(scenario: str):
    data=json.loads(pathlib.Path(scenario).read_text(encoding="utf-8"))
    frames=data.get("frames",[])
    if not frames:
        raise SystemExit("cooperative scenario contains no frames")
    pipe=CooperativePipeline()
    results=[pipe.step(f) for f in frames]
    print(json.dumps(results,indent=2,sort_keys=True))

def mission_demo(scenario: str, trace: str):
    events=run_mission(scenario,trace)
    print(json.dumps({"events":len(events),"trace":trace,"summary":summarize_trace(trace)},indent=2,sort_keys=True))

def replay_summary(trace: str):
    print(json.dumps(summarize_trace(trace),indent=2,sort_keys=True))

def main():
    p=argparse.ArgumentParser()
    sub=p.add_subparsers(dest="cmd",required=True)
    d=sub.add_parser("demo")
    d.add_argument("--scenario",required=True)
    d.add_argument("--outbox",default="/tmp/autonomymind-outbox")
    md=sub.add_parser("mission-demo")
    md.add_argument("--scenario",required=True)
    md.add_argument("--trace",required=True)
    rs=sub.add_parser("replay-summary")
    rs.add_argument("--trace",required=True)
    cd=sub.add_parser("cooperative-demo")
    cd.add_argument("--scenario",required=True)
    dd=sub.add_parser("dynamic-demo")
    dd.add_argument("--scenario",required=True)
    dd.add_argument("--trace",default="/tmp/autonomymind-dynamic-trace.jsonl")
    sub.add_parser("contract-check")
    a=p.parse_args()
    if a.cmd=="demo":
        demo(a.scenario,a.outbox)
    elif a.cmd=="dynamic-demo":
        dynamic_demo(a.scenario,a.trace)
    elif a.cmd=="cooperative-demo":
        cooperative_demo(a.scenario)
    elif a.cmd=="mission-demo":
        mission_demo(a.scenario,a.trace)
    elif a.cmd=="replay-summary":
        replay_summary(a.trace)
    else:
        contract_check()

if __name__=="__main__":
    main()
