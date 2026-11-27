"""Reference agent event replay wrapping existing mission coordinator."""
import argparse
import json
from pathlib import Path
from .agent_runtime import Runtime,AgentSpec
from .industrial_pilot import run

def replay(scenario):
    runtime=Runtime()
    output={}
    def mission(e,emit):
        emit("coordinator","mission.request",e.payload,30)
    def coordinate(e,emit):
        result=run(e.payload)
        emit("evaluation","mission.report",result,30)
    def evaluate(e,emit):
        output.update(e.payload)
    runtime.register(AgentSpec("operator","input","1",(),(),("mission.submit",)),lambda e,emit:None,60)
    runtime.register(AgentSpec("mission","mission","1",("mission",),("mission.submit",),("mission.request",)),mission,60)
    runtime.register(AgentSpec("coordinator","coordination","1",("allocate",),("mission.request",),("mission.report",)),coordinate,60)
    runtime.register(AgentSpec("evaluation","evaluation","1",("audit",),("mission.report",),()),evaluate,60)
    runtime.send("operator","mission","mission.submit",scenario,30)
    runtime.drain()
    if not output:raise RuntimeError("mission report missing")
    return {"report":output,"agent_trace":runtime.trace,"registered_agents":list(runtime.specs),"transport":"in_process_reference"}

def main():
    p=argparse.ArgumentParser();p.add_argument("--scenario",required=True);p.add_argument("--output",required=True);a=p.parse_args()
    result=replay(json.loads(Path(a.scenario).read_text()))
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps(result["report"]["metrics"],indent=2))
if __name__=="__main__":main()
