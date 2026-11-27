import json
from autonomymind_perception.common import make_observation

def test_make_observation_contract():
    msg=make_observation(prefix="t",agent_id="test",timestamp_s=1.25,frame_id="base",
        source="unit",kind="detection",confidence=1.2,position=[1,2,3],velocity=[0,0,0],
        label="box",provenance={"producer":"test","source_type":"model"},
        attributes={"distance_m":1.0})
    assert msg.schema_version=="1.0"
    assert msg.confidence==1.0
    assert json.loads(msg.provenance_json)["producer"]=="test"
    assert json.loads(msg.attributes_json)["distance_m"]==1.0
