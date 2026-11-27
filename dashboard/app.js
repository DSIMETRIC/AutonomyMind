let events=[],idx=0;
const $=id=>document.getElementById(id);
function vec(v){return Array.isArray(v)?v.map(x=>Number(x).toFixed(2)).join(", "):"—"}
function render(){
  const e=events[idx]; if(!e)return;
  $("step").textContent=`Step ${idx+1}/${events.length} · t=${e.timestamp_s}s`;
  const coop=e.cooperative||{}, sw=coop.shared_world||{}, dyn=e.dynamic_avoidance||{};
  $("robots").innerHTML=(sw.robots||[]).map(r=>`<tr><td>${r.robot_id}</td><td>${r.robot_type}</td><td>${vec(r.position_m)}</td></tr>`).join("");
  $("tracks").innerHTML=(sw.tracks||[]).map(t=>`<tr><td>${t.track_id}</td><td>${t.label}</td><td>${vec(t.position_m)}</td></tr>`).join("");
  $("assignments").textContent=JSON.stringify(coop.assignments||{},null,2);
  $("coordination").textContent=JSON.stringify({conflicts:coop.conflicts||[],directives:coop.coordination_directives||[]},null,2);
  const selected=((dyn.selected_proposal||{}).command||{}).maneuver||"—";
  $("maneuver").textContent=selected;
  $("candidates").innerHTML=(dyn.candidates||[]).map(x=>{
    const p=x.proposal||{},s=x.safety||{},m=(p.command||{}).maneuver||"—";
    return `<tr><td>${m}</td><td>${p.priority??"—"}</td><td>${Number(p.confidence||0).toFixed(3)}</td><td class="${s.accepted?"ok":"bad"}">${s.status||"—"}</td></tr>`;
  }).join("");
  $("raw").textContent=JSON.stringify(e,null,2);
}
async function load(){
  const text=await (await fetch("/trace/mission_trace.jsonl",{cache:"no-store"})).text();
  events=text.trim().split(/\n+/).filter(Boolean).map(JSON.parse);
  $("status").textContent=`${events.length} trace events loaded`;
  render();
}
$("prev").onclick=()=>{idx=Math.max(0,idx-1);render()};
$("next").onclick=()=>{idx=Math.min(events.length-1,idx+1);render()};
load().catch(e=>$("status").textContent="Trace unavailable: "+e);
