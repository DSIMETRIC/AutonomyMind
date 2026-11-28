"""Bounded deterministic multi-goal evaluation on one frozen evidence snapshot."""
from collections import Counter
import math
from .navigation import OccupancyGrid
from .collaborative_evidence import occupied_sources
from .collaborative_planning import compare_routes

def benchmark(snapshot, now, config):
    grid=OccupancyGrid(**config["map"])
    axes=config["goal_axes"]
    if len(axes)!=2 or not all(axes) or len(axes[0])*len(axes[1])>128:
        raise ValueError("expected 1..128 Cartesian goals")
    goals=[(x,y) for x in axes[0] for y in axes[1]]
    if any(not all(math.isfinite(v) for v in g) or not grid.inside(grid.cell(*g)) for g in goals):
        raise ValueError("invalid goal")
    if len({grid.cell(*g) for g in goals})!=len(goals):
        raise ValueError("duplicate goal cell")
    cells=occupied_sources(snapshot,grid,now)
    source=config["local_source"];remote=set(config["remote_sources"])
    own={c for c,s in cells.items() if source in s}
    extra={c for c,s in cells.items() if s&remote}-own
    clearance=config["clearance_m"]+grid.resolution/math.sqrt(2)
    cases=[]
    for goal in goals:
        r=compare_routes(snapshot,now,{**config,"goal":goal})
        if not r["local_route"]:
            status="local_unreachable"
        elif not r["shared_route"]:
            status="no_shared_route"
        elif r["invalidated_local_cells"]:
            status="alternate_route"
        elif not r["route_changed"]:
            status="unchanged"
        else:
            status="changed_without_invalidation"
        causes=[]
        for c in sorted(extra):
            affected=[p for p in r["invalidated_local_cells"] if math.hypot(max(0,abs(p[0]-c[0])-.5),max(0,abs(p[1]-c[1])-.5))*grid.resolution<=clearance]
            if affected:
                causes.append({"cell":list(c),"sources":sorted(cells[c]&remote),"affected_route_cells":affected})
        cases.append({"goal":list(goal),"status":status,"causes":causes,**{k:r[k] for k in ("local_route","shared_route","invalidated_local_cells","route_changed")}})
    counts=dict(Counter(c["status"] for c in cases))
    return {"world_revision":snapshot["revision"],"snapshot_evaluated_at":now,
            "goals_total":len(goals),"counts":counts,"local_cells":len(own),"remote_additional_cells":len(extra),
            "remote_cells":[{"cell":list(c),"sources":sorted(cells[c]&remote)} for c in sorted(extra)],
            "cases":cases,"coverage_complete":False,"motion_authorized":False,"cpp_rta_assessed":False}
