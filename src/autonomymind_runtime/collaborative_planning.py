"""Read-only comparison of local and shared route evidence, not execution authority."""
import math
from .navigation import OccupancyGrid, global_path
from .collaborative_evidence import occupied_sources

def compare_routes(snapshot, now, config):
    """Use one snapshot for both counterfactual plans; do not infer free space."""
    grid=OccupancyGrid(**config["map"])
    start=tuple(config["start"]);goal=tuple(config["goal"])
    clearance=config["clearance_m"]
    if len(start)!=2 or len(goal)!=2 or not all(math.isfinite(x) for x in (*start,*goal,clearance)) or clearance<=0:
        raise ValueError("invalid planning configuration")
    source=config["local_source"];remote=set(config["remote_sources"])
    if not remote or source in remote:raise ValueError("invalid sources")
    # Explicit reference prior. In a real planner this must come from validated map evidence.
    if config.get("map_prior")!="synthetic_free_reference":
        raise ValueError("reference prior required")
    grid.free={(x,y) for x in range(grid.width) for y in range(grid.height)}
    cells=occupied_sources(snapshot,grid,now)
    own={c for c,s in cells.items() if source in s}
    shared=own|{c for c,s in cells.items() if s&remote}
    inflated=clearance+grid.resolution/math.sqrt(2)
    grid.occupied=own
    local=global_path(grid,start,goal,inflated)
    grid.occupied=shared
    invalidated=[list(grid.cell(*p)) for p in local if not grid.traversable(grid.cell(*p),inflated)]
    combined=global_path(grid,start,goal,inflated)
    return {"world_revision":snapshot["revision"],"local_cells":len(own),"shared_cells":len(shared),
            "remote_additional_cells":len(shared-own),"local_route":local,"shared_route":combined,
            "invalidated_local_cells":invalidated,"route_changed":local!=combined,
            "status":"alternate_route" if local and combined and invalidated else "no_shared_route" if local and not combined else "no_demonstrated_change",
            "coverage_complete":False,"motion_authorized":False,"cpp_rta_assessed":False}
