"""Union of occupied evidence cells; preserves every contributing source."""
from .shared_world import number,vector

def occupied_sources(snapshot,grid,now,max_age=2.):
    if snapshot.get("frame_id")!="world_enu" or not number(now) or not number(max_age) or max_age<=0:raise ValueError("invalid frame or time")
    cells={}
    for record in snapshot["observations"]:
        p=record["payload"]
        if p.get("label")!="lidar_surface_return" or p.get("frame_id")!="world_enu":continue
        if not number(record["source_time"]) or not number(record["expires"]) or not 0<=now-record["source_time"]<max_age or now>=record["expires"]:continue
        if not vector(p.get("position_m"),3):raise ValueError("invalid point")
        c=grid.cell(*p["position_m"][:2])
        if grid.inside(c):cells.setdefault(c,set()).add(record["sender"])
    return cells
