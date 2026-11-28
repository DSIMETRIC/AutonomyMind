"""Read-only local trajectory proposals from shared stationary reference evidence."""
import math
from dataclasses import replace
from .navigation import OccupancyGrid,Pose2,NavigationProfile,global_path,local_candidates
from .collaborative_evidence import occupied_sources

def propose(snapshot,now,pose,config):
    grid=OccupancyGrid(**config["map"])
    if config["map_prior"]!="synthetic_free_reference":raise ValueError("explicit reference prior required")
    grid.free={(x,y) for x in range(grid.width) for y in range(grid.height)}
    cells=occupied_sources(snapshot,grid,now)
    own={c for c,s in cells.items() if config["local_source"] in s}
    remote={c for c,s in cells.items() if set(config["remote_sources"])&s}
    if not own or not remote:raise ValueError("both fresh sensor sources required")
    profile=NavigationProfile(**config["profile"])
    # Use identical inflated clearance in global and local checks.
    profile=replace(profile,margin_m=profile.margin_m+grid.resolution/math.sqrt(2))
    clearance=profile.radius_m+profile.margin_m
    grid.occupied=own
    before=global_path(grid,(pose.x,pose.y),config["goal"],clearance)
    grid.occupied=own|remote
    route=global_path(grid,(pose.x,pose.y),config["goal"],clearance)
    candidates=local_candidates(grid,pose,route[1],profile) if len(route)>1 else []
    return {"world_revision":snapshot["revision"],"local_route":before,"shared_route":route,"remote_additional_cells":len(remote-own),
            "occupied_cells":[{"cell":list(c),"sources":sorted(cells[c]),"center":grid.center(c)} for c in sorted(grid.occupied)],
            "candidates":candidates,"coverage_complete":False,"motion_authorized":False}
