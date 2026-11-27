"""Planar, stationary Gazebo scan projection. No inferred object velocities."""
import math

def project_scan(ranges,angle_min,angle_increment,range_min,range_max,pose,offset=(.45,0.,.65)):
    x,y,z,yaw=pose
    if not all(math.isfinite(v) for v in (*pose,*offset,angle_min,angle_increment,range_min,range_max)) or range_min<=0 or range_max<=range_min or angle_increment<=0 or len(ranges)>256:
        raise ValueError("invalid scan geometry")
    ox=x+offset[0]*math.cos(yaw)-offset[1]*math.sin(yaw)
    oy=y+offset[0]*math.sin(yaw)+offset[1]*math.cos(yaw)
    hits=[]
    for i,r in enumerate(ranges):
        if math.isfinite(r) and range_min<=r<range_max:
            a=yaw+angle_min+i*angle_increment
            hits.append((i,[ox+r*math.cos(a),oy+r*math.sin(a),z+offset[2]]))
    return hits

def route_from_hits(grid,start,goal,hits,clearance):
    """Known reference map prior plus observed returns; not sensor-certified free space."""
    from .navigation import global_path
    grid.occupied={grid.cell(p[0],p[1]) for p in hits if grid.inside(grid.cell(p[0],p[1]))}
    return global_path(grid,start,goal,clearance+grid.resolution/math.sqrt(2))
