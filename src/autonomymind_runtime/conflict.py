import math

def closest_approach(ego_pos, ego_vel, obj_pos, obj_vel, horizon_s=5.0):
    rx=float(obj_pos[0])-float(ego_pos[0])
    ry=float(obj_pos[1])-float(ego_pos[1])
    rz=float(obj_pos[2])-float(ego_pos[2])
    vx=float(obj_vel[0])-float(ego_vel[0])
    vy=float(obj_vel[1])-float(ego_vel[1])
    vz=float(obj_vel[2])-float(ego_vel[2])
    vv=vx*vx+vy*vy+vz*vz
    if vv <= 1e-12:
        t=0.0
    else:
        t=-(rx*vx+ry*vy+rz*vz)/vv
    t=max(0.0,min(float(horizon_s),t))
    cx=rx+vx*t; cy=ry+vy*t; cz=rz+vz*t
    d=math.sqrt(cx*cx+cy*cy+cz*cz)
    return {"ttc_s":t,"closest_distance_m":d}

def collision_risk(metrics, min_clearance_m=2.0):
    d=float(metrics["closest_distance_m"])
    t=float(metrics["ttc_s"])
    if d >= min_clearance_m:
        return 0.0
    proximity=max(0.0,1.0-d/min_clearance_m)
    urgency=1.0/(1.0+t)
    return max(0.0,min(1.0,0.7*proximity+0.3*urgency))
