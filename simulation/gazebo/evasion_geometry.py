"""Reference unicycle candidates, world ENU offsets."""
import math

def candidates(yaw):
    return [("GO",0.25,0.0),("LEFT",0.20,0.55),("RIGHT",0.20,-0.55),("BRAKE",0.0,0.0)]

def offsets(yaw,v,w,dt=0.25,count=9):
    x=[];y=[]
    for i in range(count):
        t=i*dt
        if abs(w)<1e-9:
            x.append(v*t*math.cos(yaw));y.append(v*t*math.sin(yaw))
        else:
            x.append(v/w*(math.sin(yaw+w*t)-math.sin(yaw)))
            y.append(v/w*(math.cos(yaw)-math.cos(yaw+w*t)))
    return x,y

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
