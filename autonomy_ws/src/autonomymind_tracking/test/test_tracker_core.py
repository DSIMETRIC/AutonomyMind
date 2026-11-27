from autonomymind_tracking.tracker_core import Detection3D, MultiObjectTracker

def det(oid,t,x,label="box"):
    return Detection3D(oid,t,label,0.9,(x,0.0,0.0),(0.04,0.04,0.04))

def test_track_id_persists_and_velocity_estimates():
    tr=MultiObjectTracker(gate_m=2.0,velocity_alpha=1.0)
    a=tr.update([det("a",0.0,0.0)],0.0)
    tid=a[0].track_id
    b=tr.update([det("b",1.0,1.0)],1.0)
    assert b[0].track_id==tid
    assert abs(b[0].velocity_mps[0]-1.0)<1e-9
    assert b[0].age==2

def test_prediction():
    tr=MultiObjectTracker(gate_m=2.0,velocity_alpha=1.0)
    tr.update([det("a",0.0,0.0)],0.0)
    t=tr.update([det("b",1.0,1.0)],1.0)[0]
    assert t.predict(3.0)==(3.0,0.0,0.0)

def test_separate_tracks_outside_gate():
    tr=MultiObjectTracker(gate_m=1.0)
    tr.update([det("a",0.0,0.0)],0.0)
    tracks=tr.update([det("b",1.0,5.0)],1.0)
    assert len(tracks)==2

def test_track_expires_after_misses():
    tr=MultiObjectTracker(gate_m=1.0,max_missed=1)
    tr.update([det("a",0.0,0.0)],0.0)
    tr.update([],1.0)
    tr.update([],2.0)
    assert tr.tracks=={}
