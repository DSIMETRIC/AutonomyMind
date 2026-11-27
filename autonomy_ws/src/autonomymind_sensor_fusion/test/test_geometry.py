from autonomymind_sensor_fusion.geometry import CameraModel, Extrinsics, transform_point, project_point, point_in_bbox

def test_projection_identity():
    cam=CameraModel(100,100,50,50)
    ex=Extrinsics.identity()
    p=transform_point((1,0,2),ex)
    uv=project_point(p,cam)
    assert uv==(100.0,50.0)
    assert point_in_bbox(uv,[90,40,110,60])
