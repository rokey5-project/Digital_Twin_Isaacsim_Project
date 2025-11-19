import omni
from pxr import UsdGeom, Sdf, Gf, Usd, UsdPhysics


stage = omni.usd.get_context().get_stage()

CUBE_PATH = "/World/Cube"
BASE_LINK_PATH = "/World/Nova_Carter_ROS/chassis_link/base_link"

def get_world_position(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    xform = UsdGeom.Xformable(prim)
    mat = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return mat.ExtractTranslation()

attached = False

def on_update(dt):
    global attached

    robot_pos = get_world_position(BASE_LINK_PATH)
    cube_pos  = get_world_position(CUBE_PATH)


    if robot_pos is None or cube_pos is None:
        return

    dist = (cube_pos - robot_pos).GetLength()

    if (not attached) and dist < 1.5:
        print("📦 Attaching cube!")

        old = Sdf.Path(CUBE_PATH)
        new = Sdf.Path(BASE_LINK_PATH)
        
        joint_prim = UsdPhysics.FixedJoint.Define(stage, '/World/Cube/Joints')
        joint_prim.CreateBody0Rel().SetTargets([new])
        joint_prim.CreateBody1Rel().SetTargets([old])

        joint_prim.CreateLocalPos0Attr().Set(Gf.Vec3f(-0.2, 0.0, 1.0))

        attached = True


subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
