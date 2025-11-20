import omni
from pxr import UsdGeom, Sdf, Gf, Usd, UsdPhysics


stage = omni.usd.get_context().get_stage()

BASE_LINK_PATH = "/World/Nova_Carter_ROS/chassis_link/base_link"
BOX_01 = "/World/Cardbox_A1/Cardbox_A1"
BOX_02 = "/World/Cardbox_D3/Cardbox_D3"
TARGET_01 = "/World/officewemadeday2/officeroomwemade/FlatGrid/GroundPlane/destinations/Danny_01/DannyTable"
TARGET_02 = "/World/officewemadeday2/officeroomwemade/FlatGrid/GroundPlane/destinations/Danny_02/DannyTable"
BOX_01_JOINT_PATH = f"{BOX_01}/FixedJoint"
BOX_02_JOINT_PATH = f"{BOX_02}/FixedJoint"
TARGET_01_JOINT_PATH = f"{TARGET_01}/FixedJoint"
TARGET_02_JOINT_PATH = f"{TARGET_02}/FixedJoint"
limit_distance = 1.5

def get_world_position(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    xform = UsdGeom.Xformable(prim)
    mat = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return mat.ExtractTranslation()

# 상자 획득 상태 (상자와 로봇 부착 여부)
is_attached = False
# target_01 상자 전달 완료 상태
target_01_state = False
# target_02 상자 전달 완료 상태
target_02_state = False

def on_update(dt):
    global is_attached
    global target_01_state
    global target_02_state

    # 로봇, 상자, 목적지 위치 데이터 정보
    robot_pos = get_world_position(BASE_LINK_PATH)
    box_01_pos  = get_world_position(BOX_01)
    box_02_pos  = get_world_position(BOX_02)
    target_01_pos = get_world_position(TARGET_01)
    target_02_pos = get_world_position(TARGET_02)


    if robot_pos is None or target_01_pos is None or target_02_pos is None:
        print("object is not found")
        return

    # 상자, 목적지 사이의 거리 계산
    box_01_dist = (box_01_pos - robot_pos).GetLength()
    box_02_dist = (box_02_pos - robot_pos).GetLength()
    target_01_dist = (target_01_pos - robot_pos).GetLength()
    target_02_dist = (target_02_pos - robot_pos).GetLength()

    # box1 전달 완료 x, 
    # target_01와 robot 사이의 거리가 limit_distance 이하
    # 윗 조건을 만족한 경우 box_01를 target_01에 부착 (상자 전달 완료)
    if not target_01_state and target_01_dist < limit_distance:
        stage.RemovePrim(BOX_01_JOINT_PATH)

        input0 = Sdf.Path(TARGET_01)
        input1 = Sdf.Path(BOX_01)
        joint_prim = UsdPhysics.FixedJoint.Define(stage, TARGET_01_JOINT_PATH)
        joint_prim.CreateBody0Rel().SetTargets([input0])
        joint_prim.CreateBody1Rel().SetTargets([input1])

        is_attached = False
        target_01_state = True

        print("📦 box1 complete!!")

    # box2 전달 완료 x, 
    # target_02와 robot 사이의 거리가 limit_distance 이하
    # 윗 조건을 만족한 경우 box_01를 target_01에 부착 (상자 전달 완료)
    elif not target_02_state and target_02_dist < limit_distance:
        stage.RemovePrim(BOX_02_JOINT_PATH)

        input0 = Sdf.Path(TARGET_02)
        input1 = Sdf.Path(BOX_02) 
        joint_prim = UsdPhysics.FixedJoint.Define(stage, TARGET_02_JOINT_PATH)
        joint_prim.CreateBody0Rel().SetTargets([input0])
        joint_prim.CreateBody1Rel().SetTargets([input1])

        is_attached = False
        target_02_state = True
        
        print("📦 box2 complete!!")

    else:
        if not is_attached:
            # box_01 전달 x, 
            # box_01와 robot 사이의 거리가 limit_distance 이하
            # 윗 조건을 만족한 경우 box_01를 robot에 부착 (상자 획득 완료)
            if not target_01_state and box_01_dist < limit_distance:
                input0 = Sdf.Path(BASE_LINK_PATH)
                input1 = Sdf.Path(BOX_01)

                joint_prim = UsdPhysics.FixedJoint.Define(stage, BOX_01_JOINT_PATH)
                joint_prim.CreateBody0Rel().SetTargets([input0])
                joint_prim.CreateBody1Rel().SetTargets([input1])
                joint_prim.CreateLocalPos0Attr().Set(Gf.Vec3f(-0.2, 0.0, 1.0))

                print("📦 box1 attached!!")
                is_attached = True
                
            # box_01 전달 o, 
            # box_02 전달 x
            # box_02와 robot 사이의 거리가 limit_distance 이하
            # 윗 조건을 만족한 경우 box_02를 robot에 부착 (상자 획득 완료)
            elif target_01_state and not target_02_state and box_02_dist < limit_distance:
                input0 = Sdf.Path(BASE_LINK_PATH)
                input1 = Sdf.Path(BOX_02)

                joint_prim = UsdPhysics.FixedJoint.Define(stage, BOX_02_JOINT_PATH)
                joint_prim.CreateBody0Rel().SetTargets([input0])
                joint_prim.CreateBody1Rel().SetTargets([input1])
                joint_prim.CreateLocalPos0Attr().Set(Gf.Vec3f(-0.2, 0.0, 1.0))

                print("📦 box2 attached!!")
                is_attached = True


subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)