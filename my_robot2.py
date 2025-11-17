import omni
import numpy as np
from isaacsim.examples.interactive.base_sample import BaseSample

from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.franka import Franka 
from pxr import Sdf, Gf, UsdPhysics 
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path

from isaacsim.robot.wheeled_robots.robots import WheeledRobot 


class My_Robot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        stage = omni.usd.get_context().get_stage()
  

        my_robot_asset_path = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/Clearpath/Jackal/jackal_basic.usd'
        
        jackal = world.scene.add(
            WheeledRobot(
                prim_path="/World/Jackal",
                name="Jackal",
                wheel_dof_names=["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"],
                create_robot=True,
                usd_path=my_robot_asset_path,
                position=np.array([0, 0, 0.5])
            )
        )

        franka = world.scene.add(
            Franka(
                prim_path="/World/Franka",
                name="Franka",
                position=np.array([-0.08, 0.01, 0.7])
            )
        )

        # 3. 환경 로드 (동일)
        environment_usd_path = '/home/rokey/isaacsim/extension_examples/hello_world/officeroomwemade.usd'
        env_prim_path = "/World/Environment"
        add_reference_to_stage(usd_path=environment_usd_path, prim_path=env_prim_path)

        # 4. FIXED JOINT 생성 로직 (이전 답변에서 수정된 안정적인 USD API 사용)
        
        JOINT_JACKAL_PATH = "/World/Jackal/base_link" 
        JOINT_FRANKA_PATH = "/World/Franka/panda_link0"
        JOINT_PRIM_PATH = "/World/Jackal_Franka_FixedJoint"

        jackal_link_prim = get_prim_at_path(JOINT_JACKAL_PATH)
        franka_link_prim = get_prim_at_path(JOINT_FRANKA_PATH)
        
        if jackal_link_prim and franka_link_prim:
            
            # FixedJoint Prim 생성
            joint_prim = UsdPhysics.FixedJoint.Define(stage, JOINT_PRIM_PATH).GetPrim()
            
            # body0, body1 속성을 직접 설정하여 링크 연결
            joint_api = UsdPhysics.FixedJoint(joint_prim)
            
            # Body 0 (부모): Jackal 링크
            joint_api.CreateBody0Rel().SetTargets([jackal_link_prim.GetPath()])
            # Body 1 (자식): Franka 링크
            joint_api.CreateBody1Rel().SetTargets([franka_link_prim.GetPath()])

            # # LocalPos0 (부모/Jackal 링크 기준)
            # joint_api.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0) ) 
            
            # # LocalPos1 (자식/Franka 링크 기준)
            # joint_api.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.08, 0.01, 0.2))
            
            print(f"✅ Jackal과 Franka가 고정 조인트로 성공적으로 연결되었습니다: {JOINT_PRIM_PATH}")
        else:
            print("❌ 링크 Prim을 찾을 수 없습니다. JOINT_JACKAL_PATH 또는 JOINT_FRANKA_PATH를 확인하세요.")

        world.reset()

        return
'''
import omni
import numpy as np
from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.manipulators.examples.franka import Franka
from pxr import Sdf
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
# joint 사용 추가
from omni.physx.scripts.utils import set_targets_for_physx_joint, set_targets_for_prismatic_joint
from pxr import Gf

class My_Robot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
  
        my_robot_asset_path = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Robots/Clearpath/Jackal/jackal_basic.usd'
        
        jackal = world.scene.add(
            WheeledRobot(
                prim_path="/World/Jackal",
                name="Jackal",
                wheel_dof_names=["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"],
                create_robot=True,
                usd_path=my_robot_asset_path,
            )
        )

        franka = world.scene.add(
            Franka(
                prim_path="/World/Franka",
                name="Franka",
                position=np.array([-0.08, 0.01, 0.2])
            )
        )

        environment_usd_path = '/home/rokey/isaacsim/extension_examples/hello_world/officeroomwemade.usd'
        env_prim_path = "/World/Environment"


        add_reference_to_stage(usd_path=environment_usd_path, prim_path=env_prim_path)

        # world.reset()

        JOINT_PATH = "/World/FixedJoint_Jackal_Franka"
        JOINT_JACKAL_PATH = "/World/Jackal/base_link"
        JOINT_FRANKA_PATH = "/World/Franka/panda_link0"

        JOINT_JACKAL_PATH = "/World/Jackal/base_link" 
        
        # Franka의 베이스 링크
        JOINT_FRANKA_PATH = "/World/Franka/panda_link0"

        # Jackal의 링크 Prim과 Franka의 링크 Prim을 가져옵니다.
        jackal_link_prim = get_prim_at_path(JOINT_JACKAL_PATH)
        franka_link_prim = get_prim_at_path(JOINT_FRANKA_PATH)
        
        # 조인트 Prim이 생성될 경로
        JOINT_PRIM_PATH = "/World/Jackal/Jackal_Franka_Joint"

        # Fixed Joint 생성 및 연결
        # add_fixed_joint(stage, joint_prim_path, body0_prim, body1_prim)
        # Franka의 base_link가 Jackal의 base_link에 고정됩니다.
        # add_fixed_joint(
        #     stage=omni.usd.get_context().get_stage(), 
        #     joint_prim_path=JOINT_PRIM_PATH, 
        #     body0_prim=jackal_link_prim,
        #     body1_prim=franka_link_prim
        # )

        set_targets_for_physx_joint(
            stage=omni.usd.get_context().get_stage(), 
            joint_prim_path=JOINT_PRIM_PATH, 
            body0_prim=jackal_link_prim, # Jackal 링크
            body1_prim=franka_link_prim  # Franka 링크
        )

        # Jackal의 base_link 기준 Franka 베이스의 장착 위치 (x, y, z)
        # 이 값을 조정하여 Franka의 최종 장착 위치를 결정합니다.
        # 예: Jackal 링크 중앙에서 앞(X)으로 0.2m, 위(Z)로 0.35m
        MOUNT_OFFSET_XYZ = Gf.Vec3f(0.2, 0.0, 0.35) 
        
        joint_prim = get_prim_at_path(JOINT_PRIM_PATH)
        joint_api = set_targets_for_physx_joint(joint_prim)

        if joint_api:
            # 📌 필수 로직 2: Body0 (Jackal 링크) 기준 Franka 베이스의 위치 오프셋 설정
            # LocalPos0 (부모/Jackal): Franka 베이스가 위치할 곳 (MOUNT_OFFSET_XYZ)
            joint_api.CreateLocalPos0Attr().Set(MOUNT_OFFSET_XYZ) 
            
            # LocalPos1 (자식/Franka): Franka 자체의 조인트 연결 위치 (보통 베이스 링크의 원점인 0,0,0)
            joint_api.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            
            print(f"✅ Jackal과 Franka가 고정 조인트로 성공적으로 연결되었습니다.")

        return

'''