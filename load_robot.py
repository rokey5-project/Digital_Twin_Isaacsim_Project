import os
from omni.isaac.core.utils.stage import add_reference_to_stage



def Load_Robot():
        
    root_path = os.path.dirname(__file__)

    # 환경 로드
    env_prim_path = "/World/Environment"
    environment_usd_path = os.path.join(root_path, 'usd/officeroomwemade.usd')
    add_reference_to_stage(usd_path=environment_usd_path, prim_path=env_prim_path)

    # 로봇 로드
    robot_prim_path = "/World/Robot"
    robot_usd_path = os.path.join(root_path, 'usd/my_robot.usd')
    add_reference_to_stage(usd_path=robot_usd_path, prim_path=robot_prim_path)

