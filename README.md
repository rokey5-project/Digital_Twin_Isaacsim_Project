# 디지털트윈 로봇 자동화 시뮬레이션 시스템 구현 프로젝트

![Tumbnail](https://img1.daumcdn.net/thumb/R1280x0/?scode=mtistory2&fname=https%3A%2F%2Fblog.kakaocdn.net%2Fdna%2Fbem3I6%2FdJMcaiuUShO%2FAAAAAAAAAAAAAAAAAAAAAPLtGiC6eyGGCSR8b-TmPBZTQXyvgqP9pe2PeYOwzeHQ%2Fimg.png%3Fcredential%3DyqXZFxpELC7KVnFOS48ylbz2pIh7yKj8%26expires%3D1764514799%26allow_ip%3D%26allow_referer%3D%26signature%3DZ9GaePCN5lV%252FuoDymPCXFJCwF%252FI%253D)

<br>

## 🗂️ 목차

### 1. [프로젝트 개요](#-프로젝트-개요)

### 2. [팀 구성 및 역할](#-팀-구성-및-역할)

### 3. [기술 스텍](#-기술-스텍)

### 4. [핵심 기능 로직](#-핵심-기능-로직)

### 5. [시연 영상](#-시연-영상)

### 6. [평가 및 피드백](#-평가-및-피드백)

<br>

## 📃 프로젝트 개요

### Isaac Sim 활용한 거리 기반 Nav2 Object Auto Attach 시스템 구현

본 프로젝트는 NVIDIA Isaac Sim 환경에서 Nova Carter 로봇이 Nav2 자율주행을 수행한 뒤,로봇이 특정 물체에 접근하면 자동으로 로봇에 부착되도록 하는 Distance-Based Attach 시스템을 구현한 작업.   
또한 사용자 정의 Office 모델에서 Occupancy Map 생성 및 Rviz를 통한 Nav2 Goal 설정 기능을 제공.


#### 📆 개발 기간 : 2025년 11월 17일 ~ 2025년 11월 21일


<br>

## 🧑‍💻 팀 구성 및 역할


| 조원 | 역할 | 담당 업무 |
|:---------:|:---------:|:--------:|
| 강동혁  | 팀장  | Officeroom Design, nav2 Nova Carter load 및 Ros2 Bridge 활성화 
| 김갑민 | 팀원 | Custom Map에 맞춰 Nav2 Nova Carter 최적화, Table과 Box 간 Attach 기능 구현 및 디버깅 |
| 김효원 | 팀원 | Custom Map의 Occupancy Map 생성, Box와 nav2 Nova Carter간 Attach 기능 및 Reparent 구현 |
| 황수빈 | 팀원 | Custom Occupancy Map 최적화 및 서버 등록, Box와 Nav2 Nova Carter간 Attach 기능 구현 |
| 황혜인 | 팀원 | Custom Officeroom USD file 생성, 시뮬레이션 환경 세팅 및 Rviz를 통한 map 주행 검증 |

<br>

## 🕹️ 기술 스택

- Python
- NVIDIA Isaac Sim
- ROS2
- Nav2
- Rivz2

<br>

## ⭐ 핵심 기능 로직

### Isaac Sim Script Editor를 사용하여 핵심 기능 실행  

![](https://img1.daumcdn.net/thumb/R1280x0/?scode=mtistory2&fname=https%3A%2F%2Fblog.kakaocdn.net%2Fdna%2Fc1i2Xo%2FdJMcadNT3Sp%2FAAAAAAAAAAAAAAAAAAAAAEVlAytKXxEcfE6cZ11LyXG02o8TEWPAew9FqiZJ5z6S%2Fimg.png%3Fcredential%3DyqXZFxpELC7KVnFOS48ylbz2pIh7yKj8%26expires%3D1764514799%26allow_ip%3D%26allow_referer%3D%26signature%3DX7EocS9%252FqtJdiqaRxDsPUuNUrl4%253D)

```python
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
        주요-기능
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
```

<br>

## 🙉 시연 영상

![시연 영상](https://blog.kakaocdn.net/dna/cxzYb2/dJMcagKGf76/AAAAAAAAAAAAAAAAAAAAADvhfxBE9Qb5KQtPHAeulpPgV8Y0roBC2kpuq9icklOw/img.gif?credential=yqXZFxpELC7KVnFOS48ylbz2pIh7yKj8&expires=1764514799&allow_ip=&allow_referer=&signature=Kb2TjjMXJxNNieoCCUWffJbwYmY%3D)


<br>

## 🔧 평가 및 피드백

### 완성도 평가
- 목표였던 '사무실 환경에서의 자율주행 및 배송 기능을 성공적으로 구현함
- Nav2 기반 경로 탐색, 목적지 지정 이동, 박스 자동 부착 기능까지 정상적으로 동작함

### 추후 개선점 및 보완할 점
- 촉박한 프로젝트 기간으로 인해 Gripper를 통한 Pick & Place 기능을 구현하지 못하고 단순한 박스의 이동으로 기능을 변경한 점이 아쉬움으로 남음
- Jakal 로봇과 Franka 로봇팔의 연결까지 구현하였기에 충분히 Pick & place 기능을 구현할 수 있을 것이라고 판단됨 

### 우리 팀이 잘한 부분 / 아쉬운 점
- 피드백 반영 및 팀 내 의사소통이 원활하게 이루어짐
- 문제 상황 발생 시 빠른 대처와 수정이 이루어짐
- 레퍼런스 부족으로 구현 방향을 잡는 데 많은 시간이 소요됨

### 느낌 점 및 경험한 성과
- Isaac Sim과 ROS2를 실제로 연동하면서 시뮬레이션 기반 로봇 개발 프로세스를 실전처럼 경험함
- Nav2 경로 계획의 구조와 파이프라인을 직접 다루며 자율주행 알고리즘의 전체 흐름을 체득함

