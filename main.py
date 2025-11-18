# depencency
import omni
from isaacsim.examples.interactive.base_sample import BaseSample

# 로직 파일
from .load_robot import Load_Robot


class Main(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        stage = omni.usd.get_context().get_stage()
        Load_Robot()
