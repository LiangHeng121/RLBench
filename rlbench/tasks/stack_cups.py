from typing import List
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.const import colors
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary


class StackCups(Task):

    def init_task(self) -> None:
        success_sensor = ProximitySensor('success')
        self.cup1 = Shape('cup1')
        self.cup2 = Shape('cup2')

        self.boundary = SpawnBoundary([Shape('boundary')])

        self.register_graspable_objects([self.cup1, self.cup2])
        self.register_success_conditions([
            DetectedCondition(self.cup1, success_sensor),
            NothingGrasped(self.robot.gripper)
        ])


        for obj_name in ['cup3', 'cup3_visual', 'waypoint5','waypoint6','waypoint7']:
            try:
                obj = Shape(obj_name)
                obj.remove()
            except Exception as e:
                print(f"[Warning] Could not remove {obj_name}: {e}")

        for wp in ['waypoint5', 'waypoint6', 'waypoint7','waypoint8','waypoint9']:
            try:
                obj = Dummy(wp)
                obj.remove()
            except Exception as e:
                print(f"[Warning] Could not remove {wp}: {e}")

    def init_episode(self, index: int) -> List[str]:
        self.boundary.clear()
        self.boundary.sample(self.cup2, min_distance=0.05,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        self.boundary.sample(self.cup1, min_distance=0.05,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))

        return [
            'stack the two cups together',
            'put one cup inside the other',
            'place one cup into another cup'
        ]

    def variation_count(self) -> int:
        return 1
