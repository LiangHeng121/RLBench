from typing import List, Tuple
import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition, NothingGrasped, \
    OrConditions
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task


class PlaceCups(Task):

    def init_task(self) -> None:
        self._cups = Shape('mug0')
        self._spokes = Shape('place_cups_holder_spoke0')
        self._cups_boundary = Shape('mug_boundary')
        self._w1 = Dummy('waypoint1')
        self._w4 = Dummy('waypoint4')
        success_detectors = ProximitySensor('success_detector0')
        self._on_peg_conditions = DetectedCondition(self._cups, success_detectors) 
        self.register_graspable_objects([self._cups])
        self._initial_relative_cup = self._w1.get_pose(self._cups)
        self._initial_relative_spoke = self._w4.get_pose(self._spokes)
        
        for extra_mug in ['mug1', 'mug2','mug3']:
            try:
                mug = Shape(extra_mug)
                mug.remove()
            except Exception:
                print(f"Warning: {extra_mug} not found.")

    def init_episode(self, index: int) -> List[str]:
        holder_base = Shape('place_cups_holder_base')
        holder_base.set_orientation([0.0, 0.0, -math.pi / 2], relative_to=None)
        self._cups_placed = False
        b = SpawnBoundary([self._cups_boundary])
        b.sample(self._cups, min_distance=0.10)
        success_conditions = [NothingGrasped(self.robot.gripper),
                              self._on_peg_conditions]
        self.register_success_conditions(success_conditions)
        self.register_waypoint_ability_start(
            0, self._move_above_next_target)
        self.register_waypoints_should_repeat(lambda: not self._cups_placed)

        return [
            'place the cup on the cup holder',
            'pick up the cup and put it on the mug tree',
            'slide the handle of the mug onto the holder'
        ]


    def variation_count(self) -> int:
        return 1

    def _move_above_next_target(self, waypoint):
        self._w1.set_parent(self._cups)
        self._w4.set_pose(self._initial_relative_spoke, relative_to=self._spokes)
        self._w1.set_pose(self._initial_relative_cup, relative_to=self._cups)
        self._cups_placed = True

    def _repeat(self):
        return self._cups_placed < self._index + 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -np.pi / 2], [0.0, 0.0, np.pi / 2]

