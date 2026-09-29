#!/usr/bin/python3

import os
import platform

if platform.system() == "Linux" or platform.system() == "Darwin":
    os.environ["KIVY_VIDEO"] = "ffpyplayer"

from kivy.config import Config
Config.set('kivy', 'log_level', 'info')

from pysimbotlib.core import PySimbotApp, Robot
from kivy.logger import Logger

SAFETY_DISTANCE = 30
CLOSED_DISTANCE = 5
HIT_DISTANCE = 0

FORWARD_STEP = 5
REVERSE_STEP = -2

FOOD_TURN_DEGREE = 8.0
SIDE_TURN_DEGREE = 8.0
FRONT_TURN_DEGREE = 18.0
STUCK_TURN_DEGREE = 30.0


class CollisionAvoidanceRobotRC2(Robot):

    def __init__(self, **kwargs):
        super(CollisionAvoidanceRobotRC2, self).__init__(**kwargs)

        self.avoid_dir = 1

        self.front_avoid_active = False

        self.stuck_frames = 0

        self.side_avoid_active = False

        self.current_condition = 1

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def _select_avoidance_direction(self, ir1, ir7, angle_to_food):
        if ir7 < ir1:
            return 1
        if ir1 < ir7:
            return -1
        return 1 if angle_to_food >= 0 else -1

    def _safe_food_turn(self, ir1, ir7, angle_to_food):
        food_turn = self._clamp(
            angle_to_food,
            -FOOD_TURN_DEGREE,
            FOOD_TURN_DEGREE,
        )

        if ir1 <= SAFETY_DISTANCE or ir7 <= SAFETY_DISTANCE:
            clearance_dir = self._select_avoidance_direction(
                ir1,
                ir7,
                angle_to_food,
            )

            if food_turn * clearance_dir < 0:
                return clearance_dir * min(
                    abs(food_turn),
                    SIDE_TURN_DEGREE / 2,
                )

        return food_turn

    def update(self):
        ir0 = self.distance(0)
        ir1 = self.distance(1)
        ir7 = self.distance(7)
        angle_to_food = self.smell()

        if self.stuck or ir0 <= HIT_DISTANCE:
            self.current_condition = 0
            self.side_avoid_active = False

            if not self.front_avoid_active:
                self.avoid_dir = self._select_avoidance_direction(
                    ir1,
                    ir7,
                    angle_to_food,
                )
                self.front_avoid_active = True
                self.stuck_frames = 0
            elif self.stuck:
                self.stuck_frames += 1
                if self.stuck_frames >= 2:
                    self.avoid_dir *= -1
                    self.stuck_frames = 0
            else:
                self.stuck_frames = 0

            self.turn(self.avoid_dir * STUCK_TURN_DEGREE)
            self.move(REVERSE_STEP)
            return

        self.stuck_frames = 0

        if ir0 <= SAFETY_DISTANCE:
            self.current_condition = 4
            self.side_avoid_active = False

            if not self.front_avoid_active:
                self.avoid_dir = self._select_avoidance_direction(
                    ir1,
                    ir7,
                    angle_to_food,
                )
                self.front_avoid_active = True

            self.turn(self.avoid_dir * FRONT_TURN_DEGREE)
            return

        self.front_avoid_active = False

        if ir1 < CLOSED_DISTANCE or ir7 < CLOSED_DISTANCE:
            self.current_condition = 3

            if not self.side_avoid_active:
                self.avoid_dir = self._select_avoidance_direction(
                    ir1,
                    ir7,
                    angle_to_food,
                )
                self.side_avoid_active = True

            self.turn(self.avoid_dir * SIDE_TURN_DEGREE)
            return

        self.side_avoid_active = False

        if ir1 > SAFETY_DISTANCE and ir7 > SAFETY_DISTANCE:
            self.current_condition = 1
        else:
            self.current_condition = 2

        self.turn(self._safe_food_turn(ir1, ir7, angle_to_food))
        self.move(FORWARD_STEP)


if __name__ == '__main__':
    app = PySimbotApp(
        robot_cls=CollisionAvoidanceRobotRC2,
        num_robots=1,
        num_objectives=1,
        enable_wasd_control=False,
        simulation_forever=True,
        food_move_after_eat=True
    )
    app.run()
