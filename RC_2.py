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
    """
    Improved rule-based controller for Assignment RC 2.

    The simulator setup, sensor indices, movement steps, and turn constants
    are intentionally the same as Assignment_RC_1.py. The improvements are
    limited to decision-making and controller state.
    """

    def __init__(self, **kwargs):
        super(CollisionAvoidanceRobotRC2, self).__init__(**kwargs)

        # Positive and negative values use PySimbot's normal turn convention.
        self.avoid_dir = 1

        # True only while the robot is actively handling a front obstacle.
        # This prevents a direction selected for an old obstacle from being
        # reused after the robot has entered a different situation.
        self.front_avoid_active = False

        # Used to escape repeated failed reverse movements.
        self.stuck_frames = 0

        # Locks the turn direction while a side is critically close. Without
        # this state, tiny sensor changes can make the robot turn left and
        # right on alternating frames near a corner.
        self.side_avoid_active = False

        # 0 = recovery, 1 = open-space food tracking,
        # 2 = constrained forward movement, 3 = side avoidance,
        # 4 = front avoidance.
        self.current_condition = 1

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def _select_avoidance_direction(self, ir1, ir7, angle_to_food):
        """
        Select the direction with more clearance.

        IR 1 is the front-right sensor and IR 7 is the front-left sensor.
        If both sides are equally clear, the food direction breaks the tie.
        """
        if ir7 < ir1:
            # The left side is closer, so turn toward the right.
            return 1
        if ir1 < ir7:
            # The right side is closer, so turn toward the left.
            return -1
        return 1 if angle_to_food >= 0 else -1

    def _safe_food_turn(self, ir1, ir7, angle_to_food):
        """
        Follow food without steering toward the tighter side of a corridor.

        A critical side obstacle is handled separately. When one side is only
        moderately close, the food turn is kept if it points toward the open
        side. If it points toward the tighter side, it is replaced with a
        smaller turn toward the open side.
        """
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

            # Do not let food tracking pull the robot toward the side with
            # less space. Keep a gentle correction toward the open side.
            if food_turn * clearance_dir < 0:
                return clearance_dir * min(
                    abs(food_turn),
                    SIDE_TURN_DEGREE / 2,
                )

        return food_turn

    def update(self):
        # Keep the same sensors used by Assignment_RC_1.py.
        ir0 = self.distance(0)
        ir1 = self.distance(1)
        ir7 = self.distance(7)
        angle_to_food = self.smell()

        # Priority 0: recover from a collision, a zero-distance front hit, or
        # repeated failure to move. Reversing remains the same size as RC 1,
        # but the direction is changed after repeated failed recovery steps.
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

        # Priority 4: avoid a front obstacle. The direction is selected once
        # when entering this state and retained until the front is clear.
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

        # The front is clear, so a later front obstacle should make a fresh
        # direction decision instead of inheriting stale state.
        self.front_avoid_active = False

        # Priority 3: handle a critically close side. Turn in place until the
        # sensor is safe again. Moving after an eight-degree turn is unsafe:
        # the new heading may point into the same corner even though the old
        # front sensor reading was clear.
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

        # Priorities 1 and 2: the front is safe. RC 1 moved straight whenever
        # a side was between 5 and 30 units away. RC 2 still keeps the same
        # forward step, but continuously steers toward food. When a side is
        # moderately close, it prevents a food turn into that tighter side.
        if ir1 > SAFETY_DISTANCE and ir7 > SAFETY_DISTANCE:
            self.current_condition = 1
        else:
            self.current_condition = 2

        self.turn(self._safe_food_turn(ir1, ir7, angle_to_food))
        self.move(FORWARD_STEP)


if __name__ == '__main__':
    # Keep the simulation setup unchanged from Assignment_RC_1.py.
    app = PySimbotApp(
        robot_cls=CollisionAvoidanceRobotRC2,
        num_robots=1,
        num_objectives=1,
        enable_wasd_control=False,
        simulation_forever=True,
        food_move_after_eat=True
    )
    app.run()
