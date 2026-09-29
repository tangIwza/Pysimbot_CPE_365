#!/usr/bin/python3
"""
test_fuzzy_controller.py
Unit tests verifying Fuzzy Logic Controller operations, membership functions,
rule activations, and defuzzified outputs across typical robotic navigation scenarios.
"""

from Assignment_RC_1_Fuzzy import (
    FuzzyLogicController,
    trap_left,
    triangle,
    trap_right,
)


def test_membership_functions():
    print("Testing membership functions...")
    # trap_left
    assert trap_left(5.0, 10.0, 35.0) == 1.0
    assert trap_left(40.0, 10.0, 35.0) == 0.0
    assert 0.0 < trap_left(22.5, 10.0, 35.0) < 1.0

    # triangle
    assert triangle(15.0, 20.0, 35.0, 60.0) == 0.0
    assert triangle(35.0, 20.0, 35.0, 60.0) == 1.0
    assert triangle(70.0, 20.0, 35.0, 60.0) == 0.0

    # trap_right
    assert trap_right(30.0, 45.0, 75.0) == 0.0
    assert trap_right(80.0, 45.0, 75.0) == 1.0
    assert 0.0 < trap_right(60.0, 45.0, 75.0) < 1.0
    print("  -> Membership functions passed!")


def test_flc_scenarios():
    flc = FuzzyLogicController()
    print("Testing FLC scenarios...")

    # Scenario 1: Open space, target to the left (-50 deg)
    turn, speed, rule = flc.compute(d_front=90, d_right=90, d_left=90, angle_to_food=-50)
    print(f"  [Scenario 1: Open Left] turn={turn:.2f}°, speed={speed:.2f}, rule={rule}")
    assert turn < -4.0, f"Expected turn < -4, got {turn}"
    assert speed >= 4.0, f"Expected fast speed >= 4, got {speed}"

    # Scenario 2: Open space, target to the right (+50 deg)
    turn, speed, rule = flc.compute(d_front=90, d_right=90, d_left=90, angle_to_food=50)
    print(f"  [Scenario 2: Open Right] turn={turn:.2f}°, speed={speed:.2f}, rule={rule}")
    assert turn > 4.0, f"Expected turn > 4, got {turn}"
    assert speed >= 4.0, f"Expected fast speed >= 4, got {speed}"

    # Scenario 3: Open space, target ahead (0 deg)
    turn, speed, rule = flc.compute(d_front=90, d_right=90, d_left=90, angle_to_food=0)
    print(f"  [Scenario 3: Open Ahead] turn={turn:.2f}°, speed={speed:.2f}, rule={rule}")
    assert abs(turn) < 1.0, f"Expected near zero turn, got {turn}"
    assert speed >= 4.0, f"Expected fast speed >= 4, got {speed}"

    # Scenario 4: Obstacle ahead, left wall close (d_front=15, d_left=10, d_right=80)
    turn, speed, rule = flc.compute(d_front=15, d_right=80, d_left=10, angle_to_food=0)
    print(f"  [Scenario 4: Front Near, Left Near] turn={turn:.2f}°, speed={speed:.2f}, rule={rule}")
    assert turn > 12.0, f"Expected sharp right turn > 12, got {turn}"
    assert speed < 1.0, f"Expected stop speed < 1, got {speed}"

    # Scenario 5: Obstacle ahead, right wall close (d_front=15, d_right=10, d_left=80)
    turn, speed, rule = flc.compute(d_front=15, d_right=10, d_left=80, angle_to_food=0)
    print(f"  [Scenario 5: Front Near, Right Near] turn={turn:.2f}°, speed={speed:.2f}, rule={rule}")
    assert turn < -12.0, f"Expected sharp left turn < -12, got {turn}"
    assert speed < 1.0, f"Expected stop speed < 1, got {speed}"

    # Scenario 6: Corridor glancing, left wall close, front medium (d_front=35, d_left=15, d_right=60)
    turn, speed, rule = flc.compute(d_front=35, d_right=60, d_left=15, angle_to_food=0)
    print(f"  [Scenario 6: Left Wall Glancing] turn={turn:.2f}°, speed={speed:.2f}, rule={rule}")
    assert turn > 3.0, f"Expected right turn > 3, got {turn}"
    assert speed <= 3.0, f"Expected slow speed <= 3, got {speed}"

    # Scenario 7: Corridor glancing, right wall close, front medium (d_front=35, d_right=15, d_left=60)
    turn, speed, rule = flc.compute(d_front=35, d_right=15, d_left=60, angle_to_food=0)
    print(f"  [Scenario 7: Right Wall Glancing] turn={turn:.2f}°, speed={speed:.2f}, rule={rule}")
    assert turn < -3.0, f"Expected left turn < -3, got {turn}"
    assert speed <= 3.0, f"Expected slow speed <= 3, got {speed}"

    print("  -> All FLC scenarios passed successfully!")


if __name__ == '__main__':
    test_membership_functions()
    test_flc_scenarios()
    print("\nALL FUZZY CONTROLLER TESTS PASSED!")
