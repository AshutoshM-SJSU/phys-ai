from gazebo_trust_experiments.coordinates import cell_to_world, odom_to_world_pose


def test_odom_origin_is_shifted_to_robot_spawn_cell():
    sx, sy = cell_to_world(2, 2, map_height=32, cell_size=0.5)
    x, y, yaw = odom_to_world_pose(0.0, 0.0, 0.0, start_cell=(2, 2), map_height=32, cell_size=0.5)
    assert (x, y) == (sx, sy)
    assert yaw == 0.0


def test_odom_motion_is_applied_from_spawn_pose():
    sx, sy = cell_to_world(2, 2, map_height=32, cell_size=0.5)
    x, y, _ = odom_to_world_pose(1.0, 0.0, 0.0, start_cell=(2, 2), map_height=32, cell_size=0.5)
    assert x == sx + 1.0
    assert y == sy
