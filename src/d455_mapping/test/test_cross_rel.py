"""Pinned cross in the base frame, pins file reading (no ROS)."""
import json

import numpy as np

from d455_mapping.cross_rel import load_pin_position, pin_in_base


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def test_pin_in_base_follows_the_drone():
    pin = np.array([2.0, 1.0, 1.5])
    # drone at the origin facing +x: the cross is 2 m ahead, 1 m to the left, 1.5 m up
    assert np.allclose(pin_in_base(np.eye(3), np.zeros(3), pin), [2.0, 1.0, 1.5])
    # yawed 90 deg left: what was ahead is now to the right
    assert np.allclose(pin_in_base(rot_z(np.pi / 2), np.zeros(3), pin), [1.0, -2.0, 1.5])
    # moved to the cross in x and y: 1.5 m straight up in z
    assert np.allclose(pin_in_base(np.eye(3), np.array([2.0, 1.0, 0.0]), pin), [0, 0, 1.5])
    # distance does not depend on the drone's heading
    d = [np.linalg.norm(pin_in_base(rot_z(a), np.array([0.3, -0.2, 0.1]), pin)) for a in (0, 1, 2)]
    assert np.allclose(d, d[0])


def test_load_pin_position(tmp_path):
    path = tmp_path / 'pins.json'
    assert load_pin_position(str(path)) is None                                  # missing
    path.write_text('{ not json')
    assert load_pin_position(str(path)) is None                                  # mid-write / broken
    assert path.exists()                                                         # never renamed
    path.write_text(json.dumps({'pins': [
        {'id': 3, 'cross_id': 7, 'position': [1, 2, 3]},
        {'id': 2, 'cross_id': 4, 'position': [4, 5, 6]}]}))
    pid, pos = load_pin_position(str(path))
    assert pid == 2 and np.allclose(pos, [4, 5, 6])                              # lowest id
    pid, pos = load_pin_position(str(path), 3)
    assert pid == 3 and np.allclose(pos, [1, 2, 3])
    assert load_pin_position(str(path), 9) is None
    path.write_text(json.dumps({'pins': []}))
    assert load_pin_position(str(path)) is None
