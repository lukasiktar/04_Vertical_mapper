"""Pins: toggling, snapping, persistence, distances (no ROS)."""
import json

import numpy as np
import pytest

from d455_mapping.pins_logic import PinStore, nearest_cross, pin_distances


def test_toggle_and_stable_ids(tmp_path):
    s = PinStore()
    a, b = np.array([1.0, 0.0, 1.0]), np.array([3.0, 0.0, 1.0])
    arms = np.array([[1.0, 0, 0], [0, 1.0, 0]])
    act, p1 = s.toggle_cross(0, a, arms)
    assert act == 'added' and p1.id == 1
    assert s.toggle_cross(1, b)[1].id == 2
    act, removed = s.toggle_cross(0, a + 0.01)              # same cross again -> unpin
    assert act == 'removed' and removed.id == 1 and [p.id for p in s.pins] == [2]
    assert s.toggle_cross(0, a, arms)[1].id == 3            # ids are never reused

    path = str(tmp_path / 'sub' / 'pins.json')
    assert s.save(path) == 2 and not s.dirty
    s2 = PinStore()
    assert s2.load(path) == 2
    assert [p.id for p in s2.pins] == [2, 3] and s2.next_id == 4
    assert np.allclose(s2.pins[1].position, a)
    assert s2.pins[1].arms is not None and np.allclose(s2.pins[1].arms, arms)      # arms survive save/load
    assert s2.pins[0].arms is None                                                  # pinned without arms
    assert json.load(open(path))['frame'] == 'map'


def test_snap_to_nearest_cross_only_within_radius():
    crosses = {0: np.array([0.0, 0.0, 1.0]), 1: np.array([1.0, 0.0, 1.0])}
    hit, d = nearest_cross(crosses, np.array([0.9, 0.1, 1.0]), 0.4)
    assert hit[0] == 1 and d < 0.15
    miss, d = nearest_cross(crosses, np.array([5.0, 0.0, 1.0]), 0.4)
    assert miss is None and d == pytest.approx(4.0)
    assert nearest_cross({}, np.zeros(3), 0.4) == (None, float('inf'))


def test_distances_with_known_camera_pose():
    s = PinStore()
    s.toggle_cross(0, np.array([2.0, 1.0, 0.5]))
    # camera at (0,1,0.5) looking along map +x: optical z -> map x, optical x -> -y, optical y -> -z
    R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    (d,) = pin_distances(s.pins, R, np.array([0.0, 1.0, 0.5]))
    assert d['distance_m'] == pytest.approx(2.0)
    assert np.allclose(d['camera_xyz'], [0.0, 0.0, 2.0])     # straight ahead


def test_corrupt_file_is_kept(tmp_path):
    p = tmp_path / 'pins.json'
    p.write_text('{oops')
    with pytest.raises(ValueError):
        PinStore().load(str(p))
    assert (tmp_path / 'pins.json.corrupt').exists()
