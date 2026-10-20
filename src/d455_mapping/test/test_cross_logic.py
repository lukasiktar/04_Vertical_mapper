"""Offline tests of the cross detector logic on synthetic RGB + depth (no ROS, no camera)."""
import json
import os
import time

import numpy as np
import pytest

from d455_mapping.cross_logic import (
    Cross3D, is_vertical_plane, DetectorParams, GeometricCandidateDetector, LandmarkMap, LandmarkParams,
    VerifyParams, transform_cross, unit, verify_candidate)
from synth_scene import CAM, cross, plane_axes, beam, look_at_pose, render


def run(rects, vp=None, dp=None, seed=0):
    bgr, depth = render(rects, seed=seed)
    det = GeometricCandidateDetector(dp or DetectorParams())
    rng = np.random.default_rng(0)
    cands = det.detect(bgr)
    out = [verify_candidate(c, depth, CAM, vp or VerifyParams(), rng) for c in cands]
    return cands, [c for c, why in out if c is not None], [why for c, why in out if c is None]


def nearest(found, center):
    return min(found, key=lambda c: np.linalg.norm(c.center - center))


# ---------------------------------------------------------------------------------- crosses
@pytest.mark.parametrize('seed', [0, 1, 2])
def test_two_identical_crosses_at_2m(seed):
    gt = [np.array([-0.45, 0.0, 2.0]), np.array([0.50, 0.05, 2.1])]
    rects = cross(gt[0], angle_between=90, orientation=45, yaw=10) \
        + cross(gt[1], angle_between=90, orientation=45, yaw=-10)
    cands, found, why = run(rects, seed=seed)
    assert len(found) == 2, (len(cands), why)
    for g in gt:
        c = nearest(found, g)
        assert np.linalg.norm(c.center - g) < 0.02
        assert abs(c.beam_angle_deg - 90.0) < 5.0
    assert nearest(found, gt[0]) is not nearest(found, gt[1])


@pytest.mark.parametrize('angle', [60.0, 75.0])
def test_beam_angle_and_directions(angle):
    g = np.array([0.1, -0.05, 2.0])
    e1, e2, n = plane_axes(0, 0)
    _, found, why = run(cross(g, angle_between=angle, orientation=20),
                        vp=VerifyParams(cross_angle_tol_deg=90.0))       # angle filter off
    assert len(found) == 1, why
    c = found[0]
    assert np.linalg.norm(c.center - g) < 0.02
    assert abs(c.beam_angle_deg - angle) < 4.0
    # the detected beam directions are the true ones (up to sign and order)
    true = [unit(np.cos(np.radians(a)) * e1 + np.sin(np.radians(a)) * e2)
            for a in (20 - angle / 2, 20 + angle / 2)]
    for t in true:
        assert max(abs(t @ a) for a in c.arms) > np.cos(np.radians(4.0))
    assert abs(c.normal @ n) > 0.99


def test_cross_at_6m_rejected_by_max_depth():
    rects = cross([0.0, 0.0, 6.0], half_len=1.0, width=0.15)   # big enough for min_arm_px at 6 m
    # control: detected when the range allows it
    _, far_ok, why = run(rects, vp=VerifyParams(max_depth_m=8.0))
    assert len(far_ok) == 1, why
    _, found, why = run(rects, vp=VerifyParams(max_depth_m=4.0))
    assert found == []


def l_joint(corner_xy, z=2.0, arm=0.45, width=0.08):
    e1, e2, _ = plane_axes(0, 0)
    c = np.array([corner_xy[0], corner_xy[1], z])
    a = beam(c + e1 * (arm / 2 - width / 2), e1, e2, 0, arm / 2, width, 200)
    b = beam(c + e2 * (arm / 2 - width / 2), e1, e2, 90, arm / 2, width, 170)
    return [a, b]


def t_joint(center_xy, z=2.0, arm=0.45, width=0.08):
    e1, e2, _ = plane_axes(0, 0)
    c = np.array([center_xy[0], center_xy[1], z])
    top = beam(c, e1, e2, 0, arm, width, 200)
    stem = beam(c + e2 * (arm / 2 + width / 2), e1, e2, 90, arm / 2, width, 170)
    return [top, stem]


@pytest.mark.parametrize('name,rects', [('L', l_joint((0.0, 0.0))), ('T', t_joint((0.0, 0.0)))])
def test_l_and_t_joints_rejected(name, rects):
    cands, found, why = run(rects)
    assert found == [], f'{name} joint accepted: {[c.center for c in found]}'


@pytest.mark.parametrize('offset', [0.15, 0.30])
def test_non_coplanar_beams_rejected(offset):
    cands, found, why = run(cross([0.0, 0.0, 2.0], depth_offset_b=offset))
    assert found == [], (offset, [c.center for c in found])
    assert cands, 'the 2D step must still see the crossing (rejection must come from depth)'


def test_coplanar_control_for_non_coplanar_test():
    _, found, _ = run(cross([0.0, 0.0, 2.0], depth_offset_b=0.0))
    assert len(found) == 1


def test_empty_and_textured_scene_has_no_detection():
    _, found, _ = run([])
    assert found == []


def test_detect_scale_half_gives_same_result():
    g = np.array([0.0, 0.0, 2.0])
    _, found, why = run(cross(g), dp=DetectorParams(detect_scale=0.5))
    assert len(found) == 1, why
    assert np.linalg.norm(found[0].center - g) < 0.03


def test_custom_candidate_detector_can_replace_the_geometric_one():
    from d455_mapping.cross_logic import Candidate

    class Fixed:
        def detect(self, bgr):
            return [Candidate(pt=np.array([320.0, 240.0]), dirs=np.array([[0.8, 0.6], [-0.6, 0.8]]),
                              arm=100.0, score=1.0)]
    bgr, depth = render(cross([0.0, 0.0, 2.0], angle_between=90, orientation=36.87 + 45))
    c, why = verify_candidate(Fixed().detect(bgr)[0], depth, CAM, VerifyParams(), np.random.default_rng(0))
    assert c is not None, why


def test_cross_angle_filter_is_3d():
    g = np.array([0.0, 0.0, 2.0])
    _, ok90, why = run(cross(g, angle_between=90))
    assert len(ok90) == 1, why
    # a 90-degree cross on a plane turned 45 deg away from the camera looks far from 90 in the image,
    # but its 3D angle is 90: accepted
    _, tilted, why = run(cross(g, angle_between=90, yaw=45))
    assert len(tilted) == 1, why
    assert abs(tilted[0].beam_angle_deg - 90.0) < 8.0
    # a 60-degree cross is rejected by the angle filter
    cands, f60, why = run(cross(g, angle_between=60, orientation=20))
    assert f60 == [] and 'angle' in why and cands


def test_measured_beam_width_and_width_filter():
    g = np.array([0.0, 0.0, 2.0])
    _, found, why = run(cross(g, width=0.08))
    assert len(found) == 1 and abs(found[0].beam_width_m - 0.08) < 0.015, (why, found and found[0].beam_width_m)
    _, narrow, why = run(cross(g, width=0.08), vp=VerifyParams(min_beam_width_m=0.12))
    assert narrow == [] and 'width' in why
    _, wide, why = run(cross(g, width=0.08), vp=VerifyParams(max_beam_width_m=0.05))
    assert wide == [] and 'width' in why


def test_vertical_plane_filter():
    up = np.array([0.0, -1.0, 0.0])                      # camera y is down
    assert is_vertical_plane(np.array([0.0, 0.0, -1.0]), up, 20.0)           # wall facing the camera
    assert is_vertical_plane(unit(np.array([0.3, 0.1, -1.0])), up, 20.0)
    assert not is_vertical_plane(unit(np.array([0.0, -1.0, -0.3])), up, 20.0)   # floor/ceiling


# ---------------------------------------------------------------------------------- landmarks
def fake_cross(center, rng, sigma=0.015, distance=2.0):
    e1, e2, n = plane_axes(0, 0)
    arms = np.array([unit(e1 + 0.3 * e2), unit(-0.3 * e1 + e2)])
    arms = arms + rng.normal(0, 0.03, arms.shape)
    arms = np.array([unit(a) * rng.choice([-1, 1]) for a in arms])
    if rng.random() < 0.5:
        arms = arms[::-1]
    return Cross3D(center=np.asarray(center) + rng.normal(0, sigma, 3),
                   normal=unit(n + rng.normal(0, 0.02, 3)) * rng.choice([-1, 1]), arms=arms,
                   arm_len=0.2, pixel=np.zeros(2), distance=distance)


def feed(lm, centers, frames, rng, t0=0.0, dt=0.2, outliers=0):
    t = t0
    for _ in range(frames):
        obs = [fake_cross(c, rng) for c in centers]
        if outliers and rng.random() < 0.1:
            obs.append(fake_cross(rng.uniform(-3, 3, 3), rng))      # one-off false detection
        lm.update(obs, t)
        lm.prune(t)
        t += dt
    return t


def test_landmark_fusion_two_crosses_one_meter_apart(tmp_path):
    rng = np.random.default_rng(1)
    centers = [np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 1.0])]
    lm = LandmarkMap(LandmarkParams(min_observations=5, max_landmarks=0))
    t = feed(lm, centers, 60, rng, outliers=1)
    conf = lm.confirmed()
    assert len(conf) == 2
    assert sorted(c.id for c in conf) == [0, 1]                    # noise did not use up IDs
    ids = {c.id: c.center.copy() for c in conf}
    for c in conf:
        assert min(np.linalg.norm(c.center - g) for g in centers) < 0.01
        assert c.count >= 55
    # more frames: IDs and positions stay
    feed(lm, centers, 40, rng, t0=t)
    assert sorted(c.id for c in lm.confirmed()) == [0, 1]
    for c in lm.confirmed():
        assert np.linalg.norm(c.center - ids[c.id]) < 0.01
    # arm directions: averaging with sign alignment keeps them unit and consistent
    for c in lm.confirmed():
        assert np.allclose(np.linalg.norm(c.arms, axis=1), 1.0, atol=1e-6)
        assert abs(c.arms[0] @ c.arms[1]) < 0.5

    # save / load round trip; new detections fuse into the loaded landmarks
    path = str(tmp_path / 'sub' / 'crosses.json')
    assert lm.save(path) == 2
    data = json.load(open(path))
    assert data['frame'] == 'map' and [c['id'] for c in data['crosses']] == [0, 1]
    lm2 = LandmarkMap(LandmarkParams(min_observations=5, max_landmarks=0))
    assert lm2.load(path) == 2
    before = {c.id: c.count for c in lm2.confirmed()}
    feed(lm2, centers, 20, rng)
    after = {c.id: c.count for c in lm2.confirmed()}
    assert len(lm2.landmarks) == 2 and set(after) == {0, 1}
    assert all(after[i] == before[i] + 20 for i in before)
    assert not os.path.exists(path + '.tmp')
    # a third cross gets the next ID, not a reused one
    feed(lm2, [np.array([3.0, 0.0, 1.0])], 10, rng, t0=50.0)
    assert sorted(c.id for c in lm2.confirmed()) == [0, 1, 2]


def test_top_two_rule_keeps_the_best_observed_crosses():
    rng = np.random.default_rng(5)
    lm = LandmarkMap(LandmarkParams(min_observations=5, max_landmarks=2, replace_margin=1.5))
    false_c, a, b = np.array([0.0, 0, 1]), np.array([2.0, 0, 1]), np.array([4.0, 0, 1])
    t = feed(lm, [false_c], 8, rng)                      # clutter confirmed first (id 0)
    assert [c.id for c in lm.confirmed()] == [0]
    t = feed(lm, [a], 12, rng, t0=t)                     # a true cross (id 1)
    assert sorted(c.id for c in lm.confirmed()) == [0, 1]
    t = feed(lm, [b], 6, rng, t0=t)                      # 6 obs: not enough to replace (needs > 12)
    assert sorted(c.id for c in lm.confirmed()) == [0, 1]
    feed(lm, [b], 20, rng, t0=t)                         # well observed now: replaces the weakest
    conf = {c.id: c.center for c in lm.confirmed()}
    assert sorted(conf) == [1, 2]                        # the clutter landmark (id 0) is gone, ids not reused
    assert np.linalg.norm(conf[1] - a) < 0.05 and np.linalg.norm(conf[2] - b) < 0.05
    assert len(lm.to_dict()['crosses']) == 2


def test_one_observation_per_landmark_and_frame():
    rng = np.random.default_rng(2)
    lm = LandmarkMap(LandmarkParams(min_observations=3))
    c = np.array([0.0, 0.0, 1.0])
    for k in range(3):                     # duplicate detection of the same cross in each frame
        lm.update([fake_cross(c, rng, 0.01), fake_cross(c, rng, 0.01)], k * 0.2)
    assert len(lm.landmarks) == 1 and lm.landmarks[0].count == 3 and lm.landmarks[0].id == 0


def test_tentative_landmarks_expire():
    rng = np.random.default_rng(3)
    lm = LandmarkMap(LandmarkParams(min_observations=5, tentative_ttl_s=5.0))
    lm.update([fake_cross([0, 0, 1], rng)], 0.0)
    assert lm.prune(3.0) == 0 and lm.prune(6.0) == 1 and lm.landmarks == []


def test_corrupt_landmark_file_is_kept(tmp_path):
    p = tmp_path / 'c.json'
    p.write_text('{not json')
    lm = LandmarkMap()
    with pytest.raises(ValueError):
        lm.load(str(p))
    assert (tmp_path / 'c.json.corrupt').exists() and not p.exists()
    assert lm.load(str(tmp_path / 'missing.json')) == 0


def test_end_to_end_moving_camera():
    """Render two crosses from several camera poses, fuse in the world frame."""
    world_gt = [np.array([2.0, 0.4, 1.0]), np.array([2.0, -0.6, 1.1])]
    e1 = np.array([0.0, -1.0, 0.0])             # world: the crosses face -x (toward the camera),
    e2 = np.array([0.0, 0.0, -1.0])              # in-plane axes right = -y, down = -z
    n = np.array([-1.0, 0.0, 0.0])

    def world_cross(c):
        a = beam(c, e1, e2, 0.0, 0.4, 0.08, 200)
        b = beam(c, e1, e2, 90.0, 0.4, 0.08, 170)
        return [a, b]
    rects = world_cross(world_gt[0]) + world_cross(world_gt[1])
    lm = LandmarkMap(LandmarkParams(min_observations=4))
    det, rng = GeometricCandidateDetector(), np.random.default_rng(0)
    t = 0.0
    for k, (px, py) in enumerate([(0.0, 0.0), (0.1, 0.2), (0.0, -0.2), (0.2, 0.1), (-0.1, 0.0), (0.1, -0.1)]):
        R, p = look_at_pose([px, py, 1.0], [2.0, -0.1, 1.05])
        bgr, depth = render(rects, R, p, seed=k)
        obs = []
        for c in det.detect(bgr):
            x, why = verify_candidate(c, depth, CAM, VerifyParams(), rng)
            if x is not None:
                obs.append(transform_cross(x, R, p))
        lm.update(obs, t)
        t += 0.3
    conf = lm.confirmed()
    assert len(conf) == 2, [(c.id, c.count, c.center) for c in lm.landmarks]
    for g in world_gt:
        assert min(np.linalg.norm(c.center - g) for c in conf) < 0.03


# ---------------------------------------------------------------------------------- runtime
@pytest.mark.parametrize('scale', [1.0, 0.5])
def test_runtime_per_frame(scale, capsys):
    rects = cross([-0.45, 0.0, 2.0], yaw=10) + cross([0.5, 0.05, 2.1], yaw=-10)
    bgr, depth = render(rects)
    det = GeometricCandidateDetector(DetectorParams(detect_scale=scale))
    rng = np.random.default_rng(0)
    vp = VerifyParams()
    det.detect(bgr)
    td, tv = [], []
    for _ in range(30):
        t0 = time.perf_counter()
        cands = det.detect(bgr)
        t1 = time.perf_counter()
        for c in cands:
            verify_candidate(c, depth, CAM, vp, rng)
        t2 = time.perf_counter()
        td.append(t1 - t0)
        tv.append(t2 - t1)
    with capsys.disabled():
        print(f'\n[runtime] detect_scale={scale}: detect {1e3 * np.median(td):.1f} ms, '
              f'verify {1e3 * np.median(tv):.1f} ms (median of 30, {len(cands)} candidates)')
    assert np.median(td) + np.median(tv) < 0.15
