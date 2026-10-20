"""Cross detection logic without ROS: 2D candidates, depth verification, landmark fusion.

Pipeline (see README "Cross detection"):
  1. ``CandidateDetector.detect(bgr)`` proposes crossings in the image. The default is
     ``GeometricCandidateDetector`` (Canny -> HoughLinesP -> pairs of non-parallel segments ->
     edge support on all four arms -> merge). Any object with the same ``detect`` method (for
     example a learned detector) can replace it.
  2. ``verify_candidate`` samples aligned depth along the four arms, fits a plane (RANSAC) and
     intersects the pixel rays with it -> ``Cross3D`` in the camera frame.
  3. ``LandmarkMap`` fuses ``Cross3D`` observations (already in the map frame) into persistent,
     numbered landmarks and reads/writes them as JSON.

All pixel parameters are given for the FULL-resolution color image; ``detect_scale`` rescales them
internally, so the same values work when the Orin Nano detects on a smaller image.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import List, NamedTuple, Optional, Protocol, Tuple

import cv2
import numpy as np


# --------------------------------------------------------------------------------------------
# Parameters (field names are the ROS parameter names)
# --------------------------------------------------------------------------------------------
@dataclass
class DetectorParams:
    """A beam is a PAIR of parallel edges with a consistent width. A cross is two beams whose center-lines intersect inside both,
    with at least ``min_arm_px`` of beam on each side of the crossing."""
    canny_low: float = 50.0
    canny_high: float = 150.0
    blur_ksize: int = 5              # Gaussian blur before Canny (odd)
    hough_threshold: int = 40        # accumulator votes
    min_segment_px: float = 40.0     # shortest edge segment kept
    max_line_gap_px: float = 15.0    # gap bridged inside one Hough segment
    max_segments: int = 100          # longest N edge segments (after joining) are paired
    collinear_gap_px: float = 80.0   # edge pieces on one line are joined across gaps up to this
                                     # (the other beam hides part of an edge: about one beam width)
    collinear_tol_deg: float = 3.0
    parallel_tol_deg: float = 6.0    # the two edges of a beam
    min_beam_width_px: float = 5.0   # distance between the two edges
    max_beam_width_px: float = 70.0
    max_width_ratio: float = 2.0     # both beams of a cross have about the same width
    beam_contrast: float = 10.0      # gray levels: interior vs. BOTH outsides, same sign
    min_arm_support: float = 0.6     # fraction of beam samples that pass the contrast test
    min_cross_angle_deg: float = 35.0   # loose 2D limit only: perspective distorts angles in the
                                        # image; the exact angle test is in 3D (VerifyParams)
    min_arm_px: float = 40.0         # beam length required on each side of the crossing
    max_arm_px: float = 120.0        # arm length used for depth sampling
    merge_px: float = 30.0           # duplicate candidates closer than this are one cross
    detect_scale: float = 1.0        # detection image scale (Orin Nano: 0.5)


@dataclass
class VerifyParams:
    min_depth_m: float = 0.3
    max_depth_m: float = 4.0
    plane_tol_m: float = 0.03
    min_inlier_ratio: float = 0.7
    min_valid_ratio: float = 0.5     # fraction of the arm samples that must have valid depth
    min_valid_per_arm: int = 3       # ... and this many on EACH arm (both beams must be seen)
    patch_px: int = 2                # depth window half size: (2*patch_px+1)^2 pixels
    near_percentile: float = 10.0    # "nearest" depth in the window; a percentile, not the
                                     # minimum, so single flying pixels do not win
    samples_per_arm: int = 12
    ransac_iters: int = 150
    min_view_cos: float = 0.15       # reject planes seen almost edge-on
    expected_cross_angle_deg: float = 90.0   # angle between the two beams, measured in 3D
    cross_angle_tol_deg: float = 12.0        # >= 90 switches the test off
    min_beam_width_m: float = 0.0    # physical beam width (0 = off); every detection shows its
    max_beam_width_m: float = 0.0    # measured width in the debug image, read the real one there


@dataclass
class LandmarkParams:
    landmark_merge_m: float = 0.3    # association radius
    min_observations: int = 5        # frames needed to confirm a landmark
    tentative_ttl_s: float = 15.0    # unconfirmed landmarks not seen for this long are dropped
    max_weight: float = 50.0         # caps the averaging memory (late observations still count)
    max_landmarks: int = 2           # keep only the N best-observed crosses (0 = unlimited)
    replace_margin: float = 1.5      # a challenger must have this many times the observations of
                                     # the weakest confirmed landmark to replace it


class Camera(NamedTuple):
    fx: float
    fy: float
    cx: float
    cy: float


# --------------------------------------------------------------------------------------------
# Data
# --------------------------------------------------------------------------------------------
@dataclass
class Candidate:
    pt: np.ndarray        # (2,) crossing pixel (full resolution)
    dirs: np.ndarray      # (2, 2) unit image directions of the two beams
    arm: float            # arm sampling length [px]
    score: float          # worst-arm edge support, 0..1
    members: int = 1      # raw crossings merged into this candidate
    width_px: float = 0.0  # beam width in the image [px]


@dataclass
class Cross3D:
    center: np.ndarray    # (3,)
    normal: np.ndarray    # (3,) unit, facing the camera in the camera frame
    arms: np.ndarray      # (2, 3) unit directions of the two beams (sign arbitrary)
    arm_len: float        # mean half length [m] of the sampled arms (a sampling length, not the
                          # physical beam length)
    pixel: np.ndarray     # (2,)
    score: float = 0.0
    inlier_ratio: float = 0.0
    rms: float = 0.0      # plane fit residual of the inliers [m]
    distance: float = 0.0  # camera -> center [m]
    beam_angle_deg: float = 0.0
    beam_width_m: float = 0.0


class CandidateDetector(Protocol):
    """Anything that turns a BGR image into crossing candidates (replaceable by a learned model)."""

    def detect(self, bgr: np.ndarray) -> List[Candidate]: ...


# --------------------------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------------------------
def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def depth_to_meters(depth: np.ndarray) -> np.ndarray:
    """RealSense aligned depth: 16UC1 in mm or 32FC1 in m -> float32 meters (invalid = 0)."""
    out = depth.astype(np.float32)
    if depth.dtype == np.uint16:
        out *= 0.001
    return np.nan_to_num(out, nan=0.0, posinf=0.0, neginf=0.0)


def is_vertical_plane(normal: np.ndarray, up: np.ndarray, tol_deg: float) -> bool:
    """A vertical surface has a horizontal normal: it is perpendicular to ``up`` within tol_deg
    (both in the same frame)."""
    return abs(float(unit(normal) @ unit(up))) <= math.sin(math.radians(tol_deg))


def transform_cross(c: Cross3D, R: np.ndarray, T: np.ndarray) -> Cross3D:
    return Cross3D(center=R @ c.center + T, normal=R @ c.normal, arms=(R @ c.arms.T).T,
                   arm_len=c.arm_len, pixel=c.pixel, score=c.score,
                   inlier_ratio=c.inlier_ratio, rms=c.rms, distance=c.distance,
                   beam_angle_deg=c.beam_angle_deg, beam_width_m=c.beam_width_m)


# --------------------------------------------------------------------------------------------
# 2D candidates
# --------------------------------------------------------------------------------------------
def _components(adj: np.ndarray) -> np.ndarray:
    """Connected-component labels of a boolean adjacency matrix (label propagation)."""
    n = len(adj)
    adj = adj | np.eye(n, dtype=bool)
    label = np.arange(n)
    while True:
        new = np.where(adj, label[None, :], n).min(axis=1)
        if np.array_equal(new, label):
            return label
        label = new


def _cross2(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


class GeometricCandidateDetector:
    """Canny -> Hough edge segments -> joined edges -> beams (parallel edge pairs) -> crosses."""

    def __init__(self, params: Optional[DetectorParams] = None):
        self.p = params or DetectorParams()

    def detect(self, bgr: np.ndarray) -> List[Candidate]:
        p, s = self.p, self.p.detect_scale
        img = cv2.resize(bgr, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s != 1.0 else bgr
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        k = int(p.blur_ksize) | 1
        gray = cv2.GaussianBlur(gray, (k, k), 0)
        edges = cv2.Canny(gray, p.canny_low, p.canny_high)
        segs = cv2.HoughLinesP(edges, 1, np.pi / 180, max(10, int(round(p.hough_threshold * s))),
                               minLineLength=p.min_segment_px * s, maxLineGap=p.max_line_gap_px * s)
        if segs is None:
            return []
        segs = segs.reshape(-1, 4).astype(np.float64)                          # (N,1,4) or (N,4)
        lengths = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
        segs = segs[np.argsort(-lengths)[:4 * p.max_segments]]                # a net gives thousands
        segs = self._join_collinear(segs)
        lengths = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
        segs = segs[np.argsort(-lengths)[:p.max_segments]]
        if len(segs) < 2:
            return []
        beams = self._beams(segs, gray.astype(np.float32))
        cands = self._crossings(beams, gray.shape)
        for c in cands:   # back to full-resolution pixels
            c.pt = c.pt / s
            c.arm = c.arm / s
            c.width_px = c.width_px / s
        return cands

    # -- edges ------------------------------------------------------------------------------
    def _join_collinear(self, segs: np.ndarray) -> np.ndarray:
        """Edge pieces on one line (Hough breaks an edge where the other beam crosses it) are
        joined into one long edge."""
        p, s = self.p, self.p.detect_scale
        n = len(segs)
        if n < 2:
            return segs
        P, Q = segs[:, :2], segs[:, 2:]
        L = np.hypot(*(Q - P).T)
        U = (Q - P) / L[:, None]
        M = 0.5 * (P + Q)
        i, j = np.triu_indices(n, 1)
        ok = np.abs(np.einsum('ij,ij->i', U[i], U[j])) > math.cos(math.radians(p.collinear_tol_deg))
        i, j = i[ok], j[ok]
        # the two ends that face each other must lie on the other piece's line (a fixed small
        # tolerance: far-apart pieces on slightly different lines belong to different edges)
        def near_end(a, b):
            pa, qa = P[a], Q[a]
            use_p = np.linalg.norm(pa - M[b], axis=1) < np.linalg.norm(qa - M[b], axis=1)
            return np.where(use_p[:, None], pa, qa)
        perp = np.maximum(np.abs(_cross2(U[j], near_end(i, j) - M[j])),
                          np.abs(_cross2(U[i], near_end(j, i) - M[i])))
        ok = perp < 3.0 * max(s, 0.5)
        i, j = i[ok], j[ok]
        lo_i, hi_i = -L[i] / 2, L[i] / 2
        pj, qj = np.einsum('ij,ij->i', P[j] - M[i], U[i]), np.einsum('ij,ij->i', Q[j] - M[i], U[i])
        lo_j, hi_j = np.minimum(pj, qj), np.maximum(pj, qj)
        gap = np.maximum(0.0, np.maximum(lo_i, lo_j) - np.minimum(hi_i, hi_j))
        ok = gap <= p.collinear_gap_px * s
        adj = np.zeros((n, n), bool)
        adj[i[ok], j[ok]] = adj[j[ok], i[ok]] = True
        label = _components(adj)
        out = []
        for g in np.unique(label):
            idx = np.flatnonzero(label == g)
            if len(idx) == 1:
                out.append(segs[idx[0]])
                continue
            w = L[idx]
            c = (M[idx] * w[:, None]).sum(0) / w.sum()
            d = np.linalg.eigh(sum(w[k] * np.outer(U[idx[k]], U[idx[k]]) for k in range(len(idx))))[1][:, -1]
            t = np.concatenate([(P[idx] - c) @ d, (Q[idx] - c) @ d])
            out.append(np.concatenate([c + t.min() * d, c + t.max() * d]))
        return np.array(out)

    # -- beams: pairs of parallel edges ------------------------------------------------------
    def _beams(self, segs: np.ndarray, gray: np.ndarray, samples: int = 12) -> dict:
        p, s = self.p, self.p.detect_scale
        h, w = gray.shape
        P, Q = segs[:, :2], segs[:, 2:]
        L = np.hypot(*(Q - P).T)
        U = (Q - P) / L[:, None]
        M = 0.5 * (P + Q)
        i, j = np.triu_indices(len(segs), 1)
        dot = np.einsum('ij,ij->i', U[i], U[j])
        ok = np.abs(dot) > math.cos(math.radians(p.parallel_tol_deg))
        i, j, dot = i[ok], j[ok], dot[ok]
        ax = U[i] + np.sign(dot)[:, None] * U[j]
        ax /= np.linalg.norm(ax, axis=1)[:, None]
        nrm = np.stack([-ax[:, 1], ax[:, 0]], axis=1)
        O = 0.5 * (M[i] + M[j])
        width = np.abs(np.einsum('ij,ij->i', M[j] - M[i], nrm))
        ok = (width >= p.min_beam_width_px * s) & (width <= p.max_beam_width_px * s)
        i, j, ax, nrm, O, width = i[ok], j[ok], ax[ok], nrm[ok], O[ok], width[ok]
        ti = np.stack([np.einsum('ij,ij->i', P[i] - O, ax), np.einsum('ij,ij->i', Q[i] - O, ax)], 1)
        tj = np.stack([np.einsum('ij,ij->i', P[j] - O, ax), np.einsum('ij,ij->i', Q[j] - O, ax)], 1)
        # the edges must overlap enough to be a beam; its extent is the union of both edges (the
        # other beam can hide part of an edge); the contrast test below validates the whole extent
        overlap = np.minimum(ti.max(1), tj.max(1)) - np.maximum(ti.min(1), tj.min(1))
        a = np.minimum(ti.min(1), tj.min(1))
        b = np.maximum(ti.max(1), tj.max(1))
        ok = (overlap >= p.min_arm_px * s) & ((b - a) >= 2.0 * p.min_arm_px * s)
        i, j, ax, nrm, O, width, a, b = i[ok], j[ok], ax[ok], nrm[ok], O[ok], width[ok], a[ok], b[ok]
        if len(i) == 0:
            return {'n': 0}
        # contrast test along the beam: the interior must differ from BOTH outsides, same sign
        t = a[:, None] + (b - a)[:, None] * np.linspace(0.05, 0.95, samples)[None, :]      # (K,S)
        ctr = O[:, None, :] + t[..., None] * ax[:, None, :]
        off = (0.5 * width + max(2.0, 3.0 * s))[:, None, None] * nrm[:, None, :]

        def at(xy):
            xi = np.rint(xy[..., 0]).astype(int)
            yi = np.rint(xy[..., 1]).astype(int)
            inside = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)
            return np.where(inside, gray[np.clip(yi, 0, h - 1), np.clip(xi, 0, w - 1)], np.nan)
        gc, gl, gr = at(ctr), at(ctr - off), at(ctr + off)
        d1, d2 = gc - gl, gc - gr
        good = (d1 * d2 > 0) & (np.minimum(np.abs(d1), np.abs(d2)) >= p.beam_contrast)
        score = good.mean(axis=1)
        ok = score >= p.min_arm_support
        keep = np.flatnonzero(ok)
        # strongest/longest first; clutter can produce many near-duplicates
        keep = keep[np.argsort(-((b - a)[keep] * score[keep]))][:250]
        return {'n': len(keep), 'O': O[keep], 'ax': ax[keep], 'w': width[keep],
                'a': a[keep], 'b': b[keep], 'score': score[keep]}

    # -- crosses: pairs of beams -------------------------------------------------------------
    def _crossings(self, bm: dict, shape) -> List[Candidate]:
        p, s = self.p, self.p.detect_scale
        if bm['n'] < 2:
            return []
        h, w = shape
        i, j = np.triu_indices(bm['n'], 1)
        O, ax, wd, a, b, sc = (bm[k] for k in ('O', 'ax', 'w', 'a', 'b', 'score'))
        den = _cross2(ax[i], ax[j])
        ok = np.abs(den) >= math.sin(math.radians(p.min_cross_angle_deg))
        ok &= np.maximum(wd[i], wd[j]) <= p.max_width_ratio * np.minimum(wd[i], wd[j])
        i, j, den = i[ok], j[ok], den[ok]
        si = _cross2(O[j] - O[i], ax[j]) / den             # position along beam i
        pt = O[i] + si[:, None] * ax[i]
        tj = np.einsum('ij,ij->i', pt - O[j], ax[j])       # position along beam j
        m = p.min_arm_px * s
        room = np.stack([si - a[i], b[i] - si, tj - a[j], b[j] - tj], axis=1).min(axis=1)
        ok = (room >= m) & (pt[:, 0] >= 0) & (pt[:, 0] < w) & (pt[:, 1] >= 0) & (pt[:, 1] < h)
        i, j, pt, room = i[ok], j[ok], pt[ok], room[ok]
        if len(pt) == 0:
            return []
        score = np.minimum(sc[i], sc[j])
        dirs = np.stack([ax[i], ax[j]], axis=1)
        arm = np.minimum(room, p.max_arm_px * s)
        return self._merge(pt, dirs, arm, score, 0.5 * (wd[i] + wd[j]), p.merge_px * s)

    @staticmethod
    def _merge(pts, dirs, arms, scores, widths, radius) -> List[Candidate]:
        """Duplicates (fragmented edges give several beam pairs per beam) -> one candidate."""
        label = _components(np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2) < radius)
        out = []
        for g in np.unique(label):
            idx = np.flatnonzero(label == g)
            idx = idx[np.argsort(-scores[idx])]               # best member first
            ref = dirs[idx[0]]
            acc = [np.zeros((2, 2)), np.zeros((2, 2))]
            for k in idx:
                d = dirs[k]
                if abs(d[0] @ ref[0]) + abs(d[1] @ ref[1]) < abs(d[1] @ ref[0]) + abs(d[0] @ ref[1]):
                    d = d[::-1]
                for q in range(2):
                    acc[q] += np.outer(d[q], d[q])            # orientation-free average
            avg = np.zeros((2, 2))
            for q in range(2):
                v = np.linalg.eigh(acc[q])[1][:, -1]
                avg[q] = v if v @ ref[q] >= 0 else -v
            out.append(Candidate(pt=pts[idx].mean(axis=0), dirs=avg, arm=float(np.median(arms[idx])),
                                 score=float(scores[idx[0]]), members=len(idx),
                                 width_px=float(np.median(widths[idx]))))
        return out


# --------------------------------------------------------------------------------------------
# 3D verification
# --------------------------------------------------------------------------------------------
def fit_plane_ransac(pts: np.ndarray, tol: float, iters: int, rng: np.random.Generator,
                     min_sin: float = 0.1):
    """RANSAC plane through (N,3) points. Returns (normal, d, inlier_mask, rms) with the normal
    facing the origin (the camera), or None. Sample triples that are almost collinear (they
    define the plane badly; the samples lie on two lines, so many are) are skipped."""
    n = len(pts)
    if n < 3:
        return None
    idx = rng.integers(0, n, size=(iters, 3))
    a, b, c = pts[idx[:, 0]], pts[idx[:, 1]], pts[idx[:, 2]]
    ba, ca = b - a, c - a
    nrm = np.cross(ba, ca)
    ln = np.linalg.norm(nrm, axis=1)
    ok = ln > min_sin * np.maximum(np.linalg.norm(ba, axis=1) * np.linalg.norm(ca, axis=1), 1e-12)
    if not ok.any():
        return None
    nrm, a = nrm[ok] / ln[ok, None], a[ok]
    dist = np.abs(pts @ nrm.T - np.einsum('ij,ij->i', a, nrm)[None, :])      # (N, I)
    counts = (dist < tol).sum(axis=0)
    best = int(np.argmax(counts))
    inl = dist[:, best] < tol
    if inl.sum() < 3:
        return None
    for _ in range(2):                                   # least-squares refit on the inliers
        P = pts[inl]
        centroid = P.mean(axis=0)
        nv = np.linalg.svd(P - centroid)[2][2]
        d = -nv @ centroid
        new = np.abs(pts @ nv + d) < tol
        if new.sum() < 3:
            break
        inl = new
    if nv @ centroid > 0:
        nv, d = -nv, -d
    resid = pts[inl] @ nv + d
    return nv, d, inl, float(np.sqrt(np.mean(resid ** 2)))


def verify_candidate(cand: Candidate, depth_m: np.ndarray, cam: Camera, p: VerifyParams,
                     rng: np.random.Generator) -> Tuple[Optional[Cross3D], str]:
    """Depth check of one candidate. Returns (Cross3D, 'ok') or (None, reason)."""
    h, w = depth_m.shape
    n = p.samples_per_arm
    rs = np.linspace(0.0, cand.arm, n + 1)[1:]
    uv = [cand.pt[None, :]]
    arm_id = [np.full(1, -1)]
    for b in range(2):
        for k, sgn in enumerate((-1.0, 1.0)):
            uv.append(cand.pt[None, :] + sgn * rs[:, None] * cand.dirs[b][None, :])
            arm_id.append(np.full(n, 2 * b + k))
    uv, arm_id = np.vstack(uv), np.concatenate(arm_id)

    r = p.patch_px
    ui, vi = np.rint(uv[:, 0]).astype(int), np.rint(uv[:, 1]).astype(int)
    inb = (ui >= r) & (ui < w - r) & (vi >= r) & (vi < h - r)
    dv, du = np.meshgrid(np.arange(-r, r + 1), np.arange(-r, r + 1), indexing='ij')
    patch = depth_m[np.clip(vi, r, h - r - 1)[:, None] + dv.ravel()[None, :],
                    np.clip(ui, r, w - r - 1)[:, None] + du.ravel()[None, :]]
    patch = np.where((patch >= p.min_depth_m) & (patch <= p.max_depth_m), patch, np.nan)
    ok = inb & (np.isfinite(patch).sum(axis=1) >= max(3, patch.shape[1] // 5))
    if not ok.any():
        return None, 'no_depth'
    z = np.full(len(uv), np.nan)
    z[ok] = np.nanpercentile(patch[ok], p.near_percentile, axis=1)
    ok &= np.isfinite(z)
    if ok.sum() < p.min_valid_ratio * len(uv):
        return None, 'few_depth'
    if any((ok & (arm_id == a)).sum() < p.min_valid_per_arm for a in range(4)):
        return None, 'arm_no_depth'

    pts3 = np.stack([(uv[ok, 0] - cam.cx) * z[ok] / cam.fx,
                     (uv[ok, 1] - cam.cy) * z[ok] / cam.fy, z[ok]], axis=1)
    ids = arm_id[ok]
    fit = fit_plane_ransac(pts3, p.plane_tol_m, p.ransac_iters, rng)
    if fit is None:
        return None, 'no_plane'
    nrm, d, inl, rms = fit
    ratio = float(inl.mean())
    if ratio < p.min_inlier_ratio or any((inl & (ids == a)).sum() < p.min_valid_per_arm
                                         for a in range(4)):
        return None, 'not_coplanar'

    def ray_hit(pix):
        ray = np.array([(pix[0] - cam.cx) / cam.fx, (pix[1] - cam.cy) / cam.fy, 1.0])
        den = nrm @ ray
        if abs(den) < 1e-9:
            return None
        t = -d / den
        return t * ray if t > 0 else None

    center = ray_hit(cand.pt)
    if center is None or abs(nrm @ unit(center)) < p.min_view_cos:
        return None, 'grazing'
    if not p.min_depth_m <= center[2] <= p.max_depth_m:
        return None, 'range'
    arms3, halves = [], []
    for dvec in cand.dirs:
        a, b = ray_hit(cand.pt + cand.arm * dvec), ray_hit(cand.pt - cand.arm * dvec)
        if a is None or b is None:
            return None, 'grazing'
        arms3.append(unit(a - b))
        halves.append(0.5 * float(np.linalg.norm(a - b)))
    arms3 = np.array(arms3)
    angle = math.degrees(math.acos(min(1.0, abs(float(arms3[0] @ arms3[1])))))
    expected = min(p.expected_cross_angle_deg, 180.0 - p.expected_cross_angle_deg)
    if p.cross_angle_tol_deg < 90.0 and abs(angle - expected) > p.cross_angle_tol_deg:
        return None, 'angle'
    width_m = cand.width_px * float(center[2]) / cam.fx
    if (p.min_beam_width_m > 0 and width_m < p.min_beam_width_m) or \
            (p.max_beam_width_m > 0 and width_m > p.max_beam_width_m):
        return None, 'width'
    return Cross3D(center=center, normal=nrm, arms=arms3, arm_len=float(np.mean(halves)),
                   pixel=cand.pt.copy(), score=cand.score, inlier_ratio=ratio, rms=rms,
                   distance=float(np.linalg.norm(center)), beam_angle_deg=angle,
                   beam_width_m=width_m), 'ok'


# --------------------------------------------------------------------------------------------
# Landmarks
# --------------------------------------------------------------------------------------------
@dataclass
class Landmark:
    id: Optional[int]     # None while tentative; assigned on confirmation
    count: int
    center: np.ndarray
    normal: np.ndarray
    arms: np.ndarray
    arm_len: float
    weight: float
    last_seen: float


class LandmarkMap:
    """Persistent crosses in one fixed frame (``map``). Two identical crosses are told apart
    by position only: an observation goes to the nearest landmark within ``landmark_merge_m``.
    IDs are handed out when a landmark is confirmed, from a counter that survives save/load, so
    noise never uses up IDs and an ID is never reused."""

    VERSION = 1

    def __init__(self, params: Optional[LandmarkParams] = None, frame: str = 'map'):
        self.p = params or LandmarkParams()
        self.frame = frame
        self.landmarks: List[Landmark] = []
        self.next_id = 0
        self.dirty = False

    def confirmed(self) -> List[Landmark]:
        return [lm for lm in self.landmarks if lm.id is not None]

    # -- fusion ---------------------------------------------------------------------------
    def update(self, crosses: List[Cross3D], t: float) -> int:
        """Fuse one frame's observations (in the landmark frame). One observation per landmark
        and frame: a second detection of the same cross in the same frame is dropped."""
        pairs = []
        for ci, c in enumerate(crosses):
            for li, lm in enumerate(self.landmarks):
                dist = float(np.linalg.norm(lm.center - c.center))
                if dist < self.p.landmark_merge_m:
                    pairs.append((dist, ci, li))
        done_c, done_l = set(), set()
        for _, ci, li in sorted(pairs):
            if ci in done_c:
                continue
            done_c.add(ci)
            if li not in done_l:
                done_l.add(li)
                self._fuse(self.landmarks[li], crosses[ci], t)
        created = 0
        for ci, c in enumerate(crosses):
            if ci in done_c:
                continue
            if any(np.linalg.norm(lm.center - c.center) < self.p.landmark_merge_m
                   for lm in self.landmarks[len(self.landmarks) - created:]):
                continue                                              # duplicate in this frame
            self.landmarks.append(Landmark(None, 1, c.center.copy(), c.normal.copy(),
                                           c.arms.copy(), c.arm_len, _weight(c), t))
            created += 1
        return created

    def _fuse(self, lm: Landmark, c: Cross3D, t: float) -> None:
        w = _weight(c)
        a = w / (lm.weight + w)
        lm.center = (1 - a) * lm.center + a * c.center
        n = c.normal if c.normal @ lm.normal >= 0 else -c.normal
        lm.normal = unit((1 - a) * lm.normal + a * n)
        new = c.arms
        if abs(new[0] @ lm.arms[0]) + abs(new[1] @ lm.arms[1]) < \
                abs(new[1] @ lm.arms[0]) + abs(new[0] @ lm.arms[1]):
            new = new[::-1]
        arms = []
        for b in range(2):
            v = new[b] if new[b] @ lm.arms[b] >= 0 else -new[b]
            arms.append(unit((1 - a) * lm.arms[b] + a * v))
        lm.arms = np.array(arms)
        lm.arm_len = (1 - a) * lm.arm_len + a * c.arm_len
        lm.weight = min(lm.weight + w, self.p.max_weight)
        lm.count += 1
        lm.last_seen = t
        if lm.id is None and lm.count >= self.p.min_observations:
            self._try_confirm(lm)
        if lm.id is not None:
            self.dirty = True

    def _try_confirm(self, lm: Landmark) -> None:
        """Confirm ``lm`` unless the top-N rule says no. When ``max_landmarks`` crosses are already
        confirmed, ``lm`` replaces the weakest one only with replace_margin x its observations."""
        conf = self.confirmed()
        n = self.p.max_landmarks
        if n > 0 and len(conf) >= n:
            weakest = min(conf, key=lambda c: c.count)
            if lm.count <= self.p.replace_margin * weakest.count:
                return
            self.landmarks.remove(weakest)
            self.dirty = True
        lm.id = self.next_id
        self.next_id += 1

    def prune(self, t: float) -> int:
        """Drop unconfirmed landmarks that were not seen again in time."""
        keep = [lm for lm in self.landmarks
                if lm.id is not None or t - lm.last_seen <= self.p.tentative_ttl_s]
        n = len(self.landmarks) - len(keep)
        self.landmarks = keep
        return n

    # -- persistence ----------------------------------------------------------------------
    def to_dict(self) -> dict:
        return {
            'version': self.VERSION, 'frame': self.frame, 'next_id': self.next_id,
            'crosses': [{'id': lm.id, 'count': lm.count,
                         'center': lm.center.tolist(), 'normal': lm.normal.tolist(),
                         'arms': lm.arms.tolist(), 'arm_len': lm.arm_len}
                        for lm in sorted(self.confirmed(), key=lambda x: x.id)],
        }

    def save(self, path: str) -> int:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=2)
        os.replace(tmp, path)
        self.dirty = False
        return len(self.confirmed())

    def load(self, path: str) -> int:
        """Replace the landmarks with the file's. Missing file -> 0. A file that cannot be read is
        moved to ``<path>.corrupt`` (so the next save cannot silently destroy it) and ValueError
        is raised."""
        if not path or not os.path.exists(path):
            return 0
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
            lms = [Landmark(int(c['id']), int(c['count']), np.array(c['center'], float),
                            np.array(c['normal'], float), np.array(c['arms'], float).reshape(2, 3),
                            float(c.get('arm_len', 0.0)), self.p.max_weight, -1e18)
                   for c in data['crosses']]
            next_id = max([int(data.get('next_id', 0))] + [lm.id + 1 for lm in lms])
        except (OSError, ValueError, KeyError, TypeError) as e:
            os.replace(path, path + '.corrupt')
            raise ValueError(f'{path} unreadable ({e}); moved to {path}.corrupt') from e
        self.landmarks, self.next_id, self.dirty = lms, next_id, False
        return len(lms)


def _weight(c: Cross3D) -> float:
    """Closer observations are more accurate (depth noise grows with range squared)."""
    return (2.0 / max(c.distance, 0.5)) ** 2
