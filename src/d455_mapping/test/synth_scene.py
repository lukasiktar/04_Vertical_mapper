"""Synthetic RGB + aligned depth of planar beams, for testing the cross detector without a camera.

Pinhole camera (D455 color-like), z-buffered planar rectangles ("beams"), a far textured wall,
image noise, depth noise growing with range^2, random depth dropouts and invalid depth along
beam silhouettes (real depth is unreliable at edges).
"""
import cv2
import numpy as np

from d455_mapping.cross_logic import Camera

W, H = 640, 480
CAM = Camera(615.0, 615.0, 320.0, 240.0)


class Rect:
    """Planar rectangle: center c, unit length direction u, unit width direction v (u, v in the plane)."""

    def __init__(self, c, u, v, half_len, half_wid, gray):
        self.c, self.u, self.v = np.asarray(c, float), np.asarray(u, float), np.asarray(v, float)
        self.half_len, self.half_wid, self.gray = half_len, half_wid, gray

    def corners(self):
        return [self.c + sl * self.half_len * self.u + sw * self.half_wid * self.v
                for sl, sw in ((-1, -1), (1, -1), (1, 1), (-1, 1))]

    @property
    def normal(self):
        return np.cross(self.u, self.v)


def plane_axes(yaw_deg=0.0, pitch_deg=0.0):
    """In-plane axes e1 (right), e2 (down) and normal n (toward the camera) of a plane that is
    rotated by yaw about y and pitch about x from fronto-parallel."""
    y, p = np.radians(yaw_deg), np.radians(pitch_deg)
    Ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(p), -np.sin(p)], [0, np.sin(p), np.cos(p)]])
    R = Ry @ Rx
    return R @ [1, 0, 0], R @ [0, 1, 0], R @ [0, 0, -1]


def beam(center, e1, e2, angle_deg, half_len=0.4, width=0.08, gray=200, offset=(0, 0, 0)):
    a = np.radians(angle_deg)
    u = np.cos(a) * e1 + np.sin(a) * e2
    v = -np.sin(a) * e1 + np.cos(a) * e2
    return Rect(np.asarray(center, float) + offset, u, v, half_len, width / 2, gray)


def cross(center, angle_between=90.0, orientation=45.0, yaw=0.0, pitch=0.0, half_len=0.4,
          width=0.08, depth_offset_b=0.0):
    """Two coplanar beams (B on top) crossing at ``center``. ``depth_offset_b`` moves beam B
    along the plane normal (non-coplanar test)."""
    e1, e2, n = plane_axes(yaw, pitch)
    a = beam(center, e1, e2, orientation - angle_between / 2, half_len, width, 200)
    b = beam(center, e1, e2, orientation + angle_between / 2, half_len, width, 170,
             offset=-depth_offset_b * n)
    return [a, b]


def render(rects, R_wc=None, t_wc=None, seed=0, wall_z=6.0, depth_sigma=0.0015, dropout=0.03):
    """Returns (bgr uint8, depth float32 meters). World -> camera: p_cam = R_wc.T @ (p - t_wc)."""
    rng = np.random.default_rng(seed)
    R = np.eye(3) if R_wc is None else np.asarray(R_wc)
    t = np.zeros(3) if t_wc is None else np.asarray(t_wc)
    vv, uu = np.mgrid[0:H, 0:W]
    rays = np.stack([(uu - CAM.cx) / CAM.fx, (vv - CAM.cy) / CAM.fy, np.ones((H, W))], axis=-1)

    # background wall with weak low-frequency texture
    tex = cv2.GaussianBlur(rng.normal(0, 1, (H, W)).astype(np.float32), (0, 0), 6) * 60
    gray = 80.0 + tex
    zbuf = np.full((H, W), wall_z)
    sil = np.zeros((H, W), np.uint8)

    for r in rects:
        pc = [R.T @ (p - t) for p in r.corners()]
        if min(p[2] for p in pc) < 0.1:
            continue
        uv = np.array([[CAM.fx * p[0] / p[2] + CAM.cx, CAM.fy * p[1] / p[2] + CAM.cy] for p in pc])
        mask = np.zeros((H, W), np.uint8)
        cv2.fillConvexPoly(mask, np.round(uv).astype(np.int32), 1)
        n = R.T @ r.normal
        c = R.T @ (r.c - t)
        z = (n @ c) / np.maximum(rays @ n, 1e-9)
        upd = (mask > 0) & (z <= zbuf + 1e-9)
        zbuf[upd] = z[upd]
        gray[upd] = r.gray
        sil |= cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))

    img = cv2.GaussianBlur(gray.astype(np.float32), (0, 0), 0.8) + rng.normal(0, 2.0, (H, W))
    bgr = np.repeat(np.clip(img, 0, 255).astype(np.uint8)[..., None], 3, axis=2)
    depth = (zbuf + rng.normal(0, 1, (H, W)) * depth_sigma * zbuf ** 2).astype(np.float32)
    depth[(rng.random((H, W)) < dropout) | (cv2.dilate(sil, np.ones((3, 3), np.uint8)) > 0)] = 0.0
    return bgr, depth


def look_at_pose(position, target, up=(0, 0, 1)):
    """Camera (x right, y down, z forward) at ``position`` looking at ``target``; world z up."""
    p, t = np.asarray(position, float), np.asarray(target, float)
    z = (t - p) / np.linalg.norm(t - p)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1), p
