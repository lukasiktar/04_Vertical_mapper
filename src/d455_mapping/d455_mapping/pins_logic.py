"""Pins on detected crosses, without ROS: storage, toggling, snapping and camera distances.

A pin is the position (``map`` frame) of a cross center that the user selected. It is stored when
pinned and does not follow later refinements of the cross landmark. Pins are saved as JSON next to
the RTAB-Map database.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class Pin:
    id: int
    cross_id: Optional[int]
    position: np.ndarray
    arms: Optional[np.ndarray] = None    # (2, 3) beam directions of the pinned cross, to draw the pin


class PinStore:
    """Pins in one fixed frame. IDs are handed out from a counter that survives save/load, so
    removing a pin never makes another pin change its ID."""

    VERSION = 1

    def __init__(self, frame: str = 'map'):
        self.frame = frame
        self.pins: List[Pin] = []
        self.next_id = 1
        self.dirty = False

    def toggle_cross(self, cross_id: int, center: np.ndarray, arms: Optional[np.ndarray] = None,
                     same_tol_m: float = 0.05) -> Tuple[str, Pin]:
        """Pin the cross, or remove its pin if it already has one. Returns ('added'|'removed', pin)."""
        for pin in self.pins:
            if pin.cross_id == cross_id or np.linalg.norm(pin.position - center) < same_tol_m:
                self.pins.remove(pin)
                self.dirty = True
                return 'removed', pin
        pin = Pin(self.next_id, cross_id, np.asarray(center, float).copy(),
                  None if arms is None else np.asarray(arms, float).reshape(2, 3).copy())
        self.next_id += 1
        self.pins.append(pin)
        self.dirty = True
        return 'added', pin

    def clear(self) -> int:
        n = len(self.pins)
        self.pins = []
        self.dirty = n > 0
        return n

    def to_dict(self) -> dict:
        return {'version': self.VERSION, 'frame': self.frame, 'next_id': self.next_id,
                'pins': [{'id': p.id, 'cross_id': p.cross_id, 'position': p.position.tolist(),
                          **({'arms': p.arms.tolist()} if p.arms is not None else {})}
                         for p in sorted(self.pins, key=lambda p: p.id)]}

    def save(self, path: str) -> int:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=2)
        os.replace(tmp, path)
        self.dirty = False
        return len(self.pins)

    def load(self, path: str) -> int:
        """Missing file -> 0. An unreadable file is moved to ``<path>.corrupt`` and ValueError raised."""
        if not path or not os.path.exists(path):
            return 0
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
            pins = [Pin(int(p['id']), None if p.get('cross_id') is None else int(p['cross_id']),
                        np.array(p['position'], float).reshape(3),
                        np.array(p['arms'], float).reshape(2, 3) if p.get('arms') else None)
                    for p in data['pins']]
            next_id = max([int(data.get('next_id', 1))] + [p.id + 1 for p in pins])
        except (OSError, ValueError, KeyError, TypeError) as e:
            os.replace(path, path + '.corrupt')
            raise ValueError(f'{path} unreadable ({e}); moved to {path}.corrupt') from e
        self.pins, self.next_id, self.dirty = pins, next_id, False
        return len(pins)


def nearest_cross(crosses: Dict[int, np.ndarray], click: np.ndarray,
                  radius: float) -> Tuple[Optional[Tuple[int, np.ndarray]], float]:
    """Cross whose center is closest to the clicked point. Returns ((id, center) or None if farther
    than ``radius``, distance to the closest cross; inf when there are none)."""
    if not crosses:
        return None, float('inf')
    cid = min(crosses, key=lambda k: np.linalg.norm(crosses[k] - click))
    dist = float(np.linalg.norm(crosses[cid] - click))
    return ((cid, crosses[cid]) if dist <= radius else None), dist


def pin_distances(pins: List[Pin], R_map_cam: np.ndarray, T_map_cam: np.ndarray) -> List[dict]:
    """Distance from the camera origin to each pin and the pin in the camera frame (for the optical
    frame: x right, y down, z forward)."""
    out = []
    for p in sorted(pins, key=lambda p: p.id):
        rel = p.position - T_map_cam
        cam = R_map_cam.T @ rel
        out.append({'id': p.id, 'cross_id': p.cross_id, 'distance_m': float(np.linalg.norm(rel)),
                    'camera_xyz': cam.tolist(), 'map_xyz': p.position.tolist()})
    return out
