"""Pinned cross in the drone's base frame, without ROS."""
from __future__ import annotations

import json
import os
from typing import Optional, Tuple

import numpy as np


def pin_in_base(R_map_base: np.ndarray, t_map_base: np.ndarray, p_map: np.ndarray) -> np.ndarray:
    """Vector from the base_link origin to a point given in the map frame, in base_link axes.
    R_map_base, t_map_base: pose of base_link in the map frame."""
    return np.asarray(R_map_base, float).T @ (np.asarray(p_map, float) - np.asarray(t_map_base, float))


def load_pin_position(path: str, pin_id: int = -1) -> Optional[Tuple[int, np.ndarray]]:
    """(id, position in the map frame) of a pin in a pins JSON file, or None when the file is
    missing, unreadable (it may be mid-write) or has no such pin. pin_id < 0: the lowest id.
    Read only: unlike PinStore.load this never renames the file."""
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, encoding='utf-8') as f:
            pins = json.load(f)['pins']
        pins = {int(p['id']): np.array(p['position'], float).reshape(3) for p in pins}
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if not pins:
        return None
    if pin_id < 0:
        pin_id = min(pins)
    return (pin_id, pins[pin_id]) if pin_id in pins else None
