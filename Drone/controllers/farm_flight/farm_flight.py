"""Farm-world controller: the flight logic lives in Drone/validation.

Importing inertial_flight runs the mission, so this file is only a shim that
puts the validation controller on sys.path.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / 'validation/controllers/inertial_flight'))
import inertial_flight  # noqa: E402,F401
