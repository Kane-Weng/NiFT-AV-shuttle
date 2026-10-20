"""
Re-export the vehicle constants from nift_common for older imports.

File: shuttle_config.py

Description:
  Moved to nift_common.vehicle (values in nift_common/vehicle.yaml). Kept so existing imports keep
  working; new code imports nift_common.vehicle.
"""
from nift_common.vehicle import ShuttleConfig, VehicleState  # noqa: F401
