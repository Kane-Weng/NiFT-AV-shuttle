"""
Coordinate conversions between Lanelet2 UTM space, the local map frame and RViz.

File: coord_transform.py
Author: Kane Weng
Date: Apr 11, 2026

Description:
  Coordinate frame conversions between Lanelet2 UTM absolute space,
  the local map frame, and the RViz display frame.

  Frame definitions
  -----------------
  lanelet_xy   — raw UTM easting/northing as returned by UtmProjector.forward()
  local_xy     — lanelet_xy minus the map origin (meters from origin, no display shift)
  display_xy   — local_xy minus any display_offset (what RViz shows on screen)

  In the current NiFT deployment the display_offset is (0, 0) because the simulator
  and RViz share the same origin. The offset attributes are retained so any future
  divergence can be handled without touching the node.
"""


class CoordTransform:
    """
    Converts 2D coordinates between Lanelet2 UTM space and the RViz display frame.

    Parameters
    ----------
    map_origin_x, map_origin_y : float
        UTM easting/northing of the map origin, obtained from UtmProjector.
    display_offset_x, display_offset_y : float
        Additional shift applied between the local frame and the RViz display.
        Defaults to 0.0 (the simulator and RViz origins are aligned).

    """

    def __init__(self, map_origin_x: float, map_origin_y: float,
                 display_offset_x: float = 0.0, display_offset_y: float = 0.0):
        self.map_origin_x = map_origin_x
        self.map_origin_y = map_origin_y
        self.display_offset_x = display_offset_x
        self.display_offset_y = display_offset_y

    # ------------------------------------------------------------------
    # Primitive conversions
    # ------------------------------------------------------------------

    def to_local_xy(self, lanelet_x: float, lanelet_y: float):
        """Lanelet2 UTM absolute → local map frame (subtract map origin)."""
        return lanelet_x - self.map_origin_x, lanelet_y - self.map_origin_y

    def to_lanelet_xy(self, local_x: float, local_y: float):
        """Local map frame → Lanelet2 UTM absolute (add map origin)."""
        return local_x + self.map_origin_x, local_y + self.map_origin_y

    # ------------------------------------------------------------------
    # Display-frame conversions (account for optional display offset)
    # ------------------------------------------------------------------

    def to_display_xy(self, lanelet_x: float, lanelet_y: float):
        """Lanelet2 UTM absolute → RViz display coordinates."""
        local_x, local_y = self.to_local_xy(lanelet_x, lanelet_y)
        return local_x - self.display_offset_x, local_y - self.display_offset_y

    def display_to_lanelet_xy(self, display_x: float, display_y: float):
        """Convert RViz display coordinates to Lanelet2 UTM absolute."""
        local_x = display_x + self.display_offset_x
        local_y = display_y + self.display_offset_y
        return self.to_lanelet_xy(local_x, local_y)
