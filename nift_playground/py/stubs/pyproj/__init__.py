"""
pyproj stand-in (ARCH section 5). `carla_geo.py` imports pyproj and builds its CRS and transformers in CarlaGeo's constructor, but the kinematic sim only converts map <-> WGS84, which goes through Lanelet2's UtmProjector. Only CARLA world conversions call `transform()`, so that raises NotImplementedError. Construction works and keeps the definition for error messages.
"""


class ProjError(RuntimeError):
    pass


class CRS:
    def __init__(self, definition=None, **kwargs):
        self.definition = definition if definition is not None else kwargs

    @classmethod
    def from_proj4(cls, proj_string):
        return cls(proj_string)

    @classmethod
    def from_epsg(cls, code):
        return cls(f'EPSG:{int(code)}')

    @classmethod
    def from_string(cls, text):
        return cls(text)

    @classmethod
    def from_user_input(cls, value):
        return value if isinstance(value, CRS) else cls(value)

    def to_proj4(self):
        return str(self.definition)

    def __repr__(self):
        return f'<CRS (playground stand-in): {self.definition}>'


class Transformer:
    def __init__(self, crs_from, crs_to, always_xy=False):
        self.source_crs = CRS.from_user_input(crs_from)
        self.target_crs = CRS.from_user_input(crs_to)
        self.always_xy = always_xy

    @classmethod
    def from_crs(cls, crs_from, crs_to, always_xy=False, **_kwargs):
        return cls(crs_from, crs_to, always_xy)

    def transform(self, *args, **kwargs):
        raise NotImplementedError(
            'pyproj stand-in: no projections in the playground (only CARLA world coordinates use them; '
            f'{self.source_crs} -> {self.target_crs})')

    def itransform(self, *args, **kwargs):
        self.transform()


class Proj:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError('pyproj stand-in: Proj is not available in the playground')
