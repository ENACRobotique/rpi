from typing import NamedTuple
import numpy as np
import numpy.typing as npt
try:
    import loca_lidar.config as config
except ModuleNotFoundError:
    import config

class hashabledict(dict):
    # https://stackoverflow.com/questions/1151658/python-hashable-dicts
    def __hash__(self):
        return hash(tuple(sorted(self.items())))
    

CartesianPts = np.dtype([
        ('x', np.float64),
        ('y', np.float64),
    ])

CartesianPts_t = npt.NDArray[CartesianPts]

PolarPts = np.dtype([
        ('distance', np.float64),
        ('angle', np.float64),
    ])

PolarPts_t = npt.NDArray[PolarPts]

AmalgamePolar = np.dtype([
        ('center_polar', PolarPts), 
        ('list_pts', (PolarPts, config.amalg_max_nb_pts)),
        ('size', np.float64),
    ])

AmalgamePolar_t = npt.NDArray[AmalgamePolar]

AmalgameCartesian = np.dtype([
    ('center_cartesian', CartesianPts), 
    ('list_pts', (CartesianPts, config.amalg_max_nb_pts)),
    ('size', np.float64),
    ])

class DistPts(NamedTuple):
    pt1: int
    pt2: int 
    squared_dist: float
"""
DistPts = np.dtype([
        ('pt1', np.int64),
        ('pt2', np.int64),
        ('squared_dist', np.float64),
    ])
"""



