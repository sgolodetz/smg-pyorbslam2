import numpy as np


# CLASSES

class CVMat1d(np.ndarray):
	@staticmethod
	def zeros(rows: int, cols: int) -> CVMat1d: ...

class CVMat1f(np.ndarray):
	@staticmethod
	def zeros(rows: int, cols: int) -> CVMat1f: ...

class CVMat3b(np.ndarray):
	@staticmethod
	def zeros(rows: int, cols: int) -> CVMat3b: ...

class System:
	def __init__(self, voc_file: str, settings_file: str, sensor: ESensor, use_viewer: bool): ...
	def track_monocular(self, im: CVMat3b, timestamp: float) -> CVMat1d: ...
	def track_rgbd(self, im: CVMat3b, depthmap: CVMat1f, timestamp: float) -> CVMat1d: ...

# ENUMERATIONS

class ESensor(int):
	pass

MONOCULAR: ESensor
STEREO: ESensor
RGBD: ESensor
