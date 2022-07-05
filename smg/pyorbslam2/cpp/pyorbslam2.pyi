from smg.pyopencv import CVMat1d, CVMat1f, CVMat3b


# ENUMERATIONS

class EScoringType(int):
	pass

L1_NORM: EScoringType
L2_NORM: EScoringType
CHI_SQUARE: EScoringType
KL: EScoringType
BHATTACHARYYA: EScoringType
DOT_PRODUCT: EScoringType

class ESensor(int):
	pass

MONOCULAR: ESensor
STEREO: ESensor
RGBD: ESensor

class EWeightingType(int):
	pass

TF_IDF: EWeightingType
TF: EWeightingType
IDF: EWeightingType
BINARY: EWeightingType

# CLASSES

class ORBVocabulary:
	def __init__(
		self, k: int = 10, ell: int = 5, weighting: EWeightingType = TF_IDF, scoring: EScoringType = L1_NORM
	): ...
	def load_from_binary_file(self, filename: str) -> bool: ...
	def load_from_text_file(self, filename: str) -> bool: ...
	def save_to_binary_file(self, filename: str) -> None: ...
	def save_to_text_file(self, filename: str) -> None: ...

class System:
	def __init__(self, voc_file: str, settings_file: str, sensor: ESensor, use_viewer: bool): ...
	def shutdown(self) -> None: ...
	def track_monocular(self, im: CVMat3b, timestamp: float) -> CVMat1d: ...
	def track_rgbd(self, im: CVMat3b, depthmap: CVMat1f, timestamp: float) -> CVMat1d: ...
