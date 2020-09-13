import numpy as np


#################### InputSource ####################

# CLASSES

class ImageSourceEngine:
	def get_calib(self) -> ITMRGBDCalib: ...
	def get_depth_image_size(self) -> Vector2i: ...
	def get_images(self, rgb: ORUChar4Image, raw_depth: ORShortImage): ...
	def get_rgb_image_size(self) -> Vector2i: ...
	def has_images_now(self) -> bool: ...
	def has_more_images(self) -> bool: ...

class ImageMaskPathGenerator:
	def __init__(self, rgb_image_mask: str, depth_image_mask: str): ...
	def get_rgb_image_path(self, current_frame_no: int) -> str: ...
	def get_depth_image_path(self, current_frame_no: int) -> str: ...

class ImageMaskFileReader(ImageSourceEngine):
	def __init__(self, calib_filename: str, path_generator: ImageMaskPathGenerator, initial_frame_no: int): ...

#################### ITMLib ####################

# FUNCTIONS

def read_rgbd_calib(filename: str, calib: ITMRGBDCalib) -> bool: ...

# ENUMERATIONS

class GetImageType(int):
	pass

IMAGE_ORIGINAL_RGB: GetImageType
IMAGE_COLOUR_FROM_VOLUME: GetImageType

# CLASSES

class ITMBasicEngine:
	def __init__(self, settings: ITMLibSettings, calib: ITMRGBDCalib, img_size_rgb: Vector2i, img_size_d: Vector2i): ...
	def get_view(self) -> ITMView: ...
	def get_tracking_state(self) -> ITMTrackingState: ...
	def get_scene(self) -> ITMScene: ...
	def process_frame(self, rgb_image: ORUChar4Image, raw_depth_image: ORShortImage): ...
	def save_scene_to_mesh(self, filename: str): ...
	def save_to_file(self): ...
	def load_from_file(self): ...
	def get_image_size(self) -> Vector2i: ...
	def get_image(self, out: ORUChar4Image, get_image_type: GetImageType, pose: SE3Pose, intrinsics: ITMIntrinsics): ...
	def turn_on_tracking(self): ...
	def turn_off_tracking(self): ...
	def turn_on_integration(self): ...
	def turn_off_integration(self): ...
	def turn_on_main_processing(self): ...
	def turn_off_main_processing(self): ...
	def reset_all(self): ...

class ITMDenseMapper:
	def __init__(self, settings: ITMLibSettings): ...
	def reset_scene(self, scene: ITMScene): ...
	def process_frame(self, view: ITMView, tracking_state: ITMTrackingState, scene: ITMScene, render_state_live: ITMRenderState, reset_visible_list: bool): ...
	def update_visible_list(self, view: ITMView, tracking_state: ITMTrackingState, scene: ITMScene, render_state: ITMRenderState, reset_visible_list: bool): ...

class ITMDisparityCalib:
	pass

class ITMExtrinsics:
	pass

class ITMIntrinsics:
	# TODO: Image size
	projection_params_simple: ProjectionParamsSimple

class ProjectionParamsSimple:
	fx: float
	fy: float
	px: float
	py: float

class ITMLibSettings:
	device_type: DeviceType
	use_approximate_raycast: bool
	# TODO
	scene_params: ITMSceneParams
	# TODO
	def __init__(self): ...
	def get_memory_type(self) -> MemoryDeviceType: ...

class ITMLowLevelEngine:
	pass

class ITMLowLevelEngineFactory:
	@staticmethod
	def make_low_level_engine(self, device_type: DeviceType) -> ITMLowLevelEngine: ...

class ITMRenderState:
	no_fwd_proj_missing_points: int

class ITMRenderStateFactory:
	@staticmethod
	def create_render_state(img_size: Vector2i, scene_params: ITMSceneParams, memory_type: MemoryDeviceType) -> ITMRenderState: ...

class ITMRGBDCalib:
	intrinsics_rgb: ITMIntrinsics
	intrinsics_d: ITMIntrinsics
	trafo_rgb_to_depth: ITMExtrinsics
	disparity_calib: ITMDisparityCalib
	def __init__(self): ...

class ITMScene:
	scene_params: ITMSceneParams
	def __init__(self, scene_params: ITMSceneParams, use_swapping: bool, memory_type: MemoryDeviceType): ...

class ITMSceneParams:
	voxel_size: float
	view_frustum_min: float
	view_frustum_max: float
	mu: float
	max_w: int
	stop_integrating_at_max_w: bool

class ITMTrackingState:
	pose_d: SE3Pose
	def __init__(self, img_size: Vector2i, memory_type: MemoryDeviceType): ...

class TrackingResult:
	pass

class ITMView:
	rgb: ORUChar4Image
	depth: ORFloatImage
	def __init__(self, calibration: ITMRGBDCalib, img_size_rgb: Vector2i, img_size_d: Vector2i, use_gpu: bool)

class ITMViewBuilder:
	pass

class ITMViewBuilderFactory:
	@staticmethod
	def make_view_builder(calib: ITMRGBDCalib, device_type: DeviceType) -> ITMViewBuilder: ...

class ITMVisualisationEngine:
	pass

class ITMVisualisationEngineFactory:
	@staticmethod
	def make_visualisation_engine(device_type: DeviceType) -> ITMVisualisationEngine: ...

#################### ORUtils ####################

# FUNCTIONS

def read_image_from_file(image: ORUChar4Image, filename: str) -> bool: ...

# ENUMERATIONS

class DeviceType(int):
	pass

DEVICE_CPU: DeviceType
DEVICE_CUDA: DeviceType

class MemoryDeviceType(int):
	pass

MEMORYDEVICE_CPU: MemoryDeviceType
MEMORYDEVICE_CUDA: MemoryDeviceType

# CLASSES

class Matrix4f(np.ndarray):
	def __init__(self, *args): ...
	def at(self, x: int, y: int) -> float: ...

class ORFloatImage(np.ndarray):
	no_dims: Vector2i
	def __init__(self, no_dims: Vector2i, allocate_cpu: bool, allocate_cuda: bool, metal_compatible: bool)

class ORShortImage(np.ndarray):
	no_dims: Vector2i
	def __init__(self, no_dims: Vector2i, allocate_cpu: bool, allocate_cuda: bool, metal_compatible: bool)

class ORUChar4Image(np.ndarray):
	no_dims: Vector2i
	def __init__(self, no_dims: Vector2i, allocate_cpu: bool, allocate_cuda: bool, metal_compatible: bool)

class SE3Pose:
	def __init__(self, src: Matrix4f): ...
	def get_inv_m(self) -> Matrix4f: ...
	def get_m(self) -> Matrix4f: ...
	def get_t(self) -> Vector3f: ...
	def set_t(self, t: Vector3f): ...

class Vector2i(np.ndarray):
	x: int
	y: int
	def __init__(self, x: int, y: int): ...

class Vector3f(np.ndarray):
	x: float
	y: float
	z: float
	def __init__(self, x: float, y: float, z: float): ...

class Vector4u(np.ndarray):
	x: int
	y: int
	z: int
	w: int
	def __init__(self, x: int, y: int, z: int, w: int): ...

#################### tvgutil ####################

# CLASSES

class SettingsContainer:
	def __init__(self): ...

#################### orx ####################

# CLASSES

class DualNumberd:
	r: float
	d: float
	def __init__(self, r: float, d: float): ...
	def __iadd__(self, rhs): ...
	def __isub__(self, rhs): ...
	def __imul__(self, rhs): ...
	def conjugate(self) -> DualNumberd: ...
	def inverse(self) -> DualNumberd: ...
	def is_pure(self) -> bool: ...
	def sqrt(self) -> DualNumberd: ...
	def __add__(self, rhs): ...
	def __sub__(self, rhs): ...
	def __mul__(self, rhs): ...
	def __neg__(self): ...

#################### itmx ####################

# ENUMERATIONS

class DepthType(int):
	pass

DT_EUCLIDEAN: DepthType
DT_ORTHOGRAPHIC: DepthType

# CLASSES

class DepthVisualiser:
	pass

class DepthVisualisationUtil:
	@staticmethod
	def generate_depth_from_voxels(
		output: ORFloatImage, scene: ITMScene, pose: SE3Pose, intrinsics: ITMIntrinsics,
		render_state: ITMRenderState, depth_type: DepthType,
		voxel_visualisation_engine: ITMVisualisationEngine,
		depth_visualiser: DepthVisualiser, settings: Settings
	): ...

class DepthVisualiserFactory:
	@staticmethod
	def make_depth_visualiser(device_type: DeviceType) -> DepthVisualiser: ...

class Settings(ITMLibSettings, SettingsContainer):
	def __init__(self): ...
