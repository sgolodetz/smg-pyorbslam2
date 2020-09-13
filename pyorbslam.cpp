#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
namespace py = pybind11;

#pragma warning(disable:4244 4267 4996)
#include "System.h"
#pragma warning(default:4244 4267 4996)
using namespace ORB_SLAM2;

#if 0
#include <iostream>

#include <boost/lexical_cast.hpp>

#include <InputSource/ImageSourceEngine.h>
using namespace InputSource;

#include <ITMLib/ITMLibDefines.h>
#include <ITMLib/Core/ITMBasicEngine.h>
#include <ITMLib/Core/ITMDenseMapper.h>
#include <ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h>
#include <ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <ITMLib/Objects/Scene/ITMVoxelTypes.h>
#include <ITMLib/Objects/Views/ITMView.h>
#include <ITMLib/Utils/ITMLibSettings.h>
#include <ITMLib/Utils/ITMSceneParams.h>
using namespace ITMLib;

#include <ORUtils/FileUtils.h>
using namespace ORUtils;

#include <itmx/visualisation/DepthVisualisationUtil.tpp>
#include <itmx/visualisation/DepthVisualiserFactory.h>
using namespace itmx;

#include <orx/geometry/DualNumber.h>
using namespace orx;

#include <tvgutil/misc/SettingsContainer.h>
using namespace tvgutil;

// Tell pybind about Boost's shared pointers.
PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#endif

void f(const cv::Mat& im)
{
  // TODO
}

PYBIND11_MODULE(pyorbslam, m)
{
  // FUNCTIONS

  m.def("f", [](const cv::Mat3b& im) { f(im); }, "");

  // CLASSES

  py::class_<cv::Mat1d>(m, "CVMat1d", pybind11::buffer_protocol())
    .def_buffer([](cv::Mat1d& img) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        img.data,
        sizeof(double),
        pybind11::format_descriptor<double>::format(),
        2,
        { img.rows, img.cols },
        { sizeof(double) * img.cols, sizeof(double) }
      );
    })
    .def_static("zeros", [](int rows, int cols) -> cv::Mat1d { return cv::Mat1d::zeros(rows, cols); }, "")
  ;

  py::class_<cv::Mat1f>(m, "CVMat1f", pybind11::buffer_protocol())
    .def_buffer([](cv::Mat1f& img) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        img.data,
        sizeof(float),
        pybind11::format_descriptor<float>::format(),
        2,
        { img.rows, img.cols },
        { sizeof(float) * img.cols, sizeof(float) }
      );
    })
    .def_static("zeros", [](int rows, int cols) -> cv::Mat1f { return cv::Mat1f::zeros(rows, cols); }, "")
  ;

  py::class_<cv::Mat3b>(m, "CVMat3b", pybind11::buffer_protocol())
    .def_buffer([](cv::Mat3b& img) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        img.data,
        sizeof(unsigned char),
        pybind11::format_descriptor<unsigned char>::format(),
        3,
        { img.rows, img.cols, 3 },
        { sizeof(unsigned char) * 3 * img.cols, sizeof(unsigned char) * 3, sizeof(unsigned char) }
      );
    })
    .def_static("zeros", [](int rows, int cols) -> cv::Mat3b { return cv::Mat3b::zeros(rows, cols); }, "")
  ;

  py::class_<System>(m, "System")
    .def(py::init<std::string, std::string, System::eSensor, bool>())
    .def("track_monocular", [](System& self, const cv::Mat3b& im, float timestamp) -> cv::Mat1d {
      return self.TrackMonocular(im, timestamp);
    }, "")
    .def("track_rgbd", [](System& self, const cv::Mat3b& im, const cv::Mat1f& depthmap, float timestamp) -> cv::Mat1d {
      return self.TrackRGBD(im, depthmap, timestamp);
    }, "")
  ;

  // ENUMERATIONS

  py::enum_<System::eSensor>(m, "ESensor")
    .value("MONOCULAR", System::MONOCULAR)
    .value("STEREO", System::STEREO)
    .value("RGBD", System::RGBD)
    .export_values()
  ;

#if 0
  // ENUMERATIONS

  py::enum_<ITMMainEngine::GetImageType>(m, "GetImageType")
    .value("IMAGE_ORIGINAL_RGB", ITMMainEngine::GetImageType::InfiniTAM_IMAGE_ORIGINAL_RGB)
    .value("IMAGE_COLOUR_FROM_VOLUME", ITMMainEngine::GetImageType::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME)
    // TODO
    .export_values()
  ;
#endif
#if 0
  //#################### InputSource ####################

  // CLASSES

  py::class_<ImageSourceEngine>(m, "ImageSourceEngine")
    .def("get_calib", &ImageSourceEngine::getCalib)
    .def("get_depth_image_size", &ImageSourceEngine::getDepthImageSize)
    .def("get_images", &ImageSourceEngine::getImages)
    .def("get_rgb_image_size", &ImageSourceEngine::getRGBImageSize)
    .def("has_images_now", &ImageSourceEngine::hasImagesNow)
    .def("has_more_images", &ImageSourceEngine::hasMoreImages)
  ;

  py::class_<ImageMaskPathGenerator>(m, "ImageMaskPathGenerator")
    .def(py::init<const char*,const char*>())
    .def("get_rgb_image_path", &ImageMaskPathGenerator::getRgbImagePath)
    .def("get_depth_image_path", &ImageMaskPathGenerator::getDepthImagePath)
  ;

  py::class_<ImageFileReader<ImageMaskPathGenerator>, ImageSourceEngine>(m, "ImageMaskFileReader")
    .def(py::init<const char*,const ImageMaskPathGenerator&,size_t>())
  ;

  //#################### ITMLib ####################

  // FUNCTIONS

  m.def("read_rgbd_calib", (bool(*)(const char*,ITMRGBDCalib&))&ITMLib::readRGBDCalib, "");

  // ENUMERATIONS

  py::enum_<ITMMainEngine::GetImageType>(m, "GetImageType")
    .value("IMAGE_ORIGINAL_RGB", ITMMainEngine::GetImageType::InfiniTAM_IMAGE_ORIGINAL_RGB)
    .value("IMAGE_COLOUR_FROM_VOLUME", ITMMainEngine::GetImageType::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME)
    // TODO
    .export_values()
  ;

  py::enum_<ITMTrackingState::TrackingResult>(m, "TrackingResult")
    .value("TRACKING_GOOD", ITMTrackingState::TRACKING_GOOD)
    .value("TRACKING_POOR", ITMTrackingState::TRACKING_POOR)
    .value("TRACKING_FAILED", ITMTrackingState::TRACKING_FAILED)
    .export_values()
  ;

  // CLASSES
  py::class_<ITMBasicEngine<ITMVoxel,ITMVoxelIndex> >(m, "ITMBasicEngine")
    .def(py::init<const ITMLibSettings*,const ITMRGBDCalib&,Vector2i,Vector2i>())
    .def("get_view", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::GetView, py::return_value_policy::reference)
    .def("get_tracking_state", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::GetTrackingState, py::return_value_policy::reference)
    .def("get_scene", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::GetScene, py::return_value_policy::reference)
    .def("process_frame", (ITMTrackingState::TrackingResult(ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::*)(ORUChar4Image*,ORShortImage*))&ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::ProcessFrame)
    .def("save_scene_to_mesh", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::SaveSceneToMesh)
    .def("save_to_file", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::SaveToFile)
    .def("load_from_file", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::LoadFromFile)
    .def("get_image_size", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::GetImageSize)
    .def("get_image", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::GetImage)
    .def("turn_on_tracking", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::turnOnTracking)
    .def("turn_off_tracking", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::turnOffTracking)
    .def("turn_on_integration", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::turnOnIntegration)
    .def("turn_off_integration", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::turnOffIntegration)
    .def("turn_on_main_processing", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::turnOnMainProcessing)
    .def("turn_off_main_processing", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::turnOffMainProcessing)
    .def("reset_all", &ITMBasicEngine<ITMVoxel,ITMVoxelIndex>::resetAll)
  ;

  py::class_<ITMDenseMapper<ITMVoxel,ITMVoxelIndex> >(m, "ITMDenseMapper")
    .def(py::init<const ITMLibSettings*>())
    .def("reset_scene", &ITMDenseMapper<ITMVoxel,ITMVoxelIndex>::ResetScene)
    .def("process_frame", &ITMDenseMapper<ITMVoxel,ITMVoxelIndex>::ProcessFrame)
    .def("update_visible_list", &ITMDenseMapper<ITMVoxel,ITMVoxelIndex>::UpdateVisibleList)
  ;

  py::class_<ITMDisparityCalib>(m, "ITMDisparityCalib")
  ;

  py::class_<ITMExtrinsics>(m, "ITMExtrinsics")
  ;

  py::class_<ITMIntrinsics>(m, "ITMIntrinsics")
    // TODO: Image size
    .def_readwrite("projection_params_simple", &ITMIntrinsics::projectionParamsSimple)
  ;

  py::class_<ITMIntrinsics::ProjectionParamsSimple>(m, "ProjectionParamsSimple")
    .def_readonly("fx", &ITMIntrinsics::ProjectionParamsSimple::fx)
    .def_readonly("fy", &ITMIntrinsics::ProjectionParamsSimple::fy)
    .def_readonly("px", &ITMIntrinsics::ProjectionParamsSimple::px)
    .def_readonly("py", &ITMIntrinsics::ProjectionParamsSimple::py)
  ;

  py::class_<ITMLibSettings, boost::shared_ptr<ITMLibSettings> >(m, "ITMLibSettings")
    .def(py::init<>())
    // TODO
    .def_readwrite("device_type", &ITMLibSettings::deviceType)
    .def_readwrite("use_approximate_raycast", &ITMLibSettings::useApproximateRaycast)
    // TODO
    .def_readwrite("scene_params", &ITMLibSettings::sceneParams)
    // TODO
    .def("get_memory_type", &ITMLibSettings::GetMemoryType)
  ;

  py::class_<ITMLowLevelEngine>(m, "ITMLowLevelEngine")
    // TODO
  ;

  py::class_<ITMLowLevelEngineFactory>(m, "ITMLowLevelEngineFactory")
    .def_static("make_low_level_engine", &ITMLowLevelEngineFactory::MakeLowLevelEngine)
  ;

  py::class_<ITMRenderState, boost::shared_ptr<ITMRenderState> >(m, "ITMRenderState")
    .def_readwrite("no_fwd_proj_missing_points", &ITMRenderState::noFwdProjMissingPoints)
  ;

  py::class_<ITMRenderStateFactory<ITMVoxelIndex> >(m, "ITMRenderStateFactory")
    .def_static("create_render_state", &ITMRenderStateFactory<ITMVoxelIndex>::CreateRenderState)
  ;

  py::class_<ITMRGBDCalib>(m, "ITMRGBDCalib")
    .def(py::init<>())
    .def_readwrite("intrinsics_rgb", &ITMRGBDCalib::intrinsics_rgb)
    .def_readwrite("intrinsics_d", &ITMRGBDCalib::intrinsics_d)
    .def_readwrite("trafo_rgb_to_depth", &ITMRGBDCalib::trafo_rgb_to_depth)
    .def_readwrite("disparity_calib", &ITMRGBDCalib::disparityCalib)
  ;

  py::class_<ITMScene<ITMVoxel,ITMVoxelIndex>, boost::shared_ptr<ITMScene<ITMVoxel,ITMVoxelIndex> > >(m, "ITMScene")
    .def(py::init<const ITMSceneParams*, bool, MemoryDeviceType>())
    .def_readonly("scene_params", &ITMScene<ITMVoxel,ITMVoxelIndex>::sceneParams)
  ;

  py::class_<ITMSceneParams>(m, "ITMSceneParams")
    .def(py::init<>())
    .def(py::init<float, int, float, float, float, bool>())
    .def_readwrite("voxel_size", &ITMSceneParams::voxelSize)
    .def_readwrite("view_frustum_min", &ITMSceneParams::viewFrustum_min)
    .def_readwrite("view_frustum_max", &ITMSceneParams::viewFrustum_max)
    .def_readwrite("mu", &ITMSceneParams::mu)
    .def_readwrite("max_w", &ITMSceneParams::maxW)
    .def_readwrite("stop_integrating_at_max_w", &ITMSceneParams::stopIntegratingAtMaxW)
  ;

  py::class_<ITMTrackingState>(m, "ITMTrackingState")
    .def(py::init<Vector2i, MemoryDeviceType>())
    .def_readwrite("pose_d", &ITMTrackingState::pose_d)
  ;

  py::class_<ITMView>(m, "ITMView")
    .def(py::init<ITMRGBDCalib, Vector2i, Vector2i, bool>())
    .def_readwrite("rgb", &ITMView::rgb)
    .def_readwrite("depth", &ITMView::depth)
  ;

  py::class_<ITMViewBuilder>(m, "ITMViewBuilder")
  ;

  py::class_<ITMViewBuilderFactory>(m, "ITMViewBuilderFactory")
    .def_static("make_view_builder", &ITMViewBuilderFactory::MakeViewBuilder)
  ;

  py::class_<ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>, boost::shared_ptr<ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex> > >(m, "ITMVisualisationEngine")
    // TODO
  ;

  py::class_<ITMVisualisationEngineFactory>(m, "ITMVisualisationEngineFactory")
    .def_static("make_visualisation_engine", &ITMVisualisationEngineFactory::MakeVisualisationEngine<ITMVoxel,ITMVoxelIndex>)
  ;

  //#################### ORUtils ####################

  // FUNCTIONS

  m.def("read_image_from_file", (bool(*)(ORUChar4Image*,const char*))&ReadImageFromFile, "");

  // ENUMERATIONS

  py::enum_<DeviceType>(m, "DeviceType")
    .value("DEVICE_CPU", DeviceType::DEVICE_CPU)
    .value("DEVICE_CUDA", DeviceType::DEVICE_CUDA)
    .export_values()
  ;

  py::enum_<MemoryDeviceType>(m, "MemoryDeviceType")
    .value("MEMORYDEVICE_CPU", MemoryDeviceType::MEMORYDEVICE_CPU)
    .value("MEMORYDEVICE_CUDA", MemoryDeviceType::MEMORYDEVICE_CUDA)
    .export_values()
  ;

  // CLASSES

  py::class_<Matrix4f>(m, "Matrix4f", pybind11::buffer_protocol())
    .def(py::init<>())
    .def(py::init<float>())
    .def("at", (float& (Matrix4f::*)(int, int))&Matrix4f::at)
    .def_buffer([](Matrix4f& m) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        m.m, sizeof(float), pybind11::format_descriptor<float>::format(), 2, { 4, 4 }, { sizeof(float) * 4, sizeof(float) }
      );
    })
  ;

  py::class_<ORFloatImage, ORFloatImage_Ptr>(m, "ORFloatImage", pybind11::buffer_protocol())
    .def(py::init<Vector2i, bool, bool, bool>())
    .def_readonly("no_dims", &ORFloatImage::noDims)
    .def_buffer([](ORFloatImage& img) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        img.GetData(MEMORYDEVICE_CPU),
        sizeof(float),
        pybind11::format_descriptor<float>::format(),
        2,
        { img.noDims.y, img.noDims.x },
        { sizeof(float) * img.noDims.x, sizeof(float) }
      );
    })
  ;

  py::class_<ORShortImage>(m, "ORShortImage", pybind11::buffer_protocol())
    .def(py::init<Vector2i, bool, bool, bool>())
    .def_readonly("no_dims", &ORShortImage::noDims)
    .def_buffer([](ORShortImage& img) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        img.GetData(MEMORYDEVICE_CPU),
        sizeof(short),
        pybind11::format_descriptor<short>::format(),
        2,
        { img.noDims.y, img.noDims.x },
        { sizeof(short) * img.noDims.x, sizeof(short) }
      );
    })
  ;

  py::class_<ORUChar4Image>(m, "ORUChar4Image", pybind11::buffer_protocol())
    .def(py::init<Vector2i, bool, bool, bool>())
    .def_readonly("no_dims", &ORUChar4Image::noDims)
    .def_buffer([](ORUChar4Image& img) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        (unsigned char*)img.GetData(MEMORYDEVICE_CPU),
        sizeof(unsigned char),
        pybind11::format_descriptor<unsigned char>::format(),
        3,
        { img.noDims.y, img.noDims.x, 4 },
        { sizeof(Vector4u) * img.noDims.x, sizeof(Vector4u), sizeof(unsigned char) }
      );
    })
  ;

  py::class_<SE3Pose>(m, "SE3Pose")
    .def(py::init<Matrix4f>())
    .def("get_inv_m", &SE3Pose::GetInvM)
    .def("get_m", &SE3Pose::GetM)
    .def("get_t", &SE3Pose::GetT)
    .def("set_t", &SE3Pose::SetT)
  ;

  py::class_<Vector2i>(m, "Vector2i", pybind11::buffer_protocol())
    .def(py::init<int, int>())
    .def_readwrite("x", &Vector2i::x)
    .def_readwrite("y", &Vector2i::y)
    .def_buffer([](Vector2i& v) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        v.v, sizeof(int), pybind11::format_descriptor<int>::format(), 1, { 2 }, { sizeof(int) }
      );
    })
  ;

  py::class_<Vector3f>(m, "Vector3f", pybind11::buffer_protocol())
    .def(py::init<float, float, float>())
    .def_readwrite("x", &Vector3f::x)
    .def_readwrite("y", &Vector3f::y)
    .def_readwrite("z", &Vector3f::z)
    .def_buffer([](Vector3f& v) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        v.v, sizeof(float), pybind11::format_descriptor<float>::format(), 1, { 3 }, { sizeof(float) }
      );
    })
  ;

  py::class_<Vector4u>(m, "Vector4u", pybind11::buffer_protocol())
    .def(py::init<unsigned char, unsigned char, unsigned char, unsigned char>())
    .def_readwrite("x", &Vector4u::x)
    .def_readwrite("y", &Vector4u::y)
    .def_readwrite("z", &Vector4u::z)
    .def_readwrite("w", &Vector4u::w)
    .def_buffer([](Vector4u& v) -> pybind11::buffer_info {
      return pybind11::buffer_info(
        v.v, sizeof(unsigned char), pybind11::format_descriptor<unsigned char>::format(), 1, { 4 }, { sizeof(unsigned char) }
      );
    })
  ;

  //#################### tvgutil ####################

  // CLASSES

  py::class_<SettingsContainer, SettingsContainer_Ptr>(m, "SettingsContainer")
    .def(py::init<>())
    // TODO
  ;

  //#################### orx ####################

  // CLASSES

  py::class_<DualNumberd>(m, "DualNumberd")
    .def(py::init<double, double>())
    .def_readwrite("r", &DualNumberd::r)
    .def_readwrite("d", &DualNumberd::d)
    .def(py::self += py::self)
    .def(py::self -= py::self)
    .def(py::self *= py::self)
    .def("conjugate", &DualNumberd::conjugate)
    .def("inverse", &DualNumberd::inverse)
    .def("is_pure", &DualNumberd::is_pure)
    .def("sqrt", &DualNumberd::sqrt)
    .def(py::self + py::self)
    .def(py::self - py::self)
    .def(py::self * py::self)
    .def(-py::self)
    .def("__repr__", [](const DualNumberd& rhs) { return boost::lexical_cast<std::string>(rhs); })
  ;

  //#################### itmx ####################

  // ENUMERATIONS

  py::enum_<DepthVisualiser::DepthType>(m, "DepthType")
    .value("DT_EUCLIDEAN", DepthVisualiser::DT_EUCLIDEAN)
    .value("DT_ORTHOGRAPHIC", DepthVisualiser::DT_ORTHOGRAPHIC)
    .export_values()
  ;

  // CLASSES

  py::class_<DepthVisualiser, boost::shared_ptr<DepthVisualiser> >(m, "DepthVisualiser")
    // TODO
  ;

  py::class_<DepthVisualisationUtil<ITMVoxel,ITMVoxelIndex> >(m, "DepthVisualisationUtil")
    .def_static("generate_depth_from_voxels", &DepthVisualisationUtil<ITMVoxel,ITMVoxelIndex>::generate_depth_from_voxels_py)
  ;

  py::class_<DepthVisualiserFactory>(m, "DepthVisualiserFactory")
    .def_static("make_depth_visualiser", &DepthVisualiserFactory::make_depth_visualiser)
  ;

  py::class_<Settings, Settings_Ptr, ITMLibSettings, SettingsContainer>(m, "Settings")
    .def(py::init<>())
    // TODO
  ;
#endif
}
