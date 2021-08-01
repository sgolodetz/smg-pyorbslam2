#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
namespace py = pybind11;

#include <Python.h>

#pragma warning(disable:4244 4267 4996)
#include "System.h"
#pragma warning(default:4244 4267 4996)
using namespace ORB_SLAM2;

PYBIND11_MODULE(pyorbslam2, m)
{
  // ENUMERATIONS

  py::enum_<DBoW2::ScoringType>(m, "EScoringType")
    .value("L1_NORM", DBoW2::L1_NORM)
    .value("L2_NORM", DBoW2::L2_NORM)
    .value("CHI_SQUARE", DBoW2::CHI_SQUARE)
    .value("KL", DBoW2::KL)
    .value("BHATTACHARYYA", DBoW2::BHATTACHARYYA)
    .value("DOT_PRODUCT", DBoW2::DOT_PRODUCT)
    .export_values()
  ;

  py::enum_<System::eSensor>(m, "ESensor")
    .value("MONOCULAR", System::MONOCULAR)
    .value("STEREO", System::STEREO)
    .value("RGBD", System::RGBD)
    .export_values()
  ;

  py::enum_<DBoW2::WeightingType>(m, "EWeightingType")
    .value("TF_IDF", DBoW2::TF_IDF)
    .value("TF", DBoW2::TF)
    .value("IDF", DBoW2::IDF)
    .value("BINARY", DBoW2::BINARY)
    .export_values()
  ;

  // CLASSES

  py::class_<ORBVocabulary>(m, "ORBVocabulary")
    .def(
      py::init<int, int, DBoW2::WeightingType, DBoW2::ScoringType>(),
      py::arg("k") = 10, py::arg("ell") = 5, py::arg("weighting") = DBoW2::TF_IDF, py::arg("scoring") = DBoW2::L1_NORM,
      py::call_guard<py::gil_scoped_release>()
    )
    .def("load_from_binary_file", &ORBVocabulary::loadFromBinaryFile, py::call_guard<py::gil_scoped_release>())
    .def("load_from_text_file", &ORBVocabulary::loadFromTextFile, py::call_guard<py::gil_scoped_release>())
    .def("save_to_binary_file", &ORBVocabulary::saveToBinaryFile, py::call_guard<py::gil_scoped_release>())
    .def("save_to_text_file", &ORBVocabulary::saveToTextFile, py::call_guard<py::gil_scoped_release>())
  ;

  py::class_<System>(m, "System")
    .def(py::init<std::string, std::string, System::eSensor, bool>(), py::call_guard<py::gil_scoped_release>())
    .def("shutdown", &System::Shutdown, py::call_guard<py::gil_scoped_release>())
    .def("track_monocular", [](System& self, const cv::Mat3b& im, float timestamp) -> cv::Mat1d {
      return self.TrackMonocular(im, timestamp);
    }, py::call_guard<py::gil_scoped_release>())
    .def("track_rgbd", [](System& self, const cv::Mat3b& im, const cv::Mat1f& depthmap, float timestamp) -> cv::Mat1d {
      return self.TrackRGBD(im, depthmap, timestamp);
    }, py::call_guard<py::gil_scoped_release>())
  ;
}
