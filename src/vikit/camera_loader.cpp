#include "vikit/camera_loader.h"


namespace vk {
namespace camera_loader {

bool LoadFromCamearaConfig(const std::string &config_path, vk::AbstractCamera *&cam) {
  auto yaml = YAML::LoadFile(config_path);
  std::cout << "Camera config path: " << config_path << std::endl;

  bool res = true;
  std::string cam_model(yaml["cam_model"].as<std::string>());

  if (cam_model == "Ocam") {
    cam = new vk::OmniCamera(yaml["cam_calib_file"].as<std::string>());
  } else if (cam_model == "Pinhole") {
    cam = new vk::PinholeCamera(
        yaml["cam_width"].as<double>(),
        yaml["cam_height"].as<double>(),
        yaml["scale"].as<double>(1.0),
        yaml["cam_fx"].as<double>(),
        yaml["cam_fy"].as<double>(),
        yaml["cam_cx"].as<double>(),
        yaml["cam_cy"].as<double>(),
        yaml["cam_d0"].as<double>(0.0),
        yaml["cam_d1"].as<double>(0.0),
        yaml["cam_d2"].as<double>(0.0),
        yaml["cam_d3"].as<double>(0.0),
        yaml["cam_d4"].as<double>(0.0),
        yaml["cam_d5"].as<double>(0.0),
        yaml["cam_d6"].as<double>(0.0),
        yaml["cam_d7"].as<double>(0.0),
        yaml["cam_d8"].as<double>(0.0),
        yaml["cam_d9"].as<double>(0.0),
        yaml["cam_d10"].as<double>(0.0),
        yaml["cam_d11"].as<double>(0.0),
        yaml["cam_d12"].as<double>(0.0),
        yaml["cam_d13"].as<double>(0.0));
  } else if (cam_model == "EquidistantCamera") {
    cam = new vk::EquidistantCamera(
        yaml["cam_width"].as<double>(), yaml["cam_height"].as<double>(),
        yaml["scale"].as<double>(1.0), yaml["cam_fx"].as<double>(),
        yaml["cam_fy"].as<double>(), yaml["cam_cx"].as<double>(),
        yaml["cam_cy"].as<double>(), yaml["k1"].as<double>(0.0),
        yaml["k2"].as<double>(0.0), yaml["k3"].as<double>(0.0),
        yaml["k4"].as<double>(0.0));
  } else if (cam_model == "PolynomialCamera") {
    cam = new vk::PolynomialCamera(
        yaml["cam_width"].as<double>(), yaml["cam_height"].as<double>(),
        // yaml["scale"].as<double>(1.0),
        yaml["cam_fx"].as<double>(), yaml["cam_fy"].as<double>(),
        yaml["cam_cx"].as<double>(), yaml["cam_cy"].as<double>(),
        yaml["cam_skew"].as<double>(), yaml["k2"].as<double>(0.0),
        yaml["k3"].as<double>(0.0), yaml["k4"].as<double>(0.0),
        yaml["k5"].as<double>(0.0), yaml["k6"].as<double>(0.0),
        yaml["k7"].as<double>(0.0));
  } else if (cam_model == "ATAN") {
    cam = new vk::ATANCamera(
        yaml["cam_width"].as<double>(), yaml["cam_height"].as<double>(),
        yaml["cam_fx"].as<double>(), yaml["cam_fy"].as<double>(),
        yaml["cam_cx"].as<double>(), yaml["cam_cy"].as<double>(),
        yaml["cam_d0"].as<double>());
  } else {
    cam = NULL;
    res = false;
  }
  return res;
}

// bool loadFromRosNs(const std::string &ns,
//                    std::vector<vk::AbstractCamera *> &cam_list) {
//   bool res = true;
//   std::string cam_model(getParam<std::string>(ns + "/cam_model"));
//   int cam_num = getParam<int>(ns + "/cam_num");
//   for (int i = 0; i < cam_num; i++) {
//     std::string cam_ns = ns + "/cam_" + std::to_string(i);
//     std::string cam_model(getParam<std::string>(cam_ns + "/cam_model"));
//     if (cam_model == "FishPoly") {
//       cam_list.push_back(new vk::PolynomialCamera(
//           getParam<int>(cam_ns + "/image_width"),
//           getParam<int>(cam_ns + "/image_height"),
//           // getParam<double>(cam_ns+"/scale", 1.0),
//           getParam<double>(cam_ns + "/A11"), // cam_fx
//           getParam<double>(cam_ns + "/A22"), // cam_fy
//           getParam<double>(cam_ns + "/u0"),  // cam_cx
//           getParam<double>(cam_ns + "/v0"),  // cam_cy
//           getParam<double>(cam_ns + "/A12"), // cam_skew
//           getParam<double>(cam_ns + "/k2", 0.0),
//           getParam<double>(cam_ns + "/k3", 0.0),
//           getParam<double>(cam_ns + "/k4", 0.0),
//           getParam<double>(cam_ns + "/k5", 0.0),
//           getParam<double>(cam_ns + "/k6", 0.0),
//           getParam<double>(cam_ns + "/k7", 0.0)));
//     } else if (cam_model == "Pinhole") {
//       cam_list.push_back(new vk::PinholeCamera(
//           getParam<int>(ns + "/cam_width"), getParam<int>(ns + "/cam_height"),
//           getParam<double>(ns + "/scale", 1.0),
//           getParam<double>(ns + "/cam_fx"), getParam<double>(ns + "/cam_fy"),
//           getParam<double>(ns + "/cam_cx"), getParam<double>(ns + "/cam_cy"),
//           getParam<double>(ns + "/cam_d0", 0.0),
//           getParam<double>(ns + "/cam_d1", 0.0),
//           getParam<double>(ns + "/cam_d2", 0.0),
//           getParam<double>(ns + "/cam_d3", 0.0),
//           getParam<double>(ns + "/cam_d4", 0.0),
//           getParam<double>(ns + "/cam_d5", 0.0),
//           getParam<double>(ns + "/cam_d6", 0.0),
//           getParam<double>(ns + "/cam_d7", 0.0),
//           getParam<double>(ns + "/cam_d8", 0.0),
//           getParam<double>(ns + "/cam_d9", 0.0),
//           getParam<double>(ns + "/cam_d10", 0.0),
//           getParam<double>(ns + "/cam_d11", 0.0),
//           getParam<double>(ns + "/cam_d12", 0.0),
//           getParam<double>(ns + "/cam_d13", 0.0)));
//     } else {
//       // cam_list.clear();
//       res = false;
//     }
//   }

//   return res;
// }

} // namespace camera_loader
} // namespace vk