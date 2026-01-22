/*
 * camera_loader.h
 *
 *  Created on: Feb 11, 2014
 *      Author: cforster
 */

#ifndef _VIKIT_CAMERA_LOADER_H_
#define _VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/params_helper.h>
#include <vikit/pinhole_camera.h>
#include <vikit/polynomial_camera.h>
#include <yaml-cpp/yaml.h>

namespace vk {
namespace camera_loader {

bool LoadFromCamearaConfig(const std::string &config_path, vk::AbstractCamera *&cam);

// bool loadFromRosNs(const std::string &ns, std::vector<vk::AbstractCamera *> &cam_list);

} // namespace camera_loader
} // namespace vk

#endif // _VIKIT_CAMERA_LOADER_H_
