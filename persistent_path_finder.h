#ifndef PERSISTENT_PATH_FINDER_H
#define PERSISTENT_PATH_FINDER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// (function description)
// Inputs:
//   start: Starting point
//   end: end point
//   ...
// Output: ...
std::vector<cv::Point> find_maximally_persistence_path(cv::Point start, cv::Point goal, std::string map_filename=std::string("obstacle_map_unsigned_8bit_1channel.tif"), int obsSizeThresh=320, double reduce_factor=1.0/32.0, bool vis=false);

#endif

