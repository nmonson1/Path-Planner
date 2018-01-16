#include "persistent_path_finder.h"

int main(){

  //load image, downsample, define start and end points
  std::string mapFilename = "obstacle_map_unsigned_8bit_1channel.tif"; //VT MOD
  cv::Point start(1485, 2916); //VT MOD
  cv::Point goal(1500, 800); //VT MOD
  int obsSizeThresh = 320; //VT MOD (Define size of obstacles to ignore)
  double reduce_factor = 1.0/32.0; //VT MOD (Downsample map)
  bool vis = true; //VT MOD (Change to true to see progress)

  std::vector<cv::Point> path = call_find_path(start, goal, mapFilename, obsSizeThresh, reduce_factor, vis);

  return 0;
}
