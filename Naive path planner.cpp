#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>

// Open CV:
#include <opencv2/opencv.hpp> 
#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// DOSL
#include <dosl/encapsulations/cvMulticlassPathPlanner.tcc>
#include <dosl/aux-utils/string_utils.hpp>

int main(){

  //load image, downsample, define start and end points
  cv::Mat obs_map = cv::imread("obstacle_map_unsigned_8bit_1channel.tif", CV_LOAD_IMAGE_GRAYSCALE);
  cv::resize(obs_map, obs_map, cv::Size(), 0.125, 0.125);
  cv::Point start(200,200); 
  cv::Point goal(300,200); 
  
  int obsSizeThresh = 40; //define size of obstacles to ignore
  int thresh_level = 32; //define sensitivity to ease of traversal
  bool vis = true; //visualize the search?
   
  int nPaths = 1;
  cv::Mat threshed_map;
  cv::threshold(obs_map, threshed_map, thresh_level, 255, CV_THRESH_BINARY);
  cvMulticlassPathPlanner<AStar> robotPathSearchProblem (threshed_map, start, goal, nPaths, vis, obsSizeThresh);
  robotPathSearchProblem.find_paths();
  
  threshed_map = robotPathSearchProblem.draw_paths({},2);
  cv::imshow("Final Path", threshed_map);
  
  //robotPathSearchProblem.paths[0] is our final answer
  
  for (int a=robotPathSearchProblem.paths[0].size()-1; a>=0; --a) {
        std::cout << "[" << robotPathSearchProblem.paths[0][a].x << ", " << robotPathSearchProblem.paths[0][a].y << "]";
        if (a>0) std::cout << "; ";
        else std::cout << "]\n\n";
    }
  
  cv::waitKey(0);
  
}
