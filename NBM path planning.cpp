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
#include <dosl/aux-utils/string_utils.hpp> // compute_program_path

int main(){

  //load image, set start and end pixels.
  cv::Mat obs_map = cv::imread("obstacle_map_unsigned_8bit_1channel.tif", CV_LOAD_IMAGE_GRAYSCALE);
  //cv::Mat obs_map = cv::imread("test.png", CV_LOAD_IMAGE_GRAYSCALE);
  cv::Point start(2,2); //VT MOD
  cv::Point goal(14,14); //VT MOD

  //initialize containers for objects used later
  std::vector<std::vector<cv::Point>> edges; //indexed by obstacle
  std::vector<std::vector<cv::Point>> rep_points; //indexed by image layer, then by obstacle
  std::vector<std::vector<std::vector<int>>> path_signatures; //indexed by image layer, then obstacle, then path
  std::vector<cvMulticlassPathPlanner<AStar>> robotPathSearchProblems; //indexed by image layer
  std::vector<std::vector<int>> persistence; //explained below
  cv::Point rep_point;
  int i; int j; int k; int m;
  int nPaths; bool vis = true; int obsSizeThresh = 10; //VT MOD
  int granularity = 4; //VT MOD (should be a power of 2)
  
  //create array of binary images
  cv::Mat array_of_images [granularity];
  for (i=0; (256/granularity)*i < 256; ++i){
    cv::threshold(obs_map, array_of_images[i], 256/(granularity*2) + (256/granularity*i), 255, CV_THRESH_BINARY); //Threshold
    //cv::imshow("image"+std::to_string(i), array_of_images[i]);
    std::vector<cv::Point> points = std::vector<cv::Point>(0);
    rep_points.push_back(points);

    //for each binary image, find the (edges of the) obstructions in it
    cv::findContours(array_of_images[i], edges, {}, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    
    //for each obstruction, find a representative pixel
    for (j=0; j < edges.size(); ++j){
      rep_point = cv::boundingRect(edges[j]).tl();
      points.push_back(rep_point);
      //printf("%i \n", (int)edges.size() );
      m=0;
      do{
        rep_point.x += m;
        rep_point.y += m;
        ++m;
      } while (cv::pointPolygonTest(edges[j], rep_point, false) != 1);
    } 
    
    //for each binary image, find the topologically distinct paths and their signatures
    nPaths = 3; //std::pow(2.0,(edges[i].size()));
    robotPathSearchProblems.push_back(cvMulticlassPathPlanner<AStar> (array_of_images[i], start, goal, nPaths, vis, obsSizeThresh));
    robotPathSearchProblems[i].find_paths(); 
    for (k=0; k < nPaths; ++k){
     for (m=0; m < robotPathSearchProblems[i].paths[j].size(); ++m){
        if ((robotPathSearchProblems[i].paths[k][m].x == rep_points[i][j].x) && (robotPathSearchProblems[i].paths[k][m].y > rep_points[i][j].y))
          path_signatures[i][j][k]++;
      }
    }
  }
  
  //compute persistence of paths. At each layer, i, persistence is a vector whose kth element is -1 if the kth path in image i does NOT persist, and is it's index in image i+1 if it does.
  /*for(i=0; 64*(i+1) < 256; ++i){
    int persist [robotPathSearchProblems[i].paths.size()];
    int argcount [robotPathSearchProblems[i+1].paths.size()];
    for (k=0; k<robotPathSearchProblems[i].paths.size(); ++k){
      for (m=0; m<robotPathSearchProblems[i+1].paths.size(); ++m){
        if
      }
  }
  */
  //draw paths
  //obs_map = robotPathSearchProblem.draw_paths({}, 2);
  //cv::imshow("Final Paths", obs_map);
  cv::waitKey(0);
}
