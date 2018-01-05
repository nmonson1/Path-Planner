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
  //cv::Mat obs_map = cv::imread("obstacle_map_unsigned_8bit_1channel.tif", CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat obs_map = cv::imread("test2.png", CV_LOAD_IMAGE_GRAYSCALE);
  cv::Point start(2,200); //VT MOD
  cv::Point goal(140,140); //VT MOD
  int obsSizeThresh = 10; //VT MOD
  int granularity = 4; //VT MOD (should be a power of 2)

  //initialize containers for objects used later
  std::vector<std::vector<cv::Point>> edges; //indexed by obstacle
  std::vector<std::vector<cv::Point>> rep_points; //indexed by image layer, then by obstacle
  std::vector<std::vector<std::vector<int>>> path_signatures; //indexed by image layer, then obstacle, then path
  std::vector<std::vector<std::vector<int>>> lifted_signatures; //lifted[i][j][k] is the signature of the kth path in the ith layer around the obstacles in the i-1th layer
  std::vector<cvMulticlassPathPlanner<AStar>> robotPathSearchProblems; //indexed by image layer
  std::vector<std::vector<int>> persistence; //explained below
  cv::Point rep_point;
  //lifted_signatures.push_back(persistence);
  int i; int j; int k; int m; int n;
  int nPaths; bool vis = true; 
  cv::Mat array_of_images [granularity];
    
  //create array of binary images
  for (i=0; (256/granularity)*i < 256; ++i){
    cv::threshold(obs_map, array_of_images[i], 128/(granularity) + ((256/granularity)*i), 255, CV_THRESH_BINARY); //Threshold
    cv::imshow("image"+std::to_string(i), array_of_images[i]);
    std::vector<cv::Point> points = std::vector<cv::Point>(0);
    rep_points.push_back(points);

    //for each binary image, find the (edges of the) obstructions in it
    cv::findContours(array_of_images[i], edges, {}, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    
    //for each obstruction, find a representative pixel
    for (j=0; j < edges.size(); ++j){
      rep_point = cv::boundingRect(edges[j]).tl();
      points.push_back(rep_point);
      //printf("%i \n", (int)edges.size() );
      //printf("%i \n", j);
      do{
        rep_point.x += 1;
        rep_point.y += 1;
      } while (cv::pointPolygonTest(edges[j], rep_point, false) != 1); printf("%i \n %i \n", rep_point.x, rep_points[i][1].x);
    }
    //for each binary image, find the topologically distinct paths and their signatures
    nPaths = std::pow(2.0,(rep_points[i].size()));
    robotPathSearchProblems.push_back(cvMulticlassPathPlanner<AStar> (array_of_images[i], start, goal, nPaths, vis, obsSizeThresh));
    printf("%i \n", (int) rep_points[i].size() );
    robotPathSearchProblems[i].find_paths();
    //printf("test code");
    
    for (k=0; k < nPaths; ++k){
      for (m=0; m < robotPathSearchProblems[i].paths[k].size(); ++m){
        for (j=0; j < rep_points[i].size();j++){
          if ((robotPathSearchProblems[i].paths[k][m].x == rep_points[i][j].x) && (robotPathSearchProblems[i].paths[k][m].y > rep_points[i][j].y)) {
            path_signatures[i][j][k]++;
            path_signatures[i][j][k] = (path_signatures[i][j][k] % 2);
          }
        }  
      }
      if (i != 0){
        for (j=0; j < rep_points[i-1].size(); j++){
          if ((robotPathSearchProblems[i].paths[k][m].x == rep_points[i-1][j].x) && (robotPathSearchProblems[i].paths[k][m].y > rep_points[i-1][j].y)) {
            lifted_signatures[i][j][k]++;
            lifted_signatures[i][j][k] = (lifted_signatures[i][j][k] % 2);
          }
        }
      }
    }
  }
  //compute persistence of paths. At each layer, i, persistence is a vector whose kth element is -1 if the kth path in image i does NOT persist, and is its index in image i+1 if it does.
  for(i=1; (256/granularity)*(i) < 256; ++i){
    int persist [robotPathSearchProblems[i].paths.size()]; //corresponds to CI
    int argcount [robotPathSearchProblems[i-1].paths.size()]; //corresponds to c
    for (k=0; k<robotPathSearchProblems[i].paths.size(); ++k){
      for (m=0; m<robotPathSearchProblems[i-1].paths.size(); ++m){
        n=0;
        for(j=0; j<path_signatures[i-1].size(); j++) {n += (path_signatures[i-1][j][k] - lifted_signatures[i][j][k]);}
        if (n == 0) {
          persist[k] = m;
          ++argcount[m];
          break;
        }
      }
    }
    /*for(m=0; m<robotPathSearchProblems[i-1].paths.size(); ++m){
      if (argcount[m] > 0){
        int K [persist.size()];
        n=0;
        for (k=0; k < K.size(); k++) {if(persist[k] == m) {K[k] = 1; n++;}}
        while (n>1) {
        }
      }
    }*/
  }
  
  //draw paths
  //obs_map = robotPathSearchProblem.draw_paths({}, 2);
  //cv::imshow("Final Paths", obs_map);
  cv::waitKey(0);
}
