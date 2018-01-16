#include <cstdlib>
#include <string>
#include <iostream>
#include <cmath>

// Open CV:
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// DOSL
#include <dosl/encapsulations/cvMulticlassPathPlanner.tcc>
#include <dosl/aux-utils/string_utils.hpp>

double path_distance(const std::vector< cv::Point >& p1, const std::vector< cv::Point >& p2) {
  // compute path distance
  double err = 0.0;
  for (int ratio = 1; ratio < 10; ++ratio) {
    int idx1 = round((p1.size() * ratio) / 10.0);
    int idx2 = round((p2.size() * ratio) / 10.0);
    err += cv::norm(p1[idx1] - p2[idx2]);
  }
  return err;
}

std::vector<cv::Point> find_maximally_persistent_path(const cv::Point& oldstart, const cv::Point& oldgoal, const cv::Mat& oldmap, const int& oldThresh, const double& reduce_factor, const bool& vis){
  cv::Point start = oldstart;
  cv::Point goal = oldgoal;
  cv::Mat obs_map = oldmap;
  int obsSizeThresh = oldThresh;
  start.x *=reduce_factor;
  start.y *=reduce_factor;
  goal.x *=reduce_factor;
  goal.y *=reduce_factor;
  obsSizeThresh *= reduce_factor;
  cv::resize(obs_map, obs_map, cv::Size(), reduce_factor, reduce_factor);

  //initialize containers for objects used later
  std::vector<std::vector<cv::Point>> rep_points; //indexed by image layer, then by obstacle
  std::vector<std::vector<std::vector<int>>> path_signatures; //indexed by image layer, then obstacle, then path
  std::vector<std::vector<std::vector<int>>> lifted_signatures; //lifted[i][j][k] is the signature of the kth path in the ith layer around the obstacles in the i-1th layer
  std::vector<cvMulticlassPathPlanner<AStar>*> robotPathSearchProblems; //indexed by image layer
  std::vector<std::vector<int>> persistence; //explained below
  persistence.push_back({});
  int i; int j; int k; int m; int n;
  int nPaths; 
  std::vector<cv::Mat> array_of_images (7);

  //create array of binary images
  for (i=0; i < 7; ++i){
    cv::threshold(obs_map, array_of_images[i], std::pow(2,i)-1, 255, CV_THRESH_BINARY); //Threshold

    //for each binary image, find the (edges of the) obstructions in it
    std::vector<cv::Point> points = std::vector<cv::Point> (0);
    std::vector<std::vector<cv::Point>> edges; //indexed by obstacle
    cv::Mat temp = array_of_images[i].clone();
    cv::findContours(temp, edges, {}, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    edges.erase(edges.end());
    std::vector<std::vector<int>> sig1;

    //for each obstruction, find a representative pixel
    for (j=0; j < edges.size(); ++j){
      if (std::max(cv::boundingRect(edges[j]).width, cv::boundingRect(edges[j]).height) > obsSizeThresh){
        cv::Point rep_point = cv::boundingRect(edges[j]).tl();
        std::vector<int> sig2;
        sig1.push_back(sig2);
        while (cv::pointPolygonTest(edges[j], rep_point, false) == -1) {
          rep_point.x += 1;
          rep_point.y += 1;
        }
        points.push_back(rep_point);
      }
    }
    
    rep_points.push_back(points);
    path_signatures.push_back(sig1);
    lifted_signatures.push_back(sig1);
    // std::cout << "There are " << rep_points[i].size() << " obstacles in layer " << i << std::endl;

    //for each binary image, find the topologically distinct paths
    nPaths = std::min(10.0, std::pow(2,(int)rep_points[i].size())-1);
    robotPathSearchProblems.push_back(new cvMulticlassPathPlanner<AStar> (array_of_images[i], start, goal, nPaths, vis, obsSizeThresh));
    robotPathSearchProblems[i]->find_paths();
    
    //and their signatures
    for (k=0; k < nPaths; ++k){
      for (j=0; j < rep_points[i].size();j++){
        path_signatures[i][j].push_back(0);
        for (m=0; m < robotPathSearchProblems[i]->paths[k].size(); ++m){
          if ((robotPathSearchProblems[i]->paths[k][m].x == rep_points[i][j].x) && (robotPathSearchProblems[i]->paths[k][m].y > rep_points[i][j].y)) {
            path_signatures[i][j][k]++;
            path_signatures[i][j][k] = (path_signatures[i][j][k] % 2);
          }
        }
      }
      if (i != 0){
        for (j=0; j < rep_points[i-1].size(); j++){
          lifted_signatures[i][j].push_back(0);
          for (m=0; m < robotPathSearchProblems[i]->paths[k].size(); ++m){
            if ((robotPathSearchProblems[i]->paths[k][m].x == rep_points[i-1][j].x) && (robotPathSearchProblems[i]->paths[k][m].y > rep_points[i-1][j].y)) {
              lifted_signatures[i][j][k]++;
              lifted_signatures[i][j][k] = (lifted_signatures[i][j][k] % 2);
            }
          }
        }
      }
    }
  }
  //compute "persistence" vector
  //At each layer, i, persistence is a vector whose kth element is -1 if the kth path in image i does NOT persist, and is its index in image i-1 if it does.
  for(i=1; i < 7; ++i){
    std::vector<int> pathsPersist(robotPathSearchProblems[i]->paths.size(),-1); //corresponds to CII
    for (m=0; m<robotPathSearchProblems[i-1]->paths.size(); ++m){
      double minDist = DBL_MAX;
      int argMin = -1;
      for (k=0; k<robotPathSearchProblems[i]->paths.size(); ++k){
        n=0;
        for(j=0; j<path_signatures[i-1].size(); ++j) {n += ((path_signatures[i-1][j][m] - lifted_signatures[i][j][k])%2);}
        if (n == 0) {
          double pathDist = path_distance(robotPathSearchProblems[i-1]->paths[m], robotPathSearchProblems[i]->paths[k]);
          if (minDist > pathDist) {
            minDist = pathDist;
            argMin = k;
          }
        }
      }

      if (argMin > -1) {
        pathsPersist[argMin] = m;
      }
    }

    persistence.push_back(pathsPersist);
  }

  // std::cout << "persistence computed" <<std::endl;
  
  //Use "persistence" vector to compute persistence length
  std::vector<std::vector<int>> persistLength(persistence);
  int maxLength=0;
  int maxPathi;
  int maxPathk;
  i=6;
  while(i >= 0){
    for(k=0; k < persistence[i].size() ;k++){
      int currentpathlength = 0;
      int currentindexi = i;
      int currentindexk = k;
      while(persistence[currentindexi][currentindexk] > 0){
        currentpathlength++;
        persistence[currentindexi][currentindexk];
        currentindexi--;
      }
      if(currentpathlength > maxLength){
        maxLength = currentpathlength;
        maxPathi = i;
        maxPathk = k;
      }
    }
    i--;
    if(maxLength > i){break;}
  }

  std::vector<cv::Point> max_path = robotPathSearchProblems[maxPathi]->paths[maxPathk];
  for (int i = 0; i < max_path.size(); ++i) {
    max_path[i].x = round(max_path[i].x / reduce_factor);
    max_path[i].y = round(max_path[i].y / reduce_factor);
  }

  // robotPathSearchProblems[maxPathi]->paths[maxPathk] is a maximally persistent path!
  std::cout << "A maximally persistent path (persistence = " << maxLength << " out of 7) is given by ";
  for (int a=max_path.size()-1; a>=0; --a) {
    std::cout << "[" << max_path[a].x << ", " << max_path[a].y << "]";
    if (a>0) std::cout << "; ";
    else std::cout << "]\n\n";
  }

  // clean up memory
  for (int i = 0; i < robotPathSearchProblems.size(); ++i) {
    delete robotPathSearchProblems[i];
  }

  return max_path;
}

std::vector<cv::Point> call_find_path(cv::Point start, cv::Point goal, std::string mapFilename, int obsSizeThresh, double reduce_factor, bool vis){
  cv::Mat obs_map = cv::imread(mapFilename, CV_LOAD_IMAGE_GRAYSCALE);
  return find_maximally_persistent_path(start, goal, obs_map, obsSizeThresh, reduce_factor, vis);
}

