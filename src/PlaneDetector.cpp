/*
  GEO1015.2020
  hw03 
  --
  Michiel de Jong
  4376978
  [YOUR NAME] 
  [YOUR STUDENT NUMBER] 
*/


#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iterator>
#include <algorithm>

#include "PlaneDetector.h"

/*
!!! TO BE COMPLETED !!!

Function that implements the RANSAC algorithm to detect one plane in the point cloud that is 
accessible from _input_points variable (which contains the point read from the input ply file).

NOTICE that this function can be called multiple times! On each call a *new* plane should be generated
that does not contain any inliers that belong to a previously detected plane. Each plane should 
have a unique segment_id (an int; '1' for the first plane, '2' for the second plane etc.).

Input:
    epsilon:          maximum distance from an inlier to corresponding plane.
    min_score:        minimum score (= number of inlier) of a plane. Planes with a lower score
                      should not be detected.
    k:                number of times a new minimal set and a consencus set is computed.
    
Output:
    Updated .segment_id's on the inliers in the newly detected plane. The inliers contain both the
    consensus set and minimal set.
*/
void PlaneDetector::detect_plane(double epsilon, int min_score, int k) {

  //-- TIP
  //-- access the input points from the _input_points:

  //   double x_of_point_i = _input_points[i].x;
  //   int segment_id_of_point_i = _input_points[i].segment_id;

  //-- set the segment_id of point i:

  //   _input_points[i].segment_id = 1;


  //-- TIP
  //-- Generating random numbers between 0 and 100

  // std::uniform_int_distribution<int> distrib(0, 100);
  // int my_random_number = distrib(_rand);

  //-- see https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution for more info
  int min = min_score;
  int num_iter = k;
  int min_set = 3;
  int plane_no = 1;
  double tolerance = epsilon;
  std::vector<Point> random_set;
  while(random_set.size() < 3){
      std::uniform_int_distribution<int> distrib(0,_input_points.size());
      int my_random_number = distrib(_rand);
      std::cout<< _input_points[my_random_number].x<< std::endl;
      if(_input_points[my_random_number].segment_id == 0){
          random_set.push_back(_input_points[my_random_number]);
          std::cout<< _input_points[my_random_number].x<< std::endl;
      }
  }

  double a = ((random_set[1].y - random_set[0].y) * (random_set[2].z - random_set[0].z))-((random_set[1].z - random_set[0].z) * (random_set[2].y - random_set[1].y));
  double b = ((random_set[1].z - random_set[0].z) * (random_set[2].x - random_set[0].x))-((random_set[1].x - random_set[0].x) * (random_set[2].z - random_set[1].z));
  double c = ((random_set[1].x - random_set[0].x) * (random_set[2].y - random_set[0].y))-((random_set[1].y - random_set[0].y) * (random_set[2].x - random_set[1].x));
  double d = -((a * random_set[0].x) + (b * random_set[0].y) + (c * random_set[0].z));
  std::cout<< a<<std::endl;
  std::cout<< b<<std::endl;
  std::cout<< c<<std::endl;
  std::cout<< d<<std::endl;
    for(auto i = begin(_input_points); i != end(_input_points); ++i) {
        double dist = (a * i->x + b * i->y + c * i->z + d)/ std::sqrt(a*a + b*b + c*c);
        std::cout<< dist << std::endl;
        if(dist <= tolerance){
            i->segment_id = plane_no;
        }
    }
    //for(i begin())
/*
  for(auto i = begin(_input_points); i != end(_input_points); ++i) {
      std::cout<< i->x<<" "<<i->y << std::endl;
  }
*/


}

// PLY I/O

/*
!!! TO BE COMPLETED !!!

Function that writes the entire point cloud including the segment_id of each point to a .ply file

Input:
   filepath:  path of the .ply file to write the points with segment id
*/
void PlaneDetector::write_ply(std::string filepath) {
    auto start = std::chrono::high_resolution_clock::now();
    std::ofstream outfile(filepath.c_str(), std::ofstream::out);
    outfile << "ply" << std::endl;
    outfile << "format ascii 1.0" << std::endl;
    outfile << "element vertex" << " " << _input_points.size() <<std::endl;
    outfile << "property float64 x" << std::endl;
    outfile << "property float64 y" << std::endl;
    outfile << "property float64 z" << std::endl;
    outfile << "property int segment_id" << std::endl;
    outfile << "end_header" << std::endl;
    for (const auto &e : _input_points) outfile <<std::setprecision(10)<< e.x << " " << e.y << " " << e.z << " " << e.segment_id <<  "\n";
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout<<"--- File written to "<< filepath<< " in " << elapsed.count() << " seconds ---";
    outfile.close();
}

/*
!!! DO NOT MODIFY read_ply() !!!

This function is already implemented.
*/
bool PlaneDetector::read_ply(std::string filepath) {

    std::cout << "Reading file: " << filepath << std::endl;
    std::ifstream infile(filepath.c_str(), std::ifstream::in);
    if (!infile)
    {
        std::cerr << "Input file not found.\n";
        return false;
    }
    std::string cursor;

    // start reading the header
    std::getline(infile, cursor);
    if (cursor != "ply") {
        std::cerr << "Magic ply keyword not found\n";
        return false;
    };

    std::getline(infile, cursor);
    if (cursor != "format ascii 1.0") {
        std::cerr << "Incorrect ply format\n";
        return false;
    };

    // read the remainder of the header
    std::string line = "";
    int vertex_count = 0;
    bool expectVertexProp = false, foundNonVertexElement = false;
    int property_count = 0;
    std::vector<std::string> property_names;
    int pos_x = -1, pos_y = -1, pos_z = -1, pos_segment_id = -1;

    while (line != "end_header") {
        std::getline(infile, line);
        std::istringstream linestream(line);

        linestream >> cursor;

        // read vertex element and properties
        if (cursor == "element") {
            linestream >> cursor;
            if (cursor == "vertex") {
                // check if this is the first element defined in the file. If not exit the function
                if (foundNonVertexElement) {
                    std::cerr << "vertex element is not the first element\n";
                    return false;
                };

                linestream >> vertex_count;
                expectVertexProp = true;
            } else {
                foundNonVertexElement = true;
            }
        } else if (expectVertexProp) {
            if (cursor != "property") {
                expectVertexProp = false;
            } else {
                // read property type
                linestream >> cursor;
                if (cursor.find("float") != std::string::npos || cursor == "double") {
                    // read property name
                    linestream >> cursor;
                    if (cursor == "x") {
                        pos_x = property_count;
                    } else if (cursor == "y") {
                        pos_y = property_count;
                    } else if (cursor == "z") {
                        pos_z = property_count;
                    }
                    ++property_count;
                }
                else if (cursor.find("uint") != std::string::npos || cursor == "int") {
                    // read property name
                    linestream >> cursor;
                    if (cursor == "segment_id") {
                        pos_segment_id = property_count;
                    }
                    ++property_count;
                }
            }

        }
    }

    // check if we were able to locate all the coordinate properties
    if ( pos_x == -1 || pos_y == -1 || pos_z == -1) {
        std::cerr << "Unable to locate x, y and z vertex property positions\n";
        return false;
    };

    // read the vertex properties
    for (int vi = 0; vi < vertex_count; ++vi) {
        std::getline(infile, line);
        std::istringstream linestream(line);

        double x{},y{},z{};
        int sid{};
        for (int pi = 0; pi < property_count; ++pi) {
            linestream >> cursor;
            if ( pi == pos_x ) {
                x = std::stod(cursor);
            } else if ( pi == pos_y ) {
                y = std::stod(cursor);
            } else if ( pi == pos_z ) {
                z = std::stod(cursor);
            } else if ( pi == pos_segment_id ) {
                sid = std::stoi(cursor);
            }
        }
        auto p = Point{x, y, z};
        if (pos_segment_id!=-1) {
            p.segment_id = sid;
        }
        _input_points.push_back(p);
    }

    std::cout << "Number of points read from .ply file: " << _input_points.size() << std::endl;

    return true;
}
