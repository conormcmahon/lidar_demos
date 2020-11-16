
#include <iostream>

#include <lidar_raster_stats/point_cloud_rasterizer.h>
#include <dirt_or_leaf/las_classifier.h>
#include <dirt_or_leaf/point_2d_elevation.h>
#include <dirt_or_leaf/las_conversions.h>

//#include <pdal/PointTable.hpp>
#include <proj.h>
#include <stdio.h>

struct StreamPoint
{
    pcl::PointXYZ coords;
    float order;
    int junction_id;
};


void load_csv_network(std::string filename, std::vector<StreamPoint>& points, bool strip_header=true);