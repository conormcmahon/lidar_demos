
#include <lidar_demos/lidar_demo.h>

#include <fstream>



void load_csv_network(std::string filename, std::vector<StreamPoint>& points, bool strip_header)
{
    points.clear();

    std::cout << "Loading file with name " << filename << std::endl;

    // Open CSV file containing stream network
    std::ifstream network_file(filename);
    if(!network_file.is_open())
        throw std::runtime_error("Failed to open network CSV file.");
    // Build Stream Points from file
    // Get column names
    std::string line, word;
    if(network_file.good())
    {
        // Optionally remove first line (headers)
        if(strip_header)
            std::getline(network_file, line);
    }
    else 
        std::runtime_error("Something is wrong with CSV input file.");
    // Extract stream data
    while(std::getline(network_file, line))
    {
        StreamPoint point;
        // Current line
        std::stringstream ss(line);
        // First column - Latitude
        std::getline(ss, word, ',');
        point.coords.y = std::stof(word);
        // Second column - longitude
        std::getline(ss, word, ',');
        point.coords.x = std::stof(word);
        // Third column - junction ID
        std::getline(ss, word, ',');
        point.junction_id = std::stof(word);
        // Fourth column - stream order  
        std::getline(ss, word, ',');
        point.order = std::stof(word);
        points.push_back(point);
    }
    network_file.close();
}

int main(int argc, char *argv[])
{
    std::string stream_filename = argv[1];
    std::string dem_filename = argv[2];
    std::string veg_filename = argv[3];
    




    // ------ Load CSV Data ------
    std::vector<StreamPoint> points;
    load_csv_network(stream_filename, points, true);



    // ------ Build Network Cloud ------
    // The input Channel Network CSV from LSDTopoTools is only output in EPSG:4326 lat/lon coordinates
    // We need to transform to a custom projection in ft for California to match our point clouds
    PJ_CONTEXT *C;
    PJ *P;
    PJ* P_for_GIS;
    PJ_COORD point_init, point_proj;
    C = proj_context_create();
    P = proj_create_crs_to_crs (C,
                                "EPSG:4326",
                                "+proj=lcc +lat_0=32.1666666666667 +lon_0=-116.25 +lat_1=32.7833333333333 +lat_2=33.8833333333333 +x_0=2000000 +y_0=500000.000000001 +ellps=GRS80 +towgs84=0.9956,-1.9013,-0.5215,-0.025915,-0.009426,-0.011599,0.00062 +units=us-ft +no_defs", 
                                NULL);
    pcl::PointCloud<pcl::PointXYZI>::Ptr streams(new pcl::PointCloud<pcl::PointXYZI>);
    for(std::size_t i=0; i<points.size(); i++)
    {
        // NOTE for EPSG-defined projections, the order is latitude, longitude
        // initial point
        point_init = proj_coord(points[i].coords.y, points[i].coords.x, 0, 0);
        // reprojected point in ESRI:103243
        point_proj = proj_trans(P, PJ_FWD, point_init);

        // NOTE for proj string projections, the order is longitude, latitude - DIFFERENT from the above
        pcl::PointXYZI point;
        point.x = point_proj.enu.e;        // easting
        point.y = point_proj.enu.n;        // northing
        point.intensity = points[i].order; // stream order
        // height (point.z) is left at default value here, to be set below based on DEM interpolation 
        streams->points.push_back(point); 
    }
    proj_destroy(P);
    proj_context_destroy(C);

    // ------- Load DEM -------
    pcl::PointCloud<pcl::PointXYZ>::Ptr dem(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (dem_filename, *dem) == -1) //* load the file
    {
        std::cout << "\nCouldn't read file " << dem_filename << std::endl;
        return (-1);
    }
    std::cout << "Loaded " << dem->width * dem->height << " data points from " << dem_filename << std::endl;
    
    // ------- Load Vegetation -------
    pcl::PointCloud<pcl::PointVeg>::Ptr veg(new pcl::PointCloud<pcl::PointVeg>);
    if (pcl::io::loadPCDFile<pcl::PointVeg> (veg_filename, *veg) == -1) //* load the file
    {
        std::cout << "\nCouldn't read file " << veg_filename << std::endl;
        return (-1);
    }
    std::cout << "Loaded " << veg->width * veg->height << " data points from " << veg_filename << std::endl;



    // ------- Populate Heights in Stream -------
    // First, convert input cloud to one for which we can do a 2D search tree
    pcl::PointCloud<pcl::Point2DElevation>::Ptr flat_dem(new pcl::PointCloud<pcl::Point2DElevation>);
    pcl::copyPointCloud2D(dem, flat_dem);
    pcl::KdTreeFLANN<pcl::Point2DElevation> tree;
    tree.setInputCloud(flat_dem);
    std::vector<int> nearest_indices;
    std::vector<float> nearest_dists;
    for(std::size_t i=0; i<streams->points.size(); i++)
    {
        pcl::Point2DElevation point;
        point.x = streams->points[i].x;
        point.y = streams->points[i].y;
        tree.nearestKSearch(point, 3, nearest_indices, nearest_dists);

        // Check if nearest neighbor point is exactly here
        if(nearest_dists[0] == 0)
        {
            streams->points[i].z = dem->points[nearest_indices[0]].z;
            continue;
        }

        // Get Inverse Distance-weighted Height
        float inverse_distance;
        float height;
        height = dem->points[nearest_indices[0]].z / nearest_dists[0] + 
                 dem->points[nearest_indices[1]].z / nearest_dists[1] +
                 dem->points[nearest_indices[2]].z / nearest_dists[2];
        inverse_distance = 1 / nearest_dists[0] + 
                           1 / nearest_dists[1] + 
                           1 / nearest_dists[2];

        streams->points[i].z = height / inverse_distance;    
    }

    // ------ Output Streams Cloud ------
    pcl::PCDWriter writer;
    streams->width = 1;
    streams->height = streams->points.size();
    writer.write<pcl::PointXYZI>("/mnt/c/Users/conor/Documents/laser_data/las.pcd", *streams, false);

    // ------ Veg-Stream Distances ------
    // First, generate a subset of streams cloud for only large streams (order 3+)
    int min_order = 3;
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (streams);
    pass.setFilterFieldName ("intensity");
    pass.setFilterLimits (min_order-0.01, 10.0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr streams_large(new pcl::PointCloud<pcl::PointXYZI>);
    pass.filter (*streams_large);
    std::cout << "Filtered streams to keep only those larger than " << min_order << ". Initially, " << streams->points.size() << " points, but now only " << streams_large->points.size() << std::endl;
    // Create 2D search versions of the streams clouds
    pcl::PointCloud<pcl::Point2DElevation>::Ptr streams_flat(new pcl::PointCloud<pcl::Point2DElevation>);
    pcl::PointCloud<pcl::Point2DElevation>::Ptr streams_large_flat(new pcl::PointCloud<pcl::Point2DElevation>);
    pcl::copyPointCloud2D(streams, streams_flat);
    pcl::copyPointCloud2D(streams_large, streams_large_flat);
    pcl::KdTreeFLANN<pcl::Point2DElevation> tree_streams;
    tree_streams.setInputCloud(streams_flat);
    pcl::KdTreeFLANN<pcl::Point2DElevation> tree_streams_large;
    tree_streams_large.setInputCloud(streams_large_flat);
    pcl::PointCloud<pcl::PointVeg>::Ptr veg_streamed(new pcl::PointCloud<pcl::PointVeg>);
    pcl::PointCloud<pcl::PointVeg>::Ptr veg_streamed_large(new pcl::PointCloud<pcl::PointVeg>);
    for(std::size_t i=0; i<veg->points.size(); i++)
    {
        veg_streamed->points.push_back(veg->points[i]);
        veg_streamed_large->points.push_back(veg->points[i]);
        // Flat version of target vegetation point
        pcl::Point2DElevation point; 
        point.x = veg->points[i].x; 
        point.y = veg->points[i].y;
        // Get nearest stream point
        tree_streams.nearestKSearch(point, 1, nearest_indices, nearest_dists);
        veg_streamed->points[i].intensity = veg_streamed->points[i].z - streams->points[nearest_indices[0]].z - veg_streamed->points[i].height;
        if(veg_streamed->points[i].intensity < 0)
            veg_streamed->points[i].intensity = 0;
        veg_streamed->points[i].classification = sqrt(nearest_dists[0]);
        veg_streamed->points[i].roughness = points[nearest_indices[0]].order;
        // Get nearest stream point of order > 2
        tree_streams_large.nearestKSearch(point, 1, nearest_indices, nearest_dists);
        veg_streamed_large->points[i].intensity = veg_streamed_large->points[i].z - streams_large->points[nearest_indices[0]].z - veg_streamed_large->points[i].height;
        if(veg_streamed_large->points[i].intensity < 0)
            veg_streamed_large->points[i].intensity = 0;
        veg_streamed_large->points[i].classification = sqrt(nearest_dists[0]);
        //veg_streamed_large->points[i].classification = streams_large->points[nearest_indices[0]].order;
    }

    // ------ Output Vegetation Files ------
    veg_streamed->width = 1;
    veg_streamed->height = veg_streamed->points.size();
    veg_streamed_large->width = 1;
    veg_streamed_large->height = veg_streamed_large->points.size();
    writer.write<pcl::PointVeg>("/mnt/c/Users/conor/Documents/laser_data/veg_streamed.pcd", *veg_streamed, false);
    writer.write<pcl::PointVeg>("/mnt/c/Users/conor/Documents/laser_data/veg_streamed_large.pcd", *veg_streamed_large, false);
    
    return 1;
}