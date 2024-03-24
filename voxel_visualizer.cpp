#include <iostream>
#include <thread>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h> // Include the PLY I/O header to handle .ply files
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv) {
    // Check for proper argument usage
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <point_cloud_file.ply>" << std::endl;
        return -1;
    }

    // Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read a PLY file from disk.
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1], *cloud) != 0) {
        return -1;
    }

    // The voxel grid filter will downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    float leaf_size = 0.1;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*cloud_filtered);

    // Visualize both the original cloud and the voxel grid filtered cloud
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    
    // The original cloud is white
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255);
    //viewer->addPointCloud(cloud, cloud_color_handler, "original_cloud");

    // The filtered cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_color_handler(cloud_filtered, 255, 0, 0);
    viewer->addPointCloud(cloud_filtered, cloud_filtered_color_handler, "filtered_cloud");

    // By default, the filtered cloud will be displayed
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered_cloud");

    // Viewer properties
    viewer->addCoordinateSystem(1.0);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer->initCameraParameters();

    // Main loop
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
