#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <string>
#include <X11/Xlib.h>

int main(int argc, char** argv)
{
    if (!XInitThreads()) {
        std::cerr << "Failed to initialize Xlib support for multithreading." << std::endl;
        return -1;
    }
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input.ply>" << std::endl;
        return -1;
    }

    std::string filename(argv[1]);

    // Load input point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
    pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) != 0)
    {
        std::cerr << "Error loading point cloud file: " << filename << std::endl;
        return -1;
    }

    // Calculate normals
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // Configure region growing
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    // Set the angle threshold to 5 degrees (the smaller, the more stringent the smoothness criterion)
    reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    // Set normal cloud
    reg.setInputNormals(normals);

    // Perform region growing segmentation
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    // Visualize the results
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer("Cluster Viewer");
    
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

std::vector<bool> point_classified(colored_cloud->points.size(), false);

// Mark all points that are classified
for (const pcl::PointIndices &cluster : clusters) {
    for (int idx : cluster.indices) {
        point_classified[idx] = true;
    }
}

// Copy only classified points to the new cloud
for (size_t i = 0; i < colored_cloud->points.size(); ++i) {
    if (point_classified[i]) {
        filtered_cloud->points.push_back(colored_cloud->points[i]);
    }
}
filtered_cloud->width = filtered_cloud->points.size();
filtered_cloud->height = 1;
filtered_cloud->is_dense = true;

viewer.showCloud(filtered_cloud);

    while (!viewer.wasStopped())
    {
        // Display the visualizer until 'q' key is pressed
    }

    return 0;
}
