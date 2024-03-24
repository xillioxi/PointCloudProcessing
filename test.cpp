#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
    // Create a PointCloud object for PointXYZ points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read the point cloud from a PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file input.pcd \n");
        return -1;
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from input.pcd with the following fields: "
              << std::endl;

    // Write the point cloud to a new PCD file
    pcl::io::savePCDFileASCII("output.pcd", *cloud);
    std::cout << "Saved " << cloud->points.size() << " data points to output.pcd." << std::endl;

    return 0;
}
