#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/random_sample.h>
#include <thread>

int main (int argc, char** argv)
{
  // Check for proper command-line argument (a .ply file)
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <input_ply_file.ply>" << std::endl;
    return -1;
  }

  // Create a Point Cloud pointer for the original and sampled cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Load .ply file
  if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s \n", argv[1]);
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from " << argv[1] << " with the following fields: "
            << pcl::getFieldsList(*cloud) << std::endl;

  // Randomly sample the cloud to 10% of its original size
  pcl::RandomSample<pcl::PointXYZRGB> random_sampler;
  random_sampler.setInputCloud(cloud);
  random_sampler.setSample(static_cast<int>(cloud->points.size() * 0.1)); // 10%
  random_sampler.filter(*cloud_sampled);

  // Create a PCLVisualizer object
  pcl::visualization::PCLVisualizer viewer ("PLY file viewer");
  viewer.setBackgroundColor (0, 0, 0); // Set background to black

  // Add the sampled point cloud to the viewer and pass the color handler
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_sampled);
  viewer.addPointCloud<pcl::PointXYZRGB> (cloud_sampled, rgb, "sampled cloud");

  // Set point size
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sampled cloud");

  // Enable user interface interactor
  viewer.initCameraParameters ();

  // Main loop
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
