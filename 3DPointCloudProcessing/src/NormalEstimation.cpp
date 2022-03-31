#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // 입력 포인트 클라우드 저장할 오브젝트 
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>); // 계산된 Normal을 저장할 오브젝트 
  pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

  pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud);
  std::cout << "INPUT " << cloud->points.size ()  << std::endl;
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);  
  ne.setRadiusSearch (0.03); // Use all neighbors in a sphere of radius 3cm  
                             // setKSearch() 변경 가능 
  ne.compute (*cloud_normals); // Compute the features

  // 포인트수 출력  
  std::cout << "NORMAL " << cloud_normals->points.size ()  << std::endl;

  // Copy the point cloud data
  pcl::concatenateFields (*cloud, *cloud_normals, p_n_cloud_c);
  pcl::io::savePCDFile<pcl::PointNormal>("p_n_cloud_c.pcd", p_n_cloud_c);

  std::cerr << "Cloud C: " << std::endl;
  for (std::size_t i = 0; i < 5; ++i)
    std::cerr << "    " <<
      p_n_cloud_c[i].x << " " << p_n_cloud_c[i].y << " " << p_n_cloud_c[i].z << " " <<
      p_n_cloud_c[i].normal[0] << " " << p_n_cloud_c[i].normal[1] << " " << p_n_cloud_c[i].normal[2] << std::endl;



  // Visualize them.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	// Display one normal out of 20, as a line of length 3cm.
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 20, 0.03, "normals");
	while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}