#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

int main(int argc, char **argv) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud);

  // 포인트수 출력
  std::cout << "PointCloud before filtering has: " << cloud->points.size()
            << " data points." << std::endl; //*

  // 탐색을 위한 KdTree 오브젝트 생성 //Creating the KdTree object for the
  // search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud); // KdTree 생성

  std::vector<pcl::PointIndices>
      cluster_indices; // 군집화된 결과물의 Index 저장, 다중 군집화 객체는
                       // cluster_indices[0] 순으로 저장
  // 군집화 오브젝트 생성
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setInputCloud(cloud);      // 입력
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(100);    // 최소 포인트 수
  ec.setMaxClusterSize(25000);  // 최대 포인트 수
  ec.setSearchMethod(tree);     // 위에서 정의한 탐색 방법 지정
  ec.extract(cluster_indices);  // 군집화 적용

  // 클러스터별 정보 수집, 출력, 저장
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // 포인트수 출력
    std::cout << "PointCloud representing the Cluster: "
              << cloud_cluster->points.size() << " data points." << std::endl;

    // 클러스터별 이름 생성 및 저장
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}