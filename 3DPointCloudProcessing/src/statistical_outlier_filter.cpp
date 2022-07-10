#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read<pcl::PointXYZ>(argv[1], *cloud);

  // 포인트수 출력
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // 오브젝트 생성
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);    // 입력
  sor.setMeanK(50);            // 분석시 고려한 이웃 점 수(50개)
  sor.setStddevMulThresh(1.0); // Outlier로 처리할 거리 정보
  sor.filter(*cloud_filtered); // 필터 적용

  // 생성된 포인트클라우드 수 출력
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  // 생성된 포인트클라우드(inlier) 저장
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered,
                              false);

  // 생성된 포인트클라우드(outlier) 저장
  sor.setNegative(true);
  sor.filter(*cloud_filtered);
  writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd",
                              *cloud_filtered, false);

  return (0);
}