// Copyright 2018-present, Simone Fontana
// Distributed under the GNU GPL 3.0 License (https://www.gnu.org/licenses/gpl-3.0.html)

#include <Eigen/Core>
#include <memory>
#include <string>

#include "pcl/common/transforms.h"
#include "pcl/filters/random_sample.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_types.h"

#include "pso_registration/metric.hpp"
#include "pso_registration/particle.hpp"
#include "pso_registration/swarm.hpp"
#include "pso_registration/utilities.hpp"

typedef pcl::PointXYZ PointType;

using pso_registration::Particle;
using pso_registration::Swarm;

std::string matrix_to_string(const Eigen::MatrixXd &matrix) {
  std::stringstream ret;
  for (int i = 0; i < matrix.size(); i++) {
    ret << matrix(i) << " ";
  }
  return ret.str();
}

int main(int argc, char **argv) {
  std::string source_file_name;
  std::string target_file_name;
  std::string ground_truth_name;
  std::string metric;
  bool display = false;
  float source_filter_size, target_filter_size, random_perc;
  Eigen::Matrix4d initial_transformation = Eigen::Matrix4d::Identity();

  int num_part = 0, num_gen = 0;

  try {

    TCLAP::ValueArg<int> num_part_arg("p", "num_part", "The number of particles of the swarm", false, 50, "int", cmd);
    TCLAP::ValueArg<int> num_gen_arg("e", "num_it", "The number of iterations (generations) of the algorithm", false,
                                     1000, "int", cmd);
    TCLAP::ValueArg<std::string> metric_arg("m", "metric",
                                            "The metric to use. One of: l1, l2, robust_l2, normalized_robust_l2", false,
                                            "l1", "string", cmd);
    TCLAP::ValueArg<float> source_filter_arg(
        "s", "source_filter_size", "The leaf size of the voxel filter of the source cloud", false, 0, "float", cmd);
    TCLAP::ValueArg<float> target_filter_arg(
        "t", "target_filter_size", "The leaf size of the voxel filter of the target cloud", false, 0, "float", cmd);
    TCLAP::ValueArg<float> perc_filter_arg(
        "r", "random_perc",
        "The percentage of points (of the source and target cloud) to use for the alignment (randomly sampled).", false,
        100, "float", cmd);


    source_file_name = source_file_name_arg.getValue();
    target_file_name = target_file_name_arg.getValue();
    num_part = num_part_arg.getValue();
    num_gen = num_gen_arg.getValue();
    metric = metric_arg.getValue();
    source_filter_size = source_filter_arg.getValue();
    target_filter_size = target_filter_arg.getValue();
    display = display_arg.getValue();
    random_perc = perc_filter_arg.getValue();
    if (verbose_arg.getValue()) {
      spdlog::set_level(spdlog::level::debug);
    }
    if (transformation_arg.isSet()) {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
          initial_transformation(i, j) = transformation_arg.getValue()[i * 4 + j];
        }
      }
    }
    spdlog::debug("Using {} as initial transformation", matrix_to_string(initial_transformation));
  } catch (TCLAP::ArgException &e) {
    spdlog::critical("error: {0} for arg {1}", e.error(), e.argId());
    exit(EXIT_FAILURE);
  }

  pcl::PointCloud<PointType>::Ptr source_cloud(new pcl::PointCloud<PointType>);

  pcl::PointCloud<PointType>::Ptr target_cloud(new pcl::PointCloud<PointType>);

  std::vector<int> nan_points;
  pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, nan_points);
  pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, nan_points);

  pcl::PointCloud<PointType>::Ptr moved_filtered_source_cloud(new pcl::PointCloud<PointType>);
  pcl::copyPointCloud(*source_cloud, *moved_filtered_source_cloud);
  pcl::transformPointCloud(*moved_filtered_source_cloud, *moved_filtered_source_cloud, initial_transformation);

  double initial_error = point_cloud_registration_benchmark::calculate_error(source_cloud, moved_filtered_source_cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  if (source_filter_size > 0) {
    voxel_filter.setInputCloud(moved_filtered_source_cloud);
    voxel_filter.setLeafSize(source_filter_size, source_filter_size, source_filter_size);
    voxel_filter.filter(*moved_filtered_source_cloud);
  }
  if (target_filter_size > 0) {
    voxel_filter.setInputCloud(target_cloud);
    voxel_filter.setLeafSize(target_filter_size, target_filter_size, target_filter_size);
    voxel_filter.filter(*target_cloud);
  }

  if (random_perc != 100) {
    pcl::RandomSample<PointType> sample(true);
    sample.setInputCloud(moved_filtered_source_cloud);
    sample.setSample(moved_filtered_source_cloud->size() * (random_perc / 100));
    sample.filter(*moved_filtered_source_cloud);

    sample.setInputCloud(target_cloud);
    sample.setSample(target_cloud->size() * random_perc / 100);
    sample.filter(*target_cloud);
  }

  double (*score_function)(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr,
                           pcl::KdTree<PointType>::Ptr);
  if (metric == "l2") {
    score_function = &pso_registration::l2_distance;
  } else if (metric == "robust_l2") {
    score_function = &pso_registration::robust_l2_distance;
  } else if (metric == "robust_normalized_l2") {
    score_function = &pso_registration::robust_normalized_l2_distance;
  } else {
    score_function = &pso_registration::l1_distance;
  }

  Swarm swarm;

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr target_tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  target_tree->setEpsilon(3.6);
  target_tree->setInputCloud(target_cloud);
  for (int i = 0; i < num_part; i++) {
    swarm.add_particle(Particle(moved_filtered_source_cloud, target_cloud, target_tree, i, score_function));
  }
  Particle best;
  swarm.init();
  for (int i = 0; i < num_gen; i++) {
    std::ostringstream os;
    swarm.evolve();
    best = swarm.getBest();
    os << best;
  }

  auto estimated_transformation = swarm.getBest().getTransformation();
  pcl::transformPointCloud(*source_cloud, *moved_filtered_source_cloud, estimated_transformation);
  double error = point_cloud_registration_benchmark::calculate_error(source_cloud, moved_filtered_source_cloud);

  std::cout.precision(15);
  std::cout << initial_error << ", " << error << ", ";
  Eigen::Matrix4d matrix = estimated_transformation.matrix();
  for (int i = 0; i < matrix.size(); i++) {
    std::cout << matrix(i) << " ";
  }
  return 0;
}
