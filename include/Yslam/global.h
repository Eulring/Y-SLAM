#ifndef GLOBAL_H
#define GLOBAL_H


// for g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>


// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
using Eigen::Vector2d;
using Eigen::Vector3d;


// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SO3;
using Sophus::SE3;


// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
using cv::Mat;
using cv::Point2f;
using cv::Point2d;
using cv::Point3f;
using cv::Point3d;

// for std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <ostream>
#include <strstream>
#include <algorithm>
#include <math.h>
#include <set>
#include <map>
using namespace std;
#endif
