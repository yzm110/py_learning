#include "convex_hull.h"

namespace zone {
namespace environment_model {
bc::bool_t ConvexHull::Solve(const bc::TFixedVector<Point3D, kMaxPointNum>& pts, ConvexHullInfo& convex_hull,
                             const EgoPose& ego_motion)
{
  pts_ = pts;
  if (pts_.size() == 0)
  {
    convex_hull = ConvexHullInfo();
    return bc::false_v;
  }
  else if (pts_.size() == 1)
  {
    convex_hull = ConvexHullInfo();
    convex_hull.dx_f_ = pts_[0].x;
    convex_hull.dy_f_ = pts_[0].y;
    convex_hull.heading_f = 0;
    convex_hull.length_f = 1.2f;
    convex_hull.width_f = 1.2f;
    convex_hull.valid_b_ = bc::true_v;
    return bc::true_v;
  }
  else if (pts_.size() == 2)
  {
    bc::float32_t t_length_f = 0.f;
    bc::float32_t t_width_f = 0.f;
    bc::float32_t t_denominator_f = 0.f;
    convex_hull = ConvexHullInfo();
    convex_hull.dx_f_ = (pts_[0].x + pts_[1].x) / 2;
    convex_hull.dy_f_ = (pts_[0].y + pts_[1].y) / 2;
    t_denominator_f = std::abs(pts_[1].x - pts_[0].x) > 0.000001 ? (pts_[1].x - pts_[0].x) : 0.000001;
    convex_hull.heading_f =
        std::atan((pts_[1].y - pts_[0].y) / t_denominator_f);
    t_length_f = std::sqrt((pts_[0].x - pts_[1].x) * (pts_[0].x - pts_[1].x) +
                                     (pts_[1].y - pts_[0].y) * (pts_[1].y - pts_[0].y)) +
                           1;
    t_width_f = 1.f;
    convex_hull.length_f = t_length_f > t_width_f ? t_length_f :t_width_f;
    convex_hull.width_f = t_length_f > t_width_f ? t_width_f :t_length_f;
    convex_hull.valid_b_ = bc::true_v;
    return bc::true_v;
  }
  else
  {
    pts_ = pts;
    switch (method_flag_)
    {
      case 1:
        return SolveWithEVD(convex_hull);
        break;
      case 2:
        return SolveWithSVD(convex_hull);
        break;
      default:
        return SolveWithSVD(convex_hull);
        break;
    }
    return bc::false_v;
  }
}

bc::bool_t ConvexHull::SolveWithEVD(ConvexHullInfo& convex_hull)
{
  Eigen::MatrixXd E = Eigen::MatrixXd::Identity(pts_.size(), pts_.size());
  Eigen::MatrixXd I(pts_.size(), 1);
  I.setOnes();
  Eigen::MatrixXd centralized_mtrix(2, pts_.size());
  Eigen::MatrixXd new_centralized_mtrix(2, pts_.size());
  Eigen::MatrixXd centering_matrix(pts_.size(), pts_.size());
  Eigen::MatrixXd S(2, 2);
  Point3D p1;
  Point3D p2;
  Point3D p3;
  Point3D p4;
  Eigen::VectorXd max_values;
  Eigen::VectorXd min_values;
  Eigen::Vector2d new_axis_center_point;
  Eigen::Vector2d raw_axis_center_point;
  bc::float32_t t_width_f = 0;
  bc::float32_t t_length_f = 0;
  bc::float32_t t_denominator_f = 0.f;

  for (bc::uint8_t t_col_idx_u8 = 0; t_col_idx_u8 < pts_.size(); ++t_col_idx_u8)
  {
    centralized_mtrix(0, t_col_idx_u8) = pts_[t_col_idx_u8].x;
  }

  for (bc::uint8_t t_col_idx_u8 = 0; t_col_idx_u8 < pts_.size(); ++t_col_idx_u8)
  {
    centralized_mtrix(1, t_col_idx_u8) = pts_[t_col_idx_u8].y;
  }

  centering_matrix = E - 1 / bc::max((bc::float32_t)pts_.size(), bc::float32_t(0.000001)) * I * I.transpose();
  S = 1 / bc::max(bc::float32_t(pts_.size()), bc::float32_t(0.000001)) * centralized_mtrix *
      centering_matrix * centralized_mtrix.transpose();
  
  Eigen::EigenSolver<Eigen::MatrixXd> es(S);
  Eigen::MatrixXd d = es.eigenvalues().real();
  Eigen::Matrix2d v = es.eigenvectors().real();

  // 得到投影后在新基下的坐标
  new_centralized_mtrix = v.transpose() * centralized_mtrix;
  // 每行的最大值分别对应每个维度的最大值，最小值同理
  max_values = new_centralized_mtrix.rowwise().maxCoeff();
  min_values = new_centralized_mtrix.rowwise().minCoeff();

  p1.x = min_values[0];
  p1.y = min_values[1];
  p2.x = max_values[0];
  p2.y = min_values[1];
  p3.x = max_values[0];
  p3.y = max_values[1];
  p4.x = min_values[0];
  p4.y = max_values[1];
  Eigen::MatrixXd::Index max_row, max_col;
  bc::float32_t t_max_lambda = d.maxCoeff(&max_row, &max_col);
  
  t_denominator_f = std::abs(v(0, max_row)) > 0.000001 ? v(0, max_row) : 0.000001;
  convex_hull.heading_f = std::atan(v(1,max_row) /  t_denominator_f);
  t_length_f = bc::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  t_width_f = bc::sqrt((p1.x - p4.x) * (p1.x - p4.x) + (p1.y - p4.y) * (p1.y - p4.y));
  convex_hull.length_f = t_length_f > t_width_f ? t_length_f : t_width_f;
  convex_hull.width_f = t_length_f > t_width_f ? t_width_f : t_length_f;
  new_axis_center_point[0] = (p1.x + p3.x) / 2;
  new_axis_center_point[1] = (p1.y + p3.y) / 2;
  raw_axis_center_point = v.transpose().inverse() * new_axis_center_point;
  
  convex_hull.dx_f_ = raw_axis_center_point[0];
  convex_hull.dy_f_ = raw_axis_center_point[1];
  convex_hull.valid_b_ = bc::true_v;
  return bc::true_v;
}

bc::bool_t ConvexHull::SolveWithSVD(ConvexHullInfo& convex_hull)
{ 
  Eigen::MatrixXd E = Eigen::MatrixXd::Identity(pts_.size(), pts_.size());
  Eigen::MatrixXd I(pts_.size(), 1);
  I.setOnes();
  Eigen::MatrixXd centralized_mtrix(2, pts_.size());
  Eigen::MatrixXd transformed_centralized_mtrix(2, pts_.size());
  Eigen::MatrixXd centering_matrix(pts_.size(), pts_.size());
  Point3D p1;
  Point3D p2;
  Point3D p3;
  Point3D p4;
  Eigen::VectorXd max_values;
  Eigen::VectorXd min_values;
  Eigen::Vector2d new_axis_center_point;
  Eigen::Vector2d raw_axis_center_point;
  bc::float32_t t_denominator_f = 0.f;
  bc::float32_t t_x_mean;
  bc::float32_t t_y_mean;
  for (bc::uint8_t t_col_idx_u8 = 0; t_col_idx_u8 < pts_.size(); ++t_col_idx_u8)
  {
    centralized_mtrix(0, t_col_idx_u8) = pts_[t_col_idx_u8].x;
  }

  for (bc::uint8_t t_col_idx_u8 = 0; t_col_idx_u8 < pts_.size(); ++t_col_idx_u8)
  {
    centralized_mtrix(1, t_col_idx_u8) = pts_[t_col_idx_u8].y;
  }

  t_x_mean = centralized_mtrix.row(0).mean();
  t_y_mean = centralized_mtrix.row(1).mean();


  centering_matrix = E - 1 / bc::max((bc::float32_t)pts_.size(), bc::float32_t(0.000001)) * I * I.transpose();
  centralized_mtrix = centralized_mtrix * centering_matrix;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(centralized_mtrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd A = svd.singularValues();
  Eigen::MatrixXd::Index max_row, max_col;
   
  bc::float32_t t_max_lambda = A.maxCoeff(&max_row, &max_col);
  t_denominator_f = std::abs(U(0, max_row)) > 0.000001 ? U(0, max_row) : 0.000001;
  convex_hull.heading_f = std::atan(U(1, max_row) / t_denominator_f);

  transformed_centralized_mtrix = U.transpose() * centralized_mtrix;
  max_values = transformed_centralized_mtrix.rowwise().maxCoeff();
  min_values = transformed_centralized_mtrix.rowwise().minCoeff();
  p1.x = min_values[0];
  p1.y = min_values[1];
  p2.x = max_values[0];
  p2.y = min_values[1];
  p3.x = max_values[0];
  p3.y = max_values[1];
  p4.x = min_values[0];
  p4.y = max_values[1];
  convex_hull.length_f = bc::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  convex_hull.width_f = bc::sqrt((p1.x - p4.x) * (p1.x - p4.x) + (p1.y - p4.y) * (p1.y - p4.y));
  new_axis_center_point[0] = (p1.x + p3.x) / 2;
  new_axis_center_point[1] = (p1.y + p3.y) / 2;
  raw_axis_center_point = U.transpose().inverse() * new_axis_center_point;

  convex_hull.dx_f_ = raw_axis_center_point[0] + t_x_mean;
  convex_hull.dy_f_ = raw_axis_center_point[1] + t_y_mean;
  convex_hull.valid_b_ = bc::true_v;
  return bc::true_v;
}


}  // namespace environment_model
}  // namespace zone
