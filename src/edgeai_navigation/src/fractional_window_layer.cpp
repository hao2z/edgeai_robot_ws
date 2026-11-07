#include <cmath>
#include <stdexcept>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace edgeai_navigation
{
// Nav2 Jazzy's native width/height parameters accept integer metres only.
// Load first: resize once before obstacle/inflation layers allocate their grids.
class FractionalWindowLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node || !layered_costmap_->isRolling()) {
      throw std::runtime_error("FractionalWindowLayer requires a rolling costmap");
    }
    declareParameter("width_m", rclcpp::ParameterValue(1.5));
    declareParameter("height_m", rclcpp::ParameterValue(1.5));
    double width, height;
    node->get_parameter(name_ + ".width_m", width);
    node->get_parameter(name_ + ".height_m", height);
    auto * map = layered_costmap_->getCostmap();
    const double resolution = map->getResolution();
    if (!std::isfinite(width) || !std::isfinite(height) ||
      width < resolution || height < resolution)
    {
      throw std::runtime_error("Invalid fractional costmap dimensions");
    }
    layered_costmap_->resizeMap(
      std::lround(width / resolution), std::lround(height / resolution),
      resolution, map->getOriginX(), map->getOriginY(), true);
    current_ = true;
    RCLCPP_INFO(node->get_logger(), "Rolling window: %.2f x %.2f m (%u x %u cells)",
      map->getSizeInCellsX() * resolution, map->getSizeInCellsY() * resolution,
      map->getSizeInCellsX(), map->getSizeInCellsY());
  }
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
  void reset() override {current_ = true;}
  bool isClearable() override {return false;}
};
}  // namespace edgeai_navigation
PLUGINLIB_EXPORT_CLASS(edgeai_navigation::FractionalWindowLayer, nav2_costmap_2d::Layer)
