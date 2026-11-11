#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include "nav2_core/planner_exceptions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace edgeai_navigation
{
// Sample around the whole circle, snapping to distinct native lattice headings.
// A count of zero retains all headings. Keep the previous heading within the cap.
inline std::vector<double> selectGoalHeadings(const std::vector<double> & available,
  double preferred, size_t count, const std::optional<double> & reusable)
{
  auto distance=[](double a,double b) {return std::abs(std::remainder(a-b,2*M_PI));};
  std::vector<double> selected;
  const size_t limit=count==0 ? available.size() : std::min(count,available.size());
  for (size_t i=0;i<limit;++i) {
    const double target=preferred+2*M_PI*i/limit;
    auto nearest=available.end();
    for (auto it=available.begin();it!=available.end();++it) {
      if (std::find(selected.begin(),selected.end(),*it)!=selected.end()) {continue;}
      if (nearest==available.end() || distance(*it,target)<distance(*nearest,target)) {nearest=it;}
    }
    selected.push_back(*nearest);
  }
  if (reusable && !selected.empty() &&
    std::find(selected.begin(),selected.end(),*reusable)==selected.end()) {
    const auto found=std::find(available.begin(),available.end(),*reusable);
    if (found!=available.end()) {selected.back()=*found;}
  }
  return selected;
}

struct HeadingPlan
{
  nav_msgs::msg::Path path;
  double yaw{0.0}, cost{std::numeric_limits<double>::infinity()};
  unsigned int attempted{0}, feasible{0};
  bool reused{false};
};

// Jazzy compatibility for upstream multi-goal-heading selection. Each leaf is
// still native Smac Lattice; compare complete returned paths in a shared budget.
template<typename Plan, typename Score>
HeadingPlan chooseHeadingPlan(const std::vector<double> & headings, double budget_s,
  const std::optional<double> & reusable, const std::function<bool()> & cancelled,
  Plan plan, Score score)
{
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now()+std::chrono::duration<double>(budget_s);
  const auto check_cancel = [&]() {
      if (cancelled && cancelled()) {throw nav2_core::PlannerCancelled("Heading search canceled");}
    };
  HeadingPlan best;
  bool timed_out = false;
  for (size_t i=0; i<headings.size(); ++i) {
    check_cancel();
    const double remaining = std::chrono::duration<double>(deadline-Clock::now()).count();
    if (remaining<=0) {timed_out=true;break;}
    const bool is_cached=reusable && std::abs(std::remainder(headings[i]-*reusable,2*M_PI))<1e-8;
    const double share=is_cached && headings.size()-i>1 ? remaining*.5 : remaining/(headings.size()-i);
    const auto slice_end = Clock::now()+std::chrono::duration<double>(share);
    ++best.attempted;
    try {
      auto path=plan(headings[i],[&]() {
          return (cancelled && cancelled()) || Clock::now()>=slice_end;
        });
      check_cancel();
      const double cost=path.poses.empty() ? std::numeric_limits<double>::infinity() : score(path);
      if (!std::isfinite(cost)) {continue;}
      ++best.feasible;
      // Order breaks equal-cost ties; input/cached yaw is a hint, not a penalty.
      if (cost<best.cost-1e-6) {best.path=std::move(path);best.cost=cost;best.yaw=headings[i];}
      if (is_cached) {
        best.reused=true;return best;
      }
    } catch (const nav2_core::StartOccupied &) {throw;}
    catch (const nav2_core::StartOutsideMapBounds &) {throw;}
    catch (const nav2_core::GoalOutsideMapBounds &) {throw;}
    catch (const nav2_core::PlannerTFError &) {throw;}
    catch (const nav2_core::InvalidPlanner &) {throw;}
    catch (const nav2_core::PlannerCancelled &) {check_cancel();timed_out=true;}
    catch (const nav2_core::PlannerTimedOut &) {timed_out=true;}
    catch (const nav2_core::PlannerException &) {check_cancel();}
  }
  check_cancel();
  if (!best.path.poses.empty()) {return best;}
  const std::string detail="; attempted "+std::to_string(best.attempted)+"/"+std::to_string(headings.size())+" headings";
  if (timed_out) {throw nav2_core::PlannerTimedOut("No feasible heading found within shared search budget"+detail);}
  throw nav2_core::NoValidPathCouldBeFound("No feasible path for candidate goal headings"+detail);
}

// Based on upstream Smac analytic-expansion distance/cost scoring. Rotation and
// reverse terms preserve the configured diff-drive preferences. This is a path
// comparison score, not the native A* accumulated g-cost (not exposed by Jazzy).
inline double headingPathCost(const nav_msgs::msg::Path & path,
  nav2_costmap_2d::Costmap2D & map, double cost_penalty, double rotation_penalty,
  double reverse_penalty, bool allow_unknown)
{
  if (path.poses.empty()) {return std::numeric_limits<double>::infinity();}
  double score=0.;
  for (size_t i=0;i<path.poses.size();++i) {
    const auto & pose=path.poses[i].pose;
    unsigned int x,y;
    if (!map.worldToMap(pose.position.x,pose.position.y,x,y)) {return std::numeric_limits<double>::infinity();}
    const auto value=map.getCost(x,y);
    if (value==nav2_costmap_2d::LETHAL_OBSTACLE ||
      (value==nav2_costmap_2d::NO_INFORMATION && !allow_unknown)) {return std::numeric_limits<double>::infinity();}
    if (i==0) {continue;}
    const auto & previous=path.poses[i-1].pose;
    const double dx=pose.position.x-previous.position.x,dy=pose.position.y-previous.position.y;
    const double distance=std::hypot(dx,dy);
    const double yaw=tf2::getYaw(previous.orientation);
    const double turn=std::abs(std::remainder(tf2::getYaw(pose.orientation)-yaw,2*M_PI));
    const double normalized=value==nav2_costmap_2d::NO_INFORMATION ? 1. : std::min(1.,value/252.);
    const double direction=dx*std::cos(yaw)+dy*std::sin(yaw)<-1e-6 ? reverse_penalty : 1.;
    score+=(distance*direction+rotation_penalty*map.getResolution()*turn)*(1.+cost_penalty*normalized);
  }
  return score;
}
}  // namespace edgeai_navigation
