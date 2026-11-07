#include <gtest/gtest.h>
#include "recovery_geometry.hpp"
#include "recovery_history.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
using namespace edgeai_navigation;

static Polygon body()
{
  Polygon p(4);
  p[0].x=.1225; p[0].y=.175; p[1].x=-.3225; p[1].y=.175;
  p[2].x=-.3225; p[2].y=-.175; p[3].x=.1225; p[3].y=-.175;
  return p;
}

TEST(RecoveryGeometry, NativeCollisionCheckMustUseCostmapFrameRatherThanRawOdom)
{
  nav2_costmap_2d::Costmap2D map(120,120,.05,0.,0.,0);
  // map->odom is a 1 m translation. A wall cell is beside the *wrong* odom coordinates.
  map.setCost(42,40,254);
  const RecoveryPose map_pose{3.,2.,0.}, odom_pose{2.,2.,0.};
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> native(&map);
  EXPECT_TRUE(motionFree(map,body(),map_pose,{"backup",.10}));
  EXPECT_TRUE(motionFree(map,body(),map_pose,{"spin",.35}));
  EXPECT_GE(native.footprintCostAtPose(odom_pose.x,odom_pose.y,odom_pose.yaw,body()),254.);
  EXPECT_LT(native.footprintCostAtPose(map_pose.x,map_pose.y,map_pose.yaw,body()),254.);
}

TEST(RecoveryGeometry, NarrowCorridorRejectsLargeButAllowsShortExit)
{
  nav2_costmap_2d::Costmap2D map(500,500,.01,0,0,0);
  for (unsigned y=0;y<500;++y) {
    for (unsigned x=0;x<500;++x) {
      double wx,wy;map.mapToWorld(x,y,wx,wy);
      if ((wx<=2.2 && wy>=3.05 && wy<=3.15) ||
        (wx>=1.9 && wx<=2.5 && wy>=1.5 && wy<=2.5)) {map.setCost(x,y,254);}
    }
  }
  RecoveryPose pose{1.9490441186,2.7669287444,-.2407589578};
  ASSERT_TRUE(poseFree(map,body(),pose));
  EXPECT_TRUE(motionFree(map,body(),pose,{"backup",.05}));
  EXPECT_TRUE(motionFree(map,body(),pose,{"backup",.10}));
  EXPECT_FALSE(motionFree(map,body(),pose,{"backup",.20}));
  EXPECT_FALSE(motionFree(map,body(),pose,{"spin",1.}));
  EXPECT_FALSE(motionFree(map,body(),pose,{"spin",-1.}));
  EXPECT_TRUE(motionFree(map,body(),pose,{"spin",.20}));
}

TEST(RecoveryGeometry, InteriorObstaclesUnknownAndBoundsAreNotIgnored)
{
  nav2_costmap_2d::Costmap2D map(100,100,.05,0,0,0);
  const RecoveryPose pose{2.,2.,0.};
  ASSERT_TRUE(poseFree(map,body(),pose));
  map.setCost(40,40,254);
  EXPECT_FALSE(poseFree(map,body(),pose));
  EXPECT_FALSE(motionFree(map,body(),pose,{"forward",.1}));
  map.setCost(40,40,255);
  EXPECT_FALSE(poseFree(map,body(),pose));
  map.setCost(40,40,0);
  EXPECT_FALSE(poseFree(map,body(),{.1,2.,0.}));
}

TEST(RecoveryGeometry, KnownBodyAndPaddingOverlapCanExitButNewObstaclesCannot)
{
  nav2_costmap_2d::Costmap2D map(1000,1000,.005,0,0,0);
  const RecoveryPose pose{2.,2.,0.};
  auto actual=body();for(auto & p:actual){p.x-=std::copysign(.01,p.x);p.y-=std::copysign(.01,p.y);}
  for(unsigned y=0;y<1000;++y){map.setCost(423,y,254);}
  ASSERT_FALSE(poseFree(map,body(),pose));ASSERT_TRUE(poseFree(map,actual,pose));
  const auto overlap=occupiedOverlap(map,body(),pose);
  ASSERT_TRUE(overlap.valid);ASSERT_GT(overlap.area,0.);
  EXPECT_TRUE(occupiedEscapeFree(map,actual,body(),pose,{"backup",.05},overlap));
  EXPECT_FALSE(occupiedEscapeFree(map,actual,body(),pose,{"forward",.05},overlap));
  EXPECT_FALSE(occupiedEscapeFree(map,actual,body(),pose,{"spin",.2},overlap));
  for(unsigned y=0;y<1000;++y){map.setCost(329,y,254);}
  EXPECT_FALSE(occupiedEscapeFree(map,actual,body(),pose,{"backup",.05},overlap));
  for(unsigned y=0;y<1000;++y){map.setCost(329,y,0);map.setCost(421,y,254);}
  ASSERT_FALSE(poseFree(map,actual,pose));
  const auto body_overlap=occupiedOverlap(map,body(),pose);
  EXPECT_TRUE(occupiedEscapeFree(map,actual,body(),pose,{"backup",.05},body_overlap));
  EXPECT_FALSE(occupiedEscapeFree(map,actual,body(),pose,{"forward",.05},body_overlap));
  for(unsigned y=0;y<1000;++y){map.setCost(421,y,0);map.setCost(423,y,255);}
  EXPECT_FALSE(occupiedOverlap(map,body(),pose).valid);
}

TEST(RecoveryHistory, SmallRecoveryTranslationDoesNotForgetAttempt)
{
  RecoveryHistory history;history.add({2.,2.,0.},{"backup",.1});
  EXPECT_TRUE(history.tried({1.9,2.,.2},{"backup",.1}));
  EXPECT_FALSE(history.tried({1.9,2.,.2},{"backup",.05}));
  EXPECT_FALSE(history.tried({1.5,2.,.2},{"backup",.1}));
  EXPECT_EQ(history.size(),1u);
  history.clear();EXPECT_FALSE(history.tried({2.,2.,0.},{"backup",.1}));
}

TEST(RecoveryGeometry, BodyOverlapAllowsPartialProgressAndRejectsUnknown)
{
  nav2_costmap_2d::Costmap2D map(600,600,.005,0,0,0);
  const RecoveryPose pose{2.,2.,0.};
  auto actual=body();for(auto & p:actual){p.x-=std::copysign(.01,p.x);p.y-=std::copysign(.01,p.y);}
  for(unsigned y=0;y<600;++y){for(unsigned x=411;x<=424;++x){map.setCost(x,y,254);}}
  const auto overlap=occupiedOverlap(map,body(),pose);
  ASSERT_FALSE(poseFree(map,actual,pose));
  EXPECT_TRUE(occupiedEscapeFree(map,actual,body(),pose,{"backup",.05},overlap));
  EXPECT_TRUE(occupiedEscapeFree(map,actual,body(),pose,{"backup",.10},overlap));
  EXPECT_FALSE(occupiedEscapeFree(map,actual,body(),pose,{"spin",.20},overlap));
  for(unsigned y=0;y<600;++y){map.setCost(411,y,255);}
  EXPECT_FALSE(occupiedEscapeFree(map,actual,body(),pose,{"backup",.10},overlap));
}

TEST(RecoveryHistory, SustainedActualTranslationRequiredForProgressEpoch)
{
  TrackingProgress progress;
  for(int i=0;i<=25;++i){progress.observe(i*.1,{i*.015,0.,0.},true);}
  EXPECT_EQ(progress.epoch(),1u);
  progress.failed();
  for(int i=30;i<=55;++i){progress.observe(i*.1,{.4,0.,i*.1},false);}
  EXPECT_EQ(progress.epoch(),1u);
  // A controller gap cannot count as continuous tracking.
  progress.observe(7.,{.4,0.,0.},true);progress.observe(10.,{1.,0.,0.},true);
  EXPECT_EQ(progress.epoch(),1u);
}

TEST(RecoveryGeometry, RppShortArcDistinguishesBlockedAndRetreatedPose)
{
  nav2_costmap_2d::Costmap2D map(100,100,.05,0,0,0);
  for(unsigned y=0;y<100;++y){map.setCost(44,y,254);}
  EXPECT_FALSE(rppConnectionFree(map,body(),{2.,2.,0.},2.5,2.,true));
  EXPECT_TRUE(rppConnectionFree(map,body(),{1.9,2.,0.},2.5,2.,true));
  EXPECT_FALSE(rppConnectionFree(map,body(),{2.,2.,0.},1.5,2.,false));
}
