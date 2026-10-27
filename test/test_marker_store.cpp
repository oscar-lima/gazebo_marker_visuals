// MarkerStore and the pose and visibility helpers of the plugin, without Gazebo
#include <cmath>

#include <gtest/gtest.h>

#include "gazebo_marker_visuals/marker_store.h"

using gazebo_marker_visuals::MarkerStore;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace
{
Marker M(const std::string &ns, int id, int type = Marker::CYLINDER, int action = Marker::ADD)
{
  Marker m;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = action;
  m.pose.orientation.w = 1.0;
  return m;
}

MarkerArray A(std::initializer_list<Marker> markers)
{
  MarkerArray a;
  a.markers = markers;
  return a;
}
}  // namespace

TEST(MarkerStore, KeepsDrawnTypesAndCountsTheRest)
{
  MarkerStore s;
  s.Apply(A({M("discs", 1), M("discs", 2, Marker::SPHERE), M("text", 1, Marker::TEXT_VIEW_FACING),
             M("line", 0, Marker::LINE_STRIP)}), 1.0);
  EXPECT_EQ(s.Entries().size(), 2u);
  EXPECT_EQ(s.Skipped(), 2u);
  EXPECT_EQ(s.Entries().count("discs/1"), 1u);
}

TEST(MarkerStore, KeepsOnlyTheChosenNamespaces)
{
  MarkerStore s({"cable_discs"});
  s.Apply(A({M("cable_discs", 1), M("collision_capsules", 1), M("clamps", 2, Marker::SPHERE)}), 1.0);
  EXPECT_EQ(s.Entries().size(), 1u);
  EXPECT_EQ(s.Entries().count("cable_discs/1"), 1u);
  EXPECT_EQ(s.Skipped(), 2u);
}

TEST(MarkerStore, OnlyAChangedLookRaisesTheVersion)
{
  MarkerStore s;
  s.Apply(A({M("discs", 1)}), 1.0);
  const auto v1 = s.Entries().at("discs/1").version;
  s.Apply(A({M("discs", 1)}), 1.1);   // sent again unchanged (the monitor every tick): no redraw
  EXPECT_EQ(s.Entries().at("discs/1").version, v1);
  EXPECT_DOUBLE_EQ(s.Entries().at("discs/1").received, 1.1);   // but it counts as an update for the timeout
  Marker moved = M("discs", 1);
  moved.pose.position.z = 0.1;
  s.Apply(A({moved}), 1.2);
  const auto v2 = s.Entries().at("discs/1").version;
  EXPECT_GT(v2, v1);
  Marker red = moved;
  red.color.r = 1.0;
  s.Apply(A({red}), 1.3);
  EXPECT_GT(s.Entries().at("discs/1").version, v2);
}

TEST(MarkerStore, DeleteAndDeleteAllReportRemovedMarkers)
{
  MarkerStore s;
  s.Apply(A({M("discs", 1), M("discs", 2), M("discs", 3)}), 1.0);
  s.Apply(A({M("discs", 2, Marker::CYLINDER, Marker::DELETE)}), 1.1);
  EXPECT_EQ(s.TakeRemoved(), (std::set<std::string>{"discs/2"}));
  EXPECT_TRUE(s.TakeRemoved().empty());
  // the monitor's first message starts with a DELETEALL, then adds again: the re-added ones are not removed
  s.Apply(A({M("", 0, Marker::ARROW, Marker::DELETEALL), M("discs", 1)}), 1.2);
  EXPECT_EQ(s.TakeRemoved(), (std::set<std::string>{"discs/3"}));
  EXPECT_EQ(s.Entries().size(), 1u);
}

TEST(MarkerStore, ExpiresMarkersOfAStoppedPublisher)
{
  MarkerStore s;
  s.Apply(A({M("discs", 1), M("discs", 2)}), 10.0);
  s.Apply(A({M("discs", 2)}), 11.5);
  EXPECT_TRUE(s.Expire(12.0, 2.0).empty());
  EXPECT_EQ(s.Expire(12.5, 2.0), (std::vector<std::string>{"discs/1"}));
  EXPECT_TRUE(s.Expire(100.0, 0.0).empty());   // timeout 0: never
  EXPECT_TRUE(s.Expire(1.0, 2.0).empty());     // the clock went back (sim restart): not expired
  EXPECT_EQ(s.Entries().size(), 1u);
}

TEST(Helpers, MeshesAndVisibility)
{
  EXPECT_EQ(gazebo_marker_visuals::MeshFor(Marker::CUBE), "unit_box");
  EXPECT_EQ(gazebo_marker_visuals::MeshFor(Marker::SPHERE), "unit_sphere");
  EXPECT_EQ(gazebo_marker_visuals::MeshFor(Marker::CYLINDER), "unit_cylinder");
  EXPECT_EQ(gazebo_marker_visuals::MeshFor(Marker::MESH_RESOURCE), "");
  uint32_t flags = 0;
  EXPECT_TRUE(gazebo_marker_visuals::VisibilityFlags("gui", &flags));
  EXPECT_EQ(flags, 0x00000001u);
  EXPECT_TRUE(gazebo_marker_visuals::VisibilityFlags("all", &flags));
  EXPECT_EQ(flags, 0x0FFFFFFFu);
  EXPECT_FALSE(gazebo_marker_visuals::VisibilityFlags("sensors", &flags));
}

TEST(Helpers, WorldPoseComposesFrameAndMarker)
{
  // frame 1 m along world x, turned 90 deg about z; a marker 1 m along the frame's x lies at world (1, 1, 0)
  const ignition::math::Pose3d frame(1, 0, 0, 0, 0, M_PI / 2);
  geometry_msgs::Pose marker;
  marker.position.x = 1.0;
  marker.orientation.w = 1.0;
  const auto world = gazebo_marker_visuals::WorldPose(frame, marker);
  EXPECT_NEAR(world.Pos().X(), 1.0, 1e-9);
  EXPECT_NEAR(world.Pos().Y(), 1.0, 1e-9);
  EXPECT_NEAR(world.Rot().Yaw(), M_PI / 2, 1e-9);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
