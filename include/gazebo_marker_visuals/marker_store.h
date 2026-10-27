// The latest state of a visualization_msgs/MarkerArray stream, kept apart from Gazebo so it can be tested alone:
// which markers exist, which changed since they were last drawn, and which went away (DELETE, DELETEALL, or no
// update for longer than a timeout, e.g. because the publisher stopped).
#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <ignition/math/Pose3.hh>
#include <visualization_msgs/MarkerArray.h>

namespace gazebo_marker_visuals
{
// Gazebo's visibility bits (gazebo/rendering/RenderTypes.hh), repeated here so this header needs no Gazebo
constexpr uint32_t kVisibilityAll = 0x0FFFFFFF;
constexpr uint32_t kVisibilityGui = 0x00000001;

// "gui": only cameras for user interaction (gzclient) draw the visuals, Gazebo camera sensors do not;
// "all": every camera, the sensors included. Anything else is an error (returns false).
inline bool VisibilityFlags(const std::string &name, uint32_t *flags)
{
  if (name == "gui")
    *flags = kVisibilityGui;
  else if (name == "all")
    *flags = kVisibilityAll;
  else
    return false;
  return true;
}

// Gazebo's unit mesh for a marker type (size 1 along every axis, centred, cylinder along z like the RViz marker);
// empty for the types that are not drawn (lines, points, text, meshes, ...)
inline std::string MeshFor(int type)
{
  switch (type)
  {
    case visualization_msgs::Marker::CUBE:
      return "unit_box";
    case visualization_msgs::Marker::SPHERE:
      return "unit_sphere";
    case visualization_msgs::Marker::CYLINDER:
      return "unit_cylinder";
    default:
      return "";
  }
}

// Whether two markers look the same (type, frame, pose, scale, colour): a marker sent again unchanged needs no redraw
inline bool SameLook(const visualization_msgs::Marker &a, const visualization_msgs::Marker &b)
{
  const auto &p = a.pose, &q = b.pose;
  return a.type == b.type && a.header.frame_id == b.header.frame_id && p.position.x == q.position.x &&
         p.position.y == q.position.y && p.position.z == q.position.z && p.orientation.x == q.orientation.x &&
         p.orientation.y == q.orientation.y && p.orientation.z == q.orientation.z &&
         p.orientation.w == q.orientation.w && a.scale.x == b.scale.x && a.scale.y == b.scale.y &&
         a.scale.z == b.scale.z && a.color.r == b.color.r && a.color.g == b.color.g && a.color.b == b.color.b &&
         a.color.a == b.color.a;
}

inline std::string Key(const visualization_msgs::Marker &m)
{
  return m.ns + "/" + std::to_string(m.id);
}

inline ignition::math::Pose3d ToPose(const geometry_msgs::Pose &p)
{
  return ignition::math::Pose3d(p.position.x, p.position.y, p.position.z, p.orientation.w, p.orientation.x,
                                p.orientation.y, p.orientation.z);
}

inline ignition::math::Pose3d ToPose(const geometry_msgs::Transform &t)
{
  return ignition::math::Pose3d(t.translation.x, t.translation.y, t.translation.z, t.rotation.w, t.rotation.x,
                                t.rotation.y, t.rotation.z);
}

// The pose of a marker in the world: frame_in_world is the marker frame's pose in the world frame (from TF)
inline ignition::math::Pose3d WorldPose(const ignition::math::Pose3d &frame_in_world, const geometry_msgs::Pose &marker)
{
  return frame_in_world * ToPose(marker);   // ignition math 6: a * b = b expressed in a's parent frame
}

class MarkerStore
{
public:
  struct Entry
  {
    visualization_msgs::Marker marker;
    double received = 0.0;   // time of the last update [s]
    uint64_t version = 0;    // grows with every change of the marker's look, so a drawer knows what to redraw
  };

  // Only markers of these namespaces are kept (empty: every namespace)
  explicit MarkerStore(std::set<std::string> namespaces = {}) : namespaces_(std::move(namespaces))
  {
  }

  // Applies one MarkerArray received at now: ADD/MODIFY store or update a drawn type (a new version only when its
  // look changed), DELETE removes one marker,
  // DELETEALL every marker of the store; markers of a type that is not drawn or of another namespace are only counted.
  void Apply(const visualization_msgs::MarkerArray &array, double now)
  {
    for (const auto &m : array.markers)
    {
      if (m.action == visualization_msgs::Marker::DELETEALL)
      {
        for (const auto &e : entries_)
          removed_.insert(e.first);
        entries_.clear();
        continue;
      }
      const std::string key = Key(m);
      if (m.action == visualization_msgs::Marker::DELETE)
      {
        if (entries_.erase(key))
          removed_.insert(key);
        continue;
      }
      if (MeshFor(m.type).empty() || (!namespaces_.empty() && !namespaces_.count(m.ns)))
      {
        ++skipped_;
        continue;
      }
      Entry &e = entries_[key];
      const bool same = e.version != 0 && SameLook(e.marker, m);
      e.marker = m;
      e.received = now;
      if (!same)
        e.version = ++version_;
      removed_.erase(key);
    }
  }

  // Removes and returns the markers without an update for longer than timeout [s] (timeout <= 0: never). A clock
  // that went backwards (a sim restart) counts as an update now.
  std::vector<std::string> Expire(double now, double timeout)
  {
    std::vector<std::string> gone;
    if (timeout <= 0.0)
      return gone;
    for (auto it = entries_.begin(); it != entries_.end();)
    {
      if (now < it->second.received)
        it->second.received = now;
      if (now - it->second.received > timeout)
      {
        gone.push_back(it->first);
        it = entries_.erase(it);
      }
      else
        ++it;
    }
    return gone;
  }

  // The markers removed by DELETE or DELETEALL since the last call
  std::set<std::string> TakeRemoved()
  {
    std::set<std::string> out;
    out.swap(removed_);
    return out;
  }

  const std::map<std::string, Entry> &Entries() const
  {
    return entries_;
  }

  uint64_t Skipped() const
  {
    return skipped_;
  }

private:
  std::set<std::string> namespaces_;
  std::map<std::string, Entry> entries_;
  std::set<std::string> removed_;
  uint64_t version_ = 0;
  uint64_t skipped_ = 0;
};
}  // namespace gazebo_marker_visuals
