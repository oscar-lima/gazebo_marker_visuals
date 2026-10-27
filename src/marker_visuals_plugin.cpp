// Draws a visualization_msgs/MarkerArray topic in Gazebo as visuals: shapes to look at, with no link, no collision
// and no effect on physics (they live only in the rendering scenes).
//
// A visual plugin, so it runs in every rendering scene: gzserver's (the one Gazebo's camera sensors render) and
// gzclient's. Markers of type CUBE, SPHERE and CYLINDER are drawn with their pose, scale and colour, put into the
// world frame through TF (<world_frame>, which must coincide with Gazebo's world frame); other types are skipped.
// <namespaces> (space separated, empty: all) chooses which marker namespaces are drawn. DELETE and DELETEALL are
// honoured; a marker without an update for <timeout> seconds of ROS time disappears, so the drawing never outlives
// its publisher.
//
// <visibility> gui (default) keeps the visuals out of every Gazebo camera sensor, so robot cameras (and whatever
// perceives through them) never see them; only gzclient and the camera sensors whose names contain one of the
// <show_in_cameras> tokens (separated by spaces) draw them (their visibility mask is widened by the GUI bit).
// <visibility> all: every camera.
//
// The plugin sits in a <visual> of any model, normally a static model of its own with one visual-only link (see
// sdf/marker_visuals.sdf.xacro and launch/marker_visuals.launch). Parameters in the README.
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <OGRE/OgreViewport.h>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "gazebo_marker_visuals/marker_store.h"

namespace gazebo
{
class MarkerVisualsPlugin : public VisualPlugin
{
public:
  ~MarkerVisualsPlugin() override
  {
    update_.reset();
    // at shutdown the scene, and with it the world visual our visuals hang from, may be gone already: finalise them
    // only while their scene is still registered, else only drop the pointers
    const bool scene_alive = scene_ && rendering::get_scene(scene_->Name()) != nullptr;
    for (auto &v : visuals_)
      if (scene_alive && v.second.visual && v.second.visual->GetParent())
        v.second.visual->Fini();
    visuals_.clear();
    if (nh_)
      nh_->shutdown();
  }

  void Load(rendering::VisualPtr visual, sdf::ElementPtr sdf) override
  {
    visual_ = visual;
    scene_ = visual->GetScene();
    topic_ = Param<std::string>(sdf, "topic", "markers");
    world_frame_ = Param<std::string>(sdf, "world_frame", "map");
    timeout_ = Param<double>(sdf, "timeout", 2.0);
    const std::string visibility = Param<std::string>(sdf, "visibility", "gui");
    if (!gazebo_marker_visuals::VisibilityFlags(visibility, &flags_))
    {
      gzerr << "gazebo_marker_visuals: visibility '" << visibility << "' is neither gui nor all, using gui\n";
      flags_ = gazebo_marker_visuals::kVisibilityGui;
    }
    std::istringstream tokens(Param<std::string>(sdf, "show_in_cameras", ""));
    for (std::string token; tokens >> token;)
      camera_tokens_.push_back(token);
    std::istringstream names(Param<std::string>(sdf, "namespaces", ""));
    std::set<std::string> namespaces;
    for (std::string name; names >> name;)
      namespaces.insert(name);
    store_ = gazebo_marker_visuals::MarkerStore(namespaces);
    // the plugin's own visual (any tiny shape the model needs) is drawn like the markers
    visual_->SetVisibilityFlags(flags_);

    if (!ros::isInitialized())
    {   // gzclient has no ROS node of its own
      int argc = 0;
      ros::init(argc, nullptr, "gazebo_marker_visuals", ros::init_options::NoSigintHandler |
                                                          ros::init_options::AnonymousName);
    }
    nh_ = std::make_unique<ros::NodeHandle>();
    nh_->setCallbackQueue(&queue_);
    sub_ = nh_->subscribe(topic_, 1, &MarkerVisualsPlugin::OnMarkers, this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    // all rendering work happens in the rendering thread, before each frame
    update_ = event::Events::ConnectPreRender(std::bind(&MarkerVisualsPlugin::OnPreRender, this));
    gzmsg << "gazebo_marker_visuals: drawing " << nh_->resolveName(topic_) << " in "
          << (scene_->IsServer() ? "gzserver" : "gzclient") << ", visibility " << visibility << "\n";
  }

private:
  struct Drawn
  {
    rendering::VisualPtr visual;
    std::string mesh;
    uint64_t version = 0;
    ignition::math::Pose3d frame_in_world;   // of the marker's frame when the visual was last posed
    ignition::math::Vector3d scale;
    ignition::math::Color colour;
    double alpha = -1.0;
  };

  template <typename T>
  static T Param(const sdf::ElementPtr &sdf, const std::string &key, const T &fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<T>(key) : fallback;
  }

  void OnMarkers(const visualization_msgs::MarkerArrayConstPtr &msg)
  {
    store_.Apply(*msg, ros::Time::now().toSec());
  }

  void OnPreRender()
  {
    queue_.callAvailable();   // OnMarkers runs here, in the rendering thread: no locking needed
    for (const auto &key : store_.TakeRemoved())
      Remove(key);
    for (const auto &key : store_.Expire(ros::Time::now().toSec(), timeout_))
      Remove(key);
    // one TF lookup per marker frame and rendered frame; a visual is touched only when its marker changed (the store
    // keeps the version of a marker sent again unchanged) or its frame moved in the world (e.g. the robot drives)
    std::map<std::string, std::pair<bool, ignition::math::Pose3d>> frames;
    for (const auto &item : store_.Entries())
    {
      const visualization_msgs::Marker &m = item.second.marker;
      auto frame = frames.find(m.header.frame_id);
      if (frame == frames.end())
        frame = frames.emplace(m.header.frame_id, LookUp(m.header.frame_id)).first;
      if (!frame->second.first)
        continue;
      auto found = visuals_.find(item.first);
      const bool fresh = found == visuals_.end() || !found->second.visual ||
                         found->second.mesh != gazebo_marker_visuals::MeshFor(m.type);
      Drawn &d = visuals_[item.first];
      if (fresh)
        Create(item.first, m, &d);
      const bool changed = fresh || d.version != item.second.version;
      if (!changed && d.frame_in_world == frame->second.second)
        continue;
      d.visual->SetWorldPose(gazebo_marker_visuals::WorldPose(frame->second.second, m.pose));
      d.frame_in_world = frame->second.second;
      d.version = item.second.version;
      if (!changed)
        continue;
      const ignition::math::Vector3d scale(m.scale.x, m.scale.y, m.scale.z);
      if (fresh || scale != d.scale)
        d.visual->SetScale(d.scale = scale);
      const ignition::math::Color colour(m.color.r, m.color.g, m.color.b, 1.0);
      if (fresh || colour != d.colour)
      {
        d.visual->SetAmbient(colour);
        d.visual->SetDiffuse(colour);
        d.colour = colour;
      }
      if (fresh || m.color.a != d.alpha)
        d.visual->SetTransparency(1.0 - (d.alpha = m.color.a));
    }
    ShowInCameras();
  }

  // (found, the frame's pose in the world frame) from TF
  std::pair<bool, ignition::math::Pose3d> LookUp(const std::string &frame_id)
  {
    try
    {
      return {true, gazebo_marker_visuals::ToPose(
                        tf_buffer_->lookupTransform(world_frame_, frame_id, ros::Time(0)).transform)};
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(10.0, "gazebo_marker_visuals: no transform %s -> %s: %s", world_frame_.c_str(),
                        frame_id.c_str(), ex.what());
      return {false, ignition::math::Pose3d::Zero};
    }
  }

  void Create(const std::string &key, const visualization_msgs::Marker &m, Drawn *d)
  {
    if (d->visual)
      d->visual->Fini();
    d->visual = std::make_shared<rendering::Visual>(visual_->Name() + "::" + key, scene_->WorldVisual(), false);
    d->visual->Load();
    d->mesh = gazebo_marker_visuals::MeshFor(m.type);
    d->visual->AttachMesh(d->mesh);
    d->visual->SetMaterial("Gazebo/White");
    d->visual->SetCastShadows(false);
    d->visual->SetVisibilityFlags(flags_);
  }

  void Remove(const std::string &key)
  {
    auto it = visuals_.find(key);
    if (it == visuals_.end())
      return;
    if (it->second.visual)
      it->second.visual->Fini();
    visuals_.erase(it);
  }

  // Widens the visibility mask of the named camera sensors (in gzserver's scene) by the visuals' flags; checked
  // about once per second, since sensors and their viewports can appear after the plugin loads
  void ShowInCameras()
  {
    if (camera_tokens_.empty() || (flags_ & ~gazebo_marker_visuals::kVisibilityGui) != 0)
      return;
    const auto now = common::Time::GetWallTime();
    if ((now - last_camera_check_).Double() < 1.0)
      return;
    last_camera_check_ = now;
    for (uint32_t i = 0; i < scene_->CameraCount(); ++i)
    {
      rendering::CameraPtr camera = scene_->GetCamera(i);
      if (!camera || !camera->OgreViewport())
        continue;
      const std::string name = camera->ScopedName();
      for (const auto &token : camera_tokens_)
      {
        Ogre::Viewport *viewport = camera->OgreViewport();
        if (name.find(token) == std::string::npos || (viewport->getVisibilityMask() & flags_) == flags_)
          continue;
        viewport->setVisibilityMask(viewport->getVisibilityMask() | flags_);
        gzmsg << "gazebo_marker_visuals: camera " << name << " now draws the markers\n";
      }
    }
  }

  rendering::VisualPtr visual_;
  rendering::ScenePtr scene_;
  std::string topic_, world_frame_;
  double timeout_ = 2.0;
  uint32_t flags_ = gazebo_marker_visuals::kVisibilityGui;
  std::vector<std::string> camera_tokens_;
  common::Time last_camera_check_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  ros::Subscriber sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  gazebo_marker_visuals::MarkerStore store_;
  std::map<std::string, Drawn> visuals_;
  event::ConnectionPtr update_;
};

GZ_REGISTER_VISUAL_PLUGIN(MarkerVisualsPlugin)
}  // namespace gazebo
