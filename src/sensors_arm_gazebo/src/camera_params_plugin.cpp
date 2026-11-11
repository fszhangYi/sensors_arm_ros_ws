// Runtime wrist-camera params from web:
//   - intrinsics: HFOV / size / rate + CameraInfo sync
//   - extrinsics: lens/look/right/up in tool frame → pose optical + gazebo camera links

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/msgs/visual.pb.h>
#include <gazebo/msgs/geometry.pb.h>
#include <gazebo/msgs/material.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <sensors_arm_msgs/srv/set_gazebo_camera.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ignition/math/Matrix3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace sensors_arm_gazebo
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

ignition::math::Vector3d NormOr(ignition::math::Vector3d v, ignition::math::Vector3d fallback)
{
  if (v.Length() < 1e-9) {
    return fallback;
  }
  v.Normalize();
  return v;
}
}  // namespace

class CameraParamsPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    sensor_name_ = "d405_color";
    if (sdf->HasElement("sensor_name")) {
      sensor_name_ = sdf->Get<std::string>("sensor_name");
    }
    link_name_ = "wrist_cam_d405_gazebo_link";
    if (sdf->HasElement("link_name")) {
      link_name_ = sdf->Get<std::string>("link_name");
    }
    optical_link_name_ = "wrist_cam_d405_optical_frame";
    if (sdf->HasElement("optical_link_name")) {
      optical_link_name_ = sdf->Get<std::string>("optical_link_name");
    }
    parent_link_name_ = "gripper_centor_link";
    if (sdf->HasElement("parent_link_name")) {
      parent_link_name_ = sdf->Get<std::string>("parent_link_name");
    }
    service_name_ = "/fov/set_gazebo_camera";
    if (sdf->HasElement("service_name")) {
      service_name_ = sdf->Get<std::string>("service_name");
    }
    set_camera_info_service_ = "/cell_c/d405/set_camera_info";
    if (sdf->HasElement("set_camera_info_service")) {
      set_camera_info_service_ = sdf->Get<std::string>("set_camera_info_service");
    }

    // Gazebo camera frame relative to ROS optical: rpy(0, -pi/2, +pi/2)
    T_optical_gazebo_ = ignition::math::Pose3d(
      ignition::math::Vector3d::Zero,
      ignition::math::Quaterniond(0.0, -0.5 * kPi, 0.5 * kPi));

    // Default extrinsics match d405_optical.xacro / cell_c_fov.yaml
    T_parent_optical_ = ignition::math::Pose3d(
      ignition::math::Vector3d(0.000045, 0.095734, 0.123294),
      ignition::math::Quaterniond(0.94831327, -0.00065494, 0.0));
    have_extrinsics_ = true;

    DetachCameraJoints();

    // Default FOV matches launch xacro (87° H); V from 640x360 aspect.
    viz_h_deg_ = 87.0;
    viz_v_deg_ = 2.0 * std::atan(std::tan(0.5 * viz_h_deg_ * kDeg2Rad) * (360.0 / 640.0)) / kDeg2Rad;
    frustum_near_ = 0.02;
    frustum_far_ = 0.28;

    gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gz_node_->Init(model_->GetWorld()->Name());
    // gzclient listens to ~/visual (MarkerManager often has zero subscribers).
    visual_pub_ = gz_node_->Advertise<gazebo::msgs::Visual>("~/visual");

    set_ci_client_ = ros_node_->create_client<sensor_msgs::srv::SetCameraInfo>(
      set_camera_info_service_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    srv_ = ros_node_->create_service<sensors_arm_msgs::srv::SetGazeboCamera>(
      service_name_,
      std::bind(
        &CameraParamsPlugin::OnSet, this,
        std::placeholders::_1, std::placeholders::_2));

    update_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&CameraParamsPlugin::OnUpdate, this));

    // Publish initial frustum once visuals are ready.
    update_first_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &) {
        if (frustum_published_) {
          return;
        }
        PublishFrustumVisual();
        frustum_published_ = true;
      });

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "camera_params_plugin model=%s parent=%s optical=%s sensor=%s::%s service=%s",
      model_->GetName().c_str(), parent_link_name_.c_str(),
      optical_link_name_.c_str(), link_name_.c_str(), sensor_name_.c_str(),
      service_name_.c_str());
  }

private:
  void DetachCameraJoints()
  {
    // Free optical/gazebo links from fixed URDF joints so web extrinsics can move them.
    for (auto const & name : {
           "wrist_cam_d405_optical_fixed",
           "wrist_cam_d405_gazebo_fixed"})
    {
      auto joint = model_->GetJoint(name);
      if (!joint) {
        // Sometimes scoped
        joint = model_->GetJoint(model_->GetName() + "::" + name);
      }
      if (joint) {
        joint->Detach();
        RCLCPP_INFO(ros_node_->get_logger(), "detached joint %s", name);
      }
    }
  }

  gazebo::physics::LinkPtr GetLink(std::string const & name)
  {
    auto link = model_->GetLink(name);
    if (link) {
      return link;
    }
    return model_->GetLink(model_->GetName() + "::" + name);
  }

  gazebo::sensors::CameraSensorPtr ResolveCamera()
  {
    std::string scoped =
      model_->GetName() + "::" + link_name_ + "::" + sensor_name_;
    auto sens = gazebo::sensors::get_sensor(scoped);
    auto cam = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sens);
    if (cam) {
      return cam;
    }
    auto * mgr = gazebo::sensors::SensorManager::Instance();
    if (!mgr) {
      return nullptr;
    }
    for (auto const & s : mgr->GetSensors()) {
      if (!s) {
        continue;
      }
      auto const sc = s->ScopedName();
      if (sc.find(sensor_name_) == std::string::npos) {
        continue;
      }
      if (sc.find(model_->GetName()) == std::string::npos) {
        continue;
      }
      cam = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(s);
      if (cam) {
        return cam;
      }
    }
    return nullptr;
  }

  bool SetExtrinsicsFromAxes(
    ignition::math::Vector3d lens_m,
    ignition::math::Vector3d look,
    ignition::math::Vector3d right,
    ignition::math::Vector3d up)
  {
    look = NormOr(look, ignition::math::Vector3d(0, 0, 1));
    right = NormOr(right, ignition::math::Vector3d(1, 0, 0));
    up = NormOr(up, ignition::math::Vector3d(0, 1, 0));
    // Optical: X=right, Y=down, Z=look
    ignition::math::Vector3d down = -up;
    // Re-orthonormalize for safety
    down = NormOr(look.Cross(right), down);
    right = NormOr(down.Cross(look), right);
    look = NormOr(right.Cross(down), look);

    ignition::math::Matrix3d rot(
      right.X(), down.X(), look.X(),
      right.Y(), down.Y(), look.Y(),
      right.Z(), down.Z(), look.Z());
    ignition::math::Quaterniond q(rot);
    T_parent_optical_ = ignition::math::Pose3d(lens_m, q);
    have_extrinsics_ = true;
    return true;
  }

  void ApplyExtrinsicsPose()
  {
    if (!have_extrinsics_) {
      return;
    }
    auto parent = GetLink(parent_link_name_);
    auto optical = GetLink(optical_link_name_);
    auto gazebo_link = GetLink(link_name_);
    // Fallback if fixed-joint lumping ate gripper_centor: place relative to link6.
    if (!parent) {
      parent = GetLink("link6");
    }
    if (!parent) {
      return;
    }

    ignition::math::Pose3d parent_pose = parent->WorldPose();
    ignition::math::Pose3d optical_world = parent_pose * T_parent_optical_;

    if (optical) {
      optical->SetWorldPose(optical_world);
      optical->SetLinearVel(ignition::math::Vector3d::Zero);
      optical->SetAngularVel(ignition::math::Vector3d::Zero);
    }
    if (gazebo_link) {
      gazebo_link->SetWorldPose(optical_world * T_optical_gazebo_);
      gazebo_link->SetLinearVel(ignition::math::Vector3d::Zero);
      gazebo_link->SetAngularVel(ignition::math::Vector3d::Zero);
    } else if (auto cam = ResolveCamera()) {
      // Last resort: sensor still parented to a lumped link — offset in link frame.
      // Approx: treat parent as optical parent and set sensor pose in parent frame.
      cam->SetPose(T_parent_optical_ * T_optical_gazebo_);
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = ros_node_->get_clock()->now();
    tf.header.frame_id = parent_link_name_;
    tf.child_frame_id = optical_link_name_;
    tf.transform.translation.x = T_parent_optical_.Pos().X();
    tf.transform.translation.y = T_parent_optical_.Pos().Y();
    tf.transform.translation.z = T_parent_optical_.Pos().Z();
    tf.transform.rotation.x = T_parent_optical_.Rot().X();
    tf.transform.rotation.y = T_parent_optical_.Rot().Y();
    tf.transform.rotation.z = T_parent_optical_.Rot().Z();
    tf.transform.rotation.w = T_parent_optical_.Rot().W();
    tf_broadcaster_->sendTransform(tf);
  }

  void SyncCameraInfo(unsigned int width, unsigned int height, double h_deg)
  {
    if (width < 16 || height < 16 || h_deg <= 1.0 || h_deg >= 179.0) {
      return;
    }
    double const hfov = h_deg * kDeg2Rad;
    double const fx = static_cast<double>(width) / (2.0 * std::tan(hfov / 2.0));
    double const fy = fx;
    double const cx = 0.5 * static_cast<double>(width);
    double const cy = 0.5 * static_cast<double>(height);

    auto req = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
    auto & ci = req->camera_info;
    ci.width = width;
    ci.height = height;
    ci.distortion_model = "plumb_bob";
    ci.d = {0, 0, 0, 0, 0};
    ci.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    ci.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    ci.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};

    if (!set_ci_client_->service_is_ready()) {
      set_ci_client_->wait_for_service(std::chrono::milliseconds(200));
    }
    if (!set_ci_client_->service_is_ready()) {
      return;
    }
    set_ci_client_->async_send_request(req);
  }

  void SetIdentityPose(gazebo::msgs::Pose * pose)
  {
    pose->mutable_position()->set_x(0);
    pose->mutable_position()->set_y(0);
    pose->mutable_position()->set_z(0);
    pose->mutable_orientation()->set_x(0);
    pose->mutable_orientation()->set_y(0);
    pose->mutable_orientation()->set_z(0);
    pose->mutable_orientation()->set_w(1);
  }

  void SetMaterialRed(
    gazebo::msgs::Material * mat, float r, float g, float b, float a)
  {
    mat->mutable_ambient()->set_r(r);
    mat->mutable_ambient()->set_g(g);
    mat->mutable_ambient()->set_b(b);
    mat->mutable_ambient()->set_a(a);
    mat->mutable_diffuse()->set_r(r);
    mat->mutable_diffuse()->set_g(g);
    mat->mutable_diffuse()->set_b(b);
    mat->mutable_diffuse()->set_a(a);
  }

  void PublishOrientedBox(
    std::string const & name,
    std::string const & parent,
    ignition::math::Vector3d const & center,
    ignition::math::Quaterniond const & q,
    double sx, double sy, double sz,
    double transparency,
    float r, float g, float b, float a)
  {
    gazebo::msgs::Visual vis;
    vis.set_name(name);
    vis.set_parent_name(parent);
    vis.set_cast_shadows(false);
    vis.set_transparency(transparency);
    vis.set_visible(true);
    vis.set_type(gazebo::msgs::Visual::VISUAL);
    auto * pose = vis.mutable_pose();
    pose->mutable_position()->set_x(center.X());
    pose->mutable_position()->set_y(center.Y());
    pose->mutable_position()->set_z(center.Z());
    pose->mutable_orientation()->set_x(q.X());
    pose->mutable_orientation()->set_y(q.Y());
    pose->mutable_orientation()->set_z(q.Z());
    pose->mutable_orientation()->set_w(q.W());
    SetMaterialRed(vis.mutable_material(), r, g, b, a);
    auto * geom = vis.mutable_geometry();
    geom->set_type(gazebo::msgs::Geometry::BOX);
    auto * size = geom->mutable_box()->mutable_size();
    size->set_x(sx);
    size->set_y(sy);
    size->set_z(sz);
    visual_pub_->Publish(vis);
  }

  void PublishBoxEdge(
    std::string const & name,
    std::string const & parent,
    ignition::math::Vector3d const & a,
    ignition::math::Vector3d const & b,
    double thickness)
  {
    ignition::math::Vector3d d = b - a;
    double len = d.Length();
    if (len < 1e-6) {
      return;
    }
    ignition::math::Vector3d mid = (a + b) * 0.5;
    ignition::math::Quaterniond q;
    q.From2Axes(ignition::math::Vector3d::UnitX, d.Normalized());
    PublishOrientedBox(
      name, parent, mid, q, len, thickness, thickness,
      0.1, 0.95f, 0.40f, 0.42f, 0.90f);
  }

  ignition::math::Pose3d CameraWorldPose()
  {
    // Always derive from the tool link + extrinsics. After fixed-joint detach,
    // wrist_cam_* links can drift under ODE and make the frustum look wrong.
    auto parent = GetLink(parent_link_name_);
    if (!parent) {
      parent = GetLink("link6");
    }
    if (parent) {
      return parent->WorldPose() * T_parent_optical_ * T_optical_gazebo_;
    }
    auto gazebo_link = GetLink(link_name_);
    if (gazebo_link) {
      return gazebo_link->WorldPose();
    }
    return ignition::math::Pose3d::Zero;
  }

  void DeleteVisual(std::string const & name, std::string const & parent)
  {
    gazebo::msgs::Visual gone;
    gone.set_name(name);
    gone.set_parent_name(parent);
    gone.set_delete_me(true);
    visual_pub_->Publish(gone);
  }

  void PublishFrustumVisual()
  {
    if (!visual_pub_) {
      return;
    }
    // World-rooted (always valid in gzclient) + same tool-frame math as web.
    std::string const parent = model_->GetWorld()->Name();
    std::string const prefix = model_->GetName() + "_fovX_";

    auto tool = GetLink(parent_link_name_);
    if (!tool) {
      tool = GetLink("link6");
    }
    if (!tool) {
      return;
    }
    ignition::math::Pose3d const T_wt = tool->WorldPose();

    if (!frustum_purged_) {
      for (auto const & pre : {
             model_->GetName() + "_fov_",
             model_->GetName() + "_fovW_",
             model_->GetName() + "_fovT_"})
      {
        for (auto const & leaf : {
               "p0", "p1", "p2", "p3", "far", "apex",
               "e0", "e1", "e2", "e3", "e4", "e5", "e6", "e7",
               "e8", "e9", "e10", "e11"})
        {
          DeleteVisual(pre + leaf, parent);
          DeleteVisual(pre + leaf, model_->GetName() + "::" + parent_link_name_);
        }
      }
      frustum_purged_ = true;
    }

    // Optical axes in tool frame (matches web viz.py).
    ignition::math::Vector3d const lens = T_parent_optical_.Pos();
    ignition::math::Matrix3d const R(T_parent_optical_.Rot());
    ignition::math::Vector3d const look = NormOr(R * ignition::math::Vector3d(0, 0, 1),
      ignition::math::Vector3d(0, 0, 1));
    ignition::math::Vector3d const right = NormOr(R * ignition::math::Vector3d(1, 0, 0),
      ignition::math::Vector3d(1, 0, 0));
    ignition::math::Vector3d const down = NormOr(R * ignition::math::Vector3d(0, 1, 0),
      ignition::math::Vector3d(0, 1, 0));

    auto pt_tool = [&](double depth, double hs, double vs) {
      return lens + look * depth + right * hs + down * vs;
    };
    auto to_world = [&](ignition::math::Vector3d const & p_tool) {
      return T_wt.Pos() + T_wt.Rot().RotateVector(p_tool);
    };

    double const n = frustum_near_;
    double const f = frustum_far_;
    double const hn = n * std::tan(0.5 * viz_h_deg_ * kDeg2Rad);
    double const vn = n * std::tan(0.5 * viz_v_deg_ * kDeg2Rad);
    double const hf = f * std::tan(0.5 * viz_h_deg_ * kDeg2Rad);
    double const vf = f * std::tan(0.5 * viz_v_deg_ * kDeg2Rad);
    ignition::math::Vector3d const o = to_world(pt_tool(0, 0, 0));
    ignition::math::Vector3d const npp = to_world(pt_tool(n, hn, vn));
    ignition::math::Vector3d const npm = to_world(pt_tool(n, hn, -vn));
    ignition::math::Vector3d const nmm = to_world(pt_tool(n, -hn, -vn));
    ignition::math::Vector3d const nmp = to_world(pt_tool(n, -hn, vn));
    ignition::math::Vector3d const fpp = to_world(pt_tool(f, hf, vf));
    ignition::math::Vector3d const fpm = to_world(pt_tool(f, hf, -vf));
    ignition::math::Vector3d const fmm = to_world(pt_tool(f, -hf, -vf));
    ignition::math::Vector3d const fmp = to_world(pt_tool(f, -hf, vf));

    {
      ignition::math::Vector3d look_w = T_wt.Rot().RotateVector(look);
      ignition::math::Vector3d right_w = T_wt.Rot().RotateVector(right);
      ignition::math::Vector3d down_w = T_wt.Rot().RotateVector(down);
      ignition::math::Matrix3d rot(
        look_w.X(), right_w.X(), down_w.X(),
        look_w.Y(), right_w.Y(), down_w.Y(),
        look_w.Z(), right_w.Z(), down_w.Z());
      PublishOrientedBox(
        prefix + "far", parent, to_world(pt_tool(f, 0, 0)),
        ignition::math::Quaterniond(rot),
        0.0008, 2.0 * hf, 2.0 * vf,
        0.85, 0.94f, 0.55f, 0.55f, 0.25f);
    }

    double const th = 0.003;
    PublishBoxEdge(prefix + "e0", parent, o, fpp, th);
    PublishBoxEdge(prefix + "e1", parent, o, fpm, th);
    PublishBoxEdge(prefix + "e2", parent, o, fmm, th);
    PublishBoxEdge(prefix + "e3", parent, o, fmp, th);
    PublishBoxEdge(prefix + "e4", parent, fpp, fpm, th);
    PublishBoxEdge(prefix + "e5", parent, fpm, fmm, th);
    PublishBoxEdge(prefix + "e6", parent, fmm, fmp, th);
    PublishBoxEdge(prefix + "e7", parent, fmp, fpp, th);
    PublishBoxEdge(prefix + "e8", parent, npp, npm, th);
    PublishBoxEdge(prefix + "e9", parent, npm, nmm, th);
    PublishBoxEdge(prefix + "e10", parent, nmm, nmp, th);
    PublishBoxEdge(prefix + "e11", parent, nmp, npp, th);

    if (!frustum_logged_) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "fov frustum visual parent=%s apex=(%.3f,%.3f,%.3f) look_tool=(%.3f,%.3f,%.3f) H=%.1f V=%.1f far=%.2f",
        parent.c_str(), o.X(), o.Y(), o.Z(),
        look.X(), look.Y(), look.Z(),
        viz_h_deg_, viz_v_deg_, frustum_far_);
      frustum_logged_ = true;
    }
  }

  void OnUpdate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ApplyExtrinsicsPose();
    ++frustum_tick_;
    if (frustum_tick_ % 50 == 0) {
      PublishFrustumVisual();
      frustum_published_ = true;
    }
  }

  void OnSet(
    sensors_arm_msgs::srv::SetGazeboCamera::Request::SharedPtr req,
    sensors_arm_msgs::srv::SetGazeboCamera::Response::SharedPtr res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (req->apply_extrinsics) {
      ignition::math::Vector3d lens(req->lens_m[0], req->lens_m[1], req->lens_m[2]);
      ignition::math::Vector3d look(req->look[0], req->look[1], req->look[2]);
      ignition::math::Vector3d right(req->right[0], req->right[1], req->right[2]);
      ignition::math::Vector3d up(req->up[0], req->up[1], req->up[2]);
      if (look.Length() < 1e-9 || right.Length() < 1e-9) {
        res->ok = false;
        res->error = "apply_extrinsics requires non-zero look/right";
        return;
      }
      SetExtrinsicsFromAxes(lens, look, right, up);
      ApplyExtrinsicsPose();
    }

    auto cam_sensor = ResolveCamera();
    auto camera = cam_sensor ? cam_sensor->Camera() : nullptr;
    double h_deg = req->h_deg;
    double v_deg = req->v_deg;
    uint32_t width = req->width;
    uint32_t height = req->height;
    double update_rate = req->update_rate;

    try {
      if (camera) {
        if (h_deg > 1.0 && h_deg < 179.0) {
          camera->SetHFOV(ignition::math::Angle(h_deg * kDeg2Rad));
          viz_h_deg_ = h_deg;
        }
        if (width >= 16 && height >= 16) {
          width = std::min<uint32_t>(width, 1280);
          height = std::min<uint32_t>(height, 720);
          camera->SetImageSize(width, height);
        }
      } else if (h_deg > 1.0 && h_deg < 179.0) {
        viz_h_deg_ = h_deg;
      }
      if (v_deg > 1.0 && v_deg < 179.0) {
        viz_v_deg_ = v_deg;
      } else if (camera && viz_h_deg_ > 1.0) {
        unsigned int w = std::max(1u, camera->ImageWidth());
        unsigned int h = std::max(1u, camera->ImageHeight());
        viz_v_deg_ = 2.0 * std::atan(std::tan(0.5 * viz_h_deg_ * kDeg2Rad) *
          (static_cast<double>(h) / static_cast<double>(w))) / kDeg2Rad;
      }
      if (cam_sensor && update_rate > 0.1 && update_rate < 60.0) {
        cam_sensor->SetUpdateRate(update_rate);
      }
    } catch (std::exception const & ex) {
      res->ok = false;
      res->error = ex.what();
      return;
    }

    unsigned int applied_w = camera ? camera->ImageWidth() : 0;
    unsigned int applied_h = camera ? camera->ImageHeight() : 0;
    double applied_h_deg = camera ? camera->HFOV().Degree() : viz_h_deg_;
    if (camera) {
      SyncCameraInfo(applied_w, applied_h, applied_h_deg);
    }
    PublishFrustumVisual();
    frustum_published_ = true;

    auto eul = T_parent_optical_.Rot().Euler();
    std::ostringstream oss;
    oss << "{"
        << "\"mode_id\":\"" << req->mode_id << "\","
        << "\"arm_id\":\"" << req->arm_id << "\","
        << "\"h_deg\":" << applied_h_deg << ","
        << "\"v_deg\":" << viz_v_deg_ << ","
        << "\"width\":" << applied_w << ","
        << "\"height\":" << applied_h << ","
        << "\"update_rate\":" << (cam_sensor ? cam_sensor->UpdateRate() : 0.0) << ","
        << "\"extrinsics\":{"
        << "\"lens_m\":[" << T_parent_optical_.Pos().X() << ","
        << T_parent_optical_.Pos().Y() << "," << T_parent_optical_.Pos().Z() << "],"
        << "\"rpy\":[" << eul.X() << "," << eul.Y() << "," << eul.Z() << "]"
        << "}}";
    res->ok = true;
    res->error = "";
    res->applied_json = oss.str();
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "camera_params applied %s", res->applied_json.c_str());
  }

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr visual_pub_;
  rclcpp::Service<sensors_arm_msgs::srv::SetGazeboCamera>::SharedPtr srv_;
  rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_ci_client_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  gazebo::event::ConnectionPtr update_;
  gazebo::event::ConnectionPtr update_first_;
  std::mutex mutex_;
  std::string sensor_name_;
  std::string link_name_;
  std::string optical_link_name_;
  std::string parent_link_name_;
  std::string service_name_;
  std::string set_camera_info_service_;
  ignition::math::Pose3d T_parent_optical_;
  ignition::math::Pose3d T_optical_gazebo_;
  bool have_extrinsics_{false};
  bool frustum_published_{false};
  bool frustum_logged_{false};
  bool frustum_purged_{false};
  uint64_t frustum_tick_{0};
  double viz_h_deg_{87.0};
  double viz_v_deg_{56.0};
  double frustum_near_{0.02};
  double frustum_far_{0.28};
};

GZ_REGISTER_MODEL_PLUGIN(CameraParamsPlugin)

}  // namespace sensors_arm_gazebo
