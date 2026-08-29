#ifndef RADARAYS_GAZEBO_PLUGINS_GZ_RADARAYS_EMBREE_SENSOR_SYSTEM_HPP
#define RADARAYS_GAZEBO_PLUGINS_GZ_RADARAYS_EMBREE_SENSOR_SYSTEM_HPP

#include <memory>
#include <random>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>

// The radar-physics math (Ray/DirectedWave, fresnel, back-reflection
// shader, cone sampling, denoising) lives in radarays_ros now, not a local
// copy -- see MIGRATION_HANDOFF.md, "Two independent physics copies", for
// why there used to be two and why there's only one now. Perlin noise
// (image_algorithms.h) and erfinvf (radar_math.h) are pulled in the same
// way.
#include <radarays_ros/radar_types.h>
#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/radar_math.h>
#include <radarays_ros/image_algorithms.h>

namespace radarays_gazebo_plugins
{

// This package's own material representation -- distinct from
// radarays_ros's ROS-message-based radarays_ros::msg::RadarMaterial, since
// materials here are resolved from SDF tags, not a message. Same four
// fields either way.
struct RadarMaterial
{
  double velocity;
  double ambient;
  double diffuse;
  double specular;
};

// Harmonic-native replacement for the scan-derived /radar/image bridge.
// Reuses the Embree map delegated by rmagine_gazebo_plugins (MapRegistry)
// and radarays_ros's radar-physics core (multi-bounce Fresnel
// reflection/refraction, cone-sampled beams, signal denoising, ambient
// noise) directly as a polar image publisher.
//
// Per-visual materials: <radarays_material> tags on a <visual> are read
// directly from the SDF DOM, matching the original Classic lookup. gz-sim's
// ECM only exposes typed components (Name, Pose, Geometry, ...) for a
// visual, not arbitrary custom child elements, so this can't be done
// through components alone -- and this plugin's own SDF element does not
// carry a live parent chain up to <world> (checked: GetParent() dead-ends,
// and the world entity has no SourceFilePath component either), so the
// world DOM is instead fetched once via gz-sim's own
// `/world/<name>/generate_world_sdf` service (gz.msgs.SdfGeneratorConfig ->
// gz.msgs.StringMsg), parsed, and cached. Which entity a given raycast hit
// belongs to comes from rmagine_gazebo_plugins's MapRegistry (object id ->
// Entity, populated by the map system that already builds the Embree
// scene); this system then resolves that entity's model/link/visual name
// path via the ECM and looks it up in the fetched SDF tree. Any hit object
// without a tag falls back to one default reflective material, matching
// the Classic plugin's own fallback.
class RadaraysEmbreeSensorSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  RadaraysEmbreeSensorSystem() = default;
  ~RadaraysEmbreeSensorSystem() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  void LoadParams(const std::shared_ptr<const sdf::Element> &_sdf);
  void RefreshSimulator();
  void ResolveFrameEntity(const gz::sim::EntityComponentManager &_ecm);
  std::vector<float> BuildDenoiserWeights(int &mode_out) const;

  // Fetches and caches the world's SDF DOM via the generate_world_sdf
  // service so RefreshMaterials() can walk <model>/<link>/<visual>
  // elements (including custom, non-component tags like
  // <radarays_material>). Returns false (and leaves world_sdf_ unset,
  // safe to retry later) if the service isn't up yet.
  bool EnsureWorldSdf(const gz::sim::EntityComponentManager &_ecm);

  // Re-reads per-visual radar materials. Only needs to run when the map
  // geometry actually changed (map_revision_ bumped), not every publish.
  void RefreshMaterials(const gz::sim::EntityComponentManager &_ecm);

  sdf::ElementPtr FindRadaraysMaterialElement(
    const std::string &model_name,
    const std::string &link_name,
    const std::string &visual_name) const;

  const RadarMaterial &MaterialFor(unsigned int material_id) const;

  // Live tuning of the radar-physics parameters, the ROS 2 equivalent of
  // Classic's dynamic_reconfigure/rqt_reconfigure (RadarModelConfig).
  // Structural params (frame/topic names, angular sampling, map key) stay
  // SDF-only -- those change the sensor's shape, not just its tuning.
  void DeclareReconfigurableParams();
  rcl_interfaces::msg::SetParametersResult OnSetParameters(
    const std::vector<rclcpp::Parameter> &parameters);

  // Simulates every angle's beam cone in one batched Embree query per
  // reflection pass (instead of one query per angle), then does the
  // (cheap, embree-free) signal-to-image conversion per angle. Batching is
  // valid because each angle only differs by a rotation about the sensor's
  // Z axis with zero translation, so that rotation can be pre-baked into
  // each wave's local direction and the whole batch shares one Tsm.
  void SimulateFrame(
    const rmagine::Transform &Tsm,
    const std::vector<float> &denoising_weights,
    int denoising_mode,
    std::vector<uint8_t> &image_data);

  gz::sim::Entity sensor_entity_{gz::sim::kNullEntity};
  gz::sim::Entity frame_entity_{gz::sim::kNullEntity};

  // -- SDF params --
  std::string map_key_{"default"};
  std::string parent_frame_id_{"world"};
  std::string frame_id_{"radar"};
  std::string topic_image_{"radar/image"};
  double update_rate_{4.0};
  bool debug_{false};

  unsigned int n_angles_{400};
  double min_angle_{-1.0472};
  double max_angle_{1.0472};
  double range_max_{100.0};
  unsigned int n_cells_{1024};

  unsigned int n_samples_{10};
  double beam_width_rad_{0.035};
  int beam_sample_dist_{1};
  double beam_sample_dist_p_in_cone_{0.99};
  unsigned int n_reflections_{3};
  double wave_energy_threshold_{0.001};

  double energy_max_{0.5};
  double signal_max_{255.0};

  // 0 = none, 1 = triangular, 2 = gaussian, 3 = maxwell-boltzmann
  int signal_denoising_{1};
  int signal_denoising_width_{23};
  double signal_denoising_mode_frac_{0.1};

  // 0 = none, 1 = uniform, 2 = perlin
  int ambient_noise_{2};
  double ambient_noise_at_signal_0_{0.05};
  double ambient_noise_at_signal_1_{0.01};
  double ambient_noise_energy_max_{0.08};
  double ambient_noise_energy_min_{0.05};
  double ambient_noise_energy_loss_{0.05};

  bool record_multi_reflection_{true};
  bool record_multi_path_{false};
  double multipath_threshold_{0.5};

  // material_id 0 == air. Any other id is (object_id + 1); materials_by_id_
  // holds the ones parsed from a <radarays_material> tag, material_default_
  // is the fallback for hit objects without one.
  RadarMaterial material_air_{0.3, 1.0, 0.0, 1.0};
  RadarMaterial material_default_{0.0, 1.0, 0.0, 3000.0};
  std::unordered_map<unsigned int, RadarMaterial> materials_by_id_;
  sdf::ElementPtr world_sdf_;
  sdf::SDFPtr world_sdf_root_;  // keeps world_sdf_'s backing document alive
  gz::transport::Node transport_node_;
  uint64_t materials_map_revision_{0};

  rmagine::EmbreeMapPtr map_;
  rmagine::OnDnSimulatorEmbreePtr sim_;
  // rmagine_gazebo_plugins's map system now mutates its persistent
  // EmbreeScene in place (add/remove/move geometry, then commit()) across
  // several steps of its own PostUpdate, instead of one atomic pointer
  // swap -- fetched from MapRegistry alongside map_ (same key), a
  // shared_lock around every simulate() call (paired with the map
  // system's own unique_lock around its whole sync block) is required so
  // a raycast never runs concurrently with a scene mutation.
  std::shared_ptr<std::shared_mutex> map_mutex_;
  uint64_t map_revision_{0};
  gz::math::Pose3d local_sensor_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool frame_resolved_logged_{false};

  std::vector<radarays_ros::DirectedWave> beam_samples_local_;
  std::mt19937 rng_{std::random_device{}()};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::chrono::nanoseconds last_pub_time_{0};
  bool has_published_{false};
};

}  // namespace radarays_gazebo_plugins

#endif  // RADARAYS_GAZEBO_PLUGINS_GZ_RADARAYS_EMBREE_SENSOR_SYSTEM_HPP
