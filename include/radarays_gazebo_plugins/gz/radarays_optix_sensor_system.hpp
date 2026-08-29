#ifndef RADARAYS_GAZEBO_PLUGINS_GZ_RADARAYS_OPTIX_SENSOR_SYSTEM_HPP
#define RADARAYS_GAZEBO_PLUGINS_GZ_RADARAYS_OPTIX_SENSOR_SYSTEM_HPP

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

#include <rmagine/map/OptixMap.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>
#include <rmagine/types/MemoryCuda.hpp>

// Pure radar-physics math (fresnel, back-reflection shader, cone sampling,
// denoising, Perlin noise) from radarays_ros -- see the Embree version of
// this file for why. The GPU-specific CUDA kernels (move_waves,
// signal_shader, fresnel_split) and the plain POD RadarMaterial come from
// radarays_ros's radarays_gpu target -- see MIGRATION_HANDOFF.md, "radarays
// GPU radar path".
#include <radarays_ros/radar_types.h>
#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/radar_math.h>
#include <radarays_ros/image_algorithms.h>
#include <radarays_ros/radar_algorithms.cuh>

namespace radarays_gazebo_plugins
{

// GPU/OptiX counterpart of RadaraysEmbreeSensorSystem. Same SDF/ROS 2
// parameter surface and per-visual <radarays_material> SDF lookup (reused
// almost verbatim); the physics loop is the real difference -- it runs on
// the GPU via radarays_ros's CUDA kernels instead of a host-side per-wave
// loop against Embree. See the .cpp for the multi-pass buffer strategy.
class RadaraysOptixSensorSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  RadaraysOptixSensorSystem() = default;
  ~RadaraysOptixSensorSystem() override = default;

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

  bool EnsureWorldSdf(const gz::sim::EntityComponentManager &_ecm);
  void RefreshMaterials(const gz::sim::EntityComponentManager &_ecm);
  sdf::ElementPtr FindRadaraysMaterialElement(
    const std::string &model_name,
    const std::string &link_name,
    const std::string &visual_name) const;

  void DeclareReconfigurableParams();
  rcl_interfaces::msg::SetParametersResult OnSetParameters(
    const std::vector<rclcpp::Parameter> &parameters);

  // Runs the whole multi-pass GPU physics loop (move -> shade -> fresnel
  // split, doubling the wave count each pass, mirroring radarays_ros's
  // RadarGPU) then downloads signals+hits and does the (cheap, GPU-free)
  // denoising/ambient-noise/image-normalization tail, identical math to
  // the Embree version's SimulateFrame().
  void SimulateFrame(
    const rmagine::Transform &Tsm,
    const std::vector<float> &denoising_weights,
    int denoising_mode,
    std::vector<uint8_t> &image_data);

  gz::sim::Entity sensor_entity_{gz::sim::kNullEntity};
  gz::sim::Entity frame_entity_{gz::sim::kNullEntity};

  // -- SDF params (identical surface to RadaraysEmbreeSensorSystem) --
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

  int signal_denoising_{1};
  int signal_denoising_width_{23};
  double signal_denoising_mode_frac_{0.1};

  int ambient_noise_{2};
  double ambient_noise_at_signal_0_{0.05};
  double ambient_noise_at_signal_1_{0.01};
  double ambient_noise_energy_max_{0.08};
  double ambient_noise_energy_min_{0.05};
  double ambient_noise_energy_loss_{0.05};

  bool record_multi_reflection_{true};
  bool record_multi_path_{false};
  double multipath_threshold_{0.5};

  // material_id 0 == air. Any other id is (object_id + 1). Kept as the
  // package's own plain struct (matches the Embree version) for the SDF
  // parsing side; converted to radarays_ros::RadarMaterial (a different,
  // GPU-buffer-safe plain struct with the same 4 fields) when uploaded.
  struct RadarMaterialLocal
  {
    double velocity;
    double ambient;
    double diffuse;
    double specular;
  };
  RadarMaterialLocal material_air_{0.3, 1.0, 0.0, 1.0};
  RadarMaterialLocal material_default_{0.0, 1.0, 0.0, 3000.0};
  std::unordered_map<unsigned int, RadarMaterialLocal> materials_by_id_;
  sdf::ElementPtr world_sdf_;
  sdf::SDFPtr world_sdf_root_;
  gz::transport::Node transport_node_;
  uint64_t materials_map_revision_{0};

  // GPU-side materials, rebuilt whenever materials_by_id_ changes (i.e.
  // whenever the map/scene revision changes) -- not every frame.
  rmagine::Memory<radarays_ros::RadarMaterial, rmagine::VRAM_CUDA> materials_gpu_;
  rmagine::Memory<int, rmagine::VRAM_CUDA> object_materials_gpu_;

  rmagine::OptixMapPtr map_;
  rmagine::OnDnSimulatorOptixPtr sim_;
  // See the identical member on RadaraysEmbreeSensorSystem (Embree header)
  // -- same rationale, GPU side: rmagine_gazebo_plugins's map system
  // mutates its persistent scene in place now, so a shared_lock around
  // every simulate() call is required.
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

#endif  // RADARAYS_GAZEBO_PLUGINS_GZ_RADARAYS_OPTIX_SENSOR_SYSTEM_HPP
