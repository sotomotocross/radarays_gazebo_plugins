#include "radarays_gazebo_plugins/gz/radarays_optix_sensor_system.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/msgs/sdf_generator_config.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <rmagine/simulation/SimulationResults.hpp>
#include <rmagine/types/Memory.hpp>

#include "rmagine_gazebo_plugins/gz/optix_map_registry.hpp"

// DirectedWave, Signal, fresnel, back_reflection_shader, sample_cone_local,
// make_denoiser_*, angle_between/get_incidence_angle, perlin_noise (host
// side); move_waves/signal_shader/fresnel_split, RadarMaterial (GPU side):
// all from radarays_ros. Scoped to this translation unit only.
using namespace radarays_ros;

namespace radarays_gazebo_plugins
{

namespace
{
namespace rm = rmagine;
}  // namespace

static rmagine::Transform ToRmTransform(const gz::math::Pose3d &pose)
{
  rmagine::Transform T;
  T.R.x = pose.Rot().X();
  T.R.y = pose.Rot().Y();
  T.R.z = pose.Rot().Z();
  T.R.w = pose.Rot().W();
  T.t.x = pose.Pos().X();
  T.t.y = pose.Pos().Y();
  T.t.z = pose.Pos().Z();
  return T;
}

void RadaraysOptixSensorSystem::RefreshSimulator()
{
  auto &registry = rmagine_gazebo_plugins::OptixMapRegistry::Instance();
  const auto revision = registry.GetOptixMapRevision(map_key_);
  const auto map = registry.GetOptixMap(map_key_);
  if(!map)
  {
    map_.reset();
    map_mutex_.reset();
    sim_.reset();
    map_revision_ = 0;
    std::cerr << "[RadaraysOptixSensorSystem] No map available for key '" << map_key_ << "'." << std::endl;
    return;
  }

  if(!sim_ || !map_ || map != map_ || revision != map_revision_)
  {
    map_ = map;
    map_mutex_ = registry.GetMapMutex(map_key_);
    sim_ = std::make_shared<rmagine::OnDnSimulatorOptix>(map_);
    sim_->setTsb(rmagine::Transform::Identity());
    // Proactively activate the map's CUDA context here rather than letting
    // the first simulate() call do it lazily. Root-caused via a standalone
    // repro (same map construction, same multi-pass move/shade/split loop,
    // outside gz-sim entirely) that reproduced cleanly with no crash --
    // the one real difference in the failing gz-sim run was this system's
    // first simulate() call being the one to find the context not yet
    // active ("Need to activate map context"), immediately followed by a
    // cudaErrorIllegalAddress on the next device-to-host copy. Activating
    // up front sidesteps whatever in that lazy-activation path is unsafe
    // in this environment.
    map_->context()->getCudaContext()->use();
    map_revision_ = revision;
    std::cerr << "[RadaraysOptixSensorSystem] Refreshed simulator for map key '"
              << map_key_ << "' at revision " << map_revision_ << "." << std::endl;
  }
}

void RadaraysOptixSensorSystem::ResolveFrameEntity(
  const gz::sim::EntityComponentManager &_ecm)
{
  if(frame_entity_ != gz::sim::kNullEntity)
  {
    return;
  }

  _ecm.Each<gz::sim::components::Name, gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &entity,
        const gz::sim::components::Name *nameComp,
        const gz::sim::components::ParentEntity *parentComp) -> bool
    {
      if(!nameComp || !parentComp)
      {
        return true;
      }
      if(nameComp->Data() == frame_id_ && parentComp->Data() == sensor_entity_)
      {
        frame_entity_ = entity;
        if(debug_ && !frame_resolved_logged_)
        {
          std::cerr << "[RadaraysOptixSensorSystem] Resolved frame entity '" << frame_id_
                    << "' to entity " << frame_entity_ << "." << std::endl;
          frame_resolved_logged_ = true;
        }
        return false;
      }
      return true;
    });
}

static std::string GetNameAttr(const sdf::ElementPtr &elem)
{
  if(!elem)
  {
    return {};
  }
  auto attr = elem->GetAttribute("name");
  return attr ? attr->GetAsString() : std::string();
}

bool RadaraysOptixSensorSystem::EnsureWorldSdf(const gz::sim::EntityComponentManager &_ecm)
{
  if(world_sdf_)
  {
    return true;
  }

  const auto world_entity = gz::sim::worldEntity(_ecm);
  const auto world_name = gz::sim::World(world_entity).Name(_ecm);
  if(!world_name.has_value())
  {
    return false;
  }

  const std::string service = "/world/" + *world_name + "/generate_world_sdf";
  gz::msgs::SdfGeneratorConfig request;
  gz::msgs::StringMsg response;
  bool result = false;
  const bool called = transport_node_.Request(service, request, 2000, response, result);
  if(!called || !result)
  {
    if(debug_)
    {
      std::cerr << "[RadaraysOptixSensorSystem] " << service << " not available yet, will retry." << std::endl;
    }
    return false;
  }

  auto sdf_doc = std::make_shared<sdf::SDF>();
  sdf::init(sdf_doc);
  if(!sdf::readString(response.data(), sdf_doc))
  {
    std::cerr << "[RadaraysOptixSensorSystem] Failed to parse SDF from " << service << "." << std::endl;
    return false;
  }

  sdf::ElementPtr root = sdf_doc->Root();
  world_sdf_ = (root && root->GetName() == "world") ? root : root->GetElement("world");
  world_sdf_root_ = sdf_doc;

  if(debug_)
  {
    std::cerr << "[RadaraysOptixSensorSystem] world SDF "
              << (world_sdf_ ? "fetched for material lookup." : "NOT resolved from service response.")
              << std::endl;
  }
  return world_sdf_ != nullptr;
}

sdf::ElementPtr RadaraysOptixSensorSystem::FindRadaraysMaterialElement(
  const std::string &model_name,
  const std::string &link_name,
  const std::string &visual_name) const
{
  if(!world_sdf_)
  {
    return nullptr;
  }

  for(sdf::ElementPtr model_sdf = world_sdf_->GetElement("model");
      model_sdf;
      model_sdf = model_sdf->GetNextElement("model"))
  {
    if(GetNameAttr(model_sdf) != model_name)
    {
      continue;
    }
    for(sdf::ElementPtr link_sdf = model_sdf->GetElement("link");
        link_sdf;
        link_sdf = link_sdf->GetNextElement("link"))
    {
      if(GetNameAttr(link_sdf) != link_name)
      {
        continue;
      }
      for(sdf::ElementPtr vis_sdf = link_sdf->GetElement("visual");
          vis_sdf;
          vis_sdf = vis_sdf->GetNextElement("visual"))
      {
        if(GetNameAttr(vis_sdf) != visual_name)
        {
          continue;
        }
        if(vis_sdf->HasElement("radarays_material"))
        {
          return vis_sdf->GetElement("radarays_material");
        }
        return nullptr;
      }
    }
  }
  return nullptr;
}

void RadaraysOptixSensorSystem::RefreshMaterials(const gz::sim::EntityComponentManager &_ecm)
{
  materials_by_id_.clear();

  auto object_entities = rmagine_gazebo_plugins::OptixMapRegistry::Instance().GetObjectEntities(map_key_);

  if(world_sdf_ && object_entities)
  {
    for(const auto &[object_id, entity] : *object_entities)
    {
      std::string visual_name;
      std::string link_name;
      std::string model_name;
      gz::sim::Entity link_entity = gz::sim::kNullEntity;
      gz::sim::Entity model_entity = gz::sim::kNullEntity;

      if(auto nameComp = _ecm.Component<gz::sim::components::Name>(entity))
      {
        visual_name = nameComp->Data();
      }
      if(auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(entity))
      {
        link_entity = parentComp->Data();
      }
      if(link_entity != gz::sim::kNullEntity)
      {
        if(auto nameComp = _ecm.Component<gz::sim::components::Name>(link_entity))
        {
          link_name = nameComp->Data();
        }
        if(auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(link_entity))
        {
          model_entity = parentComp->Data();
        }
      }
      if(model_entity != gz::sim::kNullEntity)
      {
        if(auto nameComp = _ecm.Component<gz::sim::components::Name>(model_entity))
        {
          model_name = nameComp->Data();
        }
      }

      if(model_name.empty() || link_name.empty() || visual_name.empty())
      {
        continue;
      }

      sdf::ElementPtr mat_sdf = FindRadaraysMaterialElement(model_name, link_name, visual_name);
      if(!mat_sdf)
      {
        continue;
      }

      RadarMaterialLocal material;
      material.velocity = mat_sdf->Get<double>("velocity");
      material.ambient = mat_sdf->Get<double>("ambient");
      material.diffuse = mat_sdf->Get<double>("diffuse");
      material.specular = mat_sdf->Get<double>("specular");
      materials_by_id_[object_id + 1] = material;

      if(debug_)
      {
        std::cerr << "[RadaraysOptixSensorSystem] material for " << model_name << "/" << link_name
                  << "/" << visual_name << ": velocity=" << material.velocity
                  << " ambient=" << material.ambient << " diffuse=" << material.diffuse
                  << " specular=" << material.specular << std::endl;
      }
    }
  }

  // Upload the GPU-side materials/object_materials buffers to match. Index
  // 0 is always air; index (object_id + 1) is either a parsed
  // <radarays_material> or the shared default, for every object the map
  // system actually knows about (not just the ones with a tag).
  unsigned int max_obj_id = 0;
  bool have_objects = false;
  if(object_entities)
  {
    for(const auto &[object_id, entity] : *object_entities)
    {
      (void)entity;
      max_obj_id = std::max(max_obj_id, object_id);
      have_objects = true;
    }
  }

  const size_t n_materials = have_objects ? static_cast<size_t>(max_obj_id) + 2 : 1;
  rmagine::Memory<radarays_ros::RadarMaterial, rmagine::RAM> materials_cpu(n_materials);
  materials_cpu[0].velocity = static_cast<float>(material_air_.velocity);
  materials_cpu[0].ambient = static_cast<float>(material_air_.ambient);
  materials_cpu[0].diffuse = static_cast<float>(material_air_.diffuse);
  materials_cpu[0].specular = static_cast<float>(material_air_.specular);

  for(size_t i = 1; i < n_materials; i++)
  {
    const unsigned int obj_id = static_cast<unsigned int>(i - 1);
    auto it = materials_by_id_.find(static_cast<unsigned int>(i));
    const RadarMaterialLocal &m = (it != materials_by_id_.end()) ? it->second : material_default_;
    materials_cpu[i].velocity = static_cast<float>(m.velocity);
    materials_cpu[i].ambient = static_cast<float>(m.ambient);
    materials_cpu[i].diffuse = static_cast<float>(m.diffuse);
    materials_cpu[i].specular = static_cast<float>(m.specular);
    (void)obj_id;
  }
  materials_gpu_ = materials_cpu;

  const size_t n_objects = have_objects ? static_cast<size_t>(max_obj_id) + 1 : 0;
  rmagine::Memory<int, rmagine::RAM> object_materials_cpu(n_objects);
  for(size_t obj_id = 0; obj_id < n_objects; obj_id++)
  {
    object_materials_cpu[obj_id] = static_cast<int>(obj_id + 1);
  }
  object_materials_gpu_ = object_materials_cpu;
}

void RadaraysOptixSensorSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  sensor_entity_ = _entity;
  LoadParams(_sdf);

  if(!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  std::string node_name = "radarays_optix_sensor_system";
  if(_sdf && _sdf->HasElement("node_name"))
  {
    node_name = _sdf->Get<std::string>("node_name");
  }
  node_ = std::make_shared<rclcpp::Node>(node_name);
  image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_image_, rclcpp::SensorDataQoS());
  DeclareReconfigurableParams();

  RefreshSimulator();
}

void RadaraysOptixSensorSystem::DeclareReconfigurableParams()
{
  auto declare_ranged_double = [this](const std::string &name, double value, double min, double max)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = min;
    range.to_value = max;
    descriptor.floating_point_range.push_back(range);
    node_->declare_parameter(name, value, descriptor);
  };
  auto declare_ranged_int = [this](const std::string &name, int value, int min, int max)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = min;
    range.to_value = max;
    descriptor.integer_range.push_back(range);
    node_->declare_parameter(name, value, descriptor);
  };

  node_->declare_parameter("update_rate", update_rate_);
  node_->declare_parameter("debug", debug_);

  declare_ranged_int("n_reflections", static_cast<int>(n_reflections_), 1, 20);
  node_->declare_parameter("wave_energy_threshold", wave_energy_threshold_);

  declare_ranged_int("n_samples", static_cast<int>(n_samples_), 1, 200);
  node_->declare_parameter("beam_width_deg", beam_width_rad_ * 180.0 / M_PI);
  declare_ranged_int("beam_sample_dist", beam_sample_dist_, 0, 3);
  declare_ranged_double("beam_sample_dist_p_in_cone", beam_sample_dist_p_in_cone_, 0.0, 1.0);

  node_->declare_parameter("energy_max", energy_max_);
  node_->declare_parameter("signal_max", signal_max_);

  declare_ranged_int("signal_denoising", signal_denoising_, 0, 3);
  declare_ranged_int("signal_denoising_width", signal_denoising_width_, 1, 500);
  declare_ranged_double("signal_denoising_mode_frac", signal_denoising_mode_frac_, 0.0, 1.0);

  declare_ranged_int("ambient_noise", ambient_noise_, 0, 2);
  declare_ranged_double("ambient_noise_at_signal_0", ambient_noise_at_signal_0_, 0.0, 1.0);
  declare_ranged_double("ambient_noise_at_signal_1", ambient_noise_at_signal_1_, 0.0, 1.0);
  declare_ranged_double("ambient_noise_energy_max", ambient_noise_energy_max_, 0.0, 1.0);
  declare_ranged_double("ambient_noise_energy_min", ambient_noise_energy_min_, 0.0, 1.0);
  declare_ranged_double("ambient_noise_energy_loss", ambient_noise_energy_loss_, 0.0, 1.0);

  node_->declare_parameter("record_multi_reflection", record_multi_reflection_);
  node_->declare_parameter("record_multi_path", record_multi_path_);
  declare_ranged_double("multipath_threshold", multipath_threshold_, 0.0, 1.0);

  declare_ranged_int("n_cells", static_cast<int>(n_cells_), 1, 100000);
  node_->declare_parameter("range_max", range_max_);

  param_callback_handle_ = node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &parameters)
    {
      return OnSetParameters(parameters);
    });
}

rcl_interfaces::msg::SetParametersResult RadaraysOptixSensorSystem::OnSetParameters(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for(const auto &param : parameters)
  {
    const auto &name = param.get_name();
    if(name == "n_reflections" && param.as_int() < 1)
    {
      result.successful = false;
      result.reason = "n_reflections must be >= 1";
    }
    else if(name == "n_samples" && param.as_int() < 1)
    {
      result.successful = false;
      result.reason = "n_samples must be >= 1";
    }
    else if(name == "n_cells" && param.as_int() < 1)
    {
      result.successful = false;
      result.reason = "n_cells must be >= 1";
    }
    else if(name == "range_max" && param.as_double() <= 0.0)
    {
      result.successful = false;
      result.reason = "range_max must be > 0";
    }
  }
  if(!result.successful)
  {
    return result;
  }

  bool beam_dirty = false;
  for(const auto &param : parameters)
  {
    const auto &name = param.get_name();
    if(name == "update_rate") { update_rate_ = param.as_double(); }
    else if(name == "debug") { debug_ = param.as_bool(); }
    else if(name == "n_reflections") { n_reflections_ = static_cast<unsigned int>(param.as_int()); }
    else if(name == "wave_energy_threshold") { wave_energy_threshold_ = param.as_double(); }
    else if(name == "n_samples") { n_samples_ = static_cast<unsigned int>(param.as_int()); beam_dirty = true; }
    else if(name == "beam_width_deg") { beam_width_rad_ = param.as_double() * M_PI / 180.0; beam_dirty = true; }
    else if(name == "beam_sample_dist") { beam_sample_dist_ = static_cast<int>(param.as_int()); beam_dirty = true; }
    else if(name == "beam_sample_dist_p_in_cone") { beam_sample_dist_p_in_cone_ = param.as_double(); beam_dirty = true; }
    else if(name == "energy_max") { energy_max_ = param.as_double(); }
    else if(name == "signal_max") { signal_max_ = param.as_double(); }
    else if(name == "signal_denoising") { signal_denoising_ = static_cast<int>(param.as_int()); }
    else if(name == "signal_denoising_width") { signal_denoising_width_ = static_cast<int>(param.as_int()); }
    else if(name == "signal_denoising_mode_frac") { signal_denoising_mode_frac_ = param.as_double(); }
    else if(name == "ambient_noise") { ambient_noise_ = static_cast<int>(param.as_int()); }
    else if(name == "ambient_noise_at_signal_0") { ambient_noise_at_signal_0_ = param.as_double(); }
    else if(name == "ambient_noise_at_signal_1") { ambient_noise_at_signal_1_ = param.as_double(); }
    else if(name == "ambient_noise_energy_max") { ambient_noise_energy_max_ = param.as_double(); }
    else if(name == "ambient_noise_energy_min") { ambient_noise_energy_min_ = param.as_double(); }
    else if(name == "ambient_noise_energy_loss") { ambient_noise_energy_loss_ = param.as_double(); }
    else if(name == "record_multi_reflection") { record_multi_reflection_ = param.as_bool(); }
    else if(name == "record_multi_path") { record_multi_path_ = param.as_bool(); }
    else if(name == "multipath_threshold") { multipath_threshold_ = param.as_double(); }
    else if(name == "n_cells") { n_cells_ = static_cast<unsigned int>(param.as_int()); }
    else if(name == "range_max") { range_max_ = param.as_double(); }
  }

  if(beam_dirty)
  {
    beam_samples_local_.clear();
  }

  return result;
}

void RadaraysOptixSensorSystem::LoadParams(const std::shared_ptr<const sdf::Element> &_sdf)
{
  if(!_sdf)
  {
    return;
  }

  if(_sdf->HasElement("map_key")) { map_key_ = _sdf->Get<std::string>("map_key"); }
  if(_sdf->HasElement("frame")) { frame_id_ = _sdf->Get<std::string>("frame"); }
  if(_sdf->HasElement("parent_frame")) { parent_frame_id_ = _sdf->Get<std::string>("parent_frame"); }
  if(_sdf->HasElement("topic_image")) { topic_image_ = _sdf->Get<std::string>("topic_image"); }
  if(_sdf->HasElement("update_rate")) { update_rate_ = _sdf->Get<double>("update_rate"); }
  if(_sdf->HasElement("debug")) { debug_ = _sdf->Get<bool>("debug"); }

  if(_sdf->HasElement("samples")) { n_angles_ = _sdf->Get<unsigned int>("samples"); }
  if(_sdf->HasElement("min_angle")) { min_angle_ = _sdf->Get<double>("min_angle"); }
  if(_sdf->HasElement("max_angle")) { max_angle_ = _sdf->Get<double>("max_angle"); }
  if(_sdf->HasElement("range_max")) { range_max_ = _sdf->Get<double>("range_max"); }
  if(_sdf->HasElement("n_cells")) { n_cells_ = _sdf->Get<unsigned int>("n_cells"); }

  if(_sdf->HasElement("beam_samples")) { n_samples_ = _sdf->Get<unsigned int>("beam_samples"); }
  if(_sdf->HasElement("beam_width_deg"))
  {
    beam_width_rad_ = _sdf->Get<double>("beam_width_deg") * M_PI / 180.0;
  }
  if(_sdf->HasElement("beam_sample_dist")) { beam_sample_dist_ = _sdf->Get<int>("beam_sample_dist"); }
  if(_sdf->HasElement("n_reflections")) { n_reflections_ = _sdf->Get<unsigned int>("n_reflections"); }
  if(_sdf->HasElement("wave_energy_threshold")) { wave_energy_threshold_ = _sdf->Get<double>("wave_energy_threshold"); }

  if(_sdf->HasElement("energy_max")) { energy_max_ = _sdf->Get<double>("energy_max"); }
  if(_sdf->HasElement("signal_max")) { signal_max_ = _sdf->Get<double>("signal_max"); }

  if(_sdf->HasElement("signal_denoising")) { signal_denoising_ = _sdf->Get<int>("signal_denoising"); }
  if(_sdf->HasElement("signal_denoising_width")) { signal_denoising_width_ = _sdf->Get<int>("signal_denoising_width"); }
  if(_sdf->HasElement("signal_denoising_mode_frac")) { signal_denoising_mode_frac_ = _sdf->Get<double>("signal_denoising_mode_frac"); }

  if(_sdf->HasElement("ambient_noise")) { ambient_noise_ = _sdf->Get<int>("ambient_noise"); }
  if(_sdf->HasElement("record_multi_reflection")) { record_multi_reflection_ = _sdf->Get<bool>("record_multi_reflection"); }
  if(_sdf->HasElement("record_multi_path")) { record_multi_path_ = _sdf->Get<bool>("record_multi_path"); }
  if(_sdf->HasElement("multipath_threshold")) { multipath_threshold_ = _sdf->Get<double>("multipath_threshold"); }
}

std::vector<float> RadaraysOptixSensorSystem::BuildDenoiserWeights(int &mode_out) const
{
  mode_out = 0;
  if(signal_denoising_ <= 0)
  {
    return {};
  }

  mode_out = static_cast<int>(signal_denoising_mode_frac_ * signal_denoising_width_);

  std::vector<float> weights;
  if(signal_denoising_ == 1)
  {
    weights = make_denoiser_triangular(signal_denoising_width_, mode_out);
  }
  else if(signal_denoising_ == 2)
  {
    weights = make_denoiser_gaussian(signal_denoising_width_, mode_out);
  }
  else if(signal_denoising_ == 3)
  {
    weights = make_denoiser_maxwell_boltzmann(signal_denoising_width_, mode_out);
  }

  if(!weights.empty() && weights[mode_out] > 0.0f)
  {
    const float mode_val = weights[mode_out];
    for(float &w : weights) { w /= mode_val; }
  }

  return weights;
}

void RadaraysOptixSensorSystem::SimulateFrame(
  const rmagine::Transform &Tsm,
  const std::vector<float> &denoising_weights,
  int denoising_mode,
  std::vector<uint8_t> &image_data)
{
  namespace rm = rmagine;

  const size_t n_rays = static_cast<size_t>(n_angles_) * n_samples_;

  if(debug_ && n_reflections_ > 6)
  {
    // Each pass doubles the wave count (no compaction of dropped-below-
    // threshold waves, unlike the CPU/Embree version) -- 2^n_reflections_
    // grows fast. This matches radarays_ros's RadarGPU, which only ever
    // hardcodes 3 passes; SDF/param range still allows up to 20 to match
    // the CPU system's surface, but large values here cost real VRAM.
    std::cerr << "[RadaraysOptixSensorSystem] n_reflections=" << n_reflections_
              << " means up to " << (1u << n_reflections_) << "x the base wave count in VRAM." << std::endl;
  }

  // Build pass-0 waves host-side (same construction as radarays_ros's
  // RadarGPU), then upload.
  rm::OnDnModel waves;
  waves.width = n_angles_;
  waves.height = n_samples_;
  waves.range.min = 0.0f;
  waves.range.max = static_cast<float>(range_max_);
  waves.origs.resize(n_rays);
  waves.dirs.resize(n_rays);
  rm::Memory<DirectedWaveAttributes> attrs(n_rays);

  for(size_t angle_id = 0; angle_id < n_angles_; angle_id++)
  {
    const double theta = (n_angles_ > 1)
      ? min_angle_ + (max_angle_ - min_angle_) * static_cast<double>(angle_id) / static_cast<double>(n_angles_ - 1)
      : min_angle_;
    const rm::EulerAngles Ras{0.0f, 0.0f, static_cast<float>(theta)};

    for(size_t sample_id = 0; sample_id < n_samples_; sample_id++)
    {
      const size_t buf_id = waves.getBufferId(sample_id, angle_id);
      const DirectedWave &local = beam_samples_local_[sample_id];
      waves.dirs[buf_id] = Ras * local.ray.dir;
      waves.origs[buf_id] = {0.0, 0.0, 0.0};

      DirectedWaveAttributes attr;
      attr.energy = local.energy;
      attr.polarization = local.polarization;
      attr.frequency = local.frequency;
      attr.velocity = local.velocity;
      attr.material_id = local.material_id;
      attr.time = local.time;
      attrs[buf_id] = attr;
    }
  }

  rm::OnDnModel_<rm::VRAM_CUDA> waves_gpu;
  waves_gpu.width = waves.width;
  waves_gpu.height = waves.height;
  waves_gpu.range = waves.range;
  waves_gpu.origs = waves.origs;
  waves_gpu.dirs = waves.dirs;
  rm::Memory<DirectedWaveAttributes, rm::VRAM_CUDA> attrs_gpu = attrs;

  rm::Memory<rm::Transform> Tsms(1);
  Tsms[0] = Tsm;

  using ResT = rm::Bundle<
    rm::Hits<rm::VRAM_CUDA>,
    rm::Ranges<rm::VRAM_CUDA>,
    rm::Normals<rm::VRAM_CUDA>,
    rm::ObjectIds<rm::VRAM_CUDA>>;

  std::vector<rm::Memory<Signal, rm::RAM>> signals_cpu_per_pass;
  std::vector<rm::Memory<uint8_t, rm::RAM>> hits_cpu_per_pass;
  std::vector<unsigned int> height_per_pass;

  sim_->setModel(waves_gpu);

  for(unsigned int pass = 0; pass < n_reflections_; pass++)
  {
    ResT results;
    rm::resize_memory_bundle<rm::VRAM_CUDA>(results, waves_gpu.width, waves_gpu.height, 1);
    {
      // See map_mutex_'s declaration (header) -- must not race the map
      // system's in-place scene mutation.
      std::shared_lock<std::shared_mutex> lock;
      if(map_mutex_)
      {
        lock = std::shared_lock<std::shared_mutex>(*map_mutex_);
      }
      sim_->simulate(Tsms, results);
    }

    move_waves(waves_gpu.origs, waves_gpu.dirs, attrs_gpu, results.ranges, results.hits);

    rm::Memory<Signal, rm::VRAM_CUDA> signals_gpu(waves_gpu.size());
    signal_shader(
      materials_gpu_, object_materials_gpu_, 0,
      waves_gpu.dirs, attrs_gpu, results.hits, results.normals, results.object_ids,
      signals_gpu);

    rm::Memory<Signal, rm::RAM> signals_cpu = signals_gpu;
    rm::Memory<uint8_t, rm::RAM> hits_cpu = results.hits;
    height_per_pass.push_back(waves_gpu.height);
    signals_cpu_per_pass.push_back(std::move(signals_cpu));
    hits_cpu_per_pass.push_back(std::move(hits_cpu));

    if(pass + 1 < n_reflections_)
    {
      rm::OnDnModel_<rm::VRAM_CUDA> waves_gpu_next;
      waves_gpu_next.width = waves_gpu.width;
      waves_gpu_next.height = waves_gpu.height * 2;
      waves_gpu_next.range = waves_gpu.range;
      waves_gpu_next.origs.resize(waves_gpu.origs.size() * 2);
      waves_gpu_next.dirs.resize(waves_gpu.dirs.size() * 2);
      rm::Memory<DirectedWaveAttributes, rm::VRAM_CUDA> attrs_gpu_next(attrs_gpu.size() * 2);

      auto lo = waves_gpu_next.origs(0, waves_gpu.origs.size());
      auto ld = waves_gpu_next.dirs(0, waves_gpu.dirs.size());
      auto la = attrs_gpu_next(0, attrs_gpu.size());

      auto ro = waves_gpu_next.origs(waves_gpu.origs.size(), waves_gpu.origs.size() * 2);
      auto rd = waves_gpu_next.dirs(waves_gpu.dirs.size(), waves_gpu.dirs.size() * 2);
      auto ra = attrs_gpu_next(attrs_gpu.size(), attrs_gpu.size() * 2);

      fresnel_split(
        materials_gpu_, object_materials_gpu_, 0,
        waves_gpu.origs, waves_gpu.dirs, attrs_gpu,
        results.hits, results.normals, results.object_ids,
        lo, ld, la,
        ro, rd, ra);

      waves_gpu = waves_gpu_next;
      attrs_gpu = attrs_gpu_next;
      sim_->setModel(waves_gpu);
    }
  }
  cudaDeviceSynchronize();

  // Tail: denoising + ambient noise + normalization to bytes. Identical
  // math to RadaraysEmbreeSensorSystem::SimulateFrame()'s tail -- this is
  // cheap, backend-agnostic host-side work either way. Sequential (not
  // parallelized across angles) to keep rng_ usage single-threaded, same
  // as the Embree version.
  const double resolution = range_max_ / static_cast<double>(n_cells_);
  std::uniform_real_distribution<float> dist_uni(0.0f, 1.0f);

  for(size_t angle_id = 0; angle_id < n_angles_; angle_id++)
  {
    std::vector<float> slice(n_cells_, 0.0f);
    float max_val = 0.0f;

    for(size_t pass = 0; pass < signals_cpu_per_pass.size(); pass++)
    {
      const unsigned int height = height_per_pass[pass];
      const auto &signals_cpu = signals_cpu_per_pass[pass];
      const auto &hits_cpu = hits_cpu_per_pass[pass];

      for(unsigned int sample_id = 0; sample_id < height; sample_id++)
      {
        const unsigned int signal_id = sample_id * n_angles_ + static_cast<unsigned int>(angle_id);
        if(!hits_cpu[signal_id])
        {
          continue;
        }
        const Signal &signal = signals_cpu[signal_id];

        const double half_time = signal.time / 2.0;
        const double signal_dist = 0.3 * half_time;
        const int cell = static_cast<int>(signal_dist / resolution);
        if(cell < 0 || cell >= static_cast<int>(n_cells_))
        {
          continue;
        }

        if(signal_denoising_ > 0 && !denoising_weights.empty())
        {
          for(size_t vid = 0; vid < denoising_weights.size(); vid++)
          {
            const int glob_id = static_cast<int>(vid) + cell - denoising_mode;
            if(glob_id > 0 && glob_id < static_cast<int>(n_cells_))
            {
              slice[glob_id] += static_cast<float>(signal.strength) * denoising_weights[vid];
              max_val = std::max(max_val, slice[glob_id]);
            }
          }
        }
        else
        {
          slice[cell] = std::max(slice[cell], static_cast<float>(signal.strength));
          max_val = std::max(max_val, slice[cell]);
        }
      }
    }

    for(float &v : slice) { v *= static_cast<float>(energy_max_); }
    max_val *= static_cast<float>(energy_max_);

    if(ambient_noise_ > 0)
    {
      constexpr double scale_low = 0.05;
      constexpr double scale_high = 0.2;
      const double random_begin = dist_uni(rng_) * 1000.0;

      for(size_t i = 0; i < slice.size(); i++)
      {
        const float signal = slice[i];
        double p = 0.0;
        if(ambient_noise_ == 1)
        {
          p = dist_uni(rng_);
        }
        else if(ambient_noise_ == 2)
        {
          const double p_low = perlin_noise(random_begin + static_cast<double>(i) * scale_low, static_cast<double>(angle_id) * scale_low);
          const double p_high = perlin_noise(random_begin + static_cast<double>(i) * scale_high, static_cast<double>(angle_id) * scale_high);
          p = 0.9 * p_low + 0.1 * p_high;
        }

        const float signal_amp = max_val;
        const float signal_norm = (signal_amp > 0.0f) ? (1.0f - (signal / signal_amp)) : 1.0f;

        const float noise_at_0 = signal_amp * static_cast<float>(ambient_noise_at_signal_0_);
        const float noise_at_1 = signal_amp * static_cast<float>(ambient_noise_at_signal_1_);
        const float signal_pow4 = std::pow(signal_norm, 4.0f);
        const float noise_amp = signal_pow4 * noise_at_0 + (1.0f - signal_pow4) * noise_at_1;

        const float noise_energy_max = signal_amp * static_cast<float>(ambient_noise_energy_max_);
        const float noise_energy_min = signal_amp * static_cast<float>(ambient_noise_energy_min_);
        const float energy_loss = static_cast<float>(ambient_noise_energy_loss_);

        float y_noise = noise_amp * static_cast<float>(p);
        const float x = (static_cast<float>(i) + 0.5f) * static_cast<float>(resolution);
        y_noise = y_noise + (noise_energy_max - noise_energy_min) * std::exp(-energy_loss * x) + noise_energy_min;
        y_noise = std::abs(y_noise);

        slice[i] = signal + y_noise;
        max_val = std::max(max_val, slice[i]);
      }
    }

    if(max_val > 0.0f)
    {
      const float scale = static_cast<float>(signal_max_) / max_val;
      for(float &v : slice) { v *= scale; }
    }

    for(size_t row = 0; row < n_cells_; row++)
    {
      const float clamped = std::clamp(slice[row], 0.0f, 255.0f);
      image_data[row * n_angles_ + angle_id] = static_cast<uint8_t>(clamped);
    }
  }
}

void RadaraysOptixSensorSystem::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if(_info.paused)
  {
    return;
  }

  rclcpp::spin_some(node_);

  ResolveFrameEntity(_ecm);

  const auto sim_now = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime);
  const auto base_pose = gz::sim::worldPose(sensor_entity_, _ecm);
  gz::math::Pose3d sensor_pose = base_pose;
  if(frame_entity_ != gz::sim::kNullEntity)
  {
    sensor_pose = gz::sim::worldPose(frame_entity_, _ecm);
    local_sensor_pose_ = base_pose.Inverse() * sensor_pose;
  }
  const rclcpp::Time stamp(sim_now.count(), RCL_ROS_TIME);

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));
  if(has_published_ && (sim_now - last_pub_time_) < period)
  {
    return;
  }

  RefreshSimulator();
  if(!map_ || !sim_)
  {
    return;
  }

  if(materials_map_revision_ != map_revision_)
  {
    // RefreshMaterials() must run even if the world SDF isn't fetched yet:
    // it's what sizes materials_gpu_/object_materials_gpu_ to match the
    // map's actual object count (it degrades gracefully to all-default
    // materials when world_sdf_ is null, it just can't look up
    // <radarays_material> tags yet). Only materials_map_revision_ itself
    // is gated on EnsureWorldSdf succeeding, so this keeps retrying the
    // SDF fetch every tick until it comes up -- matching the Embree
    // version's retry intent -- without ever leaving the GPU buffers
    // wrongly-sized in the meantime (a previous version of this code did
    // exactly that: an empty object_materials_gpu_ against a map that
    // already had real geometry, causing signal_shader/fresnel_split to
    // read out of bounds -- illegal GPU memory access, not a hang).
    const bool got_sdf = EnsureWorldSdf(_ecm);
    RefreshMaterials(_ecm);
    if(got_sdf)
    {
      materials_map_revision_ = map_revision_;
    }
  }

  if(beam_samples_local_.empty())
  {
    DirectedWave wave;
    wave.energy = 1.0;
    wave.polarization = 0.5;
    wave.frequency = 76.5;
    wave.velocity = material_air_.velocity;
    wave.material_id = 0;
    wave.time = 0.0;
    wave.ray.orig = {0.0, 0.0, 0.0};
    wave.ray.dir = {1.0, 0.0, 0.0};

    beam_samples_local_ = sample_cone_local(
      wave, static_cast<float>(beam_width_rad_), static_cast<int>(n_samples_),
      beam_sample_dist_, static_cast<float>(beam_sample_dist_p_in_cone_));
  }

  int denoising_mode = 0;
  const std::vector<float> denoising_weights = BuildDenoiserWeights(denoising_mode);

  const rmagine::Transform Tsm = ToRmTransform(sensor_pose);

  std::vector<uint8_t> image_data(static_cast<size_t>(n_cells_) * n_angles_, 0);
  SimulateFrame(Tsm, denoising_weights, denoising_mode, image_data);

  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = frame_id_;
  image.height = n_cells_;
  image.width = n_angles_;
  image.encoding = "mono8";
  image.is_bigendian = 0;
  image.step = n_angles_;
  image.data = std::move(image_data);
  image_pub_->publish(image);

  last_pub_time_ = sim_now;
  has_published_ = true;
}

}  // namespace radarays_gazebo_plugins

GZ_ADD_PLUGIN(radarays_gazebo_plugins::RadaraysOptixSensorSystem,
              gz::sim::System,
              radarays_gazebo_plugins::RadaraysOptixSensorSystem::ISystemConfigure,
              radarays_gazebo_plugins::RadaraysOptixSensorSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(radarays_gazebo_plugins::RadaraysOptixSensorSystem,
                    "radarays_optix_sensor_system")
