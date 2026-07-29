#include "radarays_gazebo_plugins/gz/radarays_embree_sensor_system.hpp"

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

#include "rmagine_gazebo_plugins/gz/map_registry.hpp"

// DirectedWave, Signal, fresnel, back_reflection_shader, sample_cone_local,
// make_denoiser_*, angle_between/get_incidence_angle, perlin_noise: all
// from radarays_ros now (see this file's .hpp for why). Scoped to this
// translation unit only, not the header.
using namespace radarays_ros;

namespace radarays_gazebo_plugins
{

namespace
{
// radarays_ros::make_model() hardcodes range.max to 1000.0; this system
// needs it configurable (range_max_, an SDF param), so this stays a small
// local wrapper rather than something to unify.
rmagine::OnDnModel MakeOnDnModel(const std::vector<DirectedWave> &waves, double range_max)
{
  rmagine::OnDnModel model;
  model.width = static_cast<uint32_t>(waves.size());
  model.height = 1;
  model.range.min = 0.0;
  model.range.max = static_cast<float>(range_max);
  model.origs.resize(model.width);
  model.dirs.resize(model.width);
  for(size_t i = 0; i < waves.size(); i++)
  {
    model.origs[i] = waves[i].ray.orig;
    model.dirs[i] = waves[i].ray.dir;
  }
  return model;
}
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

void RadaraysEmbreeSensorSystem::RefreshSimulator()
{
  auto &registry = rmagine_gazebo_plugins::MapRegistry::Instance();
  const auto revision = registry.GetEmbreeMapRevision(map_key_);
  const auto map = registry.GetEmbreeMap(map_key_);
  if(!map)
  {
    map_.reset();
    sim_.reset();
    map_revision_ = 0;
    std::cerr << "[RadaraysEmbreeSensorSystem] No map available for key '" << map_key_ << "'." << std::endl;
    return;
  }

  if(!sim_ || !map_ || map != map_ || revision != map_revision_)
  {
    map_ = map;
    sim_ = std::make_shared<rmagine::OnDnSimulatorEmbree>(map_);
    sim_->setTsb(rmagine::Transform::Identity());
    map_revision_ = revision;
    std::cerr << "[RadaraysEmbreeSensorSystem] Refreshed simulator for map key '"
              << map_key_ << "' at revision " << map_revision_ << "." << std::endl;
  }
}

void RadaraysEmbreeSensorSystem::ResolveFrameEntity(
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
          std::cerr << "[RadaraysEmbreeSensorSystem] Resolved frame entity '" << frame_id_
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

bool RadaraysEmbreeSensorSystem::EnsureWorldSdf(const gz::sim::EntityComponentManager &_ecm)
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
      std::cerr << "[RadaraysEmbreeSensorSystem] " << service << " not available yet, will retry." << std::endl;
    }
    return false;
  }

  auto sdf_doc = std::make_shared<sdf::SDF>();
  sdf::init(sdf_doc);
  if(!sdf::readString(response.data(), sdf_doc))
  {
    std::cerr << "[RadaraysEmbreeSensorSystem] Failed to parse SDF from " << service << "." << std::endl;
    return false;
  }

  sdf::ElementPtr root = sdf_doc->Root();
  world_sdf_ = (root && root->GetName() == "world") ? root : root->GetElement("world");
  world_sdf_root_ = sdf_doc;

  if(debug_)
  {
    std::cerr << "[RadaraysEmbreeSensorSystem] world SDF "
              << (world_sdf_ ? "fetched for material lookup." : "NOT resolved from service response.")
              << std::endl;
  }
  return world_sdf_ != nullptr;
}

sdf::ElementPtr RadaraysEmbreeSensorSystem::FindRadaraysMaterialElement(
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

void RadaraysEmbreeSensorSystem::RefreshMaterials(const gz::sim::EntityComponentManager &_ecm)
{
  materials_by_id_.clear();
  if(!world_sdf_)
  {
    return;
  }

  auto object_entities = rmagine_gazebo_plugins::MapRegistry::Instance().GetObjectEntities(map_key_);
  if(!object_entities)
  {
    return;
  }

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

    RadarMaterial material;
    material.velocity = mat_sdf->Get<double>("velocity");
    material.ambient = mat_sdf->Get<double>("ambient");
    material.diffuse = mat_sdf->Get<double>("diffuse");
    material.specular = mat_sdf->Get<double>("specular");
    materials_by_id_[object_id + 1] = material;

    if(debug_)
    {
      std::cerr << "[RadaraysEmbreeSensorSystem] material for " << model_name << "/" << link_name
                << "/" << visual_name << ": velocity=" << material.velocity
                << " ambient=" << material.ambient << " diffuse=" << material.diffuse
                << " specular=" << material.specular << std::endl;
    }
  }
}

const RadarMaterial &RadaraysEmbreeSensorSystem::MaterialFor(unsigned int material_id) const
{
  if(material_id == 0)
  {
    return material_air_;
  }
  auto it = materials_by_id_.find(material_id);
  if(it != materials_by_id_.end())
  {
    return it->second;
  }
  return material_default_;
}

void RadaraysEmbreeSensorSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  sensor_entity_ = _entity;
  LoadParams(_sdf);
  // World SDF (for material lookup) is fetched lazily from PostUpdate --
  // the generate_world_sdf service isn't guaranteed to be up yet here.

  if(!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  std::string node_name = "radarays_embree_sensor_system";
  if(_sdf && _sdf->HasElement("node_name"))
  {
    node_name = _sdf->Get<std::string>("node_name");
  }
  node_ = std::make_shared<rclcpp::Node>(node_name);
  // Best-effort/volatile: a reliable QoS with a shallow queue can make
  // publish() block on a slow or contended subscriber for a message this
  // large (400KB+), which stalls the whole gz-sim update loop. Image
  // streams conventionally use sensor-data QoS for exactly this reason.
  image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_image_, rclcpp::SensorDataQoS());
  DeclareReconfigurableParams();

  RefreshSimulator();
}

void RadaraysEmbreeSensorSystem::DeclareReconfigurableParams()
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

rcl_interfaces::msg::SetParametersResult RadaraysEmbreeSensorSystem::OnSetParameters(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Validate first, without mutating anything: rclcpp applies a batch of
  // parameters atomically, so one invalid value here must not leave other,
  // valid ones from the same batch half-applied.
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
    // Forces PostUpdate to rebuild the cached beam-sample cone next tick.
    beam_samples_local_.clear();
  }

  return result;
}

void RadaraysEmbreeSensorSystem::LoadParams(const std::shared_ptr<const sdf::Element> &_sdf)
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

std::vector<float> RadaraysEmbreeSensorSystem::BuildDenoiserWeights(int &mode_out) const
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

void RadaraysEmbreeSensorSystem::SimulateFrame(
  const rmagine::Transform &Tsm,
  const std::vector<float> &denoising_weights,
  int denoising_mode,
  std::vector<uint8_t> &image_data)
{
  // Flat, angle-tagged wave batch: every angle's beam cone lives in the
  // same array so the whole frame's ray tracing collapses into one Embree
  // query per reflection pass, no matter how many angles there are.
  std::vector<DirectedWave> waves;
  std::vector<uint32_t> wave_angle;
  waves.reserve(static_cast<size_t>(n_angles_) * n_samples_);
  wave_angle.reserve(static_cast<size_t>(n_angles_) * n_samples_);

  for(size_t angle_id = 0; angle_id < n_angles_; angle_id++)
  {
    const double theta = (n_angles_ > 1)
      ? min_angle_ + (max_angle_ - min_angle_) * static_cast<double>(angle_id) / static_cast<double>(n_angles_ - 1)
      : min_angle_;
    const rmagine::EulerAngles Ras{0.0f, 0.0f, static_cast<float>(theta)};

    for(const auto &local_wave : beam_samples_local_)
    {
      DirectedWave wave = local_wave;
      wave.ray.dir = Ras * local_wave.ray.dir;
      waves.push_back(wave);
      wave_angle.push_back(static_cast<uint32_t>(angle_id));
    }
  }

  std::vector<std::vector<Signal>> signals_per_angle(n_angles_);

  for(unsigned int pass_id = 0; pass_id < n_reflections_; pass_id++)
  {
    if(waves.empty())
    {
      break;
    }

    rmagine::OnDnModel model = MakeOnDnModel(waves, range_max_);
    sim_->setModel(model);

    using ResT = rmagine::Bundle<
      rmagine::Hits<rmagine::RAM>,
      rmagine::Ranges<rmagine::RAM>,
      rmagine::Normals<rmagine::RAM>,
      rmagine::ObjectIds<rmagine::RAM>>;

    ResT results;
    results.hits.resize(model.size());
    results.ranges.resize(model.size());
    results.normals.resize(model.size());
    results.object_ids.resize(model.size());

    sim_->simulate(Tsm, results);

    std::vector<DirectedWave> waves_new;
    std::vector<uint32_t> wave_angle_new;
    waves_new.reserve(waves.size() * 2);
    wave_angle_new.reserve(waves.size() * 2);

    for(size_t i = 0; i < waves.size(); i++)
    {
      const DirectedWave &wave = waves[i];
      const uint32_t angle_id = wave_angle[i];
      const float wave_range = results.ranges[i];
      const rmagine::Vector surface_normal = results.normals[i].normalize();
      const unsigned int obj_id = results.object_ids[i];

      if(obj_id > 10000)
      {
        continue;
      }

      const DirectedWave incidence = wave.move(wave_range);

      DirectedWave reflection = incidence;
      DirectedWave refraction = incidence;

      if(incidence.material_id == 0)
      {
        // Entering whatever object this ray just hit. material_id = object
        // id + 1 (0 is reserved for air) -- MaterialFor() resolves it to
        // either a parsed <radarays_material> or the shared default.
        refraction.material_id = obj_id + 1;
      }
      else
      {
        refraction.material_id = 0;
      }

      double v_refraction = 1.0;
      if(incidence.material_id != refraction.material_id)
      {
        v_refraction = MaterialFor(refraction.material_id).velocity;
      }
      else
      {
        v_refraction = incidence.velocity;
      }

      auto res = fresnel(surface_normal, incidence, v_refraction);

      reflection.ray.dir = res.first.ray.dir;
      reflection.energy = res.first.energy;

      if(reflection.energy > wave_energy_threshold_)
      {
        waves_new.push_back(reflection);
        wave_angle_new.push_back(angle_id);

        if(reflection.material_id == 0)
        {
          const RadarMaterial &material = MaterialFor(refraction.material_id);
          const double incidence_angle = get_incidence_angle(surface_normal, incidence);
          const double return_energy_path = back_reflection_shader(
            incidence_angle, reflection.energy, material.ambient, material.diffuse, material.specular);

          if(pass_id == 0 || record_multi_reflection_)
          {
            signals_per_angle[angle_id].push_back({incidence.time * 2.0, return_energy_path});
          }

          if(pass_id > 0 && record_multi_path_)
          {
            rmagine::Vector dir_sensor_to_hit = reflection.ray.orig;
            const double distance_between_sensor_and_hit = dir_sensor_to_hit.l2norm();
            dir_sensor_to_hit.normalizeInplace();
            const double time_to_sensor = distance_between_sensor_and_hit / reflection.velocity;
            const double sensor_view_scalar = wave.ray.dir.dot(dir_sensor_to_hit);

            if(sensor_view_scalar > multipath_threshold_)
            {
              const double angle_between_reflection_and_sensor_dir =
                angle_between(-reflection.ray.dir, dir_sensor_to_hit);
              const double return_energy_air = back_reflection_shader(
                angle_between_reflection_and_sensor_dir, reflection.energy,
                material.ambient, material.diffuse, material.specular);
              signals_per_angle[angle_id].push_back({incidence.time + time_to_sensor, return_energy_air});
            }
          }
        }
      }

      refraction.ray.dir = res.second.ray.dir;
      refraction.energy = res.second.energy;
      if(refraction.energy > wave_energy_threshold_)
      {
        waves_new.push_back(refraction);
        wave_angle_new.push_back(angle_id);
      }
    }

    constexpr double skip_dist = 0.001;
    for(auto &w : waves_new) { w.moveInplace(skip_dist); }
    waves = std::move(waves_new);
    wave_angle = std::move(wave_angle_new);
  }

  const double resolution = range_max_ / static_cast<double>(n_cells_);
  std::uniform_real_distribution<float> dist_uni(0.0f, 1.0f);

  for(size_t angle_id = 0; angle_id < n_angles_; angle_id++)
  {
    std::vector<float> slice(n_cells_, 0.0f);
    float max_val = 0.0f;

    for(const auto &signal : signals_per_angle[angle_id])
    {
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

void RadaraysEmbreeSensorSystem::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if(_info.paused)
  {
    return;
  }

  // Publishing never needed this (DDS writes don't go through the
  // executor), but the parameter services backing dynamic reconfiguration
  // (set_parameters et al.) do -- without a spin, ros2 param set/list just
  // hangs against this node forever.
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
    // Only mark this revision as done if the world SDF was actually
    // available -- otherwise every object silently keeps the default
    // material forever instead of retrying once the service comes up.
    if(EnsureWorldSdf(_ecm))
    {
      RefreshMaterials(_ecm);
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

GZ_ADD_PLUGIN(radarays_gazebo_plugins::RadaraysEmbreeSensorSystem,
              gz::sim::System,
              radarays_gazebo_plugins::RadaraysEmbreeSensorSystem::ISystemConfigure,
              radarays_gazebo_plugins::RadaraysEmbreeSensorSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(radarays_gazebo_plugins::RadaraysEmbreeSensorSystem,
                    "radarays_embree_sensor_system")
