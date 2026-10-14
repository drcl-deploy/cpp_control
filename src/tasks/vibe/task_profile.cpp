#include "cpp_control/tasks/vibe/task_profile.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace cpp_control {
namespace vibe {

namespace {

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

std::string lowered(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return value;
}

std::vector<std::string> term_names(const deploy::PortSpec* port) {
  std::vector<std::string> names;
  if (port)
    for (const auto& term : port->terms) names.push_back(term.name);
  return names;
}

void require_terms(const deploy::PortSpec* port,
                   const std::string& logical_group,
                   const std::vector<std::string>& expected) {
  if (!port)
    throw std::runtime_error("g1_vibe: manifest has no logical group '" +
                             logical_group + "'");
  const auto actual = term_names(port);
  if (actual != expected)
    throw std::runtime_error("g1_vibe: task group '" + logical_group +
                             "' has an unexpected term contract");
}

void validate_tiling(const deploy::PortSpec& port) {
  int next = 0;
  for (const auto& term : port.terms) {
    if (term.offset != next || term.dim <= 0)
      throw std::runtime_error("g1_vibe: terms do not tile port '" + port.name +
                               "' contiguously at term '" + term.name + "'");
    next += term.dim;
  }
  if (next != port.dim())
    throw std::runtime_error("g1_vibe: term widths do not fill port '" +
                             port.name + "'");
}

}  // namespace

const char* cube_color_name(int index) {
  static const char* kNames[NUM_CUBE_COLORS] = {"red",    "orange", "green",
                                                "yellow", "blue",   "pink"};
  return (index >= 0 && index < NUM_CUBE_COLORS) ? kNames[index] : "?";
}

TaskProfile profile_from_task_id(const std::string& task_id) {
  static const std::pair<const char*, TaskProfile> kFamilies[] = {
      {"repose", {"repose", GoalKind::COLOR, true, true, true, false}},
      {"uolm", {"uolm", GoalKind::OBJECT_POSE, true, true, true, false}},
      {"perloco", {"perloco", GoalKind::NONE, true, true, true, false}},
      {"dodge", {"dodge", GoalKind::NONE, false, false, false, true}},
  };
  const std::string id = lowered(task_id);
  const TaskProfile* found = nullptr;
  for (const auto& entry : kFamilies) {
    if (id.find(entry.first) == std::string::npos) continue;
    if (found)
      throw std::runtime_error("g1_vibe: manifest task_id '" + task_id +
                               "' names two task families, '" + found->family +
                               "' and '" + entry.second.family + "'");
    found = &entry.second;
  }
  if (!found)
    throw std::runtime_error("g1_vibe: manifest task_id '" + task_id +
                             "' names no known task family "
                             "(repose | uolm | perloco | dodge)");
  return *found;
}

std::vector<std::string> query_groups(const deploy::DeployManifest& manifest) {
  std::vector<std::string> queries;
  for (const auto& port : manifest.inputs)
    for (const auto& group : port.groups)
      if (starts_with(group, "q_") &&
          std::find(queries.begin(), queries.end(), group) == queries.end())
        queries.push_back(group);
  return queries;
}

void validate_contract(const deploy::DeployManifest& manifest,
                       const TaskProfile& profile) {
  if (manifest.model_class != "ExtractorSonicAdapterModel")
    throw std::runtime_error(
        "g1_vibe: task export model_class must be "
        "ExtractorSonicAdapterModel, got '" +
        manifest.model_class + "'");

  for (const auto& port : manifest.inputs) validate_tiling(port);

  require_terms(manifest.find_group("tokenizer"), "tokenizer",
                {"g1_tokenizer"});
  require_terms(
      manifest.find_group("policy"), "policy",
      {"base_ang_vel", "joint_pos", "joint_vel", "actions", "gravity_dir"});
  require_terms(manifest.find_group("kv_tokens"), "kv_tokens", {"img_tokens"});
  require_terms(
      manifest.find_group("q_proprio"), "q_proprio",
      {"projected_gravity", "base_ang_vel", "joint_pos", "joint_vel"});
  require_terms(manifest.find_group("q_cls"), "q_cls", {"img_cls"});

  if (profile.family == "repose") {
    require_terms(manifest.find_group("augmentation"), "augmentation",
                  {"bodywise_contact_cmd", "robot_root_lin_vel_cmd",
                   "robot_root_ang_vel_cmd"});
    require_terms(manifest.find_group("q_task_cmd"), "q_task_cmd",
                  {"object_goal_color"});
  } else if (profile.family == "uolm") {
    require_terms(manifest.find_group("augmentation"), "augmentation",
                  {"bodywise_contact_cmd", "robot_root_lin_vel_cmd",
                   "robot_root_ang_vel_cmd"});
    require_terms(manifest.find_group("q_task_cmd"), "q_task_cmd",
                  {"object_goal_ori", "object_goal_pos"});
  } else if (profile.family == "perloco") {
    const auto* augmentation = manifest.find_group("augmentation");
    require_terms(augmentation, "augmentation",
                  {"robot_root_lin_vel_cmd", "robot_root_ang_vel_cmd"});
    const auto* task = manifest.find_group("q_task_cmd");
    if (task != augmentation)
      throw std::runtime_error(
          "g1_vibe: PerLoco q_task_cmd must alias augmentation");
  } else if (profile.family == "dodge") {
    if (manifest.find_group("augmentation") ||
        manifest.find_group("q_task_cmd"))
      throw std::runtime_error(
          "g1_vibe: Dodge export must have no command stream");
  }

  const auto queries = query_groups(manifest);
  const std::vector<std::string> expected =
      profile.family == "dodge"
          ? std::vector<std::string>{"q_proprio", "q_cls"}
          : std::vector<std::string>{"q_task_cmd", "q_proprio", "q_cls"};
  if (queries != expected)
    throw std::runtime_error(
        "g1_vibe: manifest query order does not match task contract");
}

}  // namespace vibe
}  // namespace cpp_control
