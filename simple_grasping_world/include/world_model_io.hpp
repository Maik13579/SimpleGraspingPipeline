#ifndef SIMPLE_GRASPING_WORLD_MODEL_IO_HPP
#define SIMPLE_GRASPING_WORLD_MODEL_IO_HPP

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <unordered_map>

/**
 * @brief Save furnitures metadata (id, num_planes) to a YAML file.
 * 
 * @param path Output path (e.g., "furnitures.yaml")
 * @param furnitures Unordered map of furnitures keyed by their ID.
 * @param db Plane database used to count the number of planes per furniture.
 */
inline void save_world_model(const std::string &path,
                             const std::unordered_map<std::string, Furniture> &furnitures,
                             const PlaneDatabase &db)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "furnitures" << YAML::Value << YAML::BeginSeq;
  for (const auto &pair : furnitures)
  {
    const Furniture &furniture = pair.second;
    auto furniture_planes = get_planes_for_furniture(furniture.id, db);
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << furniture.id;
    out << YAML::Key << "num_planes" << YAML::Value << static_cast<int>(furniture_planes.size());
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream file(path);
  file << out.c_str();
}

/**
 * @brief Load furniture metadata (id, num_planes) from a YAML file.
 * 
 * @param path Input path (e.g., "furnitures.yaml")
 * @return Unordered map of furnitures keyed by their ID. The unique_component_id is set to 0.
 */
inline std::unordered_map<std::string, Furniture> load_world_model(const std::string &path)
{
  std::unordered_map<std::string, Furniture> furnitures;
  YAML::Node root = YAML::LoadFile(path);
  if (!root["furnitures"])
    return furnitures;

  for (const auto &node : root["furnitures"])
  {
    Furniture furniture;
    furniture.id = node["id"].as<std::string>();
    furniture.num_planes = node["num_planes"].as<int>();
    furniture.unique_component_id = 0; // Will be assigned later when the node is loaded.
    furnitures[furniture.id] = furniture;
  }
  return furnitures;
}

#endif // SIMPLE_GRASPING_WORLD_MODEL_IO_HPP
