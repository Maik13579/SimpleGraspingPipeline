#ifndef SIMPLE_GRASPING_WORLD_MODEL_IO_HPP
#define SIMPLE_GRASPING_WORLD_MODEL_IO_HPP

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <memory>

/**
 * \brief Save furnitures metadata (id, num_planes) to a YAML file.
 * \param path Output path (e.g., "furnitures.yaml")
 * \param furnitures Vector of furnitures
 * \param db Plane database for counting planes per furnitures
 */
inline void save_world_model(const std::string &path, const std::vector<Furniture> &furnitures, const PlaneDatabase &db)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "furnitures" << YAML::Value << YAML::BeginSeq;
  for (const auto &furniture : furnitures)
  {
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
 * \brief Load furniture metadata (id, num_planes) from a YAML file.
 * \param path Input path (e.g., "furnitures.yaml")
 * \return Vector of furnitures with their IDs. Plane count is not restored here.
 */
inline std::vector<Furniture> load_world_model(const std::string &path)
{
  std::vector<Furniture> furnitures;
  YAML::Node root = YAML::LoadFile(path);

  if (!root["furnitures"]) return furnitures;

  for (const auto &node : root["furnitures"])
  {
    Furniture furniture;
    furniture.id = node["id"].as<std::string>();
    furniture.num_planes = node["num_planes"].as<int>();
    furnitures.push_back(furniture);
  }

  return furnitures;
}

#endif // SIMPLE_GRASPING_WORLD_MODEL_IO_HPP