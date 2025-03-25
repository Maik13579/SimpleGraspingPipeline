#ifndef SIMPLE_GRASPING_WORLD_MODEL_IO_HPP
#define SIMPLE_GRASPING_WORLD_MODEL_IO_HPP

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <memory>

/**
 * \brief Save object metadata (id, num_planes) to a YAML file.
 * \param path Output path (e.g., "objects.yaml")
 * \param objects Vector of objects
 * \param db Plane database for counting planes per object
 */
inline void save_world_model(const std::string &path, const std::vector<Object> &objects, const PlaneDatabase &db)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "objects" << YAML::Value << YAML::BeginSeq;
  for (const auto &obj : objects)
  {
    auto obj_planes = get_planes_for_object(obj.id, db);
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << obj.id;
    out << YAML::Key << "num_planes" << YAML::Value << static_cast<int>(obj_planes.size());
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream file(path);
  file << out.c_str();
}

/**
 * \brief Load object metadata (id, num_planes) from a YAML file.
 * \param path Input path (e.g., "objects.yaml")
 * \return Vector of objects with their IDs. Plane count is not restored here.
 */
inline std::vector<Object> load_world_model(const std::string &path)
{
  std::vector<Object> objects;
  YAML::Node root = YAML::LoadFile(path);

  if (!root["objects"]) return objects;

  for (const auto &node : root["objects"])
  {
    Object obj;
    obj.id = node["id"].as<std::string>();
    obj.num_planes = node["num_planes"].as<int>();
    objects.push_back(obj);
  }

  return objects;
}

#endif // SIMPLE_GRASPING_WORLD_MODEL_IO_HPP