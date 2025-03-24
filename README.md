# Simple Grasping Pipeline

This ROS2 package implements a perception and grasp planning pipeline using GPD.  
The pipeline processes an input point cloud to detect planes and objects, then generates grasp candidates based on the detected scene.

## Services

### StartPerception.srv
**Purpose:**  
Processes the latest input point cloud to perform perception tasks. This service:
- Transforms the input cloud to a common frame.
- Filters the cloud using specified parameters.
- Detects planes in the cloud.
- Sorts detected planes either by height or by distance to a given query point.
- Detects objects above the planes using additional parameters.
- Publishes debug markers for visualization if debug mode is enabled.

**Request Options:**
- **bool return_cloud**  
  If true, the service returns the inlier cloud for each detected plane and object.
- **bool sort_planes_by_height**  
  If true, the planes are sorted by their height (z coordinate). Otherwise, they are sorted by distance to the query point.
- **geometry_msgs/Point querry_point**  
  The reference point used to sort planes and objects when not sorting by height.
- **float32 height_above_plane**  
  The minimum height above a plane at which objects are detected. If not provided, a default value (e.g., 0.3) is used.
- **float32 width_adjustment**  
  A parameter to adjust the width of detected objects (if needed).

**Response:**
- **bool success** – True if the perception processing succeeded.
- **string message** – An optional status message.
- **Plane[] planes** – List of detected planes. Each plane includes:
  - A ROS point cloud message (if `return_cloud` is true).
  - An oriented bounding box (displayed as a marker).
  - Plane equation coefficients.
- **Object[] objects** – List of detected objects. Each object includes:
  - A ROS point cloud message (if `return_cloud` is true).
  - An oriented bounding box marker.
  - The index of the supporting plane.

---

### GenerateGrasps.srv
**Purpose:**  
Generates grasp candidates using GPD based on the previously detected objects and planes. This service:
- Samples a combined point cloud from all objects on the same plane as the selected object.  
  The sampling can be performed from the sensor cloud or directly from object-oriented bounding boxes (OBBs).
- Augments the cloud with a synthetic plane cloud to simulate environmental constraints (e.g., a ceiling) by offsetting the plane.
- Transforms the combined cloud and the sensor origin into the object coordinate frame.
- Updates the grasp detector's approach direction and filtering parameters.
- Generates grasp candidates with GPD and optionally publishes debug markers.

**Request Options:**
- **int32 object_index**  
  The index of the selected object to grasp.
- **bool sample_cloud_from_obb**  
  If true, the service samples the cloud from each object's OBB rather than using the sensor cloud.
- **bool disable_top_grasp**  
  If true, an additional upward copy of the plane cloud is added so that GPD “sees” a ceiling.
- **float32 min_distance_to_plane**  
  The minimum distance that the gripper should maintain from the plane; used to offset the plane cloud.
- **int32 num_grasps_selected**  
  The number of grasp candidates to select (sorted by score).
- **geometry_msgs/Point approach_direction**  
  The desired approach direction for grasp filtering (default is (1, 0, 0) if not provided).
- **float32 thresh_rad**  
  The angular threshold (in radians) above which grasp candidates are filtered. A value of 0 disables filtering.

**Response:**
- **bool success** – True if grasp generation succeeded.
- **string message** – An optional status message.

---

## Visualization

- **Plane markers:** Blue  
- **Object markers:** Red  
- **Generated grasp markers:** Green  

An example simulation output is shown in [imgs/tiago_sim.png](imgs/tiago_sim.png).
