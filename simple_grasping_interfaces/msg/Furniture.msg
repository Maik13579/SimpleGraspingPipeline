Plane[] planes # 1-n planes furnite is made of
Object[] objects #objects on top of the plane/s

sensor_msgs/PointCloud2 cloud #complete cloud
#each point in the cloud has a label
# 0->n: planes
# n+1->n+m: objects