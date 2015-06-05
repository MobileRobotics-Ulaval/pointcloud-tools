# pointcloud-tools
A set of tools for manipulating point clouds inside ROS.

## cloud\_recorder
A service that accepts point clouds and saves them on disk. It returns the name of the file where
the point cloud was recorded. Currently supports VTK and PCD formats, in ASCII.
