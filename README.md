# pointcloud-tools
A set of tools for manipulating point clouds inside ROS.

## cloud\_recorder
A node that listens on a PointCloud2 topic and saves any cloud emitted there
to disk. 

### Parameters

- `source <string>` The name of the topic to listen to. 
- `format <pcd|vtk>` is used to specify which format should be used when saving
  the clouds. Note that because libpointmatcher can only save to ASCII format,
  the same restriction applies to this node. Default: `vtk`.
- `working_directory <string>` is used to specify the directory the clouds will
  be saved in. This is mainly used by the launchfiles. Optional.
- `target_frame <string>` is used to have the recorder transform the point
  clouds into the specified frame before recording. If no frame is specified the
  clouds are saved as is.
