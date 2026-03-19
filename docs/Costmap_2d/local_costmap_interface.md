**NOTE :  This note use the topic from ``` ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=True``` and  ```ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True```**

##

**Topic**

    /local_costmap/clearing_endpoints
    /local_costmap/costmap
    /local_costmap/costmap_raw
    /local_costmap/costmap_updates
    /local_costmap/footprint
    /local_costmap/local_costmap/transition_event
    /local_costmap/published_footprint
    /local_costmap/voxel_grid
    /local_costmap/voxel_marked_cloud

**Topic info**

1. /local_costmap/costmap

        nav_msgs/msg/OccupancyGrid, 

        Publisher count: 1

        Subscription count: 1

2. /local_costmap/voxel_grid

        Type: nav2_msgs/msg/VoxelGrid

        Publisher count: 1

        Subscription count: 0

3. /local_costmap/costmap_updates 

        Type: map_msgs/msg/OccupancyGridUpdate

        Publisher count: 1

        Subscription count: 1

4. /local_costmap/clearing_endpoints 

        Type: sensor_msgs/msg/PointCloud2

        Publisher count: 1

        Subscription count: 0

5. /local_costmap/costmap_raw

        Type: nav2_msgs/msg/Costmap

        Publisher count: 1

        Subscription count: 1

6. /local_costmap/voxel_marked_cloud

        Type: sensor_msgs/msg/PointCloud2

        Publisher count: 0

        Subscription count: 1

7. /local_costmap/footprint

        Type: geometry_msgs/msg/Polygon

        Publisher count: 0

        Subscription count: 1

8. /local_costmap/published_footprint

        Type: geometry_msgs/msg/PolygonStamped

        Publisher count: 1

        Subscription count: 
    
Interface show :

1. nav_msgs/msg/OccupancyGrid

        # This represents a 2-D grid map
        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id

        # MetaData for the map
        MapMetaData info
            builtin_interfaces/Time map_load_time
                int32 sec
                uint32 nanosec
            float32 resolution
            uint32 width
            uint32 height
            geometry_msgs/Pose origin
                Point position
                    float64 x
                    float64 y
                    float64 z
                Quaternion orientation
                    float64 x 0
                    float64 y 0
                    float64 z 0
                    float64 w 1

        # The map data, in row-major order, starting with (0,0).
        # Cell (1, 0) will be listed second, representing the next cell in the x direction.
        # Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
        # The values inside are application dependent, but frequently,
        # 0 represents unoccupied, 1 represents definitely occupied, and
        # -1 represents unknown.
        int8[] data

2. nav2_msgs/msg/VoxelGrid

        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
        uint32[] data
        geometry_msgs/Point32 origin
            #
            #
            float32 x
            float32 y
            float32 z
        geometry_msgs/Vector3 resolutions
            float64 x
            float64 y
            float64 z
        uint32 size_x
        uint32 size_y
        uint32 size_z

3. map_msgs/msg/OccupancyGridUpdate

        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
        int32 x
        int32 y
        uint32 width
        uint32 height
        int8[] data

4. sensor_msgs/msg/PointCloud2

        # This message holds a collection of N-dimensional points, which may
        # contain additional information such as normals, intensity, etc. The
        # point data is stored as a binary blob, its layout described by the
        # contents of the "fields" array.
        #
        # The point cloud data may be organized 2d (image-like) or 1d (unordered).
        # Point clouds organized as 2d images may be produced by camera depth sensors
        # such as stereo or time-of-flight.

        # Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id

        # 2D structure of the point cloud. If the cloud is unordered, height is
        # 1 and width is the length of the point cloud.
        uint32 height
        uint32 width

        # Describes the channels and their layout in the binary data blob.
        PointField[] fields
            uint8 INT8    = 1
            uint8 UINT8   = 2
            uint8 INT16   = 3
            uint8 UINT16  = 4
            uint8 INT32   = 5
            uint8 UINT32  = 6
            uint8 FLOAT32 = 7
            uint8 FLOAT64 = 8
            string name      #
            uint32 offset    #
            uint8  datatype  #
            uint32 count     #

        bool    is_bigendian # Is this data bigendian?
        uint32  point_step   # Length of a point in bytes
        uint32  row_step     # Length of a row in bytes
        uint8[] data         # Actual point data, size is (row_step*height)

        bool is_dense        # True if there are no invalid points

5. nav2_msgs/msg/Costmap

        # This represents a 2-D grid map, in which each cell has an associated cost

        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id

        # MetaData for the map
        CostmapMetaData metadata
            builtin_interfaces/Time map_load_time
                int32 sec
                uint32 nanosec
            builtin_interfaces/Time update_time
                int32 sec
                uint32 nanosec
            string layer
            float32 resolution
            uint32 size_x
            uint32 size_y
            geometry_msgs/Pose origin
                Point position
                    float64 x
                    float64 y
                    float64 z
                Quaternion orientation
                    float64 x 0
                    float64 y 0
                    float64 z 0
                    float64 w 1

        # The cost data, in row-major order, starting with (0,0).
        uint8[] data

6. geometry_msgs/msg/Polygon

        # A specification of a polygon where the first and last points are assumed to be connected

        Point32[] points
            #
            #
            float32 x
            float32 y
            float32 z

7. geometry_msgs/msg/PolygonStamped

        # This represents a Polygon with reference coordinate frame and timestamp

        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
        Polygon polygon
            Point32[] points
                #
                #
                float32 x
                float32 y
                float32 z




