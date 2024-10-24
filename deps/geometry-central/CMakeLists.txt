set(geometry-central_VERSION modified)

####################################################################################################
####################################################################################################
# geometry-central/include/geometrycentral/pointcloud/point_position_geometry.h
# Line 112 removed

# geometry-central/include/geometrycentral/pointcloud/neighborhoods.h
# geometry-central/src/pointcloud/neighborhoods.cpp
# Add constructor for fixed-ball neighborhood

# geometry-central/src/pointcloud/local_triangulation.cpp
# Utilize OpenMP for parallelization

####################################################################################################
####################################################################################################

add_subdirectory(deps)
add_subdirectory(src)

set_target_properties(geometry-central PROPERTIES
        VERSION ${geometry-central_VERSION}
)
