# -----------------------------------------------------------------------------
# Get livox_driver version from include/livox_driver.h
# -----------------------------------------------------------------------------
file(READ "${CMAKE_CURRENT_LIST_DIR}/../src/include/livox_driver.h"
     LIVOX_ROS_DRIVER2_VERSION_FILE
)
string(REGEX MATCH "LIVOX_ROS_DRIVER2_VER_MAJOR ([0-9]+)" _
             "${LIVOX_ROS_DRIVER2_VERSION_FILE}"
)
set(VER_MAJOR ${CMAKE_MATCH_1})

string(REGEX MATCH "LIVOX_ROS_DRIVER2_VER_MINOR ([0-9]+)" _
             "${LIVOX_ROS_DRIVER2_VERSION_FILE}"
)
set(VER_MINOR ${CMAKE_MATCH_1})

string(REGEX MATCH "LIVOX_ROS_DRIVER2_VER_PATCH ([0-9]+)" _
             "${LIVOX_ROS_DRIVER2_VERSION_FILE}"
)
set(VER_PATCH ${CMAKE_MATCH_1})

if(NOT DEFINED VER_MAJOR
   OR NOT DEFINED VER_MINOR
   OR NOT DEFINED VER_PATCH
)
  message(
    FATAL_ERROR "Could not extract valid version from include/livox_driver.h"
  )
endif()
set(LIVOX_ROS_DRIVER2_VERSION "${VER_MAJOR}.${VER_MINOR}.${VER_PATCH}")
