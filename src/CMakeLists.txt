
find_package(PCL REQUIRED COMPONENTS common io)
find_package(octomap REQUIRED)

include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd2octomap pcd2octomap.cpp)
target_link_libraries(pcd2octomap ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcd2colorOctomap pcd2colorOctomap.cpp)
target_link_libraries(pcd2colorOctomap ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

find_package(OpenCV 3 REQUIRED)


add_executable(match_img match_img.cpp)
target_link_libraries(match_img ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(official_match official_match.cpp)
target_link_libraries(official_match ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})