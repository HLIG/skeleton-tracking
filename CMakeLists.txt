project(keypoints_tracking)
cmake_minimum_required( VERSION 3.1 )
set( CMAKE_CXX_STANDARD 11 )
# add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC ON )
set( CMAKE_BUILD_TYPE Release )

set( OpenCV_DIR /usr/local/opencv4.2/lib/cmake/opencv4/)
find_package( OpenCV  REQUIRED)
set(keypoints_tracking_src	${CMAKE_CURRENT_SOURCE_DIR}/keypoints_tracking.cpp
							${CMAKE_CURRENT_SOURCE_DIR}/keypoints_tracking.h
							${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
			#PARENT_SCOPE
)

add_subdirectory(munkres)



# target_include_directories(test_opencv PRIVATE "${OpenCV_INCLUDE_DIRS}")
add_executable ( keypoints_tracking ${keypoints_tracking_src}  )

target_link_libraries(keypoints_tracking PUBLIC 
	-Wl,--start-group
	munkres
	${OpenCV_LIBS}
	-Wl,--end-group
	)



