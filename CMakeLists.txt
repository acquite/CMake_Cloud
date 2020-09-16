cmake_minimum_required(VERSION 3.0.0)
project(eigenTest VERSION 0.1.0)

# find_package(Torch REQUIRED)

#opencv3.4
find_package(OpenCV REQUIRED)



#PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#NEUV
set(INC_DIR ~/Documents/eigen_tutorials/neuvsdk/include)
set(LINK_DIR ~/Documents/eigen_tutorials/neuvsdk/lib)
include_directories(${INC_DIR})
link_directories(${LINK_DIR})
link_directories(neuvsdk)


# if(NOT Torch_FOUND)
#     message(FATAL_ERROR "Pytorch Not Found!")
# endif(NOT Torch_FOUND)

include(CTest)
enable_testing()

# message(STATUS "Pytorch status:")
# message(STATUS "    libraries: ${TORCH_LIBRARIES}")
# message(STATUS "OpenCV library status:")
# message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libraries: ${OpenCV_LIBS}")
# message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(eigenTest main.cpp T_PCL.cpp)


target_link_libraries(eigenTest "${TORCH_LIBRARIES}")
target_link_libraries(eigenTest "${OpenCV_LIBS}")
#target_link_libraries(eigenTest ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(eigenTest ${PCL_LIBRARIES})
target_link_libraries(eigenTest neuvsdk)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
