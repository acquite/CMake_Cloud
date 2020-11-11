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


#自定义F_PCL库
include_directories(./F_PCL)#添加目录路径
add_subdirectory(F_PCL)#添加F_PCL文件夹中的库

#自定义F_Thread库
include_directories(./F_ThreadFuture)#添加目录路径
add_subdirectory(F_ThreadFuture)#添加F_PCL文件夹中的库



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

# 手动添加目录下的文件
add_executable(eigenTest main.cpp T_PCL.cpp T_Eigen.cpp T_STL.cpp T_PCL2.cpp T_PCL3.cpp T_PCL4.cpp T_PCL5.cpp T_PCL6.cpp T_PCL7.cpp T_PCL8.cpp T_PCL9.cpp)



target_link_libraries(eigenTest "${TORCH_LIBRARIES}")
target_link_libraries(eigenTest "${OpenCV_LIBS}")
#target_link_libraries(eigenTest ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(eigenTest ${PCL_LIBRARIES})
target_link_libraries(eigenTest neuvsdk)

#将目录下的F_PCL库链接上可执行文件eigenTest
target_link_libraries(eigenTest libF_PCL)
target_link_libraries(eigenTest libF_Thread)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
