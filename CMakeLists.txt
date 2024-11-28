# cmake_minimum_required(VERSION 3.16.3)
# project(revolve)

# set(CMAKE_CXX_STANDARD 17)
# set(MUJOCO_DIR "/home/gaurav/.mujoco/mujoco-3.2.5")

# include_directories(${MUJOCO_DIR}/include)

# # add_executable(planner src/planner.cpp)
# add_executable(collision_checker src/collision_checker.cpp) 

# link_directories(${MUJOCO_DIR}/bin)
# # link_directories(${MUJOCO_DIR}/lib)
# target_link_libraries(collision_checker ${MUJOCO_LIB_DIR}/libmujoco.so mujoco glfw GL GLU rt dl)

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#--------------------------------------------------------------
cmake_minimum_required(VERSION 3.10)
project(LoadMuJoCoModel)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Specify MuJoCo include and library paths
set(MUJOCO_INCLUDE_DIR "/home/gaurav/.mujoco/mujoco-3.2.5/include")
set(MUJOCO_LIB_DIR "/home/gaurav/.mujoco/mujoco-3.2.5/lib")

# Find GLFW (install with apt, brew, or from source if needed)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

# Add executable
add_executable(collision_checker src/collision_checker.cpp)
add_executable(planner src/planner.cpp)
# add_executable(main src/main.cpp)

# Include directories
include_directories(include)
target_include_directories(collision_checker PRIVATE ${MUJOCO_INCLUDE_DIR} ${GLFW_INCLUDE_DIRS})
target_include_directories(planner PRIVATE ${MUJOCO_INCLUDE_DIR} ${GLFW_INCLUDE_DIRS})
# target_include_directories(main PRIVATE ${MUJOCO_INCLUDE_DIR} ${GLFW_INCLUDE_DIRS})

# Link libraries
target_link_libraries(collision_checker PRIVATE 
    ${MUJOCO_LIB_DIR}/libmujoco.so
    ${GLFW_LIBRARIES}
    pthread
    dl
    m
)

target_link_libraries(planner PRIVATE 
    ${MUJOCO_LIB_DIR}/libmujoco.so
    ${GLFW_LIBRARIES}
    pthread
    dl
    m
)

# target_link_libraries(main PRIVATE 
#     ${MUJOCO_LIB_DIR}/libmujoco.so
#     ${GLFW_LIBRARIES}
#     pthread
#     dl
#     m
# )
