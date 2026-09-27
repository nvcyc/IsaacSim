cmake_minimum_required(VERSION 3.24)
project(isaac_lyrical_image_probe LANGUAGES CXX CUDA)

find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(cuda_buffer REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_executable(image_probe ImageProbe.cpp ImagePattern.cu)
target_compile_features(image_probe PRIVATE cxx_std_17 cuda_std_17)
target_link_libraries(image_probe PRIVATE
    rcl::rcl
    rmw::rmw
    ${sensor_msgs_TARGETS}
    rosidl_typesupport_cpp::rosidl_typesupport_cpp
    cuda_buffer::cuda_buffer
    CUDA::cudart
)
set_target_properties(image_probe PROPERTIES CUDA_ARCHITECTURES "75;86;89")

set(ISAACSIM_SDK_INCLUDE_DIRS "" CACHE STRING "Carbonite, Fabric and nlohmann include directories for bridge validation")
if(ISAACSIM_SDK_INCLUDE_DIRS)
    set(bridge_root "${CMAKE_CURRENT_SOURCE_DIR}/../../source/extensions")
    add_executable(image_bridge_probe
        ImageProbe.cpp ImagePattern.cu
        "${bridge_root}/isaacsim.ros2.core/library/lyrical/Ros2LyricalImage.cpp"
    )
    target_compile_definitions(image_bridge_probe PRIVATE ISAAC_BRIDGE_IMAGE=1)
    target_compile_features(image_bridge_probe PRIVATE cxx_std_17 cuda_std_17)
    target_include_directories(image_bridge_probe PRIVATE
        ${ISAACSIM_SDK_INCLUDE_DIRS}
        "${bridge_root}/isaacsim.ros2.core/include"
        "${bridge_root}/isaacsim.core.includes/include"
        "${bridge_root}/isaacsim.ros2.core/library/lyrical"
    )
    target_link_libraries(image_bridge_probe PRIVATE
        rcl::rcl rmw::rmw ${sensor_msgs_TARGETS}
        rosidl_typesupport_cpp::rosidl_typesupport_cpp cuda_buffer::cuda_buffer CUDA::cudart ${CMAKE_DL_LIBS}
    )
    set_target_properties(image_bridge_probe PROPERTIES CUDA_ARCHITECTURES "75;86;89")
endif()

enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME image_transport_matrix
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_transport.py" "$<TARGET_FILE:image_probe>"
)
set_tests_properties(image_transport_matrix PROPERTIES TIMEOUT 180)
if(TARGET image_bridge_probe)
    add_test(NAME bridge_image_transport_matrix
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_transport.py"
            "$<TARGET_FILE:image_probe>" "$<TARGET_FILE:image_bridge_probe>"
    )
    set_tests_properties(bridge_image_transport_matrix PROPERTIES TIMEOUT 240)
endif()
