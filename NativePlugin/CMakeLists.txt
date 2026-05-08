cmake_minimum_required(VERSION 3.20)
project(BuoyancyDLL)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(BuoyancyDLL SHARED
    src/buoyancy_api.cpp
    src/triangle_clipper.cpp
    src/wave.cpp
)

target_include_directories(BuoyancyDLL PRIVATE
    include/
    lib/glm/          # GLM header-only library
)

if(WIN32)
    target_compile_definitions(BuoyancyDLL PRIVATE BUOYANCY_EXPORTS)
endif()


set(UNITY_PLUGINS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../BaseProject/Assets/Plugins/x86_64")

add_custom_command(TARGET BuoyancyDLL POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:BuoyancyDLL>
        "${UNITY_PLUGINS_DIR}/"
)

# find_package(OpenMP)
# if(OpenMP_CXX_FOUND)
#     target_link_libraries(BuoyancyDLL PRIVATE OpenMP::OpenMP_CXX)
# endif()
