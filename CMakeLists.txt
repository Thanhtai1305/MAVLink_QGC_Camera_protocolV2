cmake_minimum_required(VERSION 3.10)

# Ten project
project(MOckCameraDrone)

# Cau hinh chuan C++ (Nen dung C++ tro len de dam bao on dinh)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tao file thuc thi
add_executable(mock_camera_drone mock_camera_drone.cpp)

# Them duong dan header (-I.)
# ${CMAKE_CURRENT_SOURCE_DIR} chinh la thu muc hien tai (tuong ung voi dau cham .)
# Dieu nay giup trinh bien dich tim thay folder "mavlink/"
target_include_directories(mock_camera_drone PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Tat canh bao MAVLink (-Wno-address-of-packed-member)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mock_camera_drone PRIVATE -Wno-address-of-packed-member)
endif()