cmake_minimum_required(VERSION 3.10)

# Ten project
project(VideoCaptureDrone)

# Cau hinh chuan C++ (Nen dung C++ tro len de dam bao on dinh)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dat ten file chay dau ra (trung ten voi file cpp)
set(TARGET_NAME video_capture_camera_demo)

# Tao file thuc thi tu source code
add_executable(${TARGET_NAME} video_capture_camera_demo.cpp)

# Lay duong dan tuyet doi cua thu muc cha (Thư mục BT_TL_GREMSY/MAVLink_QGC...)
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

# Debugging
message(STATUS "Project Source Dir: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "Parent Dir (Looking for library here): ${PARENT_DIR}")

# Them duong dan nay vao Include
# ${CMAKE_CURRENT_SOURCE_DIR} chinh la thu muc hien tai (tuong ung voi dau cham .)
# Dieu nay giup trinh bien dich tim thay folder "mavlink/"
target_include_directories(${TARGET_NAME} PRIVATE ${PARENT_DIR})

# Tat canh bao MAVLink (-Wno-address-of-packed-member)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${TARGET_NAME} PRIVATE -Wno-address-of-packed-member)

    # Lien ket thu vien da luong (pthread) 
    target_link_libraries(${TARGET_NAME} pthread)
endif()