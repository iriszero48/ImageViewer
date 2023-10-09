cmake_minimum_required(VERSION 3.0.0)
project(image_viewer VERSION 0.1.0 LANGUAGES C CXX)
include(CMakePrintHelpers)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(wxWidgets CONFIG REQUIRED)
find_package(Stb REQUIRED)
find_package(directxtex CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(FFMPEG REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Gui)
find_package(unofficial-graphicsmagick CONFIG REQUIRED)

set(IvDefinitions
    CU_IMG_HAS_WXWIDGETS
    CU_IMG_HAS_STB
    CU_IMG_HAS_DIRECTXTEX
    CU_IMG_HAS_OPENCV
    CU_IMG_HAS_QT
    CU_IMG_HAS_GRAPHICSMAGICK
)
if(MSVC)
    list(APPEND IvDefinitions UNICODE _UNICODE)
endif()

file(GLOB_RECURSE CU_FILES "CppUtility/*.h" "CppUtility/*.hpp")
cmake_print_variables(CU_FILES)

add_executable(image_viewer WIN32 main.cpp ${CU_FILES})

target_link_directories(image_viewer PRIVATE ${FFMPEG_LIBRARY_DIRS})
target_link_libraries(image_viewer PRIVATE
    wx::core wx::base
    Microsoft::DirectXTex
    opencv_core opencv_imgcodecs
    ${FFMPEG_LIBRARIES}
    Qt6::Gui
    unofficial::graphicsmagick::graphicsmagick)

target_include_directories(image_viewer PRIVATE "CppUtility" ${Stb_INCLUDE_DIR} ${FFMPEG_INCLUDE_DIRS})
target_compile_definitions(image_viewer PRIVATE ${IvDefinitions})
