SET(LIBRARY_OUTPUT_PATH ${HELLO_SOURCE_DIR}/lib)	# 设置库文件打输出目录
SET(CURRENT_HEADERS ${HELLO_SOURCE_DIR}/include/tri_manager)	# 设置include文件目录

IF (WIN32)
FILE(GLOB TRI_HEADERS "${CURRENT_HEADERS}/*.h")
SOURCE_GROUP("Include files" FILES  ${TRI_HEADERS}) 
ENDIF ()

SET(SRC_LIST tri_manager.cpp)

#使用以下4句替换find_package(OpenCV REQUIRED) 防止版本错误。
find_package(PkgConfig)
pkg_check_modules(OpenCV opencv>=2.4.13)
set(OpenCV_LIBS ${OpenCV_LIBRARIES})
LINK_DIRECTORIES(${OpenCV_LIBDIR}) 
MESSAGE(STATUS "OpenCV Libs:" ${OpenCV_LIBS})
MESSAGE(STATUS "OpenCV Libdir:" ${OpenCV_LIBDIR})


if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)

  if(NOT OpenCV_SHARED)
    foreach(flag_var
            CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
            CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
            CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
            CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif()
      if(${flag_var} MATCHES "/MDd")
        string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
      endif()
    endforeach(flag_var)

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:msvcrt.lib /NODEFAULTLIB:msvcrtd.lib")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:libcmt.lib")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /NODEFAULTLIB:libcmtd.lib")
  endif()
endif()

#OpenCV 依赖

#SET(OPENCV_TRI_REQUIRED_DEPS opencv_core opencv_flann opencv_imgproc
#    opencv_highgui opencv_ml opencv_video opencv_objdetect opencv_photo
#    opencv_features2d opencv_calib3d opencv_legacy opencv_contrib)

#ocv_check_dependencies(${OPENCV_TRI_REQUIRED_DEPS})

#if(OCV_DEPENDENCIES_FOUND)
#  if(CMAKE_COMPILER_IS_GNUCXX AND NOT ENABLE_NOISY_WARNINGS)
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
#  endif()

#  ocv_include_modules(${OPENCV_TRI_REQUIRED_DEPS} opencv_nonfree)
#endif()

INCLUDE_DIRECTORIES(${CURRENT_HEADERS})

ADD_LIBRARY(tri_manager SHARED ${OpenCV_LIBS} ${SRC_LIST} ${TRI_HEADERS})
