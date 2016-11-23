SET(T_LIB_NAME camera_unit)#设置本库的名字

SET(LIBRARY_OUTPUT_PATH ${BWMT_SOURCE_DIR}/lib)	# 设置库文件输出目录
SET(CURRENT_HEADERS ${BWMT_SOURCE_DIR}/include/${T_LIB_NAME})	# 设置include文件目录

IF (WIN32)
FILE(GLOB HD_LIST "${CURRENT_HEADERS}/*.h")
SOURCE_GROUP("Include files" FILES  ${HD_LIST}) 
ENDIF ()


SET(SRC_LIST ${T_LIB_NAME}.cpp)

INCLUDE_DIRECTORIES(${CURRENT_HEADERS})

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

#使用以下4句替换find_package(OpenCV REQUIRED) 防止版本错误。
find_package(PkgConfig)
pkg_check_modules(OpenCV opencv>=2.4.1 REQUIRED)
#OpenCV头文件 相当于gcc -I
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

#OpenCV库目录 相当于gcc -L
LINK_DIRECTORIES(${OpenCV_LIBRARIES_DIRS})  
#MESSAGE(STATUS "OpenCV Cflags:" ${OpenCV_CFLAGS})
#MESSAGE(STATUS "OpenCV Include Dir:" ${OpenCV_INCLUDE_DIRS})
#MESSAGE(STATUS "OpenCV Lib Dir:" ${OpenCV_LIBRARY_DIRS})

ADD_LIBRARY(${T_LIB_NAME} SHARED ${SRC_LIST} ${HD_LIST})#一个猜测：这个函数会直接编译源码
TARGET_LINK_LIBRARIES(${T_LIB_NAME} 
glog 
opencv_calib3d 
opencv_core 
opencv_highgui)#添加glog
#OpenCV链接库 相当于gcc -l
#TARGET_LINK_LIBRARIES(tri_manager ${OpenCV_LIBRARIES})	#一个猜测：这个函数会调用编译好的代码
