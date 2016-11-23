SET(T_LIB_NAME camera_unit)#设置本库的名字

SET(LIBRARY_OUTPUT_PATH ${BWMT_SOURCE_DIR}/lib)	# 设置库文件输出目录
SET(CURRENT_HEADERS ${BWMT_SOURCE_DIR}/include/${T_LIB_NAME})	# 设置include文件目录

IF (WIN32)
FILE(GLOB HD_LIST "${CURRENT_HEADERS}/*.h")
SOURCE_GROUP("Include files" FILES  ${HD_LIST}) 
ENDIF ()

SET(SRC_LIST ${T_LIB_NAME}.cpp)

INCLUDE_DIRECTORIES(${CURRENT_HEADERS})


find_package(OpenCV REQUIRED)

# OpenGL
find_package(OpenGL REQUIRED)

# GLUT
find_package(GLUT REQUIRED)

# Glew
find_package(GLEW REQUIRED)

# SiftGPU：手动设置其头文件与库文件所在位置
include_directories("~/Codelibs/SiftGPU/src/SiftGPU/" ${OpenGL_INCLUDE_DIR})
set(SIFTGPU_LIBS "~/Codelibs/SiftGPU/bin/libsiftgpu.so")
ADD_LIBRARY(${T_LIB_NAME} SHARED ${SRC_LIST} ${HD_LIST})#一个猜测：这个函数会直接编译源码
TARGET_LINK_LIBRARIES(${T_LIB_NAME} 
glog 
${OPENCV_LIBS}
${SIFTGPU_LIBS}
${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES}
)#添加glog
#OpenCV链接库 相当于gcc -l
#TARGET_LINK_LIBRARIES(tri_manager ${OpenCV_LIBRARIES})	#一个猜测：这个函数会调用编译好的代码
