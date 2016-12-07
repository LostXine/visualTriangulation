SET(EXECUTABLE_OUTPUT_PATH ${BWMT_SOURCE_DIR}/bin)	# 设置输出执行文件的目录
SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

add_definitions("-std=c++11")
SET(SRC_LIST detection.cpp)

find_package(OpenCV REQUIRED)


INCLUDE_DIRECTORIES(/home/xine/Codelibs/ssd/caffe/include /usr/local/cuda-8.0/targets/x86_64-linux/include)

ADD_EXECUTABLE(detection ${SRC_LIST})

TARGET_LINK_LIBRARIES(detection glog gflags ${OpenCV_LIBS} "/home/xine/Codelibs/ssd/caffe/build/lib/libcaffe.so" boost_system)
