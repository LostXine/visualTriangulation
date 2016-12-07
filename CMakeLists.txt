cmake_minimum_required(VERSION 2.8)	#设置最低版本要求


PROJECT (BWMT)				#设置项目名称
#判断系统
IF (WIN32)
MESSAGE(STATUS "Now is windows")
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
ELSEIF (APPLE)
MESSAGE(STATUS "Now is Apple systens.")
ELSEIF (UNIX)
MESSAGE(STATUS "Now is UNIX-like OS's. Including Apple OS X and CygWin")
ENDIF ()

ADD_SUBDIRECTORY(src)			#设置子目录

#BINARY_DIR是终端执行的目录
MESSAGE(STATUS "This is BINARY dir " ${BWMT_BINARY_DIR})	# 显示消息
#SOURCE_DIR是项目所在的目录
MESSAGE(STATUS "This is SOURCE dir " ${BWMT_SOURCE_DIR})


#六、区分debug和release
#在工程目录下，cmake -DCMAKE__BUILD_TYPE=DEBUG(RELEASE)，再执行make

#七、指定编译32bit或64bit程序
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")


