cmake_minimum_required(VERSION 2.8)	#设置最低版本要求


PROJECT (HELLO)				#设置项目名称
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
MESSAGE(STATUS "This is BINARY dir " ${HELLO_BINARY_DIR})	# 显示消息
#SOURCE_DIR是项目所在的目录
MESSAGE(STATUS "This is SOURCE dir " ${HELLO_SOURCE_DIR})


