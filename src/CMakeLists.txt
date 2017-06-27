# CDDL HEADER START
#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#
# CDDL HEADER END

# Copyright 2017 Saso Kiselkov. All rights reserved.

cmake_minimum_required(VERSION 2.8)
project(bp C)

SET(SRC bp.c dr.c driving.c msg.c truck.c xplane.c)
SET(HDR bp.h dr.h driving.c msg.h truck.h xplane.h)

SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} -DDEBUG")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG -O0")

SET(ALL_SRC ${SRC} ${HDR})
LIST(SORT ALL_SRC)

if(APPLE)
	add_executable(bp ${ALL_SRC})
elseif(UNIX)
	add_library(bp SHARED ${ALL_SRC})
elseif(WIN32)
	add_library(bp MODULE ${ALL_SRC})
	SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MD")
	SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} /MD")
	SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MDd")
endif()

include_directories(bp PUBLIC "${LIBACFUTILS}/src")

#compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror --std=c99")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
    -DCHECK_RESULT_USED=\"__attribute__ ((warn_unused_result))\"")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_GNU_SOURCE -D_FILE_OFFSET_BITS=64")

#libraries
if(UNIX)
	include_directories(bp PUBLIC "../OpenAL/include")
	include_directories(bp PUBLIC "../SDK/CHeaders/XPLM"
	    "../SDK/CHeaders/Widgets" "../SDK/CHeaders/Wrappers")
else()
	include_directories(bp PUBLIC "..\\OpenAL\\include")
	include_directories(bp PUBLIC
	    "..\\SDK\\CHeaders\\XPLM" "..\\SDK\\CHeaders\\Widgets"
	    "..\\SDK\\CHeaders\\Wrappers")
endif(UNIX)
if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -arch x86_64 -Wno-deprecated")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mmacosx-version-min=10.7")
endif(APPLE)
if(WIN32)
	target_include_directories(bp PUBLIC
	    "..\\SDK\\CHeaders\\XPLM" "..\\SDK\\CHeaders\\Widgets"
	    "..\\SDK\\CHeaders\\Wrappers")
endif(WIN32)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DXPLM200=1 -DXPLM210=1")
if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=1 -DIBM=0 -DLIN=0")
elseif(UNIX)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=0 -DIBM=0 -DLIN=1")
elseif(WIN32)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=0 -DIBM=1 -DLIN=0 \
	    -D_CRT_SECURE_NO_WARNINGS /bigobj")
endif()

#linking
link_libraries(m)
if(APPLE)
	set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ../SDK/Libraries/Mac/)
	FIND_LIBRARY(XPLM_LIBRARY XPLM ../SDK/Libraries/Mac/)
	FIND_LIBRARY(XPWIDGETS_LIBRARY XPWidgets ../SDK/Libraries/Mac/)
	FIND_LIBRARY(OPENGL_LIBRARY OpenGL)
	FIND_LIBRARY(OPENAL_LIBRARY OpenAL)
	FIND_LIBRARY(CURSES_LIBRARY ncurses)
	target_link_libraries(bp ${OPENAL_LIBRARY} ${OPENGL_LIBRARY}
	    ${XPLM_LIBRARY})
elseif(WIN32)
	set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH}
	    "..\\SDK\\Libraries\\Win\\")
	FIND_LIBRARY(XPLM_LIBRARY XPLM_64
	    "..\\SDK\\Libraries\\Win\\")
	FIND_LIBRARY(XPWIDGETS_LIBRARY XPWidgets_64
	    "..\\SDK\\Libraries\\Win\\")
	target_link_libraries(bp ${OPENAL_LIBRARY}
	    ${XPLM_LIBRARY} ${XPWIDGETS_LIBRARY})
endif()

if(APPLE)
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} \
	    -L${LIBACFUTILS}/qmake/mac64 -lacfutils")
else()
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} \
	    -L${LIBACFUTILS}/qmake/lin64 -lacfutils")
endif()

SET_TARGET_PROPERTIES(bp PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES(bp PROPERTIES SUFFIX "")

if(APPLE)
	set(OUTPUT_FILENAME "mac.xpl")
	SET_TARGET_PROPERTIES(bp PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -fPIC -fvisibility=hidden \
	    -bundle")
elseif(UNIX)
	set(OUTPUT_FILENAME "lin.xpl")
	SET_TARGET_PROPERTIES(bp PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
	    -undefined_warning -fPIC -fvisibility=hidden")
elseif(WIN32)
	set(OUTPUT_FILENAME "win.xpl")
endif()

set_target_properties(bp PROPERTIES RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(bp PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(bp PROPERTIES OUTPUT_NAME "${OUTPUT_FILENAME}")
