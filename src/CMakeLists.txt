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
project(betterpushback C)

SET(SRC avl.c geom.c helpers.c list.c log.c math.c time.c wav.c riff.c)
SET(HDR avl.h avl_impl.h geom.h helpers.h log.h list.h list_impl.h
    math.h types.h time.h wav.h riff.h)

SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} -DDEBUG")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG -O0")

SET(ALL_SRC ${SRC} ${HDR})
LIST(SORT ALL_SRC)

if(APPLE)
	add_library(betterpushback ${ALL_SRC})
elseif(UNIX)
	add_library(betterpushback SHARED ${ALL_SRC})
elseif(WIN32)
	add_library(betterpushback MODULE ${ALL_SRC})
	SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MD")
	SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} /MD")
	SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MDd")
endif()

#compiler flags
if(WIN32)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DCHECK_RESULT_USED=_Check_return_")
else()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -W -Wall -Wextra -Werror \
	    --std=c99 \
	    -DCHECK_RESULT_USED=\"__attribute__ ((warn_unused_result))\"")
endif()

#libraries
if(UNIX)
	include_directories(betterpushback PUBLIC "../OpenAL/include")
	if(NOT "${TEST_STANDALONE_BUILD}" EQUAL "1")
		include_directories(betterpushback PUBLIC "../SDK/CHeaders/XPLM"
		    "../SDK/CHeaders/Widgets" "../SDK/CHeaders/Wrappers")
	endif()
else()
	include_directories(betterpushback PUBLIC "..\\OpenAL\\include")
	if(NOT "${TEST_STANDALONE_BUILD}" EQUAL "1")
		include_directories(betterpushback PUBLIC
		    "..\\SDK\\CHeaders\\XPLM" "..\\SDK\\CHeaders\\Widgets"
		    "..\\SDK\\CHeaders\\Wrappers")
	endif()
endif(UNIX)
if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -arch x86_64 -Wno-deprecated")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mmacosx-version-min=10.7")
endif(APPLE)
if(WIN32)
	target_include_directories(betterpushback PUBLIC
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
	FIND_LIBRARY(OPENAL_LIBRARY OpenAL)
	FIND_LIBRARY(CURSES_LIBRARY ncurses)
	if("${TEST_STANDALONE_BUILD}" EQUAL "1")
		target_link_libraries(betterpushback ${OPENAL_LIBRARY}
		    ${CURSES_LIBRARY} ${XPWIDGETS_LIBRARY})
	else()
		target_link_libraries(betterpushback ${OPENAL_LIBRARY}
		    ${XPLM_LIBRARY} ${XPWIDGETS_LIBRARY})
	endif()
elseif(UNIX)
	INCLUDE(FindPkgConfig)
	pkg_check_modules(UDEV REQUIRED libudev)
	include_directories(${UDEV_INCLUDE_DIRS})
	link_directories(${UDEV_LIBRARY_DIRS})
	target_link_libraries(betterpushback ${UDEV_LIBRARIES})
elseif(WIN32)
	set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH}
	    "..\\SDK\\Libraries\\Win\\")
	FIND_LIBRARY(XPLM_LIBRARY XPLM_64
	    "..\\SDK\\Libraries\\Win\\")
	FIND_LIBRARY(XPWIDGETS_LIBRARY XPWidgets_64
	    "..\\SDK\\Libraries\\Win\\")
	if("${TEST_STANDALONE_BUILD}" EQUAL "1")
		target_link_libraries(betterpushback ${OPENAL_LIBRARY})
	else()
		target_link_libraries(betterpushback ${OPENAL_LIBRARY}
		    ${XPLM_LIBRARY} ${XPWIDGETS_LIBRARY})
	endif()
endif()

SET_TARGET_PROPERTIES(betterpushback PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES(betterpushback PROPERTIES SUFFIX "")

if(APPLE)
	set(OUTPUT_FILENAME "mac.xpl")
	SET_TARGET_PROPERTIES(betterpushback PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -fPIC -fvisibility=hidden \
	    -bundle")
elseif(UNIX)
	set(OUTPUT_FILENAME "lin.xpl")
	SET_TARGET_PROPERTIES(betterpushback PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
	    -undefined_warning -fPIC -fvisibility=hidden")
elseif(WIN32)
	set(OUTPUT_FILENAME "win.xpl")
endif()

set_target_properties(betterpushback PROPERTIES RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(betterpushback PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(betterpushback PROPERTIES
    OUTPUT_NAME "${OUTPUT_FILENAME}")
