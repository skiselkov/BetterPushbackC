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

# Copyright 2022 Saso Kiselkov. All rights reserved.

cmake_minimum_required(VERSION 3.0)

project(bp)

# Platform-specific variables
if(APPLE)
	set(PLAT_SHORT "mac64")
	set(PLAT_LONG "mac-64")
	set(PLAT_ONLY "mac")
	set(PLUGIN_BIN_OUTDIR "mac_x64")
elseif(WIN32)
	set(PLAT_SHORT "win64")
	set(PLAT_LONG "win-64")
	set(PLAT_ONLY "win")
	set(PLUGIN_BIN_OUTDIR "win_x64")
else()
	set(PLAT_SHORT "lin64")
	set(PLAT_LONG "linux-64")
	set(PLAT_ONLY "linux")
	set(PLUGIN_BIN_OUTDIR "lin_x64")
endif()

# Build path setup
file(GLOB LIBACFUTILS "libacfutils")
file(GLOB GLEW_PREFIX "${LIBACFUTILS}/glew/glew-1.13.0-${PLAT_LONG}/install")
file(GLOB OPENAL_PREFIX
    "${LIBACFUTILS}/openal-soft/openal-soft-1.21.0-${PLAT_LONG}/build/install")
file(GLOB CAIRO_PREFIX "${LIBACFUTILS}/cairo/cairo-${PLAT_LONG}")
file(GLOB PIXMAN_PREFIX "${LIBACFUTILS}/cairo/pixman-${PLAT_LONG}")
file(GLOB FREETYPE_PREFIX "${LIBACFUTILS}/freetype/freetype-${PLAT_LONG}")
file(GLOB PCRE2_PREFIX "${LIBACFUTILS}/pcre2/pcre2-${PLAT_LONG}")
file(GLOB CGLM_PREFIX "${LIBACFUTILS}/cglm/cglm-0.7.9")
file(GLOB LIBPNG_PREFIX "${LIBACFUTILS}/libpng/libpng-${PLAT_LONG}")
file(GLOB LIBXML_PREFIX "${LIBACFUTILS}/libxml2/libxml2-${PLAT_LONG}")
file(GLOB OPUSFILE_PREFIX "${LIBACFUTILS}/opus/opusfile-${PLAT_LONG}")
file(GLOB LIBOGG_PREFIX "${LIBACFUTILS}/opus/libogg-${PLAT_LONG}/install")
file(GLOB OPENSSL_PREFIX "${LIBACFUTILS}/ssl/openssl-${PLAT_LONG}")
file(GLOB ICONV_PREFIX "${LIBACFUTILS}/libiconv/libiconv-${PLAT_LONG}")

file(GLOB SRC *.c)
file(GLOB HDR *.h)

set(ALL_SRC ${SRC} ${HDR})
list(SORT ALL_SRC)

if(APPLE)
	add_executable(bp ${ALL_SRC})
else()
	add_library(bp SHARED ${ALL_SRC})
endif()

include_directories(bp PUBLIC
    "${LIBACFUTILS}/src"
    "${LIBACFUTILS}/acf_apis"
    "${LIBACFUTILS}/SDK/CHeaders/XPLM"
    "${LIBACFUTILS}/SDK/CHeaders/Widgets"
    "${LIBACFUTILS}/SDK/CHeaders/Wrappers"
    "${LIBPNG_PREFIX}/include"
    "${LIBXML_PREFIX}/include/libxml2"
    "${PCRE2_PREFIX}/include"
    "${CGLM_PREFIX}/include"
    "${OPENAL_PREFIX}/include/AL"
    "${CAIRO_PREFIX}/include/cairo"
    "${FREETYPE_PREFIX}/include/freetype2"
    "${GLEW_PREFIX}/include"
    )

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror --std=c99 \
    -Wno-unused-local-typedefs -fvisibility=hidden")
if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -arch x86_64 -arch arm64 \
	    -Wno-deprecated")
endif()

# Defines
add_definitions(-D_GNU_SOURCE)
add_definitions(-D_FILE_OFFSET_BITS=64)
add_definitions(-DDEBUG)
add_definitions(-DGLEW_BUILD=GLEW_STATIC -DGLEW_MX)
add_definitions(-DPCRE2_CODE_UNIT_WIDTH=8 -DPCRE2_STATIC)
add_definitions(-DLIBXML_STATIC)
add_definitions("-DCHECK_RESULT_USED=__attribute__((warn_unused_result))")

# Yes, we need to strip a trailing newline from command output. CMake is
# "smart" like that...
execute_process(COMMAND git describe --abbrev=0 --tags
    OUTPUT_VARIABLE BP_PLUGIN_VERSION)
string(REGEX REPLACE "\n$" "" BP_PLUGIN_VERSION "${BP_PLUGIN_VERSION}")

add_definitions(-DBP_PLUGIN_VERSION="${BP_PLUGIN_VERSION}")
add_definitions(-DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM302=1)
if(APPLE)
	add_definitions(-DAPL=1 -DIBM=0 -DLIN=0)
elseif(WIN32)
	add_definitions(-DAPL=0 -DIBM=1 -DLIN=0 -D_WIN32_WINNT=0x0600)
else()
	add_definitions(-DAPL=0 -DIBM=0 -DLIN=1)
endif()

# Linking
# Careful: order matters in target_link_libraries. Dependencies must be
# ordered as first symbol callers and then as symbol providers (i.e.
# libpng16 depends on libz, so linking must be first: libpng, then libz).
link_libraries(m)
file(GLOB LIBACFUTILS_LIBRARY
    "${LIBACFUTILS}/qmake/${PLAT_SHORT}/libacfutils.a")
file(GLOB LIBPNG_LIBRARY "${LIBPNG_PREFIX}/lib/libpng16.a")
file(GLOB ZLIB_LIBRARY "${LIBACFUTILS}/zlib/zlib-${PLAT_LONG}/lib/libz.a")
file(GLOB PCRE2_LIBRARY "${PCRE2_PREFIX}/lib/libpcre2-8.a")
file(GLOB OPUSFILE_LIBRARY  "${OPUSFILE_PREFIX}/lib/libopusfile.a")
file(GLOB OPUS_LIBRARY "${LIBACFUTILS}/opus/opus-${PLAT_LONG}/lib/libopus.a")
file(GLOB OGG_LIBRARY "${LIBOGG_PREFIX}/lib/libogg.a")
file(GLOB LIBXML2_LIBRARY "${LIBXML_PREFIX}/lib/libxml2.a")
if(WIN32)
	file(GLOB OPENAL_LIBRARY "${OPENAL_PREFIX}/lib/libOpenAL32.a")
else()
	file(GLOB OPENAL_LIBRARY "${OPENAL_PREFIX}/lib/libopenal.a")
endif()
file(GLOB FREETYPE_LIBRARY "${FREETYPE_PREFIX}/lib/libfreetype.a")
file(GLOB CAIRO_LIBRARY "${CAIRO_PREFIX}/lib/libcairo.a")
file(GLOB PIXMAN_LIBRARY  "${PIXMAN_PREFIX}/lib/libpixman-1.a")
if(WIN32)
	file(GLOB GLEW_LIBRARY "${GLEW_PREFIX}/lib/libglew32mx.a")
elseif(APPLE)
	file(GLOB GLEW_LIBRARY "${GLEW_PREFIX}/lib/libGLEWmx.a")
else()
	file(GLOB GLEW_LIBRARY "${GLEW_PREFIX}/lib64/libGLEWmx.a")
endif()
file(GLOB CURL_LIBRARY "${LIBACFUTILS}/curl/libcurl-${PLAT_LONG}/lib/libcurl.a")
file(GLOB OPENSSL_LIBRARY "${OPENSSL_PREFIX}/lib/libssl.a")
file(GLOB CRYPTO_LIBRARY "${OPENSSL_PREFIX}/lib/libcrypto.a")
file(GLOB ICONV_LIBRARY "${ICONV_PREFIX}/lib/libiconv.a")
file(GLOB LZMA_LIBRARY "${LIBACFUTILS}/lzma/qmake/${PLAT_LONG}/liblzma.a")

if(APPLE)
	find_library(XPLM_FRAMEWORK XPLM "${LIBACFUTILS}/SDK/Libraries/Mac")
	find_library(XPWIDGETS_FRAMEWORK XPWidgets
	    "${LIBACFUTILS}/SDK/Libraries/Mac")
	find_library(OPENGL_FRAMEWORK OpenGL)
	find_library(IOSURFACE_FRAMEWORK IOSurface)
	find_library(AUDIOUNIT_FRAMEWORK AudioUnit)
	find_library(COREAUDIO_FRAMEWORK CoreAudio)
	find_library(COCOA_FRAMEWORK Cocoa)
	# Required for IPv6 support in libcurl
	find_library(SYSTEM_CONFIGURATION_FRAMEWORK SystemConfiguration)
	set(EXTRA_PLATFORM_LIBS
	    ${XPLM_FRAMEWORK}
	    ${XPWIDGETS_FRAMEWORK}
	    ${OPENGL_FRAMEWORK}
	    ${IOSURFACE_FRAMEWORK}
	    ${COCOA_FRAMEWORK}
	    ${AUDIOUNIT_FRAMEWORK}
	    ${COREAUDIO_FRAMEWORK}
	    ${SYSTEM_CONFIGURATION_FRAMEWORK})
elseif(WIN32)
	find_library(XPLM_LIBRARY XPLM_64 "${LIBACFUTILS}/SDK/Libraries/Win")
	find_library(XPWIDGETS_LIBRARY XPWidgets_64
	    "${LIBACFUTILS}/SDK/Libraries/Win")
	set(EXTRA_PLATFORM_LIBS
	    ${XPLM_LIBRARY}
	    ${XPWIDGETS_LIBRARY}
	    "-lglu32"
	    "-lopengl32"
	    "-lws2_32"
	    "-lwinmm"
	    "-ldbghelp"
	    "-lpsapi"
	    "-lcrypt32")
else()
	set(EXTRA_PLATFORM_LIBS "")
endif()

target_link_libraries(bp
    ${LIBACFUTILS_LIBRARY}
    ${CAIRO_LIBRARY}
    ${PIXMAN_LIBRARY}
    ${FREETYPE_LIBRARY}
    ${LIBPNG_LIBRARY}
    ${GLEW_LIBRARY}
    ${OPENAL_LIBRARY}
    ${OPUSFILE_LIBRARY}
    ${OPUS_LIBRARY}
    ${OGG_LIBRARY}
    ${PCRE2_LIBRARY}
    ${LIBXML2_LIBRARY}
    ${CURL_LIBRARY}
    ${OPENSSL_LIBRARY}
    ${CRYPTO_LIBRARY}
    ${ICONV_LIBRARY}
    ${ZLIB_LIBRARY}
    ${LZMA_LIBRARY}
    ${EXTRA_PLATFORM_LIBS})

set_target_properties(bp PROPERTIES PREFIX "")
set_target_properties(bp PROPERTIES SUFFIX "")

if(APPLE)
	set_target_properties(bp PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -fvisibility=hidden -bundle \
	    -arch x86_64 -arch arm64")
elseif(WIN32)
	set_target_properties(bp PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -undefined_warning \
	    -fvisibility=hidden -static-libgcc -static-libstdc++")
else()
	set_target_properties(bp PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
	    -undefined_warning -fPIC -fvisibility=hidden \
	    -static-libgcc -static-libstdc++")
endif()
set_target_properties(bp PROPERTIES LINKER_LANGUAGE CXX)

set_target_properties(bp PROPERTIES RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/../${PLUGIN_BIN_OUTDIR}")
set_target_properties(bp PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/../${PLUGIN_BIN_OUTDIR}")
set_target_properties(bp PROPERTIES OUTPUT_NAME "BetterPushback.xpl")
