CMAKE_MINIMUM_REQUIRED(VERSION 3.16)
project(libcamera-sctp CXX)

include(FindPackageHandleStandardArgs)

if( NOT CMAKE_CXX_STANDARD )
	set (CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
set(CMAKE_LINK_WHAT_YOU_USE OFF)


# ----------------------------------------------------------------------------
# PkgConfig is required to find libcamera
#
find_package(PkgConfig QUIET) # REQUIRED
if ( NOT PkgConfig_FOUND )
	message(FATAL_ERROR "PkgConfig NOT FOUND, libcamera can not be detected.")
	return()
endif()


# ----------------------------------------------------------------------------
# libcamera
#
pkg_check_modules(LIBCAMERA QUIET libcamera)
if ( NOT LIBCAMERA_VERSION ) 
	message(WARNING "libcamera nor found. Install please libcamera package 
		https://aur.archlinux.org/packages/libcamera-git 
		https://libcamera.org")
    return()
else()
	message(STATUS "libcamera: ${LIBCAMERA_VERSION} ${LIBCAMERA_INCLUDE_DIRS} ${LIBCAMERA_LINK_LIBRARIES}")
	add_definitions(-DHAVE_LIBCAMERA=1)
endif()


# ----------------------------------------------------------------------------
# libconfig
#
set(LIBCONFIG_MINIMUM_VERSION_REQUIRED "1.7")
find_package(libconfig QUIET)
message(STATUS "libconfig [ ${LIBCONFIG_VERSION}, ${libconfig_VERSION}] found=${libconfig_FOUND}")
if ( NOT ${libconfig_FOUND} )
    message(FATAL_ERROR "libconfig version >= ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} is not found\n \
        Please install fresh libconfig version from repo or manually from github: https://github.com/hyperrealm/libconfig.git")
	return()
endif()
if ( libconfig_VERSION AND NOT LIBCONFIG_VERSION ) # Th case of the lellers vary between ubuntu and arch linux
    set(LIBCONFIG_VERSION ${libconfig_VERSION})
endif()
if ( NOT LIBCONFIG_VERSION )
    message(FATAL_ERROR "Can not determine libconfig version, seems libconfig cmake skript is broken.
        Please install fresh version of libconfig manually from github: https://github.com/hyperrealm/libconfig.git")
endif()
if ( LIBCONFIG_VERSION VERSION_LESS ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} )
    message(FATAL_ERROR "libconfig version >= ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} is required but libconfig version ${LIBCONFIG_VERSION} found.
        Please install fresh version of libconfig manually from github: https://github.com/hyperrealm/libconfig.git")
endif()

if ( NOT LIBCONFIG_INCLUDE_DIRS )
  if (WIN32)
    get_target_property(LIBCONFIG_INCLUDE_DIRS libconfig::libconfig INTERFACE_INCLUDE_DIRECTORIES)
  else()
    get_target_property(LIBCONFIG_INCLUDE_DIRS libconfig::config INTERFACE_INCLUDE_DIRECTORIES)
  endif()
endif()

if ( NOT LIBCONFIG_LIBRARIES ) 
  if (WIN32)
    set(LIBCONFIG_LIBRARIES libconfig::libconfig)
  else() 
    set(LIBCONFIG_LIBRARIES libconfig::config)
  endif()
endif()

include_directories(${LIBCONFIG_INCLUDE_DIRS})
message (STATUS "libconfig : ${LIBCONFIG_VERSION} ${LIBCONFIG_INCLUDE_DIRS} ${LIBCONFIG_LIBRARIES}")


# ----------------------------------------------------------------------------
# sctp
#
# Look for the header file (usually in /usr/include/netinet/sctp.h) for the library file (libsctp.so)
find_path(SCTP_INCLUDE_DIR NAMES netinet/sctp.h)
find_library(SCTP_LIBRARY NAMES sctp)
find_package_handle_standard_args(SCTP DEFAULT_MSG SCTP_LIBRARY SCTP_INCLUDE_DIR)

if(SCTP_FOUND)
    set(SCTP_LIBRARIES ${SCTP_LIBRARY})
    set(SCTP_INCLUDE_DIRS ${SCTP_INCLUDE_DIR})
	add_definitions(-DHAVE_SCTP=1)
    message(STATUS "SCTP found: ${SCTP_LIBRARIES} ${SCTP_INCLUDE_DIRS}")
else()
    message(STATUS "SCTP NOT found. install sctp package, on archlinux it is lksctp-tools")
	return()
endif()

# ----------------------------------------------------------------------------
# RPI version
#
# 1. Read the system model file directly into a variable
if(EXISTS "/proc/device-tree/model")
    file(READ "/proc/device-tree/model" SYSTEM_MODEL_INFO)

    # 2. Check if the string "Raspberry Pi 5" exists in the data
    string(FIND "${SYSTEM_MODEL_INFO}" "Raspberry Pi 5" PI5_FOUND)

    # 3. If found, the index will be >= 0
    if(NOT PI5_FOUND EQUAL -1)
        set(PI_VERSION_FLAG "--pi5")
        add_definitions(-DIS_PI5)
        message(STATUS "Raspberry Pi 5 detected")
    endif()
endif()


# ----------------------------------------------------------------------------

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)

file(GLOB_RECURSE SOURCE_FILES
	${SRC}/*.c
	${SRC}/*.cc
	${SRC}/*.cpp)

file(GLOB_RECURSE HEADER_FILES
	${SRC}/*.h
	${SRC}/*.hpp)

add_executable(${PROJECT_NAME}
	${SOURCE_FILES}
	${HEADER_FILES}
	${RESOURCE_FILES})

include_directories(${SRC} ${LIBCAMERA_INCLUDE_DIRS} ${LIBCONFIG_INCLUDE_DIRS} ${SCTP_INCLUDE_DIRS} ${CORE_INCLUDES})
target_link_libraries(${PROJECT_NAME} ${CORE_LIBRARIES} ${LIBCAMERA_LINK_LIBRARIES} ${LIBCONFIG_LIBRARIES} ${SCTP_LIBRARIES})


## Install app as a service
include(GNUInstallDirs)
set(CMAKE_INSTALL_MESSAGE ALWAYS)
set(BIN_INSTALL_DIR "${CMAKE_INSTALL_BINDIR}")
set(SERVICE_NAME ${PROJECT_NAME})
set(SERVICE_FILE "${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}.service")
set(SERVICE_PATH "/etc/systemd/system")

set(SERVICE_STRING
"[Unit]
Description=libcamera-sctp streamer
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
Restart=on-failure
RestartSec=1
ExecStart=${CMAKE_INSTALL_PREFIX}/${BIN_INSTALL_DIR}/${PROJECT_NAME}
#WorkingDirectory=/opt/${PROJECT_NAME}

[Install]
WantedBy=multi-user.target")

# Generate service file
file(GENERATE
     OUTPUT ${SERVICE_FILE}
     CONTENT ${SERVICE_STRING})

install(CODE "MESSAGE(\"Installing ${PROJECT_NAME} to ${CMAKE_INSTALL_PREFIX}/${BIN_INSTALL_DIR}...\")")
install(TARGETS ${PROJECT_NAME} DESTINATION ${BIN_INSTALL_DIR})

## Enable and start service
install(CODE "MESSAGE(\"\tAdding to systemd...\")")
install(FILES ${SERVICE_FILE}
        DESTINATION "${SERVICE_PATH}/"
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_EXECUTE)
#install(CODE "MESSAGE(\"\tEnabling service...\")")
#install(CODE "execute_process(COMMAND systemctl enable --now ${SERVICE_NAME})")
install(CODE "MESSAGE(\"Installing ${PROJECT_NAME}... Done.\")")
## ==================================================================================

add_custom_target(uninstall-${PROJECT_NAME}
    COMMAND xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt
    COMMAND xargs systemctl disable --now ${SERVICE_NAME}
    COMMAND xargs systemctl stop ${SERVICE_NAME}
    COMMAND systemctl daemon-reload)
