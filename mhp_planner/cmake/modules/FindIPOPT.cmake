# Try to locate the IPOPT library
#
# If the IPOPT_DIR is set, try to locate the package in the given
# directory, otherwise use pkg-config to locate it
#
# Create the following variables:
# IPOPT_INCLUDE_DIRS - Directories to include to use IPOPT
# IPOPT_LIBRARIES    - Default library to link against to use IPOPT
# IPOPT_DEFINITIONS  - Flags to be added to compiler's options
# IPOPT_LINK_FLAGS   - Flags to be added to linker's options
# IPOPT_FOUND        - If false, don't try to use IPOPT

# Copyright (C) 2008-2010  RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


if(APPLE)

    # On APPLE we use PkgConfig to find IPOPT
    # TODO use it on UNIX as well
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)

        if(IPOPT_FIND_VERSION)
            if(IPOPT_FIND_VERSION_EXACT)
                pkg_check_modules(_PC_IPOPT QUIET ipopt=${IPOPT_FIND_VERSION})
            else()
                pkg_check_modules(_PC_IPOPT QUIET ipopt>=${IPOPT_FIND_VERSION})
            endif()
        else()
            pkg_check_modules(_PC_IPOPT QUIET ipopt)
        endif()


        if(_PC_IPOPT_FOUND)
            set(IPOPT_INCLUDE_DIRS ${_PC_IPOPT_INCLUDE_DIRS} CACHE PATH "IPOPT include directory")
            set(IPOPT_DEFINITIONS ${_PC_IPOPT_CFLAGS_OTHER} CACHE STRING "Additional compiler flags for IPOPT")
            set(IPOPT_LIBRARIES "" CACHE STRING "IPOPT libraries" FORCE)
            foreach(_LIBRARY IN ITEMS ${_PC_IPOPT_LIBRARIES})
                find_library(${_LIBRARY}_PATH
                            NAMES ${_LIBRARY}
                            PATHS ${_PC_IPOPT_LIBRARY_DIRS})
                    list(APPEND IPOPT_LIBRARIES ${${_LIBRARY}_PATH})
            endforeach(_LIBRARY)
        else()
            set(IPOPT_DEFINITIONS "")
        endif()

    endif()

    set(IPOPT_LINK_FLAGS "")

elseif(UNIX OR MINGW)

    find_package(PkgConfig QUIET)
    pkg_check_modules(_PC_IPOPT QUIET ipopt)

    if(_PC_IPOPT_FOUND)
        set(IPOPT_INCLUDE_DIRS ${_PC_IPOPT_INCLUDE_DIRS} CACHE PATH "IPOPT include directory")
        set(IPOPT_DEFINITIONS ${_PC_IPOPT_CFLAGS_OTHER} CACHE STRING "Additional compiler flags for IPOPT")
        set(IPOPT_LIBRARIES "" CACHE STRING "IPOPT libraries" FORCE)
        foreach(_LIBRARY IN ITEMS ${_PC_IPOPT_LIBRARIES})
            find_library(${_LIBRARY}_PATH
                        NAMES ${_LIBRARY}
                        PATHS ${_PC_IPOPT_LIBRARY_DIRS})
                list(APPEND IPOPT_LIBRARIES ${${_LIBRARY}_PATH})
        endforeach(_LIBRARY)
    else()
        set(IPOPT_DEFINITIONS "")
    endif()

    set(IPOPT_LINK_FLAGS "")

# Windows platforms
else()

    set(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")

    set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/include/coin)
    find_library(IPOPT_LIBRARIES_RELEASE libipopt  ${IPOPT_DIR}/lib
                                                   ${IPOPT_DIR}/lib/coin
                                                   NO_DEFAULT_PATH)
    find_library(IPOPT_LIBRARIES_DEBUG   libipoptD ${IPOPT_DIR}/lib
                                                   ${IPOPT_DIR}/lib/coin
                                                   NO_DEFAULT_PATH)


        #find_library(IPOPT_COINHSL_RELEASE libcoinhsl  ${IPOPT_DIR}/lib
    #                                               ${IPOPT_DIR}/lib/coin
    #                                               NO_DEFAULT_PATH)

        #find_library(IPOPT_MUMPS_RELEASE libcoinmumps  ${IPOPT_DIR}/lib
    #                                                  ${IPOPT_DIR}/lib/coin
    #                                                  NO_DEFAULT_PATH)

        #find_library(IPOPT_LAPACK_RELEASE libcoinlapack  ${IPOPT_DIR}/lib
        #												   ${IPOPT_DIR}/lib/coin
        #												   NO_DEFAULT_PATH)

    #find_library(IPOPT_BLAS_RELEASE libcoinblas  ${IPOPT_DIR}/lib
    #										   ${IPOPT_DIR}/lib/coin
    #										   NO_DEFAULT_PATH)

    #find_library(IPOPT_METIS_RELEASE libcoinmetis  ${IPOPT_DIR}/lib
        #										   ${IPOPT_DIR}/lib/coin
        #										   NO_DEFAULT_PATH)


    if(IPOPT_LIBRARIES_RELEASE AND IPOPT_LIBRARIES_DEBUG)
        set(IPOPT_LIBRARIES optimized ${IPOPT_LIBRARIES_RELEASE} debug ${IPOPT_LIBRARIES_DEBUG})
    elseif(IPOPT_LIBRARIES_RELEASE)
        set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES_RELEASE})
    elseif(IPOPT_LIBRARIES_DEBUG)
        set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES_DEBUG})
    endif()


        #set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${IPOPT_COINHSL_RELEASE} ${IPOPT_MUMPS_RELEASE} ${IPOPT_LAPACK_RELEASE} ${IPOPT_BLAS_RELEASE} ${IPOPT_METIS_RELEASE} -lgfortran)

    set(IPOPT_LIBRARIES_RELEASE "")
    set(IPOPT_LIBRARIES_DEBUG "")

    set(IPOPT_DEFINITIONS "")
    if(MSVC)
        #set(IPOPT_LINK_FLAGS "/NODEFAULTLIB:libcmt.lib;libcmtd.lib")
    else()
        set(IPOPT_LINK_FLAGS "")
    endif()

endif()

mark_as_advanced(IPOPT_INCLUDE_DIRS
                 IPOPT_LIBRARIES
                 IPOPT_DEFINITIONS
                 IPOPT_LINK_FLAGS)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(IPOPT DEFAULT_MSG IPOPT_LIBRARIES)




