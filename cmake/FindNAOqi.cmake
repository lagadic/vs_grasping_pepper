# Copyright (c) 2013, Miguel Sarabia
# Imperial College London
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the Imperial College London nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


# - Try to find NAOqi
# Once executed this script will define the following:
#  NAOqi_FOUND - NAOqi was succesfully found
#  NAOqi_INCLUDE_DIRS - NAOqi's include directories
#  NAOqi_LIBRARIES - NAOqi's libraries
#  NAOqi_DIR - Directory where NAOqi was found
#------------------------------------------------------------------------------
# Users can set NAOqi_DIR to force CMake to look in a particular location,
# setting the AL_DIR environment variable will have a similar effect.

macro(vp_create_list_from_string STR LST)
  if(NOT ${STR} STREQUAL "")
    set(__lst ${STR})
    string(REPLACE " " ";" __lst ${__lst})
    list(REMOVE_ITEM __lst "")
    set(${LST} ${__lst})
  endif()
endmacro()

macro(vp_get_naoqi_version naoqi_include_path version)
  set(${version} "0.0.0")
  foreach(__inc ${${naoqi_include_path}})
    set(__filename "${__inc}/alcommon/version.h")
    if(EXISTS "${__filename}")
      file(STRINGS ${__filename} __matches REGEX "#define NAOQI_VERSION")
      vp_create_list_from_string(${__matches} __list)
      list(LENGTH __list __length)
      if(__length EQUAL 3)
        list(REVERSE __list)
        list(GET __list 0 ${version})
        string(REPLACE "\"" "" ${version} ${${version}})
        break()
      else()
        message("Warning: NAOQI_VERSION macro not found in ${__filename}. The following var ${version} is se
t to 0. This may produce build issues.")
      endif()
    endif()
  endforeach()
  if(${version} VERSION_EQUAL "0.0.0")
    message("Warning: NAOQI_VERSION macro not found in ${${naoqi_include_path}}. The following var ${version} is set to 0. This may produce build issues.")
  endif()
endmacro()

cmake_minimum_required(VERSION 2.8.3)

if(NOT NAOqi_DIR)
  set(NAOqi_DIR "" CACHE INTERNAL "NAOqi sdk location")
endif()

#Set INCLUDE hints
set(NAOqi_INCLUDE_HINTS
  "$ENV{NAOqi_DIR}/include"
  "${NAOqi_DIR}/include")

# Set LIBRARY hints
set(NAOqi_LIBRARY_HINTS
  "$ENV{NAOqi_DIR}/lib"
  "${NAOqi_DIR}/lib")

vp_get_naoqi_version(NAOqi_INCLUDE_HINTS NAOqi_VERSION)

#These are NAOqi's known components (ie. libraries)
set(NAOqi_COMPONENTS
    alaudio
    albonjourdiscovery
    alextractor
    allog
    almodelutils
    alproject
    alresource
    altools
    alautomatictest
    alboxrary
    alfile
    almathinternal
    almotion
    alpythonbridge
    alserial
    altts
    albehaviorinfo
    alcommon
    allauncher
    almath
    almotionrecorder
    alpythontools
    alsoap
    alvalue
    albehavior
    alerror
    allogremote
    almemoryfastaccess
    alparammanager
    alremotecall
    althread
    alvision
    alproxies
    qi
)

if(${NAOqi_VERSION} VERSION_LESS "2.3.0")
  list(APPEND NAOqi_COMPONENTS qitype)
endif()

# Find include directories
find_path(NAOqi_INCLUDE_DIR alcommon/alproxy.h HINTS ${NAOqi_INCLUDE_HINTS} )

# Verify we know about all the components requested
# and remove those we don't know about
set(NAOqi_FILTERED_COMPONENTS ${NAOqi_FIND_COMPONENTS})

if ( NAOqi_FIND_COMPONENTS )
    foreach(comp ${NAOqi_FIND_COMPONENTS})
        list(FIND NAOqi_COMPONENTS ${comp} ${comp}_KNOWN)
        if (${comp}_KNOWN EQUAL -1)
            list(REMOVE_ITEM NAOqi_FILTERED_COMPONENTS ${comp})
            message(STATUS "Unknown NAOqi component ${comp}")
        endif()
    endforeach()
endif()

list(LENGTH NAOqi_FILTERED_COMPONENTS NAOqi_NUMBER_OF_COMPONENTS)
set(NAOqi_FOUND_COMPONENTS TRUE)

# Look for components (ie. libraries)
if( ${NAOqi_NUMBER_OF_COMPONENTS}  )
    foreach(comp ${NAOqi_FILTERED_COMPONENTS})
        #Look for the actual library here
        find_library(${comp}_LIBRARY NAMES ${comp} HINTS ${NAOqi_LIBRARY_HINTS} NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH)
        if ( ${${comp}_LIBRARY} STREQUAL ${comp}_LIBRARY-NOTFOUND)
            message(STATUS "Could not find NAOqi's ${comp}")
            set(NAOqi_FOUND_COMPONENTS FALSE)
        else()
            #If everything went well append this component to list of libraries
            list(APPEND NAOqi_LIBRARY ${${comp}_LIBRARY})
        endif()
    endforeach()
else()
    message(STATUS "No NAOqi components specified")
endif()


# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    NAOqi #Package name
    DEFAULT_MSG
    # Variables required to evaluate as TRUE
    NAOqi_LIBRARY
    NAOqi_INCLUDE_DIR
    NAOqi_FOUND_COMPONENTS)

# Copy the values of the advanced variables to the user-facing ones
set(NAOqi_LIBRARIES ${NAOqi_LIBRARY} )
set(NAOqi_INCLUDE_DIRS ${NAOqi_INCLUDE_DIR} )
set(NAOqi_FOUND ${NAOQI_FOUND})

# If NAOqi was found, update NAOqi_DIR to show where it was found
if(NAOqi_FOUND)
  get_filename_component(NAOqi_NEW_DIR "${NAOqi_INCLUDE_DIRS}/../" ABSOLUTE)
endif()
set(NAOqi_DIR ${NAOqi_NEW_DIR} CACHE FILEPATH "NAOqi root directory" FORCE)

#Hide these variables
mark_as_advanced(NAOqi_INCLUDE_DIR NAOqi_LIBRARY NAOQI_FOUND)

message("NAOqi version     : ${NAOqi_VERSION}")
message("NAOqi include dirs: ${NAOqi_INCLUDE_DIRS}")
message("NAOqi libraries   : ${NAOqi_LIBRARIES}")
