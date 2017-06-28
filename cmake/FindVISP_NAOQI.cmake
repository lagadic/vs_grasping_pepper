#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find visp_naoqi library
#
# VISP_NAOQI_FOUND
# VISP_NAOQI_INCLUDE_DIRS
# VISP_NAOQI_LIBRARIES
# VISP_NAOQI_VERSION
#
# Authors:
# Giovanni Claudio
#
#############################################################################

find_path(VISP_NAOQI_INCLUDE_DIR visp_naoqi/vpNaoqiGrabber.h
  PATHS
    $ENV{VISP_NAOQI_HOME}/../include
)

find_library(VISP_NAOQI_LIBRARY
  NAMES visp_naoqi
  PATHS
    $ENV{VISP_NAOQI_HOME}
)
find_library(METAPOD_ROMEO_LIBRARY
  NAMES metapod_romeo
  PATHS
    $ENV{VISP_NAOQI_HOME}
)
find_library(METAPOD_PEPPER_LIBRARY
  NAMES metapod_pepper
  PATHS
    $ENV{VISP_NAOQI_HOME}
)

if(VISP_NAOQI_LIBRARY)
  set(VISP_NAOQI_LIBRARIES ${VISP_NAOQI_LIBRARY})
endif()

if(METAPOD_ROMEO_LIBRARY)
  list(APPEND VISP_NAOQI_LIBRARIES ${METAPOD_ROMEO_LIBRARY})
endif()

if(METAPOD_PEPPER_LIBRARY)
  list(APPEND VISP_NAOQI_LIBRARIES ${METAPOD_PEPPER_LIBRARY})
endif()

set(VISP_NAOQI_INCLUDE_DIRS
        ${VISP_NAOQI_INCLUDE_DIR}
#$ENV{VISP_NAOQI_HOME}/include
)

if(VISP_NAOQI_LIBRARIES AND VISP_NAOQI_INCLUDE_DIRS)
  set(VISP_NAOQI_FOUND TRUE)
  message("VISP_NAOQI found")
  message("VISP_NAOQI_LIBRARIES     : ${VISP_NAOQI_LIBRARIES}")
  message("VISP_NAOQI_INCLUDE_DIRS  : ${VISP_NAOQI_INCLUDE_DIRS}")

else()
  set(VISP_NAOQI_FOUND FALSE)
  message("VISP_NAOQI NOT found")
endif()
  
mark_as_advanced(
  VISP_NAOQI_INCLUDE_DIRS
  VISP_NAOQI_LIBRARY
  METAPOD_NAOQI_LIBRARY
)
