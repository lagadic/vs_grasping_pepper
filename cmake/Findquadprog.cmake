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
# Try to find quadprog library
#
# QUADPROG_FOUND
# QUADPROG_INCLUDE_DIRS
# QUADPROG_LIBRARIES
# QUADPROG_VERSION
#
# Authors:
# Giovanni Claudio
#
#############################################################################

find_path(QUADPROG_INCLUDE_DIR QuadProg++.hh
  PATHS
    $ENV{QUADPROG_HOME}/include/QuadProg++/
)

 message(" find_path QUADPROG_INCLUDE_DIR     : $ENV{QUADPROG_HOME}/include/QuadProg++/")
 message(" Found in      : ${QUADPROG_INCLUDE_DIR}")

find_library(QUADPROG_LIBRARY
  NAMES quadprog
  PATHS
    $ENV{QUADPROG_HOME}/lib/
)

if(QUADPROG_LIBRARY)
  set(QUADPROG_LIBRARIES ${QUADPROG_LIBRARY})
endif()



set(QUADPROG_INCLUDE_DIRS
        ${QUADPROG_INCLUDE_DIR}
#$ENV{VISP_NAOQI_HOME}/include
)

if(QUADPROG_LIBRARIES AND QUADPROG_INCLUDE_DIRS)
  set(QUADPROG_FOUND TRUE)
  message("QUADPROG found")
  message("QUADPROG_LIBRARIES     : ${QUADPROG_LIBRARIES}")
  message("QUADPROG_INCLUDE_DIRS  : ${QUADPROG_INCLUDE_DIRS}")

else()
  set(QUADPROG_FOUND FALSE)
  message("QUADPROG NOT found")
endif()
  
mark_as_advanced(
  QUADPROG_INCLUDE_DIRS
  QUADPROG_LIBRARY
)
