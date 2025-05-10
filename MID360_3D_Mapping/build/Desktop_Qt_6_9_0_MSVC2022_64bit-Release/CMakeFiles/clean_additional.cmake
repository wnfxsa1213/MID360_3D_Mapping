# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "CMakeFiles\\MID360_3D_Mapping_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\MID360_3D_Mapping_autogen.dir\\ParseCache.txt"
  "MID360_3D_Mapping_autogen"
  )
endif()
