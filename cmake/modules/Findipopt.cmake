# - Try to find ImageMagick++
# Once done, this will define
#
#  ipopt_FOUND - system has pthread
#  ipopt_INCLUDE_DIRS - the pthread include directories
#  ipopt_LIBRARIES - link these to use pthread

include(LibFindMacros)

# Dependencies
# libfind_package(pthread windows)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules( ipopt_PKGCONF ipopt )

# Include dir
find_path( ipopt_INCLUDE_DIR
  NAMES IpIpoptApplication.hpp
  PATHS 
      ${ipopt_PKGCONF_INCLUDE_DIRS}
      c:/mingw/include
      ${CMAKE_INCLUDE_PATH}
      ${CMAKE_INSTALL_PREFIX}/include
      ENV CPLUS_INCLUDE_PATH
)

# Finally the library itself
find_library( ipopt_LIBRARY
  NAMES ipopt
  PATHS 
      ${ipopt_PKGCONF_LIBRARY_DIRS}
      ${CMAKE_LIBRARY_PATH}
      c:/mingw/lib
      ${CMAKE_INSTALL_PREFIX}/lib
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set( ipopt_PROCESS_INCLUDES ipopt_INCLUDE_DIR )
set( ipopt_PROCESS_LIBS ipopt_LIBRARY )
libfind_process( ipopt )
