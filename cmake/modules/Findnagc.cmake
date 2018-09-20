# - Try to find ImageMagick++
# Once done, this will define
#
#  nagc_FOUND - system has pthread
#  nagc_INCLUDE_DIRS - the pthread include directories
#  nagc_LIBRARIES - link these to use pthread

set( NAME nagc )

include(LibFindMacros)

# Dependencies
# libfind_package(pthread windows)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules( ${NAME}_PKGCONF ${NAME} )

# Include dir
find_path( ${NAME}_INCLUDE_DIR
  NAMES nagx04.h
  PATHS 
      ${${NAME}_PKGCONF_INCLUDE_DIRS}
      c:/mingw/include
      ${CMAKE_INCLUDE_PATH}
      ${CMAKE_INSTALL_PREFIX}/include
      ENV CPLUS_INCLUDE_PATH
)

# Finally the library itself
find_library( ${NAME}_LIBRARY
  NAMES ${NAME}
  PATHS 
      ${${NAME}_PKGCONF_LIBRARY_DIRS}
      ${CMAKE_LIBRARY_PATH}
      c:/mingw/lib
      ${CMAKE_INSTALL_PREFIX}/lib
      ENV LD_LIBRARY_PATH
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set( ${NAME}_PROCESS_INCLUDES ${NAME}_INCLUDE_DIR )
set( ${NAME}_PROCESS_LIBS ${NAME}_LIBRARY )
libfind_process( ${NAME} )
