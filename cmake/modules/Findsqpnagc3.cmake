# - Try to find ImageMagick++
# Once done, this will define
#
#  yane_FOUND - system has pthread
#  yane_INCLUDE_DIRS - the pthread include directories
#  yane_LIBRARIES - link these to use pthread
#  yane_VERSION - version_number (in format 1.5.3) based on libversion_yane

set( NAME sqpnagc3 )

include(LibFindMacros)

# Dependencies
# libfind_package(pthread windows)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules( ${NAME}_PKGCONF ${NAME} )

# Include dir
find_path( ${NAME}_INCLUDE_DIR
  NAMES ${NAME}.h
  PATHS
      ${${NAME}_PKGCONF_INCLUDE_DIRS}
      c:/mingw/include
      ${CMAKE_INCLUDE_PATH}
      ${CMAKE_INSTALL_PREFIX}/include
)

# Finally the library itself
find_library( ${NAME}_LIBRARY
  NAMES ${NAME}
  PATHS
      ${${NAME}_PKGCONF_LIBRARY_DIRS}
      ${CMAKE_LIBRARY_PATH}
      c:/mingw/lib
      ${CMAKE_INSTALL_PREFIX}/lib
)

find_program( ${NAME}_VPROG NAMES versioninfo-${NAME} PATHS ${CMAKE_INSTALL_PREFIX}/bin )
exec_program( ${${NAME}_VPROG} OUTPUT_VARIABLE ostring RETURN_VALUE retval )
string( REGEX MATCHALL "(([0-9]+)[.])+([0-9]+)" ${NAME}_VERSION ${ostring} )
#message( STATUS "${NAME}_VERSION = ${${NAME}_VERSION}" )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set( ${NAME}_PROCESS_INCLUDES ${NAME}_INCLUDE_DIR )
set( ${NAME}_PROCESS_LIBS ${NAME}_LIBRARY)
libfind_process( ${NAME} )