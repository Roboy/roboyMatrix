# - Find the matrix includes and libraries
# This module defines
#  matrix_INCLUDE_DIRS
#  matrix_LIB, the lib required to use matrix
#  matrix_FOUND


FIND_LIBRARY(matrix_LIB matrix_creator_hal
        /usr/local/lib)

SET(matrix_INCLUDE_DIRS
        /usr/local/include/matrix_hal
      )

MARK_AS_ADVANCED(
        matrix_INCLUDE_DIRS
        matrix_LIB)

SET( matrix_FOUND "NO" )
IF(matrix_LIB)
    SET( matrix_FOUND "YES" )
ENDIF(matrix_LIB)

IF(matrix_FOUND)
    MESSAGE(STATUS "Found matrix library")
    MESSAGE(STATUS "matrix_FOUND include dir: ${matrix_INCLUDE_DIRS}" )
    MESSAGE(STATUS "matrix_FOUND lib: ${matrix_LIB}" )
ELSE(matrix_FOUND)
    IF(matrix_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find matrix,
-- please make sure the matrix is installed on your system https://matrix-io.github.io/matrix-documentation/")
    ENDIF(matrix_FIND_REQUIRED)
ENDIF(matrix_FOUND)