FIND_PACKAGE(Doxygen)

IF(DOXYGEN_FOUND)

SET(DOXYGEN_SOURCE_DIR ${CMAKE_SOURCE_DIR})

SET(DOXYGEN_OUTPUT_DIR html)
STRING(REGEX REPLACE ";" " " CMAKE_DOXYGEN_INPUT_LIST "${DOXYGEN_SOURCE_DIR}")

CONFIGURE_FILE(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
SET(HTML_TARGET "html")

ADD_CUSTOM_TARGET(${HTML_TARGET} ALL
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

ENDIF(DOXYGEN_FOUND)
