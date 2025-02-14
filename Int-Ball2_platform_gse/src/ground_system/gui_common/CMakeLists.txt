cmake_minimum_required(VERSION 3.1.0)
project(gui_common)

find_package(catkin REQUIRED COMPONENTS roscpp rviz ib2_msgs platform_msgs communication_software)
catkin_package(
    INCLUDE_DIRS "src"
)

set(QT5_BASE_DIR "/opt/Qt/5/gcc_64")
include_directories(
  "src"
  "/usr/local/include"
  "${QT5_BASE_DIR}/include"
  #"${QT5_BASE_DIR}/include/Qt3DAnimation"
  #"${QT5_BASE_DIR}/include/Qt3DCore"
  #"${QT5_BASE_DIR}/include/Qt3DExtras"
  #"${QT5_BASE_DIR}/include/Qt3DInput"
  #"${QT5_BASE_DIR}/include/Qt3DLogic"
  #"${QT5_BASE_DIR}/include/Qt3DQuick"
  #"${QT5_BASE_DIR}/include/Qt3DQuickAnimation"
  #"${QT5_BASE_DIR}/include/Qt3DQuickExtras"
  #"${QT5_BASE_DIR}/include/Qt3DQuickInput"
  #"${QT5_BASE_DIR}/include/Qt3DQuickRender"
  #"${QT5_BASE_DIR}/include/Qt3DQuickScene2D"
  #"${QT5_BASE_DIR}/include/Qt3DRender"
  #"${QT5_BASE_DIR}/include/Qt3DStudioRuntime2"
  "${QT5_BASE_DIR}/include/QtAccessibilitySupport"
  "${QT5_BASE_DIR}/include/QtBluetooth"
  "${QT5_BASE_DIR}/include/QtConcurrent"
  "${QT5_BASE_DIR}/include/QtCore"
  "${QT5_BASE_DIR}/include/QtDBus"
  "${QT5_BASE_DIR}/include/QtDesigner"
  "${QT5_BASE_DIR}/include/QtDesignerComponents"
  "${QT5_BASE_DIR}/include/QtDeviceDiscoverySupport"
  "${QT5_BASE_DIR}/include/QtEdidSupport"
  "${QT5_BASE_DIR}/include/QtEglFSDeviceIntegration"
  "${QT5_BASE_DIR}/include/QtEglSupport"
  "${QT5_BASE_DIR}/include/QtEventDispatcherSupport"
  "${QT5_BASE_DIR}/include/QtFbSupport"
  "${QT5_BASE_DIR}/include/QtFontDatabaseSupport"
  "${QT5_BASE_DIR}/include/QtGamepad"
  "${QT5_BASE_DIR}/include/QtGlxSupport"
  "${QT5_BASE_DIR}/include/QtGui"
  "${QT5_BASE_DIR}/include/QtHelp"
  "${QT5_BASE_DIR}/include/QtInputSupport"
  "${QT5_BASE_DIR}/include/QtKmsSupport"
  "${QT5_BASE_DIR}/include/QtLinuxAccessibilitySupport"
  "${QT5_BASE_DIR}/include/QtLocation"
  "${QT5_BASE_DIR}/include/QtMultimedia"
  "${QT5_BASE_DIR}/include/QtMultimediaGstTools"
  "${QT5_BASE_DIR}/include/QtMultimediaQuick"
  "${QT5_BASE_DIR}/include/QtMultimediaWidgets"
  "${QT5_BASE_DIR}/include/QtNetwork"
  "${QT5_BASE_DIR}/include/QtNfc"
  "${QT5_BASE_DIR}/include/QtOpenGL"
  "${QT5_BASE_DIR}/include/QtOpenGLExtensions"
  "${QT5_BASE_DIR}/include/QtPacketProtocol"
  "${QT5_BASE_DIR}/include/QtPlatformCompositorSupport"
  "${QT5_BASE_DIR}/include/QtPlatformHeaders"
  "${QT5_BASE_DIR}/include/QtPositioning"
  "${QT5_BASE_DIR}/include/QtPositioningQuick"
  "${QT5_BASE_DIR}/include/QtPrintSupport"
  #"${QT5_BASE_DIR}/include/QtQml"
  #"${QT5_BASE_DIR}/include/QtQmlDebug"
  #"${QT5_BASE_DIR}/include/QtQuick"
  #"${QT5_BASE_DIR}/include/QtQuickControls2"
  #"${QT5_BASE_DIR}/include/QtQuickParticles"
  #"${QT5_BASE_DIR}/include/QtQuickShapes"
  #"${QT5_BASE_DIR}/include/QtQuickTemplates2"
  #"${QT5_BASE_DIR}/include/QtQuickTest"
  #"${QT5_BASE_DIR}/include/QtQuickWidgets"
  "${QT5_BASE_DIR}/include/QtRemoteObjects"
  "${QT5_BASE_DIR}/include/QtRepParser"
  "${QT5_BASE_DIR}/include/QtScxml"
  "${QT5_BASE_DIR}/include/QtSensors"
  "${QT5_BASE_DIR}/include/QtSerialBus"
  "${QT5_BASE_DIR}/include/QtSerialPort"
  "${QT5_BASE_DIR}/include/QtServiceSupport"
  "${QT5_BASE_DIR}/include/QtSql"
  "${QT5_BASE_DIR}/include/QtSvg"
  "${QT5_BASE_DIR}/include/QtTest"rviz_default_plugin
  "${QT5_BASE_DIR}/include/QtTextToSpeech"
  "${QT5_BASE_DIR}/include/QtThemeSupport"
  "${QT5_BASE_DIR}/include/QtUiPlugin"
  "${QT5_BASE_DIR}/include/QtUiTools"
  "${QT5_BASE_DIR}/include/QtWaylandClient"
  "${QT5_BASE_DIR}/include/QtWaylandCompositor"
  "${QT5_BASE_DIR}/include/QtWebChannel"
  "${QT5_BASE_DIR}/include/QtWebSockets"
  "${QT5_BASE_DIR}/include/QtWebView"
  "${QT5_BASE_DIR}/include/QtWidgets"
  "${QT5_BASE_DIR}/include/QtX11Extras"
  "${QT5_BASE_DIR}/include/QtXml"
  "${QT5_BASE_DIR}/include/QtXmlPatterns"
  "${QT5_BASE_DIR}/include/QtXmlPatterns"
  ${catkin_INCLUDE_DIRS})

link_directories(
  "${QT5_BASE_DIR}/lib"
  ${catkin_LIBRARY_DIRS})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

set(QT5_MODULES Widgets Quick)
find_package(
  Qt5
  REQUIRED CONFIG
  PATHS "${QT5_BASE_DIR}" NO_DEFAULT_PATH
  COMPONENTS ${QT5_MODULES}
)

## Add compile options
add_compile_options(-std=c++11 -fPIC -DQT_FORCE_ASSERTS)

file(GLOB_RECURSE SRC_FILES
    src/*.cpp
    src/model/*.cpp
    src/view/*.cpp
    src/exception/*.cpp
    src/ui/*.cpp
)

file(GLOB_RECURSE UI_FILES src/ui/*.ui)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
  ${UI_FILES}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp rviz_default_plugin)

# Qt5 modules
QT5_USE_MODULES(${PROJECT_NAME} ${QT5_MODULES})

## Install
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
