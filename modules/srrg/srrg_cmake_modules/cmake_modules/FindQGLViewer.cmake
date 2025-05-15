#ds check if qt5 is available - otherwise fallback to qt4
find_package(Qt5Xml QUIET)

# Need to find both Qt4 and QGLViewer if the QQL support is to be built
if(${Qt5Xml_FOUND})
  find_package(Qt5 COMPONENTS Xml OpenGL Gui)
  set(SRRG_QT_INCLUDE_DIRS
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS})
  set(SRRG_QT_LIBRARIES Qt5::OpenGL Qt5::Xml Qt5::Gui  )
else()
  find_package(Qt4 COMPONENTS QtXml QtOpenGL QtGui)
  set(SRRG_QT_INCLUDE_DIRS ${QT_INCLUDES})
  set(SRRG_QT_LIBRARIES
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTXML_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    )
endif()

find_path(QGLVIEWER_INCLUDE_DIR qglviewer.h
  /usr/include/QGLViewer
  /opt/local/include/QGLViewer
  /usr/local/include/QGLViewer
  /sw/include/QGLViewer
  )

if(${Qt5Xml_FOUND})
  find_library(QGLVIEWER_LIBRARY NAMES qglviewer qglviewer-qt5 qglviewer-dev-qt5 QGLViewer QGLViewer-qt5
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    /var/lib
    /usr/lib/x86_64-linux-gnu
    )
else()
  find_library(QGLVIEWER_LIBRARY NAMES qglviewer qglviewer-qt4 qglviewer-dev-qt4 QGLViewer QGLViewer-qt4
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    /var/lib
    /usr/lib/x86_64-linux-gnu
    )
endif()

if(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
  set(QGLVIEWER_FOUND TRUE)
else(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)
  set(QGLVIEWER_FOUND FALSE)
endif(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARY)

set(QGLViewer_FOUND ${QGLVIEWER_FOUND})
set(QGLViewer_INCLUDE_DIRS ${QGLVIEWER_INCLUDE_DIR})
set(QGLViewer_LIBRARIES ${QGLVIEWER_LIBRARY})
