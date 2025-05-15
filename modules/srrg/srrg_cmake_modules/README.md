# srrg_cmake_modules

This package contains the cmake modules needed to compile the srrg packages


## Setting up
To prevent version clashes between libraries, we unified the build system.
We encountered several problems when having Qt4/Qt5 simultaneously linked.

If not absolutely necessary, use the find_packages in this folder
within your project
add the following line to your CMakeLists.txt

```
find_package(srrg_cmake_modules REQUIRED)
set(CMAKE_MODULE_PATH ${srrg_cmake_modules_INCLUDE_DIRS})
```

If you need to include qt, they are automatically included
with qglviewer

add this to your top level CMakeLists.txt

```
find_package(QGLViewer REQUIRED)
include_directories(${QGLVIEWER_INCLUDE_DIR})
include_directories(${SRRG_QT_INCLUDE_DIRS})
```

When linking qt/qglviewer, specify in your target

```
target_link_libraries( my_target
  <other stuff>
  ${QGLVIEWER_LIBRARY} 
  ${SRRG_QT_LIBRARIES}
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
)
```

That's all.

## Authors

** Giorgio Grisetti 

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

BSD 2.0 
