add_library(servable 
    BleApplication.cpp
    dbus.cpp
    plugin.c
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(deps REQUIRED IMPORTED_TARGET glib-2.0)
target_link_libraries(servable PRIVATE PkgConfig::deps)
target_compile_definitions(servable PRIVATE STORAGEDIR="." VERSION="1.0" CONFIGDIR="." PLUGINDIR=".")
set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../bluez)
set(SRC_S "")
set(GLOBS
    ${ROOT}/src/*.c
    ${ROOT}/lib/*.c
    ${ROOT}/attrib/*.c
    ${ROOT}/btio/*.c
    ${ROOT}/src/shared/*.c
)

foreach(g ${GLOBS})
    file(GLOB l ${g})
    list(APPEND SRC_S ${l})
endforeach()

target_include_directories(servable PUBLIC
    ${ROOT}/lib
    ${ROOT}
    ${ROOT}/src
    ${CMAKE_CURRENT_LIST_DIR}
)
#target_include_directories(servable PUBLIC ${CMAKE_CURRENT_LIST_DIR})

set(EXCLUDES
    "btp"
    "-ell"
    "/main.c"
    "mainloop.c"
    "gatttool.c"
    "plugin.c"
    "advertising.c"
)

foreach(ex ${EXCLUDES})
    list(FILTER SRC_S EXCLUDE REGEX ${ex})
endforeach()
target_sources(servable PRIVATE ${SRC_S})

find_package(Threads)
find_library(LIBUDEV udev)
target_link_libraries(servable PRIVATE ${LIBUDEV} readline ${CMAKE_DL_LIBS} Threads::Threads)
target_compile_options(servable PRIVATE 
    -Wno-extern-c-compat
    -Wno-uninitialized
)
