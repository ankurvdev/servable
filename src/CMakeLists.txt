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

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bluetooth)
file(GLOB bluetoothhdrs "${ROOT}/lib/*.h")
foreach(hdr ${bluetoothhdrs})
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different "${hdr}" "${CMAKE_CURRENT_BINARY_DIR}/bluetooth" )
endforeach()
foreach(g ${GLOBS})
    file(GLOB l ${g})
    list(APPEND SRC_S ${l})
endforeach()

target_include_directories(servable PUBLIC
    ${ROOT}/lib
    ${ROOT}
    ${ROOT}/src
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
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
target_link_libraries(servable PRIVATE readline ${CMAKE_DL_LIBS} Threads::Threads)
target_compile_options(servable PRIVATE 
    -Wno-extern-c-compat
    -Wno-uninitialized
)
