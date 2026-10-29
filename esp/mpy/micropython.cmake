# User C modules for our MicroPython (host/esp32/build.sh passes this as USER_C_MODULES).
add_library(usermod_cutter INTERFACE)
target_sources(usermod_cutter INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/cutter/modcutter.c
    ${CMAKE_CURRENT_LIST_DIR}/../cutter/cutter.c
)
target_include_directories(usermod_cutter INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/../cutter
)
target_link_libraries(usermod INTERFACE usermod_cutter)
