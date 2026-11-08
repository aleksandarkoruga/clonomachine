# User C modules for our MicroPython (host/esp32/build.sh passes this as USER_C_MODULES).
add_library(usermod_cutter INTERFACE)
target_sources(usermod_cutter INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/cutter/modcutter.c
    ${CMAKE_CURRENT_LIST_DIR}/../cutter/cutter.c
    ${CMAKE_CURRENT_LIST_DIR}/../harmony/harmony.c
    ${CMAKE_CURRENT_LIST_DIR}/../../fw/scales.c
)
target_include_directories(usermod_cutter INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/../cutter
    ${CMAKE_CURRENT_LIST_DIR}/../harmony
)
target_link_libraries(usermod INTERFACE usermod_cutter)
