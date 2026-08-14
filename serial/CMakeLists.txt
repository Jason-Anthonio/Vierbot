cmake_minimum_required(VERSION 3.5)
project(serial)

find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

if(APPLE)
    target_sources(${PROJECT_NAME} PRIVATE src/impl/unix.cc src/impl/list_ports/list_ports_osx.cc)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_link_libraries(${PROJECT_NAME} ${IOKIT_LIBRARY} ${FOUNDATION_LIBRARY})
elseif(UNIX)
    target_sources(${PROJECT_NAME} PRIVATE src/impl/unix.cc src/impl/list_ports/list_ports_linux.cc)
    target_link_libraries(${PROJECT_NAME} rt pthread)
else()
    target_sources(${PROJECT_NAME} PRIVATE src/impl/win.cc src/impl/list_ports/list_ports_win.cc)
    target_link_libraries(${PROJECT_NAME} setupapi)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_package()
