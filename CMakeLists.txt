#########################################################################
#                                                                       #
#   SECURE DRONE SWARM SYSTEM - CORE MODULE                             #
#                                                                       #
#   Developer : Md Shahanur Islam Shagor                                #
#   Role      : Project Architect & Lead Developer                      #
#   Version   : 1.0.2                                                   #
#   Status    : Production Ready                                        #
#                                                                       #
#   "Protecting the skies with decentralized intelligence."             #
#                                                                       #
#########################################################################

cmake_minimum_required(VERSION 3.10)
project(DroneSwarmController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(Threads REQUIRED)
option(BUILD_DRONE_TEST "Build the hardware-oriented drone_test executable" OFF)

add_library(drone_controller SHARED
    dronecontroller.cpp
    dronecontroller.h
)

if(MSVC)
    target_compile_definitions(drone_controller PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

target_include_directories(drone_controller PUBLIC
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(drone_controller PRIVATE
    Threads::Threads
)

if(BUILD_DRONE_TEST)
    add_executable(drone_test
        main_test.cpp
    )

    target_link_libraries(drone_test PRIVATE
        drone_controller
    )
endif()

install(TARGETS drone_controller
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES dronecontroller.h
    DESTINATION include/drone_swarm
)
