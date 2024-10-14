cmake_minimum_required(VERSION 3.10)

project(mtsTrueTechChamp)

find_package(cpr CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(task_1 task_1.cpp)
add_executable(task_2 task_2.cpp)
add_executable(task_3 task_3.cpp)

target_link_libraries(task_1 PRIVATE cpr::cpr)
target_link_libraries(task_1 PRIVATE nlohmann_json::nlohmann_json)

target_link_libraries(task_2 PRIVATE cpr::cpr)
target_link_libraries(task_2 PRIVATE nlohmann_json::nlohmann_json)

target_link_libraries(task_3 PRIVATE cpr::cpr)
target_link_libraries(task_3 PRIVATE nlohmann_json::nlohmann_json)