cmake_minimum_required(VERSION 3.21)

# Subdirectories
add_subdirectory(drivers)

# TODO FAS - don't think I need the below
# Include headers
zephyr_include_directories(drivers)
