# Create a library for the module
#   - Name of library
#   - Type (STATIC, SHARED, MODULE)
#   - Source file
add_library(gnss_module STATIC
    src/gnss.c
)

# Make the include directory available
#   - Name of library
#   - Library is available to everyone (PUBLIC, INTERFACE, PUBLIC)
#   - Outside data available to other programs
target_include_directories(gnss_module PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)