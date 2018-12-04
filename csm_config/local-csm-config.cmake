# Configure package config file
configure_file(
  ${CMAKE_SOURCE_DIR}/cmake/csmConfig.cmake.in
  ${CMAKE_BINARY_DIR}/cmake/csmConfig.cmake @ONLY
)



set(ConfigPackageLocation lib/csm)

install(
  FILES
    ${CMAKE_BINARY_DIR}/cmake/csmConfig.cmake
  DESTINATION
    ${ConfigPackageLocation})

