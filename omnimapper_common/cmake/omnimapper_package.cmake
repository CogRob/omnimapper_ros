#
# Standard Omnimapper project setup
#
# @public
#
macro(omnimapper_package)

  # Default to C99
  if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
      -Wall
      -Wextra
      -Wpedantic
      -Werror
      -Wdeprecated
    )
  endif()
endmacro()
