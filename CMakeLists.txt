# 文件名：drivers/touchpad/CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
project(touchpad_driver LANGUAGES C)

# 添加驱动源文件[6,8](@ref)
add_library(touchpad_a320 STATIC
  touchpad_a320.c
)

# 链接Zephyr核心库[4,8](@ref)
target_link_libraries(touchpad_a320 PRIVATE
  zephyr_interface
)

# 包含Zephyr头文件路径[6](@ref)
target_include_directories(touchpad_a320 PRIVATE
  ${ZEPHYR_BASE}/include
  ${ZEPHYR_BASE}/drivers
)

# 设置编译选项[6,8](@ref)
target_compile_options(touchpad_a320 PRIVATE
  -Wall
  -Werror
)
