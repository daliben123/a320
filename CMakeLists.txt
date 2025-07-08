zephyr_include_directories(.)
zephyr_library()
zephyr_library_sources(a320.c)
zephyr_library_compile_options(-O3) # 启用编译器最高优化
