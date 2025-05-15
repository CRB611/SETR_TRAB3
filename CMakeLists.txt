cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(SETR_TRAB3)

target_sources(main PRIVATE src/main.c)
target_sources(test PRIVATE tests/test.c)

