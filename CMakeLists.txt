cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(SETR_TRAB3)

target_sources(app PRIVATE
    src/main.c
    modules/tc74.c       # módulo I²C/TC74
    modules/rtdb.c       # módulo RTDB
    modules/pid.c
    modules/heater.c
    modules/uart.c
)

