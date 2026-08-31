#pragma once

#include "hardware/gpio.h"

struct ioqspi_status_ctrl_hw_t {
    io_rw_32 status;
    io_rw_32 ctrl;
};

struct ioqspi_hw_t {
    ioqspi_status_ctrl_hw_t io[6];
};

extern ioqspi_hw_t* ioqspi_hw;
