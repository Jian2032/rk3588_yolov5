#ifndef ROBOTIC_ARM_CONTROL_H
#define ROBOTIC_ARM_CONTROL_H

#include <iostream>
#include <unistd.h>
#include "cpp_interface/nrc_interface.h"
#include "cpp_interface/nrc_queue_operate.h"
#include "cpp_interface/nrc_job_operate.h"

void power_on(int fd);
void wait_for_running_over(int fd);

#endif