# manage wheel task
## notification bits
first 10 bits are for checking for wheel data update
11th bit is for updating number of wheel
12th bit is for wating for destructor of wheel object to finish



# Improvements to make
## communicationInterface
    - 

## mpu 6050 reader
    - remove shared data pointer and use local variable instead.

## struct serialiation
    - use std::array instead of std::vector
