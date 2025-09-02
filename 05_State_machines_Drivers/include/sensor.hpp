//Sensor generic interface
#pragma once

#include <cstdint>

enum class SensorError {
    None = 0,
    Communication,
    SensorID,
    InvalidData,
    Timeout
};

class Isensor {
public:
    virtual ~Isensor() = default;
    virtual SensorError init() = 0; //return true on success
    virtual SensorError read(uint32_t& out) = 0; //read one sample, true if new data
};