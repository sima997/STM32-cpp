#pragma once

#include "sensor.hpp"
#include "i2c_template_new.hpp"

using I2C1Bus = I2C<1, 128'000'000, 100'000>;

class ArduinoSensor : public Isensor {

public:

    
//Constructor
    ArduinoSensor(I2C1Bus& bus,uint8_t address, uint8_t sensorID);
    ~ArduinoSensor() override = default;

    SensorError init() override; //Sensor interface initialization
    SensorError read(uint32_t& out) override; //New sample read

private:
   
    

    
    enum class Command : uint8_t {
        Id = 0x0, //Sensor ID
        DataLow = 0x1, //Data Low
        DataHigh = 0x2 //Data high
               
    };

    I2C1Bus& bus_; //reference to i2c bus
    uint8_t address_;
    uint8_t sensorID_;
    

    SensorError mapI2CError (I2cError e) const;

};