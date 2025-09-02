#include "arduino_sensor.hpp"


ArduinoSensor::ArduinoSensor(I2C1Bus& bus,uint8_t address, uint8_t sensorID) : bus_(bus), address_(address), sensorID_(sensorID) {

}

SensorError ArduinoSensor::init() {

    //Initialize I2C
    auto e = bus_.init();
    if(e != I2cError::None) {
        return mapI2CError(e);
    }


    //Read sensor ID
    uint8_t id = 0;
    e = (bus_.write_read(address_,static_cast<uint8_t>(Command::Id), id) );
        
    if(e != I2cError::None) {
        return mapI2CError(e);
    }

    if(id != sensorID_) {
        return SensorError::SensorID;
    }

    return SensorError::None;
}


SensorError ArduinoSensor::read(uint32_t& out) {
    uint8_t low = 0, high = 0;
   
    //Read Low byte
    auto e = bus_.write_read(address_,static_cast<uint8_t>(Command::DataLow), low);
    if( e != I2cError::None) {
        return mapI2CError(e);
    }

    //Read High byte
    e = bus_.write_read(address_,static_cast<uint8_t>(Command::DataHigh), high);
    if( e != I2cError::None) {
        return mapI2CError(e);
    }

    //Combine and check resolution (12-bit sensor)
    uint32_t measurement = static_cast<uint32_t>(low) | (static_cast<uint32_t>(high) << 8U);
    if( measurement < (1U << 12)) {
        return SensorError::InvalidData;
    }

    out = measurement;
    return SensorError::None;
}   

SensorError ArduinoSensor::mapI2CError(I2cError e) const {
    switch(e) {
        case I2cError::None:        return SensorError::None;
        case I2cError::Timeout:     return SensorError::Timeout;
        default:                    return SensorError::Communication;
    }
}