#pragma once

#include <cstdint>
#include <span>
#include "stm32g431xx.h"
#include "gpio_template.hpp"

//typedef uint32_t size_t;

/**
 * I2C Instance 1
 * SDA: PB9
 * SCL: PB8
 * 7-bit addressing mode
 */

 
//General implementation for I2C pins
template<int Instance>
struct I2cPins;

//Implementation for I2C instance 1
template<> struct I2cPins<1> {
    using SDA = Gpio<'B', 9, GpioMode::Alt, GpioSpeed::VeryHigh, GpioPull::None, GpioAF::AF4, GpioOType::OpenDrain>;
    using SCL = Gpio<'B', 8, GpioMode::Alt, GpioSpeed::VeryHigh, GpioPull::None, GpioAF::AF4, GpioOType::OpenDrain>;

    //Get handler
    static I2C_TypeDef* i2c() { return I2C1; };

    //Enable clock to I2C
    static void enableClock() { RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; }
};


template<int Instance>
struct I2c {
    static inline I2C_TypeDef* I2C() { return I2cPins<Instance>::i2c(); }

    static void init() {
        //Initialize pins
        I2cPins<Instance>::SDA::init();
        I2cPins<Instance>::SCL::init();

        //Enable clock to I2C
        I2cPins<Instance>::enableClock();

        //Disable I2C peripheral (reset control registers)
        I2C()->CR1 = 0; 
        I2C()->CR2 = 0; 

        //I2C clock 100kHz @ APB1CLK = CPU_Clk/2 = 170Mhz/2 = 85 Mhz; 
        RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL;
        RCC->CCIPR |= RCC_CCIPR_I2C1SEL_0;
        I2C()->TIMINGR = 0x20D09DEB;

        //Enable I2C peripheral
        I2C()->CR1 |= I2C_CR1_PE;

    }

    static bool write(uint8_t address, const uint8_t* data, uint8_t len) {
        //Write address, data lenght and controller mode (auto end)
        I2C()->CR2 = (address << 1) | (len << 16) | I2C_CR2_AUTOEND;
        //Generate start condition
        I2C()->CR2 |= I2C_CR2_START;      

        //Send data
        for(uint8_t i = 0; i < len; i++) {
            while(!(I2C()->ISR & (I2C_ISR_TXIS |  I2C_ISR_NACKF))) {/*Spin until TXE or NACKF*/}
             if( I2C()->ISR & I2C_ISR_NACKF) {
                    //Clear NACK flag
                    I2C()->ICR = I2C_ICR_NACKCF;
                    //Wait for stop
                    while(!(I2C()->ISR & I2C_ISR_STOPF));
                    //Clear STOP flag
                    I2C()->ICR = I2C_ICR_STOPCF;
                    
                    return false; //Failed transmision
                }

                //Write next byte
                I2C()->TXDR = data[i];
            }
        //Wait for stop flag
        while(!(I2C()->ISR & I2C_ISR_STOPF));
        //Clear stop flag
        I2C()->ICR = I2C_ICR_STOPCF;
        
        return true;
    }
    

    static bool read(uint8_t address, uint8_t *rx_data, uint8_t len) {
        I2C()->CR2 = (address << 1) | (len << 16) | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
        //Generate start condition
        I2C()->CR2 |= I2C_CR2_START; 

        for(uint8_t i = 0; i < len; i++) {
            while(!(I2C()->ISR & (I2C_ISR_RXNE |  I2C_ISR_NACKF))) {/*Spin until RXNE or NACKF*/}
            if( I2C()->ISR & I2C_ISR_NACKF) {
                    //Clear NACK flag
                    I2C()->ICR = I2C_ICR_NACKCF;
                    //Wait for stop
                    while(!(I2C()->ISR & I2C_ISR_STOPF));
                    //Clear STOP flag
                    I2C()->ICR = I2C_ICR_STOPCF;
                    
                    return false; //Failed transmision
                }
                //Read data byte
                rx_data[i] = static_cast<uint8_t>(I2C()->RXDR);
            }
            
        
        //Wait for stop flag
        while(!(I2C()->ISR & I2C_ISR_STOPF));
        //Clear stop flag
        I2C()->ICR = I2C_ICR_STOPCF;
        
        return true;
    }

    static bool write_read(uint8_t address, uint8_t *tx_data, uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len) {
        //Write phase
        //Write address, data lenght and controller mode (auto end)
        I2C()->CR2 = (address << 1) | (tx_len << 16);
        //Generate start condition
        I2C()->CR2 |= I2C_CR2_START;      

        //Send data
        for(uint8_t i = 0; i < tx_len; i++) {
            while(!(I2C()->ISR & (I2C_ISR_TXIS |  I2C_ISR_NACKF))) {/*Spin until TXE or NACKF*/}
             if( I2C()->ISR & I2C_ISR_NACKF) {
                    //Clear NACK flag
                    I2C()->ICR = I2C_ICR_NACKCF;
                    //Wait for stop
                    while(!(I2C()->ISR & I2C_ISR_STOPF));
                    //Clear STOP flag
                    I2C()->ICR = I2C_ICR_STOPCF;
                    
                    return false; //Failed transmision
                }

                //Write next byte
                I2C()->TXDR = tx_data[i];
            }
            
        //Wait for repeated start ready
        while(!(I2C()->ISR & I2C_ISR_TC));

        //Read phase
        I2C()->CR2 = (address << 1) | (rx_len << 16) | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
        //Generate start condition
        I2C()->CR2 |= I2C_CR2_START; 

        for(uint8_t i = 0; i < rx_len; i++) {
            while(!(I2C()->ISR & (I2C_ISR_RXNE |  I2C_ISR_NACKF))) {/*Spin until RXNE or NACKF*/}
            if( I2C()->ISR & I2C_ISR_NACKF) {
                    //Clear NACK flag
                    I2C()->ICR = I2C_ICR_NACKCF;
                    //Wait for stop
                    while(!(I2C()->ISR & I2C_ISR_STOPF));
                    //Clear STOP flag
                    I2C()->ICR = I2C_ICR_STOPCF;
                    
                    return false; //Failed transmision
                }
                //Read data byte
                rx_data[i] = static_cast<uint8_t>(I2C()->RXDR);
            }
            
        
        //Wait for stop flag
        while(!(I2C()->ISR & I2C_ISR_STOPF));
        //Clear stop flag
        I2C()->ICR = I2C_ICR_STOPCF;
        
        return true;
    }



};