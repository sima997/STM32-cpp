#pragma once

#include <cstdint>
#include <span>
#include "stm32g431xx.h"
#include "gpio_template.hpp"

/**
 * SPI configuration
 * MISO: PA6
 * MOSI: PA7
 * SCK:  PA5
 * CS:   PA15 (only for hardware CS)
 */



//SPI Pin Definitions
template<int Instance, bool SwCS>
struct SpiPins;

//SPI1 hardware CS
template<>
struct SpiPins<1, false> {
    using Miso = Gpio<'A',6,GpioMode::Alt,GpioSpeed::VeryHigh,GpioPull::None,GpioAF::AF5>;
    using Mosi = Gpio<'A',7,GpioMode::Alt,GpioSpeed::VeryHigh,GpioPull::None,GpioAF::AF5>;
    using Sck  = Gpio<'A',5,GpioMode::Alt,GpioSpeed::VeryHigh,GpioPull::None,GpioAF::AF5>;
    using Cs   = Gpio<'A',15,GpioMode::Alt,GpioSpeed::VeryHigh,GpioPull::None,GpioAF::AF5>;

    static SPI_TypeDef* spi() { return SPI1; }
    static void enableClock() { RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; }
};

//SPI1 software CS
template<>
struct SpiPins<1, true> {
    using Miso = Gpio<'A',6,GpioMode::Alt,GpioSpeed::VeryHigh,GpioPull::None,GpioAF::AF5>;
    using Mosi = Gpio<'A',7,GpioMode::Alt,GpioSpeed::VeryHigh,GpioPull::None,GpioAF::AF5>;
    using Sck  = Gpio<'A',5,GpioMode::Alt,GpioSpeed::VeryHigh,GpioPull::None,GpioAF::AF5>;
    using Cs   = Gpio<'A',15,GpioMode::Output,GpioSpeed::VeryHigh,GpioPull::None>; // software-controlled

    static SPI_TypeDef* spi() { return SPI1; }
    static void enableClock() { RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; }
};

//SPI Template
template<int Instance, bool SwCS = false>
struct Spi {
    using Pins = SpiPins<Instance, SwCS>;

    static inline SPI_TypeDef* SPI() { return Pins::spi(); }

    //Type-safe SPI Register Masks
    struct SpiCR1Mask {
        static constexpr uint32_t Master                = SPI_CR1_MSTR;
        static constexpr uint32_t BaudDiv256            = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
        static constexpr uint32_t CPOLHigh              = SPI_CR1_CPOL;
        static constexpr uint32_t CPHA0                 = 0;
        static constexpr uint32_t SoftwareCS            = SPI_CR1_SSM | SPI_CR1_SSI;
        static constexpr uint32_t SPE                   = SPI_CR1_SPE;
    };

    struct SpiCR2Mask {
        static constexpr uint32_t Data8Bit   = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
        static constexpr uint32_t SSOE       = SPI_CR2_SSOE;
        static constexpr uint32_t RX8Bit     = SPI_CR2_FRXTH;
        static constexpr uint32_t Motorola   = 0;  
    };

     //Initialize SPI peripheral and pins
    static void init() {
        // Initialize pins
        Pins::Miso::init();
        Pins::Mosi::init();
        Pins::Sck::init();

        if constexpr(SwCS) {
            Pins::Cs::init();
            Pins::Cs::set(); // idle high
        } else {
            Pins::Cs::init();
        }

        //Enable SPI clock
        Pins::enableClock();

        //Reset CR1/CR2
        SPI()->CR1 = 0;
        SPI()->CR2 = 0;

        //CR1: Master, baudrate div16, CPOL=1, CPHA=0
        SPI()->CR1 = SpiCR1Mask::Master | SpiCR1Mask::BaudDiv256 | SpiCR1Mask::CPOLHigh | SpiCR1Mask::CPHA0;;

        //Software CS configuration
        if constexpr(SwCS) {
            SPI()->CR1 |= SpiCR1Mask::SoftwareCS;
        } else {
            SPI()->CR2 |= SpiCR2Mask::SSOE;
        }

        SPI()->CR2 = SpiCR2Mask::Data8Bit | SpiCR2Mask::RX8Bit | SpiCR2Mask::Motorola;
        
        SPI()->CR1 |= SpiCR1Mask::SPE;
    }

    //Software CS helpers
    static void csLow()  { if constexpr(SwCS) Pins::Cs::clear(); }
    static void csHigh() { if constexpr(SwCS) Pins::Cs::set(); }

    //Write buffer (std::span is C++20 feature)
    static void write(std::span<const uint8_t>data) {
        csLow();
        for(auto b : data) {
            while (!(SPI()->SR & SPI_SR_TXE));
            SPI()->DR = static_cast<uint16_t>(b);
        }
        while (SPI()->SR & SPI_SR_BSY);
        csHigh();
    }

    //Full-duplex transfer
    static void transfer(std::span<const uint8_t> tx, std::span<const uint8_t> rx) {
        csLow();
        size_t len = tx.size() < rx.size() ? tx.size() : rx.size();
        for(size_t i = 0; i < len; ++i) {
            while (!(SPI()->SR & SPI_SR_TXE));
            SPI()->DR = static_cast<uint16_t>(tx[i]);

            while (!(SPI()->SR & SPI_SR_RXNE));
            rx[i] = static_cast<uint8_t>(SPI()->DR);
        }
        while (SPI()->SR & SPI_SR_BSY);
        csHigh();
    }

    //Single-byte transfer
    static uint8_t transferByte(uint8_t b) {
        csLow();
        while (!(SPI()->SR & SPI_SR_TXE));
        SPI()->DR = b;
        while (!(SPI()->SR & SPI_SR_RXNE));
        uint8_t r = static_cast<uint8_t>(SPI()->DR);
        csHigh();
        return r;
    }


};
