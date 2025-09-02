#pragma once
#include <cstdint>

//Class constructed as Button IRQ handler
class IButtonHandler {
public:
    virtual void onButtonPressed() = 0;
    virtual void onButtonRelease() = 0;
    virtual ~IButtonHandler() = default;
};


class Button {

public :
    Button(uint8_t pin, IButtonHandler* handler);

    static void handleIrq();


private :
    static IButtonHandler* handler_;
    uint8_t pin_;

};