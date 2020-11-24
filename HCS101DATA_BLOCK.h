#ifndef HCS101_H
#define HCS101_H              

typedef unsigned long uint32_t;  // the usigned long to act as a container for each section of the message

enum HCS101_button_mask          //bit mask for the buttons in hex
{  
    BM_S3 = 0x1,
    BM_S0 = 0x2,
    BM_S1 = 0x4,
    BM_S2 = 0x8
};

struct HCS101_keycode
{
    uint32_t serial;             // container for the serial in hex
    unsigned char buttons;       // container for the button using enum definition
};

#endif 
