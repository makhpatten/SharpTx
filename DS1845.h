#define ID_BYTE 1
#define ADDRESS_BYTE 2
#define DATA_BYTE 3

#define POT0 0x12  // 0xF9 is Pot 0 (100 pos'ns i.e. 0 - 100 (64h))
#define POT0_MAX  0xFF
#define POT1 0x11  // 0xF8 is Pot 1 (256 pos'ns i.e. 0 - 255 (FFh))
#define POT1_MAX  0xFF

#define SCLK_PERIOD 3 //the minimum timing delay (us) between logic level changes to the Digital Pot SCL, WP and SDA lines

