#include <stdint.h>
#include "tl_common.h"
#include "main.h"
#include "epd.h"
#include "epd_spi.h"
#include "epd_bwr_213.h"
#include "drivers.h"
#include "stack/ble/ble.h"

// SSD1675 mixed with SSD1680 EPD Controller  (2.13" 250x122, 3-color BWR)
//
//   0x24 = BW plane (black/white)      0x26 = RED plane (full) / OLD RAM (diff)
//
// FULL : write frame to 0x24, clear red plane (0x26=0), trigger 0xC7 (OTP waveform).
// PARTIAL: see EPD_BWR213_PART_MODE below.

// ------------------------------------------------------------------
// Partial refresh variant (single-variable knob):
//   0 = full-config OTP  : re-init (0x12 + registers) + 0xB1 + 0x24 + 0xFF
//                          -> erases correctly, flashes (previous behaviour)
//   1 = blozi-style light: no re-init, just 0x18 + 0xB1 + 0x3C=0x80 + 0x24 + 0xFF
//                          -> this is what blozi-etag does; does NOT erase
//   3 = middle ground    : registers but WITHOUT the 0x12 SW reset + 0xB1 + 0x24 + 0xFF
// ------------------------------------------------------------------
#define EPD_BWR213_PART_MODE       3

// Partial trigger (0x22). OTP differential.
#define EPD_BWR213_PART_TRIG       0xFF

#define BWR_213_Len 50
uint8_t LUT_bwr_213_part[] = {

0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

BWR_213_Len, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 
0x00, 0x00, 0x00, 

};

// Last frame actually shown on the panel (baseline for the differential).
static uint8_t epd_prev[250 * 16];
static uint8_t epd_prev_valid = 0;

#define EPD_BWR_213_test_pattern 0xA5
_attribute_ram_code_ uint8_t EPD_BWR_213_detect(void)
{
    // SW Reset
    EPD_WriteCmd(0x12);
    WaitMs(10);

    EPD_WriteCmd(0x32);
    int i;
    for (i = 0; i < 153; i++)// This model has a 159 bytes LUT storage so we test for that
    {
        EPD_WriteData(EPD_BWR_213_test_pattern);
    }
    EPD_WriteCmd(0x33);
    for (i = 0; i < 153; i++)
    {
        if(EPD_SPI_read() != EPD_BWR_213_test_pattern)
            return 0;
    }
    return 1;
}

_attribute_ram_code_ uint8_t EPD_BWR_213_read_temp(void)
{
    uint8_t epd_temperature = 0 ;
    
    // SW Reset
    EPD_WriteCmd(0x12);

    EPD_CheckStatus_inverted(100);

    // Set Analog Block control
    EPD_WriteCmd(0x74);
    EPD_WriteData(0x54);
    // Set Digital Block control
    EPD_WriteCmd(0x7E);
    EPD_WriteData(0x3B);

    // Booster soft start
    EPD_WriteCmd(0x0C);
    EPD_WriteData(0x8B);
    EPD_WriteData(0x9C);
    EPD_WriteData(0x96);
    EPD_WriteData(0x0F);

    // Driver output control
    EPD_WriteCmd(0x01);
    EPD_WriteData(0x28);
    EPD_WriteData(0x01);
    EPD_WriteData(0x01);

    // Data entry mode setting
    EPD_WriteCmd(0x11);
    EPD_WriteData(0x01);

    // Set RAM X- Address Start/End
    EPD_WriteCmd(0x44);
    EPD_WriteData(0x00);
    EPD_WriteData(0x0F);

    // Set RAM Y- Address Start/End
    EPD_WriteCmd(0x45);
    EPD_WriteData(0x28);
    EPD_WriteData(0x01);
    EPD_WriteData(0x2E);
    EPD_WriteData(0x00);

    // Border waveform control
    EPD_WriteCmd(0x3C);
    EPD_WriteData(0x05);

    // Display update control
    EPD_WriteCmd(0x21);
    EPD_WriteData(0x00);
    EPD_WriteData(0x80);

    // Temperature sensor control
    EPD_WriteCmd(0x18);
    EPD_WriteData(0x80);

    // Display update control
    EPD_WriteCmd(0x22);
    EPD_WriteData(0xB1);
    
    // Master Activation
    EPD_WriteCmd(0x20);

    EPD_CheckStatus_inverted(100);

    // Temperature sensor read from register
    EPD_WriteCmd(0x1B);
    epd_temperature = EPD_SPI_read();    
    EPD_SPI_read();

    WaitMs(5);
    
    // deep sleep
    EPD_WriteCmd(0x10);
    EPD_WriteData(0x01);

    return epd_temperature;
}

// Common panel registers (used by the full refresh and the "middle ground" partial)
_attribute_ram_code_ void EPD_BWR_213_Regs(void)
{
    EPD_WriteCmd(0x74);
    EPD_WriteData(0x54);
    EPD_WriteCmd(0x7E);
    EPD_WriteData(0x3B);
    EPD_WriteCmd(0x0C);
    EPD_WriteData(0x8B);
    EPD_WriteData(0x9C);
    EPD_WriteData(0x96);
    EPD_WriteData(0x0F);
    EPD_WriteCmd(0x01);
    EPD_WriteData(0x28);
    EPD_WriteData(0x01);
    EPD_WriteData(0x01);
    EPD_WriteCmd(0x11);
    EPD_WriteData(0x01);
    EPD_WriteCmd(0x44);
    EPD_WriteData(0x00);
    EPD_WriteData(0x0F);
    EPD_WriteCmd(0x45);
    EPD_WriteData(0x28);
    EPD_WriteData(0x01);
    EPD_WriteData(0x2E);
    EPD_WriteData(0x00);
    EPD_WriteCmd(0x3C);
    EPD_WriteData(0x80);
    EPD_WriteCmd(0x21);
    EPD_WriteData(0x00);
    EPD_WriteData(0x80);
    EPD_WriteCmd(0x18);
    EPD_WriteData(0x80);
}

_attribute_ram_code_ uint8_t EPD_BWR_213_Display(unsigned char *image, int size, uint8_t full_or_partial)
{
    uint8_t epd_temperature = 0;
    int i;
    int n = size;
    if (n > (250 * 16))
        n = 250 * 16; // clamp to one full frame (4000 bytes)
    if (n < 0)
        n = 0;

    /* ---------------- FULL refresh (also seeds the OLD frame) --------- */
    if (full_or_partial || !epd_prev_valid)
    {
        EPD_WriteCmd(0x12);
        EPD_CheckStatus_inverted(100);

        EPD_WriteCmd(0x74);
        EPD_WriteData(0x54);
        EPD_WriteCmd(0x7E);
        EPD_WriteData(0x3B);
        EPD_WriteCmd(0x0C);
        EPD_WriteData(0x8B);
        EPD_WriteData(0x9C);
        EPD_WriteData(0x96);
        EPD_WriteData(0x0F);
        EPD_WriteCmd(0x01);
        EPD_WriteData(0x28);
        EPD_WriteData(0x01);
        EPD_WriteData(0x01);
        EPD_WriteCmd(0x11);
        EPD_WriteData(0x01);
        EPD_WriteCmd(0x44);
        EPD_WriteData(0x00);
        EPD_WriteData(0x0F);
        EPD_WriteCmd(0x45);
        EPD_WriteData(0x28);
        EPD_WriteData(0x01);
        EPD_WriteData(0x2E);
        EPD_WriteData(0x00);
        EPD_WriteCmd(0x3C);
        EPD_WriteData(0x05);
        EPD_WriteCmd(0x21);
        EPD_WriteData(0x00);
        EPD_WriteData(0x80);
        EPD_WriteCmd(0x18);
        EPD_WriteData(0x80);
        EPD_WriteCmd(0x22);
        EPD_WriteData(0xB1);
        EPD_WriteCmd(0x20);
        EPD_CheckStatus_inverted(100);
        EPD_WriteCmd(0x1B);
        epd_temperature = EPD_SPI_read();
        EPD_SPI_read();
        WaitMs(5);

        EPD_WriteCmd(0x4E);
        EPD_WriteData(0x00);
        EPD_WriteCmd(0x4F);
        EPD_WriteData(0x28);
        EPD_WriteData(0x01);
        EPD_LoadImage(image, n, 0x24);

        EPD_WriteCmd(0x4E);
        EPD_WriteData(0x00);
        EPD_WriteCmd(0x4F);
        EPD_WriteData(0x28);
        EPD_WriteData(0x01);
        EPD_WriteCmd(0x26);
        for (i = 0; i < n; i++)
            EPD_WriteData(0x00);

        EPD_WriteCmd(0x22);
        EPD_WriteData(0xC7);
        EPD_WriteCmd(0x20);

        memcpy(epd_prev, image, n);
        epd_prev_valid = 1;
        return epd_temperature;
    }

    /* ---------------- PARTIAL refresh ---------------- */
#if EPD_BWR213_PART_MODE == 1
    // blozi-style light partial: no SW reset, no register re-send.
    EPD_WriteCmd(0x18);
    EPD_WriteData(0x80);
    EPD_WriteCmd(0x22);
    EPD_WriteData(0xB1);
    EPD_WriteCmd(0x20);
    EPD_CheckStatus_inverted(100);
    EPD_WriteCmd(0x3C);
    EPD_WriteData(0x80);
#elif EPD_BWR213_PART_MODE == 3
    // middle ground: send registers but WITHOUT the 0x12 SW reset
    EPD_BWR_213_Regs();
    EPD_WriteCmd(0x22);
    EPD_WriteData(0xB1);
    EPD_WriteCmd(0x20);
    EPD_CheckStatus_inverted(100);
#else
    // full-config OTP (previous behaviour)
    EPD_WriteCmd(0x12);
    EPD_CheckStatus_inverted(100);
    EPD_BWR_213_Regs();
    EPD_WriteCmd(0x22);
    EPD_WriteData(0xB1);
    EPD_WriteCmd(0x20);
    EPD_CheckStatus_inverted(100);
    EPD_WriteCmd(0x1B);
    EPD_SPI_read();
    EPD_SPI_read();
    WaitMs(5);
#endif

    // NEW frame -> 0x24
    EPD_WriteCmd(0x4E);
    EPD_WriteData(0x00);
    EPD_WriteCmd(0x4F);
    EPD_WriteData(0x28);
    EPD_WriteData(0x01);
    EPD_LoadImage(image, n, 0x24);

    // OTP differential trigger
    EPD_WriteCmd(0x22);
    EPD_WriteData(EPD_BWR213_PART_TRIG);
    EPD_WriteCmd(0x20);

    memcpy(epd_prev, image, n);

    return epd_temperature;
}

_attribute_ram_code_ void EPD_BWR_213_set_sleep(void)
{
    // deep sleep
    EPD_WriteCmd(0x10);
    EPD_WriteData(0x01);

}
