#include <stdint.h>
#include <stdbool.h>

// Indexes of colors 

#define RED       0
#define YELLOW    1
#define GREEN     2
#define TURQUOISE 3
#define BLUE      4
#define PURPLE    5
#define WHITE     6  

#define NUMBER_OF_COLORS      7
#define RGB_CODE_SIZE         3


static uint8_t colors_pwm_codes[NUMBER_OF_COLORS][RGB_CODE_SIZE] = 

{
 {0xFF,0x00,0x00},   // RED
 {0xFF,0xFF,0x00},   // YELLOW
 {0x00,0xFF,0x00},   // GREEN
 {0x00,0x7F,0xFF},   // TURQUOISE
 {0x00,0x00,0xFF},   // BLUE
 {0xFF,0x00,0xFF},   // PURPLE
 {0xFF,0xFF,0xFF}    // WHITE
};

static uint8_t[RGB_CODE_SIZE] get_rgb_code();

