#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include "esp_sntp.h"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) > (b)) ? (b) : (a))
#define ADD8(a, b) (uint8_t)(((uint16_t)((a) + (b)) > 255) ? 255 : (a) + (b))
#define SUB8(a, b) (uint8_t)(((int16_t)((a) - (b)) < 0) ? 0 : (a) - (b))

#define PIXEL_COUNT 12
#define PIXEL_PIN D10

#define BUTTON_R_PIN D0
#define BUTTON_G_PIN D1
#define BUTTON_B_PIN D3
#define BUTTON_K_PIN D4
#define BUTTON_W_PIN D8

enum {
  BUTTON_R,
  BUTTON_G,
  BUTTON_B,
  BUTTON_K,
  BUTTON_W,
  BUTTON_COUNT
};

#define BUTTON_DEBOUNCE_MS 50  //ms for debounce
#define BUTTON_HOLD_MS 500     //ms below which a button is "pressed" and above which is "held"
#define BUTTON_DECAY_MS 60     //ms to coalesce multiple button events into a single message

typedef enum {
  BUTTON_IDLE,          //Button is doing nothing
  BUTTON_PRESS,         //Button was pressed (and released) as a single-click
  BUTTON_HOLD_START,    //Button hold started
  BUTTON_HOLD,          //Button is held (only sent with other events)
  BUTTON_HOLD_RELEASE,  //Button hold was released
} button_event_t;

// Define Queue
QueueHandle_t qButton;
const int qButtonSize = 10;
typedef struct {
  button_event_t evt[BUTTON_COUNT];
} qbutton_msg_t;

// Define pixel ring
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Struct for clock time
typedef struct {
  uint8_t h;
  uint8_t m;
  uint8_t s;
} clocktime_t;

// Day/Night Cloud/Star state
typedef enum {
  IDLE,
  ATTACK,
  DECAY,
} starmode_t;

// Palette state
typedef enum {
  MODE_DISPLAY,
  MODE_R,
  MODE_RW,
  MODE_RK,
  MODE_G,
  MODE_GW,
  MODE_GK,
  MODE_B,
  MODE_BW,
  MODE_BK,
} palette_mode_t;

// Device overall mode
typedef enum {
  MODE_NIGHT,
  MODE_DAY,
  MODE_PALETTE,
  MODE_CLOCK,
  MODE_COUNT,
} led_mode_t;

// Define button pins
const uint32_t button_pin[BUTTON_COUNT] = { BUTTON_R_PIN, BUTTON_G_PIN, BUTTON_B_PIN, BUTTON_K_PIN, BUTTON_W_PIN };

// Eastern Time POSIX TZ String
// EST+5: Eastern Standard Time is UTC-5
// EDT+4: Eastern Daylight Time is UTC-4
// M3.2.0/02:00:00: DST starts 2nd Sunday in March at 2AM
// M11.1.0/02:00:00: DST ends 1st Sunday in November at 2AM
const char *TZ_STRING = "EST5EDT4,M3.2.0/02:00:00,M11.1.0/02:00:00";

// Returns the pixel to the left (CCW)
uint32_t left_pixel(uint32_t p) {
  if (p == 0) {
    return PIXEL_COUNT - 1;
  } else {
    return p - 1;
  }
}

// Returns the pixel to the right (CW)
uint32_t right_pixel(uint32_t p) {
  return (p + 1) % PIXEL_COUNT;
}

uint8_t next_color(uint8_t c, int8_t inc) {
  if (inc == 0) return c;

  if (inc > 0) {
    // Calculate the step to the next higher multiple
    uint16_t next = (uint16_t)c + (inc - (c % inc));
    // Clamp at 255
    return (next > 255) ? 255 : (uint8_t)next;
  } else {
    // Convert to absolute to simplify math
    uint8_t abs_inc = (uint8_t)(-inc);

    // If already at a multiple, jump down one full step
    if (c % abs_inc == 0) {
      return (c > abs_inc) ? (c - abs_inc) : 0;
    }

    // Otherwise, floor to the nearest multiple (clamping at 0 happens naturally)
    return (c / abs_inc) * abs_inc;
  }
}

void taskButtonHandler(void *pvParameters) {
  Serial.println("Started the button task");

  uint32_t p;
  qbutton_msg_t msg = { .evt = { BUTTON_IDLE, BUTTON_IDLE, BUTTON_IDLE, BUTTON_IDLE, BUTTON_IDLE } };
  uint32_t decayStart = 0;

  struct {
    uint32_t lastTime;   //Time since last state change, for debounce
    uint32_t startTime;  //Time since button was pressed, for press/hold
    uint32_t lastState;  //Last state of button, for debounce
    uint32_t currState;  //Current (valid) state of button
    bool hold;           //If currently in a hold or not
  } button_state[BUTTON_COUNT];

  for (p = 0; p < BUTTON_COUNT; p++) {
    button_state[p].lastTime = 0;
    button_state[p].startTime = 0;
    button_state[p].lastState = HIGH;
    button_state[p].currState = HIGH;
    button_state[p].hold = false;
  }

  for (;;) {
    bool send = false;

    //Debounce
    unsigned long now = millis();
    for (p = 0; p < BUTTON_COUNT; p++) {
      int reading = digitalRead(button_pin[p]);
      if (reading != button_state[p].lastState) {
        button_state[p].lastTime = now;
      }

      if ((now - button_state[p].lastTime) > BUTTON_DEBOUNCE_MS) {
        if (reading != button_state[p].currState) {
          //A button has changed state
          button_state[p].currState = reading;

          if (reading == LOW) {
            //The button is now pressed
            button_state[p].startTime = now;
          } else {
            //The button was released
            if (button_state[p].hold) {
              button_state[p].hold = false;
              msg.evt[p] = BUTTON_HOLD_RELEASE;
              send = true;
            } else {
              msg.evt[p] = BUTTON_PRESS;
              send = true;
            }
          }
        }
      }

      //Check if we've started a hold
      if ((button_state[p].currState == LOW) && ((now - button_state[p].startTime) >= BUTTON_HOLD_MS)) {
        if (!button_state[p].hold) {
          msg.evt[p] = BUTTON_HOLD_START;
          send = true;
          button_state[p].hold = true;
        }
      }

      button_state[p].lastState = reading;
    }

    /* TODO: There should be a decay timer as well, to collect events for up to x milliseconds so we send only a single message if a user releases two buttons */

    if (send) {
      decayStart = now;
    }

    if ((decayStart > 0) && ((now - decayStart) >= BUTTON_DECAY_MS)) {
      //Reset the decay clock
      decayStart = 0;

      //If we're otherwise sending a message, any button currently in a hold should report that
      for (p = 0; p < BUTTON_COUNT; p++) {
        if (button_state[p].hold && msg.evt[p] == BUTTON_IDLE) {
          msg.evt[p] = BUTTON_HOLD;
        }
      }

      int ret = xQueueSend(qButton, (void *)&msg, 0);
      if (ret == pdTRUE) {
        // The message was successfully sent.
      } else if (ret == errQUEUE_FULL) {
        // Since we are checking uxQueueSpacesAvailable this should not occur, however if more than one task should
        //   write into the same queue it can fill-up between the test and actual send attempt
        Serial.println("The `taskButtonHandler` was unable to send data into the Queue");
      }  // Queue send check

      memset(&msg, 0, sizeof(msg));
    }

    vTaskDelay(10);
  }
}

void animate_night(bool valid, button_event_t *evt) {
  static bool initialized = false;
  static uint32_t time = 0;
  static const uint32_t delay_ms = 50;
  static const uint32_t num_stars = 3;
  static const uint32_t pct_new_star = 4;  //Percentage chance of a new star birth
  static const uint32_t colors = 120;
  static const uint32_t palette[] = {
    0x0A000F, 0x0C0211, 0x0E0413, 0x100615, 0x120817, 0x140A19, 0x160C1B, 0x180F1D, 0x1A111F, 0x1C1321, 0x1E1523, 0x201725, 0x221927, 0x241B29, 0x261E2B, 0x28202D, 0x2A222F, 0x2D2431, 0x2F2633, 0x312835, 0x332A37, 0x352D39, 0x372F3B, 0x39313D, 0x3B333F, 0x3D3541, 0x3F3743, 0x413945, 0x433C47, 0x453E49, 0x47404B, 0x49424D, 0x4B444F, 0x4D4651, 0x504853, 0x524B55, 0x544D57, 0x564F59, 0x58515B, 0x5A535D, 0x5C555F, 0x5E5761, 0x605A63, 0x625C65, 0x645E67, 0x666069, 0x68626B, 0x6A646D, 0x6C666F, 0x6E6971, 0x706B73, 0x736D75, 0x756F77, 0x777179, 0x79737B, 0x7B757D, 0x7D787F, 0x7F7A81, 0x817C83, 0x837E85, 0x858088, 0x87828A, 0x89848C, 0x8B878E, 0x8D8990, 0x8F8B92, 0x918D94, 0x938F96, 0x969198, 0x98939A, 0x9A969C, 0x9C989E, 0x9E9AA0, 0xA09CA2, 0xA29EA4, 0xA4A0A6, 0xA6A2A8, 0xA8A5AA, 0xAAA7AC, 0xACA9AE, 0xAEABB0, 0xB0ADB2, 0xB2AFB4, 0xB4B1B6, 0xB6B4B8, 0xB9B6BA, 0xBBB8BC, 0xBDBABE, 0xBFBCC0, 0xC1BEC2, 0xC3C0C4, 0xC5C3C6, 0xC7C5C8, 0xC9C7CA, 0xCBC9CC, 0xCDCBCE, 0xCFCDD0, 0xD1CFD2, 0xD3D2D4, 0xD5D4D6, 0xD7D6D8, 0xD9D8DA, 0xDCDADC, 0xDEDCDE, 0xE0DEE0, 0xE2E1E2, 0xE4E3E4, 0xE6E5E6, 0xE8E7E8, 0xEAE9EA, 0xECEBEC, 0xEEEDEE, 0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6, 0xF8F8F8, 0xFAFAFA, 0xFCFCFC, 0xFFFFFF
  };

  static struct {
    int32_t color = 0;  //allow signed to go less than zero
    uint32_t max = 0;
    uint32_t attack = 0;
    uint32_t decay = 0;
    starmode_t mode;
    uint32_t idx = 0;
  } stars[num_stars];

  uint32_t pixval[PIXEL_COUNT] = { 0 };
  uint32_t i;

  // First run initialize
  if (!initialized) {
    initialized = true;
    time = millis();
    for (i = 0; i < num_stars; i++) {
      stars[i].mode = IDLE;
      stars[i].color = 0;
      stars[i].max = 0;
      stars[i].idx = 0;
    }
    Serial.println("Initialized the Night.");
  }

  // Iterate
  uint32_t now = millis();
  if (now - time >= delay_ms) {
    time = now;

    //Initialize pixel array in the base color (it will be redrawn after iterating)
    for (uint32_t i = 0; i < PIXEL_COUNT; i++) {
      pixval[i] = 0;
    }
    // Serial.println("Reset the Pixels");

    //Advance the stars array:
    // If a star is active, step its mode
    // If no star is active, maybe create one;
    for (int i = 0; i < num_stars; i++) {
      if (stars[i].mode == IDLE) {
        //Star is idle; maybe birth one
        if (random(100) < pct_new_star) {
          stars[i].mode = ATTACK;
          stars[i].color = 0;
          stars[i].max = random(colors);
          stars[i].idx = random(PIXEL_COUNT);
          stars[i].decay = 1 + random(6);  //decay will always be slower than attack
          stars[i].attack = stars[i].decay + random(8);
          // Serial.printf("New Star at %d: max %d\n", stars[i].idx, stars[i].max);
        }
      } else {
        //advance an existing star
        if (stars[i].mode == ATTACK) {
          stars[i].color += stars[i].attack;
          if (stars[i].color >= stars[i].max) {
            stars[i].color = stars[i].max;
            stars[i].mode = DECAY;
          }
        } else if (stars[i].mode == DECAY) {
          stars[i].color -= stars[i].decay;
          if (stars[i].color <= 0) {
            stars[i].color = 0;
            stars[i].mode = IDLE;
          }
        }

        pixval[stars[i].idx] = MAX(stars[i].color, pixval[stars[i].idx]);                                 //star
        pixval[left_pixel(stars[i].idx)] = MAX(stars[i].color / 10, pixval[left_pixel(stars[i].idx)]);    //left bleed
        pixval[right_pixel(stars[i].idx)] = MAX(stars[i].color / 10, pixval[right_pixel(stars[i].idx)]);  //right bleed
      }

      // Serial.printf("Star %d: idx %d, mode %d, color %d, max %d\n",
      //   i,
      //   stars[i].idx,
      //   stars[i].mode,
      //   stars[i].color,
      //   stars[i].max);
    }

    // Draw
    // Serial.printf("@%d Draw.\n", time);
    for (i = 0; i < PIXEL_COUNT; i++) {
      pixels.setPixelColor(i, palette[pixval[i]]);
    }
    pixels.show();
  }
}

void animate_day(bool valid, button_event_t *evt) {
  static bool initialized = false;
  static uint32_t time = 0;
  static const uint32_t delay_ms = 100;
  static const uint32_t num_stars = 3;
  static const uint32_t pct_new_star = 25;  //Percentage chance of a new cloud birth
  static const uint32_t colors = 120;
  static const uint32_t palette[] = {
    0x000010,
    0x020212,
    0x040414,
    0x060616,
    0x080818,
    0x0A0A1A,
    0x0C0C1C,
    0x0F0F1E,
    0x111120,
    0x131322,
    0x151524,
    0x171726,
    0x191928,
    0x1B1B2A,
    0x1E1E2C,
    0x20202E,
    0x222230,
    0x242432,
    0x262634,
    0x282836,
    0x2A2A38,
    0x2D2D3A,
    0x2F2F3C,
    0x31313E,
    0x333340,
    0x353542,
    0x373744,
    0x393946,
    0x3C3C48,
    0x3E3E4A,
    0x40404C,
    0x42424E,
    0x444450,
    0x464652,
    0x484854,
    0x4B4B56,
    0x4D4D58,
    0x4F4F5A,
    0x51515C,
    0x53535E,
    0x555560,
    0x575762,
    0x5A5A64,
    0x5C5C66,
    0x5E5E68,
    0x60606A,
    0x62626C,
    0x64646E,
    0x666670,
    0x696972,
    0x6B6B74,
    0x6D6D76,
    0x6F6F78,
    0x71717A,
    0x73737C,
    0x75757E,
    0x787880,
    0x7A7A82,
    0x7C7C84,
    0x7E7E86,
    0x808088,
    0x82828A,
    0x84848C,
    0x87878E,
    0x898990,
    0x8B8B92,
    0x8D8D94,
    0x8F8F96,
    0x919198,
    0x93939A,
    0x96969C,
    0x98989E,
    0x9A9AA0,
    0x9C9CA2,
    0x9E9EA4,
    0xA0A0A6,
    0xA2A2A8,
    0xA5A5AA,
    0xA7A7AC,
    0xA9A9AE,
    0xABABB0,
    0xADADB2,
    0xAFAFB4,
    0xB1B1B6,
    0xB4B4B8,
    0xB6B6BA,
    0xB8B8BC,
    0xBABABE,
    0xBCBCC0,
    0xBEBEC2,
    0xC0C0C4,
    0xC3C3C6,
    0xC5C5C8,
    0xC7C7CA,
    0xC9C9CC,
    0xCBCBCE,
    0xCDCDD0,
    0xCFCFD2,
    0xD2D2D4,
    0xD4D4D6,
    0xD6D6D8,
    0xD8D8DA,
    0xDADADC,
    0xDCDCDE,
    0xDEDEE0,
    0xE1E1E2,
    0xE3E3E4,
    0xE5E5E6,
    0xE7E7E8,
    0xE9E9EA,
    0xEBEBEC,
    0xEDEDEE,
    0xF0F0F0,
    0xF2F2F2,
    0xF4F4F4,
    0xF6F6F6,
    0xF8F8F8,
    0xFAFAFA,
    0xFCFCFC,
    0xFFFFFF,
  };

  static struct {
    int32_t color = 0;  //allow signed to go less than zero
    uint32_t max = 0;
    uint32_t attack = 0;
    uint32_t decay = 0;
    starmode_t mode;
    uint32_t idx = 0;
  } stars[num_stars];

  uint32_t pixval[PIXEL_COUNT] = { 0 };
  uint32_t i;

  // First run initialize
  if (!initialized) {
    initialized = true;
    time = millis();
    for (i = 0; i < num_stars; i++) {
      stars[i].mode = IDLE;
      stars[i].color = 0;
      stars[i].max = 0;
      stars[i].idx = 0;
    }
    Serial.println("Initialized the Day.");
  }

  // Iterate
  uint32_t now = millis();
  if (now - time >= delay_ms) {
    time = now;

    //Initialize pixel array in the base color (it will be redrawn after iterating)
    for (uint32_t i = 0; i < PIXEL_COUNT; i++) {
      pixval[i] = 0;
    }
    // Serial.println("Reset the Pixels");

    //Advance the stars array:
    // If a star is active, step its mode
    // If no star is active, maybe create one;
    for (int i = 0; i < num_stars; i++) {
      if (stars[i].mode == IDLE) {
        //Star is idle; maybe birth one
        if (random(100) < pct_new_star) {
          stars[i].mode = ATTACK;
          stars[i].color = 0;
          stars[i].max = random(colors);
          stars[i].idx = random(PIXEL_COUNT);
          stars[i].decay = 1 + random(3);
          stars[i].attack = stars[i].decay;  //1+random(3);
          // Serial.printf("New Star at %d: max %d\n", stars[i].idx, stars[i].max);
        }
      } else {
        //advance an existing star
        if (stars[i].mode == ATTACK) {
          stars[i].color += stars[i].attack;
          if (stars[i].color >= stars[i].max) {
            stars[i].color = stars[i].max;
            stars[i].mode = DECAY;
          }
        } else if (stars[i].mode == DECAY) {
          stars[i].color -= stars[i].decay;
          if (stars[i].color <= 0) {
            stars[i].color = 0;
            stars[i].mode = IDLE;
          }
        }

        pixval[stars[i].idx] = MAX(stars[i].color, pixval[stars[i].idx]);                                                           //star
        pixval[left_pixel(stars[i].idx)] = MAX(stars[i].color / 5, pixval[left_pixel(stars[i].idx)]);                               //left bleed
        pixval[right_pixel(stars[i].idx)] = MAX(stars[i].color / 5, pixval[right_pixel(stars[i].idx)]);                             //right bleed
        pixval[left_pixel(left_pixel(stars[i].idx))] = MAX(stars[i].color / 10, pixval[left_pixel(left_pixel(stars[i].idx))]);      //left bleed 2
        pixval[right_pixel(right_pixel(stars[i].idx))] = MAX(stars[i].color / 10, pixval[right_pixel(right_pixel(stars[i].idx))]);  //right bleed 2
      }

      // Serial.printf("Star %d: idx %d, mode %d, color %d, max %d\n",
      //   i,
      //   stars[i].idx,
      //   stars[i].mode,
      //   stars[i].color,
      //   stars[i].max);
    }

    // Draw
    // Serial.printf("@%d Draw.\n", time);
    for (i = 0; i < PIXEL_COUNT; i++) {
      pixels.setPixelColor(i, palette[pixval[i]]);
    }
    pixels.show();
  }
}

void animate_palette(bool valid, button_event_t *evt) {
  static bool initialized = false;
  static uint32_t time = 0;
  static const uint32_t reset_ms = 2000;  //Time to reset to pure color mode (from control mode)
  static struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
  } color;
  static palette_mode_t mode;

  if (!initialized) {
    initialized = true;
    time = millis();
    mode = MODE_DISPLAY;
    color.R = 0;
    color.G = 0;
    color.B = 0;
    pixels.fill(0, 0, PIXEL_COUNT);
    pixels.show();
    Serial.println("Initialized the Palette.");
  }

  uint32_t now = millis();
  bool update = false;

  //Check buttons and act accordingly
  if (valid) {
    if (evt[BUTTON_R] == BUTTON_PRESS) {
      mode = MODE_R;
    } else if (evt[BUTTON_G] == BUTTON_PRESS) {
      mode = MODE_G;
    } else if (evt[BUTTON_B] == BUTTON_PRESS) {
      mode = MODE_B;
    }

    if (mode == MODE_DISPLAY) {
      if (evt[BUTTON_W] == BUTTON_HOLD_START) {
        color.R = 0;
        color.G = 0;
        color.B = 0;
        mode = MODE_DISPLAY;
      }
    } else {
      if (evt[BUTTON_W] == BUTTON_PRESS) {
        switch (mode) {
          case MODE_R:
            color.R = ADD8(color.R, 1);
            break;
          case MODE_G:
            color.G = ADD8(color.G, 1);
            break;
          case MODE_B:
            color.B = ADD8(color.B, 1);
            break;
        }
      }
      if (evt[BUTTON_W] == BUTTON_HOLD_START) {
        switch (mode) {
          case MODE_R:
            mode = MODE_RW;
            break;
          case MODE_G:
            mode = MODE_GW;
            break;
          case MODE_B:
            mode = MODE_BW;
            break;
        }
      }
      if (evt[BUTTON_W] == BUTTON_HOLD_RELEASE) {
        switch (mode) {
          case MODE_RW:
            mode = MODE_R;
            break;
          case MODE_GW:
            mode = MODE_G;
            break;
          case MODE_BW:
            mode = MODE_B;
            break;
        }
      }
      if (evt[BUTTON_K] == BUTTON_PRESS) {
        switch (mode) {
          case MODE_R:
            color.R = SUB8(color.R, 1);
            break;
          case MODE_G:
            color.G = SUB8(color.G, 1);
            break;
          case MODE_B:
            color.B = SUB8(color.B, 1);
            break;
        }
      }
      if (evt[BUTTON_K] == BUTTON_HOLD_START) {
        switch (mode) {
          case MODE_R:
            mode = MODE_RK;
            break;
          case MODE_G:
            mode = MODE_GK;
            break;
          case MODE_B:
            mode = MODE_BK;
            break;
        }

        //clear this event so it's not handled by the main task
        evt[BUTTON_K] = BUTTON_IDLE;
      }
      if (evt[BUTTON_K] == BUTTON_HOLD_RELEASE) {
        switch (mode) {
          case MODE_RK:
            mode = MODE_R;
            break;
          case MODE_GK:
            mode = MODE_G;
            break;
          case MODE_BK:
            mode = MODE_B;
            break;
        }
      }
    }

    time = millis();
    update = true;
  }

  //Do a hold color change
  if ((now - time) > 500) {
    switch (mode) {
      case MODE_RW:
        color.R = next_color(color.R, 8);
        time = now;
        update = true;
        break;
      case MODE_GW:
        color.G = next_color(color.G, 8);
        time = now;
        update = true;
        break;
      case MODE_BW:
        color.B = next_color(color.B, 8);
        time = now;
        update = true;
        break;
      case MODE_RK:
        color.R = next_color(color.R, -8);
        time = now;
        update = true;
        break;
      case MODE_GK:
        color.G = next_color(color.G, -8);
        time = now;
        update = true;
        break;
      case MODE_BK:
        color.B = next_color(color.B, -8);
        time = now;
        update = true;
        break;
    }
  }

  //Reset the display if it's been a while (and we're not doing a button-hold operation)
  if (((mode == MODE_R) || (mode == MODE_G) || (mode == MODE_B)) && ((now - time) > reset_ms)) {
    mode = MODE_DISPLAY;
    update = true;
  }

  //Do the displaying
  if (update) {
    Serial.printf("Mode is %d, color is %d %d %d\n", mode, color.R, color.G, color.B);
    pixels.fill(pixels.Color(color.R, color.G, color.B, 0), 0, PIXEL_COUNT);
    uint16_t bar = 0;
    switch (mode) {
      case MODE_R:
      case MODE_RW:
      case MODE_RK:
        pixels.fill(0, 7, 5);
        bar = color.R * 5 + 5;
        if ((bar >> 8) > 0) pixels.fill(0xFF0000, 7, bar >> 8);
        pixels.setPixelColor(7 + (bar >> 8), (bar & 0xFF) << 16);
        break;
      case MODE_G:
      case MODE_GW:
      case MODE_GK:
        pixels.fill(0, 7, 5);
        bar = color.G * 5 + 5;
        if ((bar >> 8) > 0) pixels.fill(0x00FF00, 7, bar >> 8);
        pixels.setPixelColor(7 + (bar >> 8), (bar & 0xFF) << 8);
        break;
      case MODE_B:
      case MODE_BW:
      case MODE_BK:
        pixels.fill(0, 7, 5);
        bar = color.B * 5 + 5;
        if ((bar >> 8) > 0) pixels.fill(0x0000FF, 7, bar >> 8);
        pixels.setPixelColor(7 + (bar >> 8), bar & 0xFF);
        break;
      case MODE_DISPLAY:
        //do nothing, we've already filled the pixels
        break;
    }
    pixels.show();
  }
}

void get_clock(clocktime_t *tm) {
  struct timeval tv;
  gettimeofday(&tv, NULL);

  // Convert tv_sec to a local calendar time structure
  struct tm *local_time = localtime(&tv.tv_sec);

  if (local_time != NULL) {
    tm->h = local_time->tm_hour;
    tm->m = local_time->tm_min;
    tm->s = local_time->tm_sec;
  }
}

void animate_clock(bool valid, button_event_t *evt) {
  static bool initialized = false;
  static uint32_t time = 0;

  if (!initialized) {
    initialized = true;
    time = millis();
    Serial.println("Initialized the Clock.");
  }

  clocktime_t clk;
  get_clock(&clk);

  bool pm = clk.h / 12;  // 1 if hour >= 12

  uint32_t now = millis();
  if (now - time > 100) {
    time = now;

    // Serial.printf("It is %02d:%02d:%02d\n", clk.h, clk.m, clk.s);
    if (pm) {
      pixels.fill(0, 0, PIXEL_COUNT);
      pixels.setPixelColor(clk.h % 12, 0x101010);
      pixels.setPixelColor(clk.m / 5, 0x100010);
      pixels.show();
    } else {
      pixels.fill(0, 0, PIXEL_COUNT);
      pixels.setPixelColor(clk.h % 12, 0x101010);
      pixels.setPixelColor(clk.m / 5, 0x201000);
      pixels.show();
    }
  }
}

void animate_boot() {
  static bool initialized = false;
  static uint32_t time = 0;
  static uint32_t idx = 0;

  if (!initialized) {
    initialized = true;
    time = millis();
    pixels.fill(0, 0, PIXEL_COUNT);
    pixels.setPixelColor(idx, 0x666666);
    pixels.show();
  }

  uint32_t now = millis();
  if (now - time > 100) {
    time = now;
    idx = right_pixel(idx);
    pixels.fill(0, 0, PIXEL_COUNT);
    pixels.setPixelColor(idx, 0x666666);
    pixels.show();
  }
}

void taskMain(void *pvParameters) {  // This is a task.
  Serial.println("Started the main task");

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.begin("308", "albemarle");

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(10);
    animate_boot();
    if (millis() - start > 5000) {
      //timeout
      break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected ");
    Serial.println(WiFi.localIP());

    configTime(0, 0, "pool.ntp.org");  // Set 0/0 and use TZ_STRING for proper DST
    setenv("TZ", TZ_STRING, 1);
    tzset();
  }

  uint32_t mode = (uint32_t)MODE_NIGHT;
  for (;;) {
    qbutton_msg_t msg;
    bool valid_msg = false;

    //Check the button queue
    //the timeout here clocks the actual LED display mode also
    int ret = xQueueReceive(qButton, &msg, 50);
    if (ret == pdPASS) {
      Serial.println("Got Button State");
      Serial.println(msg.evt[0]);
      Serial.println(msg.evt[1]);
      Serial.println(msg.evt[2]);
      Serial.println(msg.evt[3]);
      Serial.println(msg.evt[4]);
      valid_msg = true;
    }

    //Do whatever we're doing
    switch ((led_mode_t)mode) {
      case MODE_NIGHT:
        animate_night(valid_msg, msg.evt);
        break;
      case MODE_DAY:
        animate_day(valid_msg, msg.evt);
        break;
      case MODE_PALETTE:
        animate_palette(valid_msg, msg.evt);
        break;
      case MODE_CLOCK:
        animate_clock(valid_msg, msg.evt);
        break;
    }

    if (valid_msg && msg.evt[BUTTON_K] == BUTTON_HOLD_START) {
      mode = (mode + 1) % MODE_COUNT;
      Serial.printf("New Mode %d\n", mode);
    }


  }  // Infinite loop
}

void setup() {
  Serial.begin(115200);
  Serial.println("Begin");
  randomSeed(analogRead(A0));

  // Init Button HW
  uint32_t p;
  for (p = 0; p < 5; p++) {
    pinMode(button_pin[p], INPUT_PULLUP);
  }

  // Init NeoPixel ring
  pixels.begin();
  pixels.fill(0xFFFFFF, 0, PIXEL_COUNT);
  pixels.show();

  //Init main Task
  qButton = xQueueCreate(qButtonSize, sizeof(qbutton_msg_t));
  xTaskCreate(taskMain, "main", 4096, NULL, 1, NULL);

  //Init button Task
  xTaskCreate(taskButtonHandler, "button", 1024, NULL, 2, NULL);
}

void loop() {
  //Do nothing, FreeRTOS is on it
  vTaskDelay(1000);
}
