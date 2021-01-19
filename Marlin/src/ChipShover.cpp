;
#include "MarlinCore.h"

#if ENABLED(MARLIN_DEV_MODE)
  #warning "WARNING! Disable MARLIN_DEV_MODE for the final build!"
#endif
#define USE_BEEPER 1

#include "HAL/shared/Delay.h"
#include "HAL/shared/esp_wifi.h"

#ifdef ARDUINO
  #include <pins_arduino.h>
#endif
#include <math.h>

#include "core/utility.h"
#include "module/motion.h"
#include "module/planner.h"
#include "module/endstops.h"
#include "module/temperature.h"
#include "module/settings.h"
#include "module/printcounter.h" // PrintCounter or Stopwatch

#include "module/stepper.h"
#include "module/stepper/indirection.h"

#include "gcode/gcode.h"
#include "gcode/parser.h"
#include "gcode/queue.h"

#include "sd/cardreader.h"

#include "lcd/marlinui.h"
#if USE_BEEPER
  #include "libs/buzzer.h"
#endif
#if HAS_TRINAMIC_CONFIG && DISABLED(PSU_DEFAULT_OFF)
  #include "feature/tmc_util.h"
#endif

bool UI_update = false;
/***************************************************
  This is our GFX example for the Adafruit ILI9341 TFT FeatherWing
  ----> http://www.adafruit.com/products/3315

  Check out the links above for our tutorials and wiring diagrams

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

Adafruit_ILI9341 tft = Adafruit_ILI9341(77, 73, 87);

void chipshover_setup()
{
    tft.begin();

    //test LCD register reads
    volatile uint8_t x = tft.readcommand8(ILI9341_RDMODE);
    x = tft.readcommand8(ILI9341_RDMADCTL);
    x = tft.readcommand8(ILI9341_RDPIXFMT);
    x = tft.readcommand8(ILI9341_RDIMGFMT);
    x = tft.readcommand8(ILI9341_RDSELFDIAG);

    tft.fillScreen(ILI9341_BLACK);
    tft.setRotation(1);

    pinMode(22, OUTPUT);
    pinMode(72, OUTPUT);
    pinMode(85, OUTPUT);

}


//check if LCD dead, maybe not needed?
void check_LCD()
{
    if (UI_update) {
        if (tft.readcommand8(ILI9341_RDMADCTL) == 0xFF) {
            //probably dead
            tft.sendCommand(0x01);
            tft = Adafruit_ILI9341(77, 73, 87);
        }
    }
}

/**
 * The main Marlin program loop
 *
 *  - Call idle() to handle all tasks between G-code commands
 *      Note that no G-codes from the queue can be executed during idle()
 *      but many G-codes can be called directly anytime like macros.
 *  - Check whether SD card auto-start is needed now.
 *  - Check whether SD print finishing is needed now.
 *  - Run one G-code command from the immediate or main command queue
 *    and open up one space. Commands in the main queue may come from sd
 *    card, host, or by direct injection. The queue will continue to fill
 *    as long as idle() or manage_inactivity() are being called.
 */
#include "module/motion.h"
  uint16_t cnt = 0;
  uint8_t cnt2 = 0;
  uint8_t is_idle = 1;
  #define CHARSZ 2
  #define CHARWIDTH 6 * CHARSZ
  #define CHARHEIGHT 8 * CHARSZ

extern uint8_t CS_STATUS;
void LCD_clear_line(uint8_t line, uint16_t colour=ILI9341_WHITE)
{
    tft.fillRect(CHARWIDTH*0, CHARHEIGHT*line, CHARWIDTH*50, CHARHEIGHT, ILI9341_BLACK);
    tft.setTextColor(colour); tft.setTextSize(CHARSZ);
    tft.setCursor(0, CHARHEIGHT*line);
}

  void delete_status() {
    tft.fillRect(CHARWIDTH*12, 0, CHARWIDTH*10, CHARHEIGHT, ILI9341_BLACK);
  }

  void set_cursor_at_status() {
    tft.setCursor(CHARWIDTH*12, 0);
  }

  void set_status_busy() {
    is_idle = 0;
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(CHARSZ);
    delete_status();
    set_cursor_at_status();
    tft.print("Busy");
  }

  void set_status_idle() {
    if (is_idle) {
      return;
    }
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(CHARSZ);
    delete_status();
    set_cursor_at_status();
    tft.print("Idle");
    is_idle = 1;
  }



double stored_x = -101, stored_y = -100 , stored_z = -100;
void update_xyz(float x, float y, float z) {
  if (UI_update) {
    if (stored_x != x) {
      LCD_clear_line(2, ILI9341_YELLOW);
      tft.print("X: ");
      tft.print(x, 3);
      stored_x = x;
    }
    if (stored_y != y) {

      LCD_clear_line(3, ILI9341_YELLOW);
      tft.print("Y: ");
      tft.print(y, 3);
      stored_y = y;
    }
    if (stored_z != z) {
      LCD_clear_line(4, ILI9341_YELLOW);
      tft.print("Z: ");
      tft.print(z, 3);
      stored_z = z;
    }
  }
}

void lcd_print_bin(uint8_t n)
{
    for (uint8_t i = 0; i < 8; i++) {
        tft.print((n >> i) & 1, 10);
    }
}
const char *STEPPER_STATUSES[] = {"GOOD", "IDLE", "OPENLOADA", "OPENLOADB", "OVERTEMP WARN", "SHORTA", "SHORTB", "OVERTEMP SHUTDOWN", "MOTOR STALL"};

uint8_t motor_err_index(uint8_t err) 
{
    for (uint8_t i = 0; i < 8; i++) {
        if ((err >> (7 - i)) & 1)
            return 8 - i;
    }
    return 0;
}

#include "src/feature/e_parser.h"
uint8_t i = 0;
#include "gcode/gcode.h"

void handle_estop()
{
    if (UI_update) {
        //EmergencyParser::State st = EmergencyParser::EP_M410;
        EmergencyParser::State st = EmergencyParser::EP_M112;
        emergency_parser.update(st, '\n');
    }
}

void handle_pause()
{
    if (UI_update) {
        //cleaning_buffer_counter = 0;
        //quickstop_stepper();
        EmergencyParser::State st = EmergencyParser::EP_M410;
        emergency_parser.update(st, '\n');
    }
}

void update_UI_status_msg(const char *msg, bool clear=true)
{
    if (UI_update) {
        if (clear) {
            LCD_clear_line(0);
        }
        //tft.print("ChipSHOVER: ");
        tft.print(msg);
    }
}

struct encoder_value {
    bool A, B;
};

encoder_value ENCODER_SEQUENCE[] = {{.A = 0, .B = 0}, {.A = 1, .B = 0}, {.A = 1, .B = 1}, {.A = 0, .B = 1}};

int get_encoder_state(encoder_value value, int n)
{
    if (n > 3)
        return -1;
    return (value.A == ENCODER_SEQUENCE[n].A) && (value.B == ENCODER_SEQUENCE[n].B);
}

int get_encoder_direction(encoder_value o, encoder_value n)
{
    if ((get_encoder_state(o, 3)) && get_encoder_state(n, 0)){
        //special case - wrap
        return 1;
    }

    if ((get_encoder_state(o, 0)) && (get_encoder_state(n, 3))) {
        //special case - wrap
        return -1;
    }
    int st_o = 0, st_n = 0;
    for (; st_o < 4; st_o++) {
        if (get_encoder_state(o, st_o))
            break;
    }
    for (; st_n < 4; st_n++) {
        if (get_encoder_state(n, st_n))
            break;
    }

    if (st_n > st_o) return 1;
    else if (st_n < st_o) return -1;
    else return 0;

}



bool JS_MOVE_LEFT=false, JS_MOVE_RIGHT=false, JS_MOVE_UP=false, JS_MOVE_DOWN = false;
bool JS_MOVE_ZP = false;
bool JS_MOVE_ZM = false;
encoder_value enc_last = {.A = 0, .B = 0};
void handle_js()
{
    if (UI_update) {
        JS_MOVE_LEFT = digitalRead(37); //Left
        JS_MOVE_RIGHT = digitalRead(38); //Right
        JS_MOVE_DOWN = digitalRead(39); //Down
        JS_MOVE_UP = digitalRead(40); //Up

        bool A = digitalRead(42);
        bool B = digitalRead(43);
        encoder_value enc_val = {.A = A, .B = B};
        int rot = get_encoder_direction(enc_last, enc_val);
        if (rot == 1) {
            JS_MOVE_ZP = true;
            JS_MOVE_ZM = false;
        } else if (rot == -1) {
            JS_MOVE_ZM = true;
            JS_MOVE_ZP = false;
        } else {
            JS_MOVE_ZP = false;
            JS_MOVE_ZM = false;
        }
        enc_last = enc_val;
    }
}

bool XH_last, XL_last, YH_last, YL_last;
bool SW_last = 0, A_last = 0, B_last = 0;
void LCD_update_js()
{
    if (UI_update) {

        bool XH, XL, YH, YL;
        bool SW, A, B;
        XH = digitalRead(37); //Left
        XL = digitalRead(38); //Right
        YH = digitalRead(39); //Down
        YL = digitalRead(40); //Up
        SW = !digitalRead(56);
        A = digitalRead(42);
        B = digitalRead(43);
        if ((XH != XH_last) || (XL != XL_last)) {

            LCD_clear_line(8);
            if (XH) {
                tft.print("XH ");
            }
            if (XL) {
                tft.print("XL ");
            }
            XH_last = XH; XL_last = XL;
        }

        if ((YH != YH_last) || (YL != YL_last)) {

            LCD_clear_line(9);
            if (YH) {
                tft.print("YH ");
            }
            if (YL) {
                tft.print("YL ");
            }
            YH_last = YH; YL_last = YL;
        }

        if ((A != A_last) || (B != B_last) || (SW != SW_last)) {
            LCD_clear_line(10);
            encoder_value enc_val = {.A = A, .B = B};
            int rot = get_encoder_direction(enc_last, enc_val);
            if (rot == 1) {
                tft.print("Right ");
            } else if (rot == -1) {
                tft.print("Left ");
            }
            //enc_last = enc_val;
            // if (A) {
            //     tft.print("A ");
            // }
            // if (B) {
            //     tft.print("B ");
            // }

            if (SW) {
                tft.print("SW ");
            }
            A_last = A; B_last = B; SW_last = SW;
        }
    }
}

extern GcodeSuite gcode;

bool REQ_BUTTON_HOME = false;
bool REQ_STEPPER_INIT = false;
void button_homing()
{
    if (UI_update) {
        //watchdog (I think?) unhappy with enqueue here
        //queue.enqueue_one("G28 XZ\n"); //just XZ since Y driver is buggy
        if (CS_STATUS == 0x03) //estop
            REQ_STEPPER_INIT = true;
        REQ_BUTTON_HOME = true;
    }
}

void udc_detach();
void erase_firmware()
{
    /* Turn off connected stuff */
    //board_power(0);

    /* Clear ROM-mapping bit. */
    efc_perform_command(EFC0, EFC_FCMD_CGPB, 1);	

    /* Disconnect USB (will kill connection) */
    //udc_detach();

    /* With knowledge that I will rise again, I lay down my life. */
    while (RSTC->RSTC_SR & RSTC_SR_SRCMP);			
    RSTC->RSTC_CR |= RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;				
    while(1);
}

uint8_t status_x = 0, status_y = 0, status_z = 0;
#include "module/stepper/trinamic.h"
template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
uint8_t get_stepper_status(TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st);
void ui_error_update()
{
    // volatile uint32_t x = stepperX.DRVCONF();
    // volatile auto y = stepperX.isEnabled();
    // volatile auto errcnt = stepperX.error_count;
    // volatile auto current = stepperX.rms_current();
    // volatile auto status = stepperX.DRVSTATUS();
    // volatile auto wtf1 = stepperX.stst();
    if (UI_update) {
        uint8_t status = get_stepper_status(stepperX);
        if (status_x != status) {
            tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(CHARSZ);
            tft.fillRect(0, CHARHEIGHT * 5, 40 * CHARWIDTH, 1 * CHARHEIGHT, ILI9341_BLACK);
            tft.setCursor(0, CHARHEIGHT * 5);
            tft.print("Status X: ");
            //tft.print(status, 2);
            //lcd_print_bin(status);
            tft.print(STEPPER_STATUSES[motor_err_index(status)]);
            status_x = status;
        }

        status = get_stepper_status(stepperY);
        if (status_y != status) {
            tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(CHARSZ);
            tft.fillRect(0, CHARHEIGHT * 6, 40 * CHARWIDTH, 1 * CHARHEIGHT, ILI9341_BLACK);
            tft.setCursor(0, CHARHEIGHT * 6);
            tft.print("Status Y: ");
            //lcd_print_bin(status);
            tft.print(STEPPER_STATUSES[motor_err_index(status)]);
            status_y = status;
        }

        status = get_stepper_status(stepperZ);
        if (status_z != status) {
            tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(CHARSZ);
            tft.fillRect(0, CHARHEIGHT * 7, 40 * CHARWIDTH, 1 * CHARHEIGHT, ILI9341_BLACK);
            tft.setCursor(0, CHARHEIGHT * 7);
            tft.print("Status Z: ");
            //lcd_print_bin(status);
            tft.print(STEPPER_STATUSES[motor_err_index(status)]);
            status_z = status;
        }
    }
}

void heartbeat_itr()
{
    if (UI_update) {
        if (cnt++ == 100) {
        cnt = 0;
        tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(CHARSZ);
        tft.fillRect(10*18, 14+3, 10*4, 14+1, ILI9341_BLACK);
        tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(CHARSZ);
        tft.setCursor(10*18, 14+3);
        tft.print(cnt2++, 10);
        }
    }
}

#define MYXSTR(X) MYSTRINGIFY(X)
#define MYSTRINGIFY(X) #X

float XSTEPS_MM=0, YSTEPS_MM=0, ZSTEPS_MM=0;
bool PRINT_BUILD = true;
void print_build_info()
{
    if (UI_update && PRINT_BUILD) {
        PRINT_BUILD = false;
        LCD_clear_line(9, ILI9341_GREEN);
        tft.setTextSize(1);
        tft.print("Build: ");
        tft.print(__DATE__);
        tft.print(" ");
        tft.print(__TIME__);
        tft.print("\n");

        //LCD_clear_line(10);
        tft.print("Version: ");
        tft.print(MYXSTR(GITVERSION));
        tft.print(" EEPROM: ");
        #if ENABLED(EEPROM_SETTINGS)
            tft.print("YES");
        #else
            tft.print("NO");
        #endif
        tft.print("\n");
        //LCD_clear_line(12, ILI9341_GREEN);
        tft.print("Limits: ");
        tft.print(MYXSTR(X_MAX_POS) "/");
        tft.print(MYXSTR(Y_MAX_POS) "/");
        tft.print(MYXSTR(Z_MAX_POS) " ");
        tft.print("\n");

    }

    if (UI_update) {
        if ((planner.settings.axis_steps_per_mm[0] != XSTEPS_MM) || 
        (planner.settings.axis_steps_per_mm[1] != YSTEPS_MM) || 
        (planner.settings.axis_steps_per_mm[2] != ZSTEPS_MM)) {
            //LCD_clear_line(10);
            tft.print("Steps/mm: ");
            tft.print(planner.settings.axis_steps_per_mm[0],0);
            tft.print("/");
            tft.print(planner.settings.axis_steps_per_mm[1],0);
            tft.print("/");
            tft.print(planner.settings.axis_steps_per_mm[2],0);
            XSTEPS_MM = planner.settings.axis_steps_per_mm[0];
            YSTEPS_MM = planner.settings.axis_steps_per_mm[1];
            ZSTEPS_MM = planner.settings.axis_steps_per_mm[2];
            tft.print("\n");
        }
    }

    if (UI_update && PRINT_BUILD) {

        //LCD_clear_line(14, ILI9341_GREEN);
        tft.print("Microsteps: ");
        tft.print(MYXSTR(X_MICROSTEPS) "/");
        tft.print(MYXSTR(Y_MICROSTEPS) "/");
        tft.print(MYXSTR(Z_MICROSTEPS) " ");
    }
}

void print_steps_mm()
{
    if (UI_update) {
        if ((planner.settings.axis_steps_per_mm[0] != XSTEPS_MM) || 
        (planner.settings.axis_steps_per_mm[1] != YSTEPS_MM) || 
        (planner.settings.axis_steps_per_mm[2] != ZSTEPS_MM)) {
            LCD_clear_line(10);
            tft.print("Steps/mm: ");
            tft.print(planner.settings.axis_steps_per_mm[0],0);
            tft.print("/");
            tft.print(planner.settings.axis_steps_per_mm[1],0);
            tft.print("/");
            tft.print(planner.settings.axis_steps_per_mm[2],0);
            XSTEPS_MM = planner.settings.axis_steps_per_mm[0];
            YSTEPS_MM = planner.settings.axis_steps_per_mm[1];
            ZSTEPS_MM = planner.settings.axis_steps_per_mm[2];
        }
    }
}

template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
uint8_t get_stepper_status(TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st) 
{
    //0 == no error, 1 == error
    // uint8_t rtn = st.stst(); //stst
    // rtn |= st.olb() << 1; //open load 
    // rtn |= st.ola() << 2; //open load
    // rtn |= st.s2gb() << 3; //short detection
    // rtn |= st.s2ga() << 4; //short detection
    // rtn |= st.ot() << 6; //overtemp shutdown
    // rtn |= st.sg() << 7; //motor stall
    // return rtn;

    //reorder above for error priority
    uint8_t rtn = st.stst(); //stst
    rtn |= st.olb() << 1; //open load 
    rtn |= st.ola() << 2; //open load
    rtn |= st.otpw() << 3; //temp warn
    rtn |= st.s2gb() << 4; //short detection
    rtn |= st.s2ga() << 5; //short detection
    rtn |= st.ot() << 6; //overtemp shutdown
    rtn |= st.sg() << 7; //motor stall
    return rtn;
}

void chipshover_loop()
{
    char cmdbuf[20];

    if (JS_MOVE_UP) {
        snprintf(cmdbuf, 19, "G0 Y%f", current_position.y-1);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_UP=false;
    } 

    if (JS_MOVE_DOWN) {
        snprintf(cmdbuf, 19, "G0 Y%f", current_position.y+1);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_DOWN=false;
    } 

    if (JS_MOVE_RIGHT) {
        snprintf(cmdbuf, 19, "G0 X%f", current_position.x-1);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_RIGHT=false;
    } 

    if (JS_MOVE_LEFT) {
        snprintf(cmdbuf, 19, "G0 X%f", current_position.x+1);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_LEFT=false;
    } 

    if (JS_MOVE_ZP) {
        snprintf(cmdbuf, 19, "G0 Z%f", current_position.z+1);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_ZP = false;
    }
    if (JS_MOVE_ZM) {
        snprintf(cmdbuf, 19, "G0 Z%f", current_position.z-1);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_ZM = false;
    }
  //update_xyz(current_position.x, current_position.y, current_position.z);
    if (REQ_STEPPER_INIT) {
        REQ_STEPPER_INIT = false;
        stepper.init();          // Init stepper. This enables interrupts!

    }

    if (REQ_BUTTON_HOME) {
        queue.enqueue_one_now("G28 XYZ");
        REQ_BUTTON_HOME = false;
    }
    UI_update = true;
}

bool released = false;

void update_xyz(float x, float y, float z);
void ui_error_update();
void heartbeat_itr();
void handle_pause();
//void update_UI_status_msg(const char *msg, bool clear=true);
void check_LCD();
void handle_estop();
void LCD_update_js();
void handle_js();
void print_build_info();
void print_steps_mm();
void button_homing();
extern uint32_t usb_conn_active;


enum {
    CS_STAT_RUNNING = 0,
    CS_STAT_BUSY = 1,
    CS_STAT_UNHOMED = 2,
    CS_STAT_ESTOP = 4

} CHIPSHOVER_STATUS;

uint8_t CS_STATUS = CS_STAT_UNHOMED;
uint8_t CS_STATUS_PREV = 255; //should be unknown one
uint16_t HOME_BUTTON_COUNTER = 0;
uint8_t LCD_update_div = 0;


void chipshover_tick()
{

    cli();
    handle_js();
    const xyze_pos_t lops = current_position.asLogical();
    if (!digitalRead(57)) {
        CS_STATUS |= CS_STAT_ESTOP;
        handle_estop();
    }
    if (!digitalRead(55)) {
        //pause button
        if (released) {
            handle_pause();
            if (!(CS_STATUS & CS_STAT_ESTOP))
                CS_STATUS |= CS_STAT_UNHOMED;
        }
        released = false;
        if ((HOME_BUTTON_COUNTER++ >= 1464)) { //should be ~3 seconds if this really is 488Hz
            if (CS_STATUS & CS_STAT_ESTOP) {

            }
            button_homing();
            HOME_BUTTON_COUNTER = 0;
        }
    } else {
        released = true;
        HOME_BUTTON_COUNTER = 0;
    }
    //LCD_update_js();


    if (LCD_update_div++ > 5) {
        
        LCD_update_div = 0;
        update_xyz(lops.x, lops.y, lops.z);
        print_build_info();
        //print_steps_mm();
        ui_error_update();
        if (usb_conn_active > 0) {
            digitalWrite(85, 1);
            usb_conn_active--;
        } else {
            digitalWrite(85, 0);
        }
        if ((CS_STATUS != CS_STATUS_PREV) && UI_update) {
            update_UI_status_msg("ChipSHOVER: ", true);
            CS_STATUS_PREV = CS_STATUS;
            if (CS_STATUS & CS_STAT_BUSY) {
                update_UI_status_msg("Busy ", false);
                digitalWrite(22, 1);
            } else {
                update_UI_status_msg("Idle ", false);
                digitalWrite(22, 0);
            }

            if (CS_STATUS & CS_STAT_UNHOMED) {
                update_UI_status_msg("Unhomed", false);
                digitalWrite(72, 1);
            } else {

                digitalWrite(72, 0);
            }

            if (CS_STATUS & CS_STAT_ESTOP) {
                update_UI_status_msg("ChipSHOVER: ESTOP");
            }

        }
    }
    sei();
}