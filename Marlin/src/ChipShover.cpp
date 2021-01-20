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
#include "module/stepper/trinamic.h"

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
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
  #define CHARSZ 2
  #define CHARWIDTH 6 * CHARSZ
  #define CHARHEIGHT 8 * CHARSZ

#define MYXSTR(X) MYSTRINGIFY(X)
#define MYSTRINGIFY(X) #X

bool REQ_BUTTON_HOME = false;
bool REQ_STEPPER_INIT = false;

#define JOG_EN_LED_PIN 10
#define ESTOP_SW_PIN 57
#define JOG_EN_SW_PIN 66
#define PAUSE_SW_PIN 55
#define USB_CON_LED_PIN 85

#define BUSY_LED_PIN 22
#define UNHOMED_LED_PIN 72

#define ENC_PIN_A 42
#define ENC_PIN_B 43

#define JS_PIN_XP 37
#define JS_PIN_XN 38
#define JS_PIN_YP 39
#define JS_PIN_YN 40

#define FUSE_24V_PIN 96
#define FUSE_5V_PIN 36


#define STEPPER_STEP_H_MM 2
#define STEPPER_STEP_L_MM 0.2

#define JOG_FAST_SW_PIN 67

float STEPPER_STEP_SZ = STEPPER_STEP_L_MM;

Adafruit_ILI9341 tft = Adafruit_ILI9341(77, 73, 87);

extern GcodeSuite gcode;

bool JOG_EN = false;

struct encoder_value {
    bool A, B;
};
encoder_value ENCODER_SEQUENCE[] = {{.A = 0, .B = 0}, {.A = 1, .B = 0}, {.A = 1, .B = 1}, {.A = 0, .B = 1}};

void update_xyz(float x, float y, float z);
void ui_error_update();
void heartbeat_itr();
void handle_pause();
void check_LCD();
void handle_estop();
void LCD_update_js();
void handle_js();
void print_build_info();
void print_steps_mm();
void button_homing();
extern uint32_t usb_conn_active;

const char *STEPPER_STATUSES[] = {"GOOD", "IDLE", "OPENLOADA", "OPENLOADB", "OVERTEMP WARN", "SHORTA", "SHORTB", "OVERTEMP SHUTDOWN", "MOTOR STALL"};

enum CHIPSHOVER_STATUS {
    CS_STAT_RUNNING = 0,
    CS_STAT_BUSY = 1,
    CS_STAT_UNHOMED = 2,
    CS_STAT_ESTOP = 4,
    CS_STAT_5V_FUSE = 8,
    CS_STAT_24V_FUSE = 16,
} ;

// float XSTEPS_MM=0, YSTEPS_MM=0, ZSTEPS_MM=0;

uint8_t CS_STATUS = CS_STAT_UNHOMED;
uint8_t CS_STATUS_PREV = 255; //should be unknown one
encoder_value enc_last = {.A = 0, .B = 0};


#include <Wire.h>

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

    pinMode(BUSY_LED_PIN, OUTPUT);
    pinMode(UNHOMED_LED_PIN, OUTPUT);
    pinMode(USB_CON_LED_PIN, OUTPUT);
    pinMode(JOG_EN_LED_PIN, OUTPUT);

    bool A = digitalRead(ENC_PIN_A);
    bool B = digitalRead(ENC_PIN_B);
    encoder_value enc_val = {.A = A, .B = B};
    enc_last = enc_val;
    Wire1.begin();
    #define I2C_LED_PIN 60
    pinMode(I2C_LED_PIN, OUTPUT);
    digitalWrite(I2C_LED_PIN, 1);
    Wire1.setClock(100000);

    // Wire1.beginTransmission(0b01010000);
    // Wire1.write(0x00);//addr
    // Wire1.write(0x00);//addr
    // Wire1.write(0xAA);
    // Wire1.endTransmission();

    #define TEMP_ADDR_X 0b1001011
    #define TEMP_ADDR_Y 0b1001010
    #define TEMP_ADDR_Z 0b1001001

    // Wire1.beginTransmission(TEMPADDR);
    // Wire1.write(0x00);
    // Wire1.endTransmission(false);
    // volatile uint8_t from = Wire1.requestFrom(TEMPADDR, 2);
    // volatile uint16_t temp = 0;
    // if (from > 0) {
    //     temp = Wire1.read() << 8;
    //     temp |= Wire1.read();

    //     //Wire1.read((uint8_t *)&temp);
    //     temp >>= 5;
    // }


    //  Wire1.write(byte(0b10011101)); //x
    //  Wire1.write(byte(0x00));
    //  Wire1.endTransmission(false);
    //  //Wire1.write(byte(0b10011100)); //x
    //  Wire1.requestFrom(byte(0b10011100), byte(1)); //x
    //  while(!Wire1.available());
    //  volatile uint16_t temp = (uint8_t)Wire1.read();
    //  while(!Wire1.available());
    //  temp |= ((uint8_t)Wire1.read() << 8);
    //  Wire1.endTransmission();
    // Wire1.write(0x00);
    // Wire1.write(0x00);
    // Wire1.write(0x10);
    // Wire1.endTransmission();

}

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
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(CHARSZ);
    delete_status();
    set_cursor_at_status();
    tft.print("Busy");
  }

  void set_status_idle() {
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(CHARSZ);
    delete_status();
    set_cursor_at_status();
    tft.print("Idle");
  }

double STORED_X = -101, STORED_Y = -100 , STORED_Z = -100;
float XTEMP = 0;

float read_temp(uint8_t addr)
{

    Wire1.beginTransmission(addr);
    Wire1.write(0x00);
    Wire1.endTransmission(false);
    uint8_t from = Wire1.requestFrom(addr, 2);
    uint16_t temp = 0;
    if (from > 0) {
        temp = Wire1.read() << 8;
        temp |= Wire1.read();

        //Wire1.read((uint8_t *)&temp);
        temp >>= 5;
    }
    return (float)temp * 0.125f;
}

void display_temp()
{
    static float old_xtemp, old_ytemp, old_ztemp;
    if (UI_update) {
        float xtemp = read_temp(TEMP_ADDR_X);
        float ytemp = read_temp(TEMP_ADDR_Y);
        float ztemp = read_temp(TEMP_ADDR_Z);
        bool update_xtemp = fabs(xtemp - old_xtemp) > 0.5f;
        bool update_ytemp = fabs(ytemp - old_ytemp) > 0.5f;
        bool update_ztemp = fabs(ztemp - old_ztemp) > 0.5f;

        if (update_xtemp || update_ytemp || update_ztemp) {
            LCD_clear_line(9);
            tft.print("X: ");
            tft.print(xtemp);

            tft.print(" Y: ");
            tft.print(ytemp);

            tft.print(" Z: ");
            tft.print(ztemp);

            old_xtemp = xtemp;
            old_ytemp = ytemp;
            old_ztemp = ztemp;
        }

    }
}

void update_xyz(float x, float y, float z) {
  if (UI_update) {
    if (STORED_X != x) {
      LCD_clear_line(2, ILI9341_YELLOW);
      tft.print("X: ");
      tft.print(x, 3);

      STORED_X = x;
    }
    if (STORED_Y != y) {

      LCD_clear_line(3, ILI9341_YELLOW);
      tft.print("Y: ");
      tft.print(y, 3);
      STORED_Y = y;
    }
    if (STORED_Z != z) {
      LCD_clear_line(4, ILI9341_YELLOW);
      tft.print("Z: ");
      tft.print(z, 3);
      STORED_Z = z;
    }
  }
}

void lcd_print_bin(uint8_t n)
{
    for (uint8_t i = 0; i < 8; i++) {
        tft.print((n >> i) & 1, 10);
    }
}

uint8_t motor_err_index(uint8_t err) 
{
    for (uint8_t i = 0; i < 8; i++) {
        if ((err >> (7 - i)) & 1)
            return 8 - i;
    }
    return 0;
}

#include "src/feature/e_parser.h"
#include "gcode/gcode.h"

void handle_estop()
{
    if (UI_update) {
        EmergencyParser::State st = EmergencyParser::EP_M410;
        // EmergencyParser::State st = EmergencyParser::EP_M112;
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

void update_UI_status_msg(const char *msg, bool clear=true, uint16_t colour=ILI9341_WHITE)
{
    if (UI_update) {
        if (clear) {
            LCD_clear_line(0, colour);
        }
        //tft.print("ChipSHOVER: ");
        tft.print(msg);
    }
}



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
void handle_js()
{
    if (UI_update && JOG_EN) {
        JS_MOVE_LEFT = digitalRead(JS_PIN_XP); //Left
        JS_MOVE_RIGHT = digitalRead(JS_PIN_XN); //Right
        JS_MOVE_DOWN = digitalRead(JS_PIN_YP); //Down
        JS_MOVE_UP = digitalRead(JS_PIN_YN); //Up

        bool A = digitalRead(ENC_PIN_A);
        bool B = digitalRead(ENC_PIN_B);
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

// bool XH_last, XL_last, YH_last, YL_last;
// bool SW_last = 0, A_last = 0, B_last = 0;
// void LCD_update_js()
// {
//     if (UI_update) {

//         bool XH, XL, YH, YL;
//         bool SW, A, B;
//         XH = digitalRead(37); //Left
//         XL = digitalRead(38); //Right
//         YH = digitalRead(39); //Down
//         YL = digitalRead(40); //Up
//         SW = !digitalRead(56);
//         A = digitalRead(42);
//         B = digitalRead(43);
//         if ((XH != XH_last) || (XL != XL_last)) {

//             LCD_clear_line(8);
//             if (XH) {
//                 tft.print("XH ");
//             }
//             if (XL) {
//                 tft.print("XL ");
//             }
//             XH_last = XH; XL_last = XL;
//         }

//         if ((YH != YH_last) || (YL != YL_last)) {

//             LCD_clear_line(9);
//             if (YH) {
//                 tft.print("YH ");
//             }
//             if (YL) {
//                 tft.print("YL ");
//             }
//             YH_last = YH; YL_last = YL;
//         }

//         if ((A != A_last) || (B != B_last) || (SW != SW_last)) {
//             LCD_clear_line(10);
//             encoder_value enc_val = {.A = A, .B = B};
//             int rot = get_encoder_direction(enc_last, enc_val);
//             if (rot == 1) {
//                 tft.print("Right ");
//             } else if (rot == -1) {
//                 tft.print("Left ");
//             }
//             //enc_last = enc_val;
//             // if (A) {
//             //     tft.print("A ");
//             // }
//             // if (B) {
//             //     tft.print("B ");
//             // }

//             if (SW) {
//                 tft.print("SW ");
//             }
//             A_last = A; B_last = B; SW_last = SW;
//         }
//     }
// }


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

uint8_t STATUS_X = 0, STATUS_Y = 0, STATUS_Z = 0;
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
        if (STATUS_X != status) {
            tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(CHARSZ);
            tft.fillRect(0, CHARHEIGHT * 5, 40 * CHARWIDTH, 1 * CHARHEIGHT, ILI9341_BLACK);
            tft.setCursor(0, CHARHEIGHT * 5);
            tft.print("Status X: ");
            //tft.print(status, 2);
            //lcd_print_bin(status);
            tft.print(STEPPER_STATUSES[motor_err_index(status)]);
            STATUS_X = status;
        }

        status = get_stepper_status(stepperY);
        if (STATUS_Y != status) {
            tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(CHARSZ);
            tft.fillRect(0, CHARHEIGHT * 6, 40 * CHARWIDTH, 1 * CHARHEIGHT, ILI9341_BLACK);
            tft.setCursor(0, CHARHEIGHT * 6);
            tft.print("Status Y: ");
            //lcd_print_bin(status);
            tft.print(STEPPER_STATUSES[motor_err_index(status)]);
            STATUS_Y = status;
        }

        status = get_stepper_status(stepperZ);
        if (STATUS_Z != status) {
            tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(CHARSZ);
            tft.fillRect(0, CHARHEIGHT * 7, 40 * CHARWIDTH, 1 * CHARHEIGHT, ILI9341_BLACK);
            tft.setCursor(0, CHARHEIGHT * 7);
            tft.print("Status Z: ");
            //lcd_print_bin(status);
            tft.print(STEPPER_STATUSES[motor_err_index(status)]);
            STATUS_Z = status;
        }
    }
}

void ui_display_step_size()
{
    static float last_step_sz;
    if (UI_update && (STEPPER_STEP_SZ != last_step_sz)) {
        LCD_clear_line(8, ILI9341_YELLOW);
        tft.print("JS STEP SZ: ");
        tft.print(STEPPER_STEP_SZ, 2);
        last_step_sz = STEPPER_STEP_SZ;
        tft.print("mm");
    }
}


void print_build_info()
{
    static float XSTEPS_MM, YSTEPS_MM, ZSTEPS_MM;
    static bool NOT_PRINT_BUILD; //reversed since static starts as 0
    if (UI_update && !NOT_PRINT_BUILD) {
        LCD_clear_line(9, ILI9341_GREEN);
        tft.setTextSize(1);
        tft.print("\n\n\n\n\n\nBuild: ");
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

    if (UI_update && !NOT_PRINT_BUILD) {

        //LCD_clear_line(14, ILI9341_GREEN);
        tft.print("Microsteps: ");
        tft.print(MYXSTR(X_MICROSTEPS) "/");
        tft.print(MYXSTR(Y_MICROSTEPS) "/");
        tft.print(MYXSTR(Z_MICROSTEPS) " ");

        tft.print("\nBased on Marlin 3D Printer Firmware");
        NOT_PRINT_BUILD = true;
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

bool REINIT_STEPPERS=false;
void chipshover_loop()
{
    char cmdbuf[20];
    if (REINIT_STEPPERS) {
        queue.clear();
        queue.enqueue_one_now("M122 I");
        REINIT_STEPPERS=false;
    }

    else if (JS_MOVE_UP) {
        snprintf(cmdbuf, 19, "G0 Y%f", current_position.y+STEPPER_STEP_SZ);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_UP=false;
    } 

    else if (JS_MOVE_DOWN) {
        snprintf(cmdbuf, 19, "G0 Y%f", current_position.y-STEPPER_STEP_SZ);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_DOWN=false;
    } 

    else if (JS_MOVE_RIGHT) {
        snprintf(cmdbuf, 19, "G0 X%f", current_position.x+STEPPER_STEP_SZ);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_RIGHT=false;
    } 

    else if (JS_MOVE_LEFT) {
        snprintf(cmdbuf, 19, "G0 X%f", current_position.x-STEPPER_STEP_SZ);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_LEFT=false;
    } 

    else if (JS_MOVE_ZP) {
        snprintf(cmdbuf, 19, "G0 Z%f", current_position.z+STEPPER_STEP_SZ);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_ZP = false;
    }
    else if (JS_MOVE_ZM) {
        snprintf(cmdbuf, 19, "G0 Z%f", current_position.z-STEPPER_STEP_SZ);
        queue.enqueue_one_now(cmdbuf);
        queue.enqueue_one_now("M400");
        JS_MOVE_ZM = false;
    }
  //update_xyz(current_position.x, current_position.y, current_position.z);

    else if (REQ_BUTTON_HOME) {
        queue.enqueue_one_now("G28 XYZ");
        REQ_BUTTON_HOME = false;
    }

    if (!UI_update) {
        queue.enqueue_one_now("M106 S 150"); //set fan speed to 150/255
    }
    UI_update = true;
}


void chipshover_tick()
{
    static bool PAUSE_RELEASED;
    static bool JOG_EN_RELEASED;
    static bool JOG_FAST_RELEASED;
    static uint16_t HOME_BUTTON_COUNTER = 0;
    static uint8_t LCD_update_div = 0;
    static uint8_t temp_update_div = 0;

    cli();
    handle_js();

    digitalWrite(JOG_EN_LED_PIN, JOG_EN);

    const xyze_pos_t lops = current_position.asLogical();
    if (!digitalRead(ESTOP_SW_PIN)) {
        CS_STATUS |= CS_STAT_ESTOP;
        handle_estop();
    }

    if (!digitalRead(JOG_EN_SW_PIN)) {
        if (JOG_EN_RELEASED) {
            JOG_EN = !JOG_EN;
        }
        JOG_EN_RELEASED = false;
    } else {
        JOG_EN_RELEASED = true;
    }

    if (!digitalRead(JOG_FAST_SW_PIN)) {
        if (JOG_FAST_RELEASED) {
            if (STEPPER_STEP_SZ == STEPPER_STEP_H_MM)
                STEPPER_STEP_SZ = STEPPER_STEP_L_MM;
            else 
                STEPPER_STEP_SZ = STEPPER_STEP_H_MM;
        }
        JOG_FAST_RELEASED = false;
    } else {
        JOG_FAST_RELEASED = true;
    }

    if (!digitalRead(PAUSE_SW_PIN)) {
        //pause button
        if (PAUSE_RELEASED) {
            handle_pause();
            if (!(CS_STATUS & CS_STAT_ESTOP))
                CS_STATUS |= CS_STAT_UNHOMED;
        }
        PAUSE_RELEASED = false;
        if ((HOME_BUTTON_COUNTER++ >= 1464)) { //should be ~3 seconds if this really is 488Hz
            if (CS_STATUS & CS_STAT_ESTOP) {

            }
            button_homing();
            HOME_BUTTON_COUNTER = 0;
        }
    } else {
        PAUSE_RELEASED = true;
        HOME_BUTTON_COUNTER = 0;
    }

    if (!digitalRead(FUSE_24V_PIN) && digitalRead(ESTOP_SW_PIN)) {
        CS_STATUS |= CS_STAT_24V_FUSE;
    }
    if (!digitalRead(FUSE_5V_PIN)) {
        CS_STATUS |= CS_STAT_5V_FUSE;
    }

    if (LCD_update_div++ > 5) {
        
        LCD_update_div = 0;
        update_xyz(lops.x, lops.y, lops.z);
        print_build_info();
        ui_error_update();
        if (usb_conn_active > 0) {
            digitalWrite(USB_CON_LED_PIN, 1);
            usb_conn_active--;
        } else {
            digitalWrite(USB_CON_LED_PIN, 0);
        }
        if ((CS_STATUS != CS_STATUS_PREV) && UI_update) {
            update_UI_status_msg("ChipShover: ", true);
            CS_STATUS_PREV = CS_STATUS;
            if (CS_STATUS & CS_STAT_BUSY) {
                update_UI_status_msg("Busy ", false);
                digitalWrite(BUSY_LED_PIN, 1);
            } else {
                update_UI_status_msg("Idle ", false);
                digitalWrite(BUSY_LED_PIN, 0);
            }

            if (CS_STATUS & CS_STAT_UNHOMED) {
                update_UI_status_msg("Unhomed", false);
                digitalWrite(UNHOMED_LED_PIN, 1);
            } else {

                digitalWrite(UNHOMED_LED_PIN, 0);
            }

            if (CS_STATUS & CS_STAT_ESTOP) {
                update_UI_status_msg("ESTOP: RELEASE ESTOP\n", true, ILI9341_RED);
                while (!digitalRead(ESTOP_SW_PIN)) {
                    watchdog_refresh();
                }
                REINIT_STEPPERS = true;
                LCD_clear_line(0);
                LCD_clear_line(1);
                CS_STATUS = CS_STAT_UNHOMED;
            }


            if ((CS_STATUS & (CS_STAT_24V_FUSE | CS_STAT_5V_FUSE)) == (CS_STAT_5V_FUSE | CS_STAT_24V_FUSE)) {
                update_UI_status_msg("ALL FUSES BLOWN", true, ILI9341_RED);
            } else if (CS_STATUS & CS_STAT_5V_FUSE) {
                update_UI_status_msg("5V FUSE BLOWN", true, ILI9341_RED);
            } else if (CS_STATUS & CS_STAT_24V_FUSE) {
                update_UI_status_msg("24V FUSE BLOWN", true, ILI9341_RED);
                //stop from dying
                while (!digitalRead(FUSE_24V_PIN)) {
                    watchdog_refresh();
                }
            }


        }
        ui_display_step_size();
        if (temp_update_div++ > 20) {
            display_temp();
            temp_update_div = 0;
        }
    }
    sei();
}

void M14400()
{
    SERIAL_CHAR(CS_STATUS & 1);
}