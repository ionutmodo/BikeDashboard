/*
 Name:		BikeDashboard.ino
 Created:	06-June-2020 16:17:48
 Author:	ionut
*/

#include <LiquidCrystal.h>
#include <eeprom.h>
#include "DHT.h"

//// typedefs
typedef unsigned long ulong;

//// defines for DHT11 sensor
#define DHT11_PIN 13
#define DHT11_TYPE DHT11

//// defines for LCD
#define LCD_ROWS 2
#define LCD_COLS 16
#define LCD_PIN_RS 12
#define LCD_PIN_EN 11
#define LCD_PIN_D4 6
#define LCD_PIN_D5 5
#define LCD_PIN_D6 4
#define LCD_PIN_D7 3

#define LCD_ROW_VAL_SPEED 0
#define LCD_COL_VAL_SPEED 0
#define LCD_ROW_TXT_SPEED 0
#define LCD_COL_TXT_SPEED 4

#define LCD_ROW_VAL_TEMP 0
#define LCD_COL_VAL_TEMP 10
#define LCD_ROW_TXT_TEMP 0
#define LCD_COL_TXT_TEMP 14

#define LCD_ROW_VAL_DISTANCE 1
#define LCD_COL_VAL_DISTANCE 0
#define LCD_ROW_VAL_DISTANCE_TOTAL 1
#define LCD_COL_VAL_DISTANCE_TOTAL 6
#define LCD_ROW_TXT_DISTANCE 1
#define LCD_COL_TXT_DISTANCE 12

//// defines to compute constant for speed
// half of wheel length in meters since I have 2 reflectors (full length of 29 inch wheel, 231 cm = 2.31 m)
#define WHEEL_LENGTH_M_HALF 1.155 
#define WHEEL_LENGTH_KM_HALF 0.001155

//// defines for interrupt
#define INTERRUPT_IR_SENSOR 0

//// global variables
char line[17];
byte symbol_celsius_degree[8] = { B01110, B10001, B10001, B01110, B00000, B00000, B00000 };
ulong last_pulse_time_us, last_lcd_update_ms;
float temperature;
const float C = WHEEL_LENGTH_M_HALF * 3600000.; // formula: L((micros()-last) / 10^6)* 3.6
volatile float speed, distance, distance_total;

//// global objects
LiquidCrystal LCD(LCD_PIN_RS, LCD_PIN_EN, LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7);
DHT dht(DHT11_PIN, DHT11_TYPE);

//// methods
void enable_interrupt()
{
    attachInterrupt(INTERRUPT_IR_SENSOR, ISR_count_IR_pulses, FALLING);
}

void disable_interrupt()
{
    detachInterrupt(INTERRUPT_IR_SENSOR);
}

void lcd_display(int col, int row, float value, int total_length, int n_decimals)
{
    LCD.setCursor(col, row);
    if (isnan(value))
    {
        LCD.print("ERR");
    }
    else
    {
        dtostrf(value, total_length, n_decimals, line);
        LCD.print(line);
    }
}

volatile ulong t_us;
void ISR_count_IR_pulses()
{
    t_us = micros() - last_pulse_time_us;
    if (t_us > 42000) // threshold such that max speed is approx 100km/h
    {
        speed = C / t_us;
        distance += WHEEL_LENGTH_KM_HALF;
        distance_total += WHEEL_LENGTH_KM_HALF;
        last_pulse_time_us = micros();
    }
}

void write_total_distance_to_eeprom(float d)
{
    // Since d has 2 decimals, multiply it with 100 and then
    // store it in the following way:
    // EEPROM[0]: bits 0-7 of _distance_x_100
    // EEPROM[1]: bits 8-15 of _distance_x_100
    // EEPROM[2]: bits 16-23 of _distance_x_100

    //Serial.print("write ");
    //Serial.println(d);
    ulong _distance_x_100 = (ulong)(d * 100.0);

    EEPROM[0] = (byte)(_distance_x_100 & 0xFF);
    _distance_x_100 = _distance_x_100 >> 8;

    EEPROM[1] = (byte)(_distance_x_100 & 0xFF);
    _distance_x_100 = _distance_x_100 >> 8;

    EEPROM[2] = (byte)(_distance_x_100 & 0xFF);
}

float read_total_distance_to_eeprom()
{
    // Reads first 3 bytes from EEPROM and creates an ulong
    // This ulong is stored in EEPROM multiplied by 100 and distance
    // needs to be divided by 100 to have 2 decimal points an saved as float
    // EEPROM[0]: bits 0-7
    // EEPROM[1]: bits 8-15
    // EEPROM[2]: bits 16-23
    ulong _distance_x_100 = ((ulong)EEPROM[2] << 16) | ((ulong)EEPROM[1] << 8) | ((ulong)EEPROM[0]);
    float d = _distance_x_100 / 100.0;
    //Serial.print("read ");
    //Serial.println(d);
    return d;
}

void setup()
{
    Serial.begin(9600);
    dht.begin();
    LCD.begin(LCD_COLS, LCD_ROWS);

    LCD.setCursor(LCD_COL_TXT_SPEED, LCD_ROW_TXT_SPEED);
    LCD.print("km/h");

    // create celsius symbol
    LCD.createChar(0, symbol_celsius_degree);
    LCD.setCursor(LCD_COL_TXT_TEMP, LCD_ROW_TXT_TEMP);
    LCD.write(byte(0));

    LCD.setCursor(LCD_COL_TXT_TEMP + 1, LCD_ROW_TXT_TEMP);
    LCD.print("C");

    LCD.setCursor(5, 1);
    LCD.print("/");

    LCD.setCursor(LCD_COL_TXT_DISTANCE, LCD_ROW_TXT_DISTANCE);
    LCD.print("km");

    distance = speed = 0.00;
    distance_total = read_total_distance_to_eeprom();
    last_pulse_time_us = micros();
    last_lcd_update_ms = millis();
    enable_interrupt();
}

void loop()
{
    // if no pulses are received for more than a few seconds, set speed to zero
    if (micros() - last_pulse_time_us > 5000000)
    {
        speed = 0.00;
    }

    if (millis() - last_lcd_update_ms > 1000)
    {
        write_total_distance_to_eeprom(distance_total);
        lcd_display(LCD_COL_VAL_SPEED, LCD_ROW_VAL_SPEED, speed, 4, 1);
        lcd_display(LCD_COL_VAL_TEMP, LCD_ROW_VAL_TEMP, dht.readTemperature(), 4, 1);
        lcd_display(LCD_COL_VAL_DISTANCE, LCD_ROW_VAL_DISTANCE, distance, 5, 2);
        lcd_display(LCD_COL_VAL_DISTANCE_TOTAL, LCD_ROW_VAL_DISTANCE_TOTAL, distance_total, 6, 2);
        last_lcd_update_ms = millis();
    }
}
