/*
    LCD_display_i2c: https://lastminuteengineers.com/i2c-lcd-arduino-tutorial/
    TinyGPSPlus: http://arduiniana.org/libraries/tinygpsplus/
    GPS Guide: https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/
    Senzor IR:
    - bec aprins => 0 logic
    - bec stins => 1 logic
    - puls: tranzitie 1 -> 0

    Arduino Nano settings:
    - Board: Arduino Nano
    - Processor: ATMega328P simple
*/

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "DHT.h"

typedef unsigned long ulong;

struct Node
{
    ulong data;
    Node *next;
};

class MovingAverage
{
private:
    ulong window_size, sum, current_size;
    Node *head, *tail;
public:
    MovingAverage(ulong window_size)
    {
        this->window_size = window_size;
        this->current_size = 0;
        this->sum = 0;
        this->head = this->tail = NULL;
    }
    void add_data(ulong data)
    {
        // whatever we run below, add the data to the sum
        this->sum += data;
        if(this->current_size == 0)
        {
            ++(this->current_size);
            this->head = new Node;
            this->head->data = data;
            this->head->next = NULL;
            this->tail = this->head;
        }
        else if(0 < this->current_size && this->current_size < this->window_size)
        {
            ++(this->current_size);
            Node *node = new Node;
            node->data = data;
            node->next = NULL;
            this->tail->next = node;
            this->tail = node;
        }
        else if(this->current_size == this->window_size)
        {
            // remove head data from the sum
            this->sum -= this->head->data;
            // set node to head
            Node *node = this->head;
            // move head to the second node
            this->head = this->head->next;
            // replace first node's data with new data
            node->data = data;
            // link it to null
            node->next = NULL;
            // make it last node in the list
            this->tail->next = node;
            // update tail
            this->tail = node;
        }
    }
    float compute_average()
    {
        float average = this->sum / ((float)this->current_size);
        return average;
    }
    void show_data()
    {
        Serial.print(" ( ");
        for(Node *p=this->head; p != NULL; p=p->next)
        {
            Serial.print(p->data);
            Serial.print(' ');
        }
        Serial.print(") ");
    }
};


class MemoryHandler
{
private:
    static int const LSB_POS_TOTAL_DIST=0; // 0 to 3
    static int const LSB_POS_CRT_DIST=4; // 4 to 8
    
    void _write_distance(ulong dist, int start)
    {
        int i;
        for(i=start; i<start+4; ++i)
        {
            EEPROM[i] = (byte) (dist & 0xFF);
            dist = dist >> 8;
        }
    }
    
    ulong _read_distance(int start)
    {
        // Reads the 4 bytes from EEPROM at positions [pos_start, pos_stop] and creates an ulong
        // This ulong represents the distance travelled in centimeters (total or current)
        // The distance needs to be divided by 100 000 to have two decimal points in km
        // EEPROM[start+0]: bits 0-7
        // EEPROM[start+1]: bits 8-15
        // EEPROM[start+2]: bits 16-23
        // EEPROM[start+3]: bits 24-31
        ulong dist_cm = 0;
        int i, shift_pos=0;
        for(i=0; i<4; ++i)
        {
            dist_cm = dist_cm | ((ulong)EEPROM[start+i] << shift_pos);
            shift_pos += 8;
        }
        return dist_cm;
    }
public:
    static int const MODE_TOTAL_DIST=0, MODE_CRT_DIST=1;

    static void show_memory()
    {
        for (int i = 0; i < EEPROM.length(); ++i)
        {
            Serial.print(EEPROM[i]);
            if (i % 32 == 31)
                Serial.println();
            else
                Serial.print(' ');
        }
    }
    
    static void clear_memory()
    {
        for (int i = 0; i < EEPROM.length(); ++i)
            EEPROM[i] = (byte)0;
    }
    
    void write_distance(ulong dist, int mode)
    {
        if(mode == MODE_TOTAL_DIST)
        {
            _write_distance(dist, LSB_POS_TOTAL_DIST);
        }
        if(mode == MODE_CRT_DIST)
        {
            _write_distance(dist, LSB_POS_CRT_DIST);
        }
    }
    
    ulong read_distance(int mode)
    {
        if(mode == MODE_TOTAL_DIST)
        {
            return _read_distance(LSB_POS_TOTAL_DIST);
        }
        if(mode == MODE_CRT_DIST)
        {
            return _read_distance(LSB_POS_CRT_DIST);
        }
        return (ulong) 0;
    }
};

/* LCD DEFINES */
//#define LCD_PIN_RS 12
//#define LCD_PIN_EN 11
//#define LCD_PIN_D4 4
//#define LCD_PIN_D5 5
//#define LCD_PIN_D6 6
//#define LCD_PIN_D7 7

#define LCD_COLS 20
#define LCD_ROWS 4
#define LCD_I2C_ADDRESS 0x27

#define LCD_ROW_TXT_SPEED 0 // the "Speed(km/h):" text
#define LCD_COL_TXT_SPEED 0
#define LCD_ROW_VAL_SPEED 0 // speed value
#define LCD_COL_VAL_SPEED 15

#define LCD_ROW_TXT_DISTANCE_CRT 1 // the "Crt dist:" text
#define LCD_COL_TXT_DISTANCE_CRT 0
#define LCD_ROW_VAL_DISTANCE_CRT 1 // the current distance value
#define LCD_COL_VAL_DISTANCE_CRT 15

#define LCD_ROW_TXT_DISTANCE_TOTAL 2 // the "Total dist:" text
#define LCD_COL_TXT_DISTANCE_TOTAL 0
#define LCD_ROW_VAL_DISTANCE_TOTAL 2 // the total distance value
#define LCD_COL_VAL_DISTANCE_TOTAL 14

#define LCD_ROW_TXT_TEMP 3 // the "Temp. (*C):" text
#define LCD_COL_TXT_TEMP 0
#define LCD_ROW_VAL_TEMP 3 // the temp value
#define LCD_COL_VAL_TEMP 15

/* LCD DEFINES */
#define DHT11_PIN 10
#define DHT11_TYPE DHT11

#define HALL_PIN 2 // pin #2 has interrupt 0 (INT0)
#define RESET_DISTANCE_BUTTON_PIN 3 // pin #3 has interrupt 1 (INT1), but I use digitalRead
#define RESET_CRT_DIST_TIMEOUT_MS 3000
#define RESET_TOTAL_DIST_TIMEOUT_MS 5000

#define DEFAULT_BAUD_RATE 9600
#define UPDATE_INTERVAL_MS 1000
#define MOVING_AVERAGE_WINDOW_SIZE 2
#define PULSE_RATIO_THRESHOLD 10
#define WHEEL_LENGTH_CM 235
#define CONST_SPEED_KMH (3600000 * WHEEL_LENGTH_CM / 100 / 1000) // = 8460
#define CONST_MAX_INVALID_PULSES 3
#define CONST_ZERORIZE_SPEED_THRESHOLD_MS 3000


char line[17];
int display_length;
byte symbol_celsius_degree[8] = { B01110, B10001, B10001, B01110, B00000, B00000, B00000 };
volatile MovingAverage movingAverage(MOVING_AVERAGE_WINDOW_SIZE);
volatile long pulse_current_millis, pulse_last_time_millis;
volatile long pulse_last_diff_millis, pulse_diff_millis, diff_pulse_ratio, pulse_count_invalids;
volatile long var_distance_temp, var_distance_total, var_rpm;
volatile float average_diff, var_speed_kmh;
ulong update_current_millis, update_last_millis, button_last_time, button_time_diff;

LiquidCrystal_I2C LCD(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);
DHT dht(DHT11_PIN, DHT11_TYPE);
MemoryHandler mem;

void setup()
{
    LCD.init();
    LCD.clear();
    LCD.backlight();

    LCD.setCursor(LCD_COL_TXT_SPEED, LCD_ROW_TXT_SPEED);
    LCD.print("Speed(km/h):");

    LCD.setCursor(LCD_COL_TXT_TEMP, LCD_ROW_TXT_TEMP);
    LCD.print("Temp( C):");

    LCD.createChar(0, symbol_celsius_degree);
    LCD.setCursor(LCD_COL_TXT_TEMP + 5, LCD_ROW_TXT_TEMP);
    LCD.write(byte(0));

    LCD.setCursor(LCD_COL_TXT_DISTANCE_CRT, LCD_ROW_TXT_DISTANCE_CRT);
    LCD.print("Crt dist(km):");
    
    LCD.setCursor(LCD_COL_TXT_DISTANCE_TOTAL, LCD_ROW_TXT_DISTANCE_TOTAL);
    LCD.print("Tot dist(km):");
    
    dht.begin();
    Serial.begin(DEFAULT_BAUD_RATE);

    pinMode(HALL_PIN, INPUT_PULLUP);
    pinMode(RESET_DISTANCE_BUTTON_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), ISR_count_hall_pulses, FALLING);

    var_distance_temp = mem.read_distance(MemoryHandler::MODE_CRT_DIST);
    var_distance_total = mem.read_distance(MemoryHandler::MODE_TOTAL_DIST);
    
    pulse_last_time_millis = millis();
    pulse_last_diff_millis = 0;
    pulse_count_invalids = 0;
    update_last_millis = millis();
    
    delay(100);
}

void loop()
{
    update_current_millis = millis();

    // update display data
    if(update_current_millis - update_last_millis > UPDATE_INTERVAL_MS)
    {
        update_last_millis = update_current_millis;
        mem.write_distance(var_distance_temp, MemoryHandler::MODE_CRT_DIST);
        mem.write_distance(var_distance_total, MemoryHandler::MODE_TOTAL_DIST);
        lcd_display(LCD_COL_VAL_SPEED, LCD_ROW_VAL_SPEED, var_speed_kmh, 5, 2); // (var_speed_kmh < 10) ? 3 : 4
        lcd_display(LCD_COL_VAL_DISTANCE_CRT, LCD_ROW_VAL_DISTANCE_CRT, var_distance_temp / 100000.0, 5, 2);
        lcd_display(LCD_COL_VAL_DISTANCE_TOTAL, LCD_ROW_VAL_DISTANCE_TOTAL, var_distance_total / 100000.0, 6, 2);
        lcd_display(LCD_COL_VAL_TEMP, LCD_ROW_VAL_TEMP, dht.readTemperature(), 5, 2);
        /*
        Serial.print(var_rpm);
        Serial.print(' ');
        Serial.print(var_speed_kmh);
        Serial.print(' ');
        Serial.println(var_distance_temp);
        */
    }

    if(digitalRead(RESET_DISTANCE_BUTTON_PIN) == HIGH) {
        button_last_time = millis();
    }
    else {
        button_time_diff = millis() - button_last_time;
        if(button_time_diff > RESET_CRT_DIST_TIMEOUT_MS) {
            var_distance_temp = 0;
            mem.write_distance(var_distance_temp, MemoryHandler::MODE_CRT_DIST);
            if(button_time_diff > RESET_TOTAL_DIST_TIMEOUT_MS) {
                var_distance_total = 0;
                mem.write_distance(var_distance_total, MemoryHandler::MODE_TOTAL_DIST);
            }
        }
    }
    
    // set speed to zero if no pulses are received for a specific time
    if(millis() - pulse_last_time_millis > CONST_ZERORIZE_SPEED_THRESHOLD_MS)
    {
        var_speed_kmh = 0.00;
    }
}

void ISR_count_hall_pulses()
{
    pulse_current_millis = millis();
    pulse_diff_millis = pulse_current_millis - pulse_last_time_millis;
    diff_pulse_ratio = max(pulse_last_diff_millis, pulse_diff_millis) / min(pulse_last_diff_millis, pulse_diff_millis);
//    Serial.print(pulse_current_millis);
//    Serial.print(' ');
//    Serial.print(pulse_diff_millis);
//    Serial.print(' ');
    if(diff_pulse_ratio > PULSE_RATIO_THRESHOLD) // filter out
    {
        ++pulse_count_invalids;
//        Serial.print(pulse_diff_millis);
        if(pulse_count_invalids == CONST_MAX_INVALID_PULSES)
        {
//            Serial.print(" R ");
            pulse_count_invalids = 0; // reset this counter and reset the state
            pulse_last_diff_millis = 0; // shold have set it to zero, but above i might have division by zero
            pulse_last_time_millis = pulse_current_millis;
        }
//        Serial.println(" -");
    }
    else // keep current value
    {
        movingAverage.add_data(pulse_diff_millis);
        pulse_count_invalids = 0;
        pulse_last_diff_millis = pulse_diff_millis;
        pulse_last_time_millis = pulse_current_millis;
        average_diff = movingAverage.compute_average();
        var_distance_temp += WHEEL_LENGTH_CM;
        var_distance_total += WHEEL_LENGTH_CM;
        var_speed_kmh = CONST_SPEED_KMH / average_diff;
//        movingAverage.show_data();
//        Serial.println(average_diff);
//        Serial.println(var_speed_kmh);
        //var_rpm = 60000 / average_diff;
        /*
            Formula for speed:
                average  ms ... 2.35 m
                3600 000 ms ... x km/h !!!
                x = speed(km/h) = (3 600 000 ms * 2.35 m) / average ms  / 1000 = 8460 / average
            Formula for distance travelled:
                add wheel length at each revolution
            Formula for var_rpm:
                average ms ... 1 rot
                 60 000 ms ... x rot
                 x = var_rpm = 60 000 ms * rot / average ms
        */
    }
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

void find_i2c_address() {
    Serial.begin (9600);

  // Leonardo: wait for serial port to connect
  while (!Serial) 
    {
    }

  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}
