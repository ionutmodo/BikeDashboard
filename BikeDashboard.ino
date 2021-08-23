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

    LCD menu:
    1) speed, crt dist, total dist, temperature
    2) moving time, max speed (km/h), avg speed (kmh), avg pace (time/km)

    Formula for speed:
        average       us ... 2.35 m
        3,600,000,000 us ... x km
        x = speed(km/h) = (3,600,000,000 us * 2.35 m) / average ms / 1000 (m to km) = 8,460,000 / average
    Formula for distance travelled:
        add wheel length at each revolution
    Formula for var_rpm:
        average ms ... 1 rot
         60 000 ms ... x rot
         x = var_rpm = 60 000 ms * rot / average ms
*/

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "DHT.h"

typedef unsigned long ulong;
#define LCD_COLS 20
#define LCD_ROWS 4
#define LCD_I2C_ADDRESS 0x27

// DISPLAY 1
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

#define LCD_ROW_TXT_TEMPERATURE 3 // the "Temp. (*C):" text
#define LCD_COL_TXT_TEMPERATURE 0
#define LCD_ROW_VAL_TEMPERATURE 3 // the temp value
#define LCD_COL_VAL_TEMPERATURE 15

// DISPLAY 2
#define LCD_ROW_TXT_TIME_ELAPSED 0  // the "TimeElapsed:" text
#define LCD_COL_TXT_TIME_ELAPSED 0
#define LCD_ROW_VAL_TIME_ELAPSED 0  // the time elapsed value
#define LCD_COL_VAL_TIME_ELAPSED 12

#define LCD_ROW_TXT_MAX_SPEED 1  // the "MaxSpeed(km/h):" text
#define LCD_COL_TXT_MAX_SPEED 0
#define LCD_ROW_VAL_MAX_SPEED 1  // the max speed value
#define LCD_COL_VAL_MAX_SPEED 15

#define LCD_ROW_TXT_AVG_SPEED 2  // the "AvgSpeed(km/h):" text
#define LCD_COL_TXT_AVG_SPEED 0
#define LCD_ROW_VAL_AVG_SPEED 2  // the avg speed value
#define LCD_COL_VAL_AVG_SPEED 15

#define LCD_ROW_TXT_AVG_PACE 3  // the "AvgPace(t/km):" text
#define LCD_COL_TXT_AVG_PACE 0
#define LCD_ROW_VAL_AVG_PACE 3  // the avg pace value
#define LCD_COL_VAL_AVG_PACE 14

// TEMPERATURE SENSOR
#define DHT11_PIN 10
#define DHT11_TYPE DHT11

// HALL SEOSOR
#define HALL_PIN 2 // pin #2 has interrupt 0 (INT0)

// PUSH BUTTON
#define RESET_DISTANCE_BUTTON_PIN 3 // pin #3 has interrupt 1 (INT1), but I use digitalRead
#define DISPLAY_NEXT_SCREEN_US 50000
#define RESET_CRT_DIST_TIMEOUT_US 3000000
#define RESET_TOTAL_DIST_TIMEOUT_US 5000000

#define DEFAULT_BAUD_RATE 9600
#define UPDATE_INTERVAL_US 1000000
#define MOVING_AVERAGE_WINDOW_SIZE 8
#define PULSE_RATIO_THRESHOLD 10
#define WHEEL_LENGTH_CM 235.0 // wheel circumference
#define WHEEL_TICKS_PER_REVOLUTION 4.0 // have 4 magnets on the wheel
#define WHEEL_ARC_CM (WHEEL_LENGTH_CM / WHEEL_TICKS_PER_REVOLUTION) // convert cm to m
#define CONST_SPEED_KMH (3600000.0 * WHEEL_ARC_CM) // = 8460 000 / WHEEL_TICKS_PER_REVOLUTION;
#define CONST_MAX_INVALID_PULSES 3
#define CONST_ZERORIZE_SPEED_THRESHOLD_US 3000000 // set it to high value to get smaller speeds
#define MAX_LCD_DISPLAYS 2

char line[17];
char time_string[12]; // "10h35m43s#"
int display_length;
byte symbol_celsius_degree[8] = { B01110, B10001, B10001, B01110, B00000, B00000, B00000 };
volatile ulong pulse_current_micros = 0, pulse_last_time_micros = 0;
volatile ulong pulse_last_diff_micros = 0, pulse_diff_micros = 0, diff_pulse_ratio = 0, pulse_count_invalids = 0;
volatile ulong var_distance_temp = 0, var_distance_total = 0, var_rpm = 0;
volatile float average_diff = 0, var_speed_kmh = 0;
float var_max_speed_kmh = 0, var_avg_speed_kmh = 0;
ulong var_avg_pace_skm = 0, ulong total_moving_time_secs = 0, button_last_value = 0; // use button_last_value to avoid screen bouncing
ulong update_current_micros = 0, update_last_micros = 0, button_last_time_high = 0, button_time_diff = 0;

LiquidCrystal_I2C LCD(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);
DHT dht(DHT11_PIN, DHT11_TYPE);

class Average
{
private:
    float sum;
    ulong count;
public:
    Average()
    {
        this->sum = 0;
        this->count = 0;
    }

    void add(float val)
    {
        this->sum += val;
        ++(this->count);
    }

    float compute_average()
    {
        if(this->count == 0) return 0;
        float avg = this->sum / this->count;
        return avg;
    }
};
Average avgSpeed;

class MovingAverage
{
private:
    struct Node { ulong data; Node *next; };
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
    
    void add(ulong data)
    {
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
            // move head to the next node
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
        if(this->current_size == 0) return(float) 0;
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
volatile MovingAverage movingAverage(MOVING_AVERAGE_WINDOW_SIZE);

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
MemoryHandler mem;

class TimeClock
{
private:
    byte hh, mm, ss;
    ulong total_seconds;
public:
    TimeClock()
    {
        this->reset();
    }

    void reset()
    {
        this->hh = 0;
        this->mm = 0;
        this->ss = 0;
        this->total_seconds = 0;
    }
    
    void tick()
    {
        ++(this->total_seconds);
        ++(this->ss);
        if(this->ss == 60)
        {
            this->ss = 0;
            ++(this->mm);
            if(this->mm == 60)
            {
                this->mm = 0;
                ++(this->hh);
            }
        }
    }
    
    ulong get_total_seconds() { return this->total_seconds; }
    
    byte get_hours() { return this->hh; }
    
    byte get_minutes() { return this->mm; }
    
    byte get_seconds() { return this->ss; }
};
TimeClock timeClock;

class DisplayMenu
{
private:
    // indicates the screen number
    byte screen_no;
    
    // indicates whether text was displayed to avoid displaying it at each update
    byte is_text_displayed;
    
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
public:
    DisplayMenu()
    {
        this->screen_no = 1;
        this->is_text_displayed = 0;
    }

    byte get_screen_no()
    {
        return this->screen_no;
    }
    
    void next_screen()
    {
        if(++this->screen_no > MAX_LCD_DISPLAYS)
        {
            this->screen_no = 1;
        }
        this->is_text_displayed = 0;
        LCD.clear();
        this->display_screen();
    }
    
    void display_screen()
    {
        switch(this->screen_no)
        {
            case 1: _display_screen_1(); break;
            case 2: _display_screen_2(); break;
            default: _display_error(); break;
        }
    }

    void _display_error()
    {
        LCD.setCursor(0, 0);
        LCD.print("There was an error!");
        LCD.setCursor(0, 2);
        LCD.print("Please check code!");
    }
    
    void _display_screen_1()
    {
        if(this->is_text_displayed == 0)
        {
            this->is_text_displayed = 1;
            LCD.setCursor(LCD_COL_TXT_SPEED, LCD_ROW_TXT_SPEED);
            LCD.print("Speed:");
        
            LCD.setCursor(LCD_COL_TXT_TEMPERATURE, LCD_ROW_TXT_TEMPERATURE);
            LCD.print("Temperature:");
        
            // LCD.createChar(0, symbol_celsius_degree);
            // LCD.setCursor(LCD_COL_TXT_TEMPERATURE + 5, LCD_ROW_TXT_TEMPERATURE);
            // LCD.write(byte(0));
        
            LCD.setCursor(LCD_COL_TXT_DISTANCE_CRT, LCD_ROW_TXT_DISTANCE_CRT);
            LCD.print("Crt distance:");
            
            LCD.setCursor(LCD_COL_TXT_DISTANCE_TOTAL, LCD_ROW_TXT_DISTANCE_TOTAL);
            LCD.print("Tot distance:");
        }
        lcd_display(LCD_COL_VAL_SPEED, LCD_ROW_VAL_SPEED, var_speed_kmh, 5, 2);
        lcd_display(LCD_COL_VAL_DISTANCE_CRT, LCD_ROW_VAL_DISTANCE_CRT, var_distance_temp / 100000.0, 5, 2);
        lcd_display(LCD_COL_VAL_DISTANCE_TOTAL, LCD_ROW_VAL_DISTANCE_TOTAL, var_distance_total / 100000.0, 6, 2);
        lcd_display(LCD_COL_VAL_TEMPERATURE, LCD_ROW_VAL_TEMPERATURE, dht.readTemperature(), 5, 2);
    }
    
    void _display_screen_2()
    {
        if(this->is_text_displayed == 0)
        {
            this->is_text_displayed = 1;
            
            LCD.setCursor(LCD_COL_TXT_TIME_ELAPSED, LCD_ROW_TXT_TIME_ELAPSED);
            LCD.print("Elapsed:");
            
            LCD.setCursor(LCD_COL_TXT_MAX_SPEED, LCD_ROW_TXT_MAX_SPEED);
            LCD.print("Max speed:");
            
            LCD.setCursor(LCD_COL_TXT_AVG_SPEED, LCD_ROW_TXT_AVG_SPEED);
            LCD.print("Avg  speed:");
        
            LCD.setCursor(LCD_COL_TXT_AVG_PACE, LCD_ROW_TXT_AVG_PACE);
            LCD.print("Time / km:");
        }
        
        LCD.setCursor(LCD_COL_VAL_TIME_ELAPSED, LCD_ROW_VAL_TIME_ELAPSED);
        sprintf(time_string, "% d:%02d:%02d", timeClock.get_hours(), timeClock.get_minutes(), timeClock.get_seconds());
        LCD.print(time_string);

        lcd_display(LCD_COL_VAL_MAX_SPEED, LCD_ROW_VAL_MAX_SPEED, var_max_speed_kmh, 5, 2);
        lcd_display(LCD_COL_VAL_AVG_SPEED, LCD_ROW_VAL_AVG_SPEED, avgSpeed.compute_average(), 5, 2);

        LCD.setCursor(LCD_COL_VAL_AVG_PACE, LCD_ROW_VAL_AVG_PACE);
        sprintf(time_string, "% 2dm%2ds", (int)(var_avg_pace_skm / 60), (int)(var_avg_pace_skm % 60));
        LCD.print(time_string);
    }
};
DisplayMenu menu;

void setup()
{
    LCD.init();
    LCD.clear();
    LCD.backlight();
    
    dht.begin();
    Serial.begin(DEFAULT_BAUD_RATE);

    pinMode(HALL_PIN, INPUT_PULLUP);
    pinMode(RESET_DISTANCE_BUTTON_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), ISR_count_hall_pulses, FALLING);

    var_distance_temp = mem.read_distance(MemoryHandler::MODE_CRT_DIST);
    var_distance_total = mem.read_distance(MemoryHandler::MODE_TOTAL_DIST);
        
    pulse_last_time_micros = micros();
    pulse_last_diff_micros = 0;
    pulse_count_invalids = 0;
    update_last_micros = micros();

    menu.display_screen();
    
    delay(100);
}

void loop()
{
    update_current_micros = micros();

    // update display data
    if(update_current_micros - update_last_micros > UPDATE_INTERVAL_US)
    {
        update_last_micros = update_current_micros;
        mem.write_distance(var_distance_temp, MemoryHandler::MODE_CRT_DIST);
        mem.write_distance(var_distance_total, MemoryHandler::MODE_TOTAL_DIST);
        
        var_avg_pace_skm = (ulong)(timeClock.get_total_seconds() / ((ulong)var_distance_temp / 100000.0));
        if(var_speed_kmh > 0.01) // for 2.35m as wheel diameter and CONST_ZERORIZE_SPEED_THRESHOLD_US=3000000ms, min speed is 3600000000*0.00235/3000000=2.82
        {
            timeClock.tick();
            avgSpeed.add(var_speed_kmh);
        }

        menu.display_screen();
    }

    if(digitalRead(RESET_DISTANCE_BUTTON_PIN) == HIGH)
    {
        button_last_time_high = micros();
        button_last_value = 0;
    }
    else
    {
        button_time_diff = micros() - button_last_time_high;
        if(button_time_diff > RESET_TOTAL_DIST_TIMEOUT_US)
        {
            if(button_last_value != RESET_TOTAL_DIST_TIMEOUT_US)
            {
                button_last_value = RESET_TOTAL_DIST_TIMEOUT_US;
                var_distance_total = 0;
                mem.write_distance(var_distance_total, MemoryHandler::MODE_TOTAL_DIST);
            }
        }
        else if(button_time_diff > RESET_CRT_DIST_TIMEOUT_US)
        {
            if(button_last_value != RESET_CRT_DIST_TIMEOUT_US)
            {
                button_last_value = RESET_CRT_DIST_TIMEOUT_US;
                var_distance_temp = 0;
                timeClock.reset();
                mem.write_distance(var_distance_temp, MemoryHandler::MODE_CRT_DIST);
            }
        }
        else if(button_time_diff > DISPLAY_NEXT_SCREEN_US)
        {
            if(button_last_value != DISPLAY_NEXT_SCREEN_US)
            {
                button_last_value = DISPLAY_NEXT_SCREEN_US;
                menu.next_screen();
            }
        }
    }
    
    // set speed to zero if no pulses are received for a specific time
    if(micros() - pulse_last_time_micros > CONST_ZERORIZE_SPEED_THRESHOLD_US)
    {
        var_speed_kmh = 0.00;
    }
}

void ISR_count_hall_pulses()
{
    pulse_current_micros = micros();
    pulse_diff_micros = pulse_current_micros - pulse_last_time_micros;
    diff_pulse_ratio = max(pulse_last_diff_micros, pulse_diff_micros) / min(pulse_last_diff_micros, pulse_diff_micros);
    if(diff_pulse_ratio > PULSE_RATIO_THRESHOLD) // filter out
    {
        ++pulse_count_invalids;
        if(pulse_count_invalids == CONST_MAX_INVALID_PULSES)
        {
            pulse_count_invalids = 0; // reset this counter and reset the state
            pulse_last_diff_micros = 0; // shold have set it to zero, but above i might have division by zero
            pulse_last_time_micros = pulse_current_micros;
        }
    }
    else // keep current value
    {
        movingAverage.add(pulse_diff_micros);
        pulse_count_invalids = 0;
        pulse_last_diff_micros = pulse_diff_micros;
        pulse_last_time_micros = pulse_current_micros;
        average_diff = movingAverage.compute_average();
        var_distance_temp += WHEEL_ARC_CM;
        var_distance_total += WHEEL_ARC_CM;
        var_speed_kmh = CONST_SPEED_KMH / average_diff;

        if(var_speed_kmh > var_max_speed_kmh)
        {
            var_max_speed_kmh = var_speed_kmh;
        }
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
