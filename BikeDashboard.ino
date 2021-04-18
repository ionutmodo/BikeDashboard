/*
    TinyGPSPlus: http://arduiniana.org/libraries/tinygpsplus/
    GPS Guide: https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/
    Senzor IR:
    - bec aprins => 0 logic
    - bec stins => 1 logic
    - puls: tranzitie 1 -> 0
*/
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

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "DHT.h"

/* LCD DEFINES */
#define LCD_PIN_RS 12
#define LCD_PIN_EN 11
#define LCD_PIN_D4 4
#define LCD_PIN_D5 5
#define LCD_PIN_D6 6
#define LCD_PIN_D7 7

#define LCD_COLS 16
#define LCD_ROWS 2

#define LCD_ROW_VAL_SPEED 0
#define LCD_COL_VAL_SPEED 0
#define LCD_ROW_TXT_SPEED 0
#define LCD_COL_TXT_SPEED 5

#define LCD_ROW_VAL_TEMP 0
#define LCD_COL_VAL_TEMP 10
#define LCD_ROW_TXT_TEMP 0
#define LCD_COL_TXT_TEMP 14

#define LCD_ROW_DISTANCE_SEPARATOR 1
#define LCD_COL_DISTANCE_SEPARATOR 5
#define LCD_ROW_VAL_DISTANCE 1
#define LCD_COL_VAL_DISTANCE 0
#define LCD_ROW_VAL_DISTANCE_TOTAL 1
#define LCD_COL_VAL_DISTANCE_TOTAL 6
#define LCD_ROW_TXT_DISTANCE 1
#define LCD_COL_TXT_DISTANCE 12

/* LCD DEFINES */
#define DHT11_PIN 10
#define DHT11_TYPE DHT11

#define IR_PIN 2 // pin 2 has interrupt 0
#define DEFAULT_BAUD_RATE 9600
#define UPDATE_INTERVAL_MS 1000
#define MOVING_AVERAGE_WINDOW_SIZE 2
#define PULSE_RATIO_THRESHOLD 10
#define WHEEL_LENGTH_CM 235
#define CONST_SPEED_KMH (3600000 * WHEEL_LENGTH_CM / 100 / 1000) // = 8460
#define CONST_MAX_INVALID_PULSES 3

char line[17];
byte symbol_celsius_degree[8] = { B01110, B10001, B10001, B01110, B00000, B00000, B00000 };
volatile MovingAverage movingAverage(MOVING_AVERAGE_WINDOW_SIZE);
volatile long pulse_current_millis, pulse_last_time_millis;
volatile long pulse_last_diff_millis, pulse_diff_millis, diff_pulse_ratio, pulse_count_invalids;
volatile long var_distance_temp, var_distance_total, var_rpm;
volatile float average_diff, var_speed_kmh;
ulong update_current_millis, update_last_millis;

LiquidCrystal LCD(LCD_PIN_RS, LCD_PIN_EN, LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7);
DHT dht(DHT11_PIN, DHT11_TYPE);

void setup()
{
    LCD.begin(LCD_COLS, LCD_ROWS);
    LCD.setCursor(LCD_COL_TXT_SPEED, LCD_ROW_TXT_SPEED);
    LCD.print("km/h");
    
    LCD.createChar(0, symbol_celsius_degree);
    LCD.setCursor(LCD_COL_TXT_TEMP, LCD_ROW_TXT_TEMP);
    LCD.write(byte(0));

    LCD.setCursor(LCD_COL_TXT_TEMP + 1, LCD_ROW_TXT_TEMP);
    LCD.print("C");

    LCD.setCursor(LCD_COL_DISTANCE_SEPARATOR, LCD_ROW_DISTANCE_SEPARATOR);
    LCD.print("/");

    LCD.setCursor(LCD_COL_TXT_DISTANCE, LCD_ROW_TXT_DISTANCE);
    LCD.print("km");
    
    dht.begin();
    Serial.begin(DEFAULT_BAUD_RATE);

    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_PIN), ISR_count_IR_pulses, FALLING);

    var_distance_temp = 0;
    // read total_distance from EEPROM   
    //EEPROM_clear();
    
    pulse_last_time_millis = millis();
    pulse_last_diff_millis = 0;
    pulse_count_invalids = 0;
    update_last_millis = millis();
    
    delay(100);
}

void loop()
{
    update_current_millis = millis();
    if(update_current_millis - update_last_millis > UPDATE_INTERVAL_MS)
    {
        update_last_millis = update_current_millis;
        write_total_distance_to_eeprom(var_distance_total);
        lcd_display(LCD_COL_VAL_SPEED, LCD_ROW_VAL_SPEED, var_speed_kmh, 4, 2);
        lcd_display(LCD_COL_VAL_TEMP, LCD_ROW_VAL_TEMP, dht.readTemperature(), 4, 1);
        lcd_display(LCD_COL_VAL_DISTANCE, LCD_ROW_VAL_DISTANCE, var_distance_temp / 100000.0, 5, 2);
        lcd_display(LCD_COL_VAL_DISTANCE_TOTAL, LCD_ROW_VAL_DISTANCE_TOTAL, var_distance_total / 100000.0, 6, 2);
        Serial.print(var_rpm);
        Serial.print(' ');
        Serial.print(var_speed_kmh);
        Serial.print(' ');
        Serial.println(var_distance_temp);
    }
}

void ISR_count_IR_pulses()
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

void write_total_distance_to_eeprom(ulong d)
{
    // Save bytes of d in reverse order in EEPROM
    // EEPROM[0]: save bits 0-7 of d (LSB)
    // EEPROM[1]: save bits 8-15 of d
    // EEPROM[2]: save bits 16-23 of d
    // EEPROM[3]: save bits 24-31 of d (MSB)

    EEPROM[0] = (byte)(d & 0xFF); d = d >> 8;
    EEPROM[1] = (byte)(d & 0xFF); d = d >> 8;
    EEPROM[2] = (byte)(d & 0xFF); d = d >> 8;
    EEPROM[3] = (byte)(d);
}

ulong read_total_distance_from_eeprom()
{
    // Reads first 4 bytes from EEPROM and creates an ulong
    // This ulong represents the total distance travelled in centimeters
    // The distance needs to be divided by 100 000 to have two decimal points in km/h
    // EEPROM[0]: bits 0-7
    // EEPROM[1]: bits 8-15
    // EEPROM[2]: bits 16-23
    // EEPROM[3]: bits 24-31
    ulong total_dist_cm = ((ulong)EEPROM[3] << 24) | ((ulong)EEPROM[2] << 16) | ((ulong)EEPROM[1] << 8) | ((ulong)EEPROM[0]);
    return total_dist_cm;
}

void EEPROM_show()
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

void EEPROM_clear()
{
    for (int i = 0; i < EEPROM.length(); ++i)
        EEPROM[i] = (byte)0;
}

/*void lcd_display_distance_cm(int dec_col, int dec_row, int frac_col, int frac_row, ulong dist_cm)
{
    ulong decimal = dist_cm / 100000;
    ulong fractional = (dist_cm % 100000) / 1000;
    lcd_display(dec_col, dec_row,
}*/
