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
};

#include <LiquidCrystal.h>

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

#define IR_PIN 2 // pin 2 has interrupt 0
#define DEFAULT_BAUD_RATE 9600
#define UPDATE_INTERVAL_MS 1000
#define MOVING_AVERAGE_WINDOW_SIZE 2
#define PULSE_RATIO_THRESHOLD 10
#define WHEEL_LENGTH_CM 235
#define CONST_SPEED_KMH (3600000 * WHEEL_LENGTH_CM / 100 / 1000) // = 8460
//#define CONST_CM_TO_KM 1000000

char line[17];
LiquidCrystal LCD(LCD_PIN_RS, LCD_PIN_EN, LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7);
volatile MovingAverage movingAverage(MOVING_AVERAGE_WINDOW_SIZE);
volatile ulong current_pulse_millis, pulse_last_time_millis, pulse_last_diff_millis, diff_pulse_millis, diff_pulse_ratio, pulse_count_invalids;
volatile float average_diff, speed_kmh, distance_covered, rpm;
ulong update_current_millis, update_last_millis;

void setup()
{  
    Serial.begin(DEFAULT_BAUD_RATE);

    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_PIN ), ISR_count_IR_pulses, FALLING);
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
        Serial.print(rpm);
        Serial.print(' ');
        Serial.print(speed_kmh);
        Serial.print(' ');
        Serial.println(distance_covered);
    }
}

void ISR_count_IR_pulses()
{
    current_pulse_millis = millis();
    diff_pulse_millis = current_pulse_millis - pulse_last_time_millis;
    diff_pulse_ratio = max(pulse_last_diff_millis, diff_pulse_millis) / min(pulse_last_diff_millis, diff_pulse_millis);
    if(diff_pulse_ratio > PULSE_RATIO_THRESHOLD) // filter out
    {
        ++pulse_count_invalids;
        if(pulse_count_invalids == 3)
        {
            pulse_count_invalids = 0; // reset this counter and reset the state
            pulse_last_diff_millis = 0; // shold have set it to zero, but above i might have division by zero
            pulse_last_time_millis = current_pulse_millis;
        }
    }
    else // keep current value
    {
        movingAverage.add_data(diff_pulse_millis);
        pulse_count_invalids = 0;
        pulse_last_diff_millis = diff_pulse_millis;
        pulse_last_time_millis = current_pulse_millis;
        average_diff = movingAverage.compute_average();
        distance_covered += WHEEL_LENGTH_CM; // wheel length in cm
        speed_kmh = CONST_SPEED_KMH / average_diff;
        //rpm = 60000 / average_diff;
        /*
            Formula for speed:
                average  ms ... 2.35 m
                3600 000 ms ... x km/h !!!
                x = speed(km/h) = (3 600 000 ms * 2.35 m) / average ms  / 1000 = 8460 / average
            Formula for distance travelled:
                add wheel length at each revolution
            Formula for RPM:
                average ms ... 1 rot
                 60 000 ms ... x rot
                 x = RPM = 60 000 ms * rot / average ms
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
