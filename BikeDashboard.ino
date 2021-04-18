/*
    TinyGPSPlus: http://arduiniana.org/libraries/tinygpsplus/
    GPS Guide: https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/
    Senzor IR:
    - bec aprins => 0 logic
    - bec stins => 1 logic
    - puls: tranzitie 1 -> 0
*/

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

#define MOVING_AVERAGE_WINDOW_SIZE 2
#define DIFF_RATIO_THRESHOLD 10

typedef unsigned long ulong;
typedef unsigned int uint;

struct Node
{
    uint data;
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

char line[17];
LiquidCrystal LCD(LCD_PIN_RS, LCD_PIN_EN, LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7);
volatile MovingAverage movingAverage(MOVING_AVERAGE_WINDOW_SIZE);
volatile long current_millis, last_time_millis, last_diff_millis, diff_millis, diff_ratio, false_pulses_count;
volatile float average_diff, speed_kmh, distance_covered, rpm;

void setup()
{  
    Serial.begin(9600);

    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), ISR_count_IR_pulses, FALLING);
    last_time_millis = millis();
    last_diff_millis = 0;
    false_pulses_count = 0;
    delay(100);
}

void loop()
{
    Serial.print(rpm);
    Serial.print(' ');
    Serial.print(speed_kmh);
    Serial.print(' ');
    Serial.println(distance_covered);
}

void ISR_count_IR_pulses()
{
    current_millis = millis();
    diff_millis = current_millis - last_time_millis;
    diff_ratio = max(last_diff_millis, diff_millis) / min(last_diff_millis, diff_millis);
    if(diff_ratio > DIFF_RATIO_THRESHOLD) // filter out
    {
        ++false_pulses_count;
        if(false_pulses_count == 3)
        {
            false_pulses_count = 0; // reset this counter and reset the state
            last_diff_millis = 0; // shold have set it to zero, but above i might have division by zero
            last_time_millis = current_millis;
        }
    }
    else // keep current value
    {
        movingAverage.add_data(diff_millis);
        false_pulses_count = 0;
        last_diff_millis = diff_millis;
        last_time_millis = current_millis;
        average_diff = movingAverage.compute_average();
        distance_covered += 235; // wheel length in cm
        speed_kmh = 8460 / average_diff;
        rpm = 60000 / average_diff;
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
void temp_code()
{
    /*LCD.begin(16, 2);
    LCD.setCursor(0, 0);
    LCD.print("R=");
    LCD.setCursor(6, 0);
    LCD.print("F=");
    LCD.setCursor(0, 1);
    LCD.print("RP=");
    LCD.setCursor(9, 1);
    LCD.print("TM=");
    lcd_display(2, 0, AmountOfReadings, 3, 0);
    lcd_display(9, 0, FrequencyReal, 6, 0);
    lcd_display(3, 1, RPM, 5, 0);  
    lcd_display(12, 1, average, 4, 0);*/
}
