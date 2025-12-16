/*
 * CPE 301 Final Project - Swamp Cooler
 * Team 44: Bousso Seck
 */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <RTClib.h>
#include <Stepper.h>

/* ----------------- STATE DEFINITIONS ----------------- */
enum State {
  DISABLED,
  IDLE,
  ERROR_STATE,
  RUNNING
};

/* ----------------- PIN DEFINITIONS ----------------- */
// LCD
#define LCD_RS 12
#define LCD_E  11
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 6

// Sensors
#define DHT_PIN 7
#define DHT_TYPE DHT11

// Buttons
#define START_BUTTON_PIN 18
#define STOP_BUTTON_PIN  19
#define RESET_BUTTON_PIN 2

// Fan + Stepper
#define FAN_PIN 10
#define STEPPER_PIN1 22
#define STEPPER_PIN2 24
#define STEPPER_PIN3 26
#define STEPPER_PIN4 28

/* ----------------- CONSTANTS ----------------- */
#define WATER_THRESHOLD 100
#define TEMP_THRESHOLD 25.0
const unsigned long debounceDelay = 50;

/* ----------------- GPIO REGISTERS ----------------- */
// LEDs on Port C (pins 30,32,34,36)
#define LED_DDR  DDRC
#define LED_PORT PORTC

// Buttons
#define BTN_DDR_D DDRD
#define BTN_PORT_D PORTD
#define BTN_DDR_B DDRB
#define BTN_PORT_B PORTB

/* -----------------OBJECTS ----------------- */
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
DHT dht(DHT_PIN, DHT_TYPE);
RTC_DS1307 rtc;
Stepper stepper(2048, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);

/* ================= GLOBALS ================= */
State currentState = DISABLED;

volatile bool startPressed = false;
volatile bool stopPressed  = false;
volatile bool resetPressed = false;

volatile unsigned long lastStart = 0;
volatile unsigned long lastStop  = 0;
volatile unsigned long lastReset = 0;

unsigned long lastVentCheck = 0;
int lastVentPosition = 0;

/* ----------------- UART ----------------- */
void uart_init(unsigned long baud) {
  unsigned long FCPU = 16000000;
  unsigned int ubrr = (FCPU / 16 / baud) - 1;

  UBRR0  = ubrr;
  UCSR0A = 0x20;
  UCSR0B = 0x18;
  UCSR0C = 0x06;
}

void uart_putchar(char c) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void uart_println(const char* s) {
  while (*s) uart_putchar(*s++);
  uart_putchar('\r');
  uart_putchar('\n');
}

/* ----------------- ADC ----------------- */
void adc_init() {
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADMUX  |= (1 << REFS0);
}

int adc_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xE0) | channel;
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

/* ----------------- ISR ----------------- */
void startISR() {
  unsigned long t = millis();
  if (t - lastStart > debounceDelay) {
    startPressed = true;
    lastStart = t;
  }
}

void stopISR() {
  unsigned long t = millis();
  if (t - lastStop > debounceDelay) {
    stopPressed = true;
    lastStop = t;
  }
}

void resetISR() {
  unsigned long t = millis();
  if (t - lastReset > debounceDelay) {
    resetPressed = true;
    lastReset = t;
  }
}

/* ----------------- HELPERS ----------------- */
void updateLEDs() {
  LED_PORT &= ~((1 << PC7) | (1 << PC5) | (1 << PC3) | (1 << PC1));

  switch (currentState) {
    case DISABLED:    LED_PORT |= (1 << PC7); break;
    case IDLE:        LED_PORT |= (1 << PC5); break;
    case ERROR_STATE: LED_PORT |= (1 << PC3); break;
    case RUNNING:     LED_PORT |= (1 << PC1); break;
  }
}

void controlFan(bool on) {
  analogWrite(FAN_PIN, on ? 255 : 0);
}

void changeState(State s) {
  if (s == currentState) return;

  currentState = s;
  updateLEDs();
  lcd.clear();

  switch (s) {
    case DISABLED:
      lcd.print("System Disabled");
      controlFan(false);
      break;

    case IDLE:
      lcd.print("System Idle");
      controlFan(false);
      break;

    case RUNNING:
      lcd.print("System Running");
      break;

    case ERROR_STATE:
      lcd.print("ERROR: Low Water");
      controlFan(false);
      break;
  }
}

/* ----------------- VENT ----------------- */
void controlVent() {
  unsigned long now = millis();
  if (now - lastVentCheck < 50) return;
  lastVentCheck = now;

  int pot = adc_read(1);
  int target = map(pot, 0, 1023, 0, 180);
  int diff = target - lastVentPosition;

  if (abs(diff) > 5) {
    stepper.step(diff > 0 ? 2 : -2);
    lastVentPosition += (diff > 0 ? 1 : -1);
  }
}

/* ----------------- SETUP ----------------- */
void setup() {
  // LEDs (PC7, PC5, PC3, PC1)
  LED_DDR |= (1 << PC7) | (1 << PC5) | (1 << PC3) | (1 << PC1);

  // Fan (PB4 / Pin 10)
  DDRB |= (1 << PB4);

  // Buttons (inputs + pullups)
  BTN_DDR_D &= ~((1 << PD3) | (1 << PD2));
  BTN_DDR_B &= ~(1 << PB2);

  BTN_PORT_D |= (1 << PD3) | (1 << PD2);
  BTN_PORT_B |= (1 << PB2);

  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), startISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN),  stopISR,  FALLING);
  attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), resetISR, FALLING);

  lcd.begin(16, 2);
  lcd.print("Swamp Cooler");

  dht.begin();
  rtc.begin();
  stepper.setSpeed(10);
  adc_init();
  uart_init(9600);

  currentState = DISABLED;
  updateLEDs();
  lcd.clear();
  lcd.print("System Disabled");
}

/* ----------------- LOOP ----------------- */
void loop() {
  if (startPressed) {
    startPressed = false;
    if (currentState == DISABLED)
      changeState(IDLE);
  }

  if (stopPressed) {
    stopPressed = false;
    changeState(DISABLED);
  }

  if (resetPressed) {
    resetPressed = false;
    if (currentState == ERROR_STATE && adc_read(0) > WATER_THRESHOLD)
      changeState(IDLE);
  }

  if (currentState == IDLE || currentState == RUNNING) {
    if (adc_read(0) < WATER_THRESHOLD) {
      changeState(ERROR_STATE);
    } else {
      float temp = dht.readTemperature();
      if (!isnan(temp)) {
        if (currentState == IDLE && temp > TEMP_THRESHOLD)
          changeState(RUNNING);
        else if (currentState == RUNNING && temp <= TEMP_THRESHOLD)
          changeState(IDLE);
      }
      controlFan(currentState == RUNNING);
      controlVent();
    }
  }
}
