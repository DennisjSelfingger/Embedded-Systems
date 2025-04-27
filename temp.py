#!/usr/bin/env python3
#------------------------------------------------------------------
# Thermostat.py - My CS 350 final project prototype code
# Implements a smart thermostat on Raspberry Pi with:
#  - AHT20 temp sensor (I2C)
#  - 2‑line CharLCD display (I2C)
#  - 3 buttons (toggle mode, ↑temp, ↓temp) via gpiozero interrupts
#  - 2 LEDs (red=heat, blue=cool) with PWM fading via RPi.GPIO
#  - UART output every 30 s to simulate sending data to server
#------------------------------------------------------------------

import time
import datetime
import board
import busio
import serial
import RPi.GPIO as GPIO
import adafruit_ahtx0
from RPLCD.i2c import CharLCD
from gpiozero import Button

# ==== Pin & display setup ====
RED_LED_PIN   = 17    # GPIO 17 → red LED (heating)
BLUE_LED_PIN  = 27    # GPIO 27 → blue LED (cooling)
MODE_BTN_PIN  = 25    # GPIO 25 → cycle off/heat/cool
UP_BTN_PIN    = 12    # GPIO 12 → increase set‑point
DOWN_BTN_PIN  = 16    # GPIO 16 → decrease set‑point

LCD_I2C_ADDR  = 0x27  # PCF8574 expander address
LCD_COLS      = 16
LCD_ROWS      = 2

# ==== Thermostat states ====
STATE_OFF  = 0
STATE_HEAT = 1
STATE_COOL = 2
state_names = ['off', 'heat', 'cool']

# ==== Globals & defaults ====
current_state   = STATE_OFF
set_point       = 72         # default set‑point = 72 °F
pwm_value       = 0          # for LED fading
pwm_direction   = 1          # +1 or −1 step
display_toggle  = True       # flip LCD line 2 each update
uart_interval   = 30         # seconds between UART sends
last_uart_time  = time.time()

# ==== RPi.GPIO PWM setup ====
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(BLUE_LED_PIN, GPIO.OUT)
red_pwm  = GPIO.PWM(RED_LED_PIN, 100)   # 100 Hz
blue_pwm = GPIO.PWM(BLUE_LED_PIN,100)
red_pwm.start(0)
blue_pwm.start(0)

# ==== I2C sensor & LCD init ====
i2c   = busio.I2C(board.SCL, board.SDA)
aht20 = adafruit_ahtx0.AHTx0(i2c)
lcd   = CharLCD(i2c_expander='PCF8574',
                address=LCD_I2C_ADDR,
                cols=LCD_COLS,
                rows=LCD_ROWS,
                backlight_enabled=True)

# ==== UART to simulate server ====
uart = serial.Serial(port='/dev/serial0',
                     baudrate=115200,
                     timeout=1)

# ==== Buttons with gpiozero interrupts ====
mode_btn = Button(MODE_BTN_PIN, pull_up=True, bounce_time=0.2)
up_btn   = Button(UP_BTN_PIN,   pull_up=True, bounce_time=0.2)
down_btn = Button(DOWN_BTN_PIN, pull_up=True, bounce_time=0.2)

def toggle_state():
    """Cycle OFF → HEAT → COOL → OFF."""
    global current_state
    current_state = (current_state + 1) % 3
    print(f"[BTN] Mode → {state_names[current_state]}")  # for debugging

def increase_set_point():
    global set_point
    set_point += 1
    print(f"[BTN] SP ↑ → {set_point}°F")

def decrease_set_point():
    global set_point
    set_point -= 1
    print(f"[BTN] SP ↓ → {set_point}°F")

mode_btn.when_pressed = toggle_state
up_btn.when_pressed   = increase_set_point
down_btn.when_pressed = decrease_set_point

# ==== State‑machine action functions ====
def action_off(temp):
    red_pwm.ChangeDutyCycle(0)
    blue_pwm.ChangeDutyCycle(0)

def action_heat(temp):
    """If below set pt: fade red LED; else solid red."""
    global pwm_value, pwm_direction
    blue_pwm.ChangeDutyCycle(0)
    if temp < set_point:
        pwm_value += pwm_direction * 5
        if pwm_value >= 100:
            pwm_value, pwm_direction = 100, -1
        elif pwm_value <= 0:
            pwm_value, pwm_direction = 0, 1
        red_pwm.ChangeDutyCycle(pwm_value)
    else:
        red_pwm.ChangeDutyCycle(100)

def action_cool(temp):
    """If above set pt: fade blue LED; else solid blue. even though its not 
    working this code should it works """
    global pwm_value, pwm_direction
    red_pwm.ChangeDutyCycle(0)
    if temp > set_point:
        pwm_value += pwm_direction * 5
        if pwm_value >= 100:
            pwm_value, pwm_direction = 100, -1
        elif pwm_value <= 0:
            pwm_value, pwm_direction = 0, 1
        blue_pwm.ChangeDutyCycle(pwm_value)
    else:
        blue_pwm.ChangeDutyCycle(100)

# Map states → action functions
state_actions = {
    STATE_OFF:  action_off,
    STATE_HEAT: action_heat,
    STATE_COOL: action_cool
}

# ==== Helper functions ====
def read_temperature():
    """Read AHT20 Celsius, convert to Fahrenheit."""
    c = aht20.temperature
    f = (c * 9/5) + 32
    return round(f, 1)

def update_lcd(temp):
    """Line 1: date/time; Line 2 toggles between Temp and State/SP."""
    global display_toggle
    lcd.clear()
    now = datetime.datetime.now().strftime("%m/%d/%y %H:%M:%S")
    lcd.cursor_pos = (0, 0)
    lcd.write_string(now)
    lcd.cursor_pos = (1, 0)
    if display_toggle:
        lcd.write_string(f"Temp:{temp}F")
    else:
        lcd.write_string(f"{state_names[current_state]} SP:{set_point}F")
    display_toggle = not display_toggle

def send_uart(temp):
    """Send comma‑delimited: state,temp,set‑point\n"""
    line = f"{state_names[current_state]},{temp},{set_point}\n"
    uart.write(line.encode())

# ==== Main loop ====
try:
    print("Thermostat running… press Ctrl‑C to exit.")
    while True:
        temp = read_temperature()
        update_lcd(temp)
        # drive LEDs based on current state
        state_actions[current_state](temp)
        # send UART every uart_interval seconds
        if time.time() - last_uart_time >= uart_interval:
            send_uart(temp)
            last_uart_time = time.time()
        time.sleep(1)  # update once per second

except KeyboardInterrupt:
    print("😅  Program stopped by user.")

finally:
    # clean up
    lcd.clear()
    red_pwm.stop()
    blue_pwm.stop()
    GPIO.cleanup()
    uart.close()
    print("Cleaned up, exiting.")
