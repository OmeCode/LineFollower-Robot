from machine import Pin, PWM, ADC
import time

# --- Motors ---
# Motor A (left)
motorA_pwm = PWM(Pin(8))
motorA_dir = Pin(9, Pin.OUT)

# Motor B (right)
motorB_pwm = PWM(Pin(10))
motorB_dir = Pin(11, Pin.OUT)

motorA_pwm.freq(1000)
motorB_pwm.freq(1000)

# --- Sensors ---
left_sensor = ADC(Pin(26))
center_sensor = ADC(Pin(27))
right_sensor = ADC(Pin(28))

# --- Parameters ---
THRESHOLD = 30000  # ρύθμισε το μετά από δοκιμή
BASE_SPEED = 40000

# --- Motor control ---
def set_motor_A(speed):
    if speed >= 0:
        motorA_dir.value(1)
        motorA_pwm.duty_u16(int(speed))
    else:
        motorA_dir.value(0)
        motorA_pwm.duty_u16(int(-speed))


def set_motor_B(speed):
    if speed >= 0:
        motorB_dir.value(0)  
        motorB_pwm.duty_u16(int(speed))
    else:
        motorB_dir.value(1)
        motorB_pwm.duty_u16(int(-speed))

# --- Main loop ---
while True:
    left = left_sensor.read_u16()
    center = center_sensor.read_u16()
    right = right_sensor.read_u16()

    print(left, center, right)

    # Ανίχνευση γραμμής (μαύρο = μικρότερη τιμή συνήθως)
    left_on = left < THRESHOLD
    center_on = center < THRESHOLD
    right_on = right < THRESHOLD

    # --- Logic ---
    if center_on:
        # ευθεία
        set_motor(motorA_pwm, motorA_dir, BASE_SPEED)
        set_motor(motorB_pwm, motorB_dir, BASE_SPEED)

    elif left_on:
        # στρίψε αριστερά
        set_motor(motorA_pwm, motorA_dir, BASE_SPEED // 2)
        set_motor(motorB_pwm, motorB_dir, BASE_SPEED)

    elif right_on:
        # στρίψε δεξιά
        set_motor(motorA_pwm, motorA_dir, BASE_SPEED)
        set_motor(motorB_pwm, motorB_dir, BASE_SPEED // 2)

    else:
        # χάθηκε η γραμμή
        set_motor(motorA_pwm, motorA_dir, 0)
        set_motor(motorB_pwm, motorB_dir, 0)

    time.sleep(0.01)