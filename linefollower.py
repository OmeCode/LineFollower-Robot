from machine import Pin, PWM, ADC
import time

# --- 1. Ρυθμίσεις Hardware ---

# Κουμπί Εκκίνησης (Pin 20)
button = Pin(20, Pin.IN, Pin.PULL_DOWN)

# Κινητήρας Αριστερά (Pins 8 & 9)
m_left_a = PWM(Pin(9))
m_left_b = PWM(Pin(8))
m_left_a.freq(1000)
m_left_b.freq(1000)

# Κινητήρας Δεξιά (Pins 10 & 11)
m_right_a = PWM(Pin(10))
m_right_b = PWM(Pin(11))
m_right_a.freq(1000)
m_right_b.freq(1000)

# Αισθητήρες (Αναλογικά Pins 26, 27, 28)
sensor_left = ADC(Pin(26))
sensor_center = ADC(Pin(27))
sensor_right = ADC(Pin(28))

# --- 2. Μεταβλητές & Κατώφλια (Thresholds) ---

SPEED_BASE = 64000 #50000 
SPEED_SEARCH = 50000 #45000

TH_LEFT = 35000
TH_CENTER = 29000
TH_RIGHT = 36000

# Μεταβλητές PID 
Kp = 150.0  
Ki = 0.0    
Kd = 50.0   

# --- 3. Συναρτήσεις Κίνησης ---

def drive(speed_left, speed_right):
    """Ελέγχει τους κινητήρες (θετικές τιμές: εμπρός, αρνητικές: πίσω)."""
    if speed_left > 0:
        m_left_a.duty_u16(int(speed_left))
        m_left_b.duty_u16(0)
    elif speed_left < 0:
        m_left_a.duty_u16(0)
        m_left_b.duty_u16(int(abs(speed_left)))
    else:
        m_left_a.duty_u16(0)
        m_left_b.duty_u16(0)

    if speed_right > 0:
        m_right_a.duty_u16(int(speed_right))
        m_right_b.duty_u16(0)
    elif speed_right < 0:
        m_right_a.duty_u16(0)
        m_right_b.duty_u16(int(abs(speed_right)))
    else:
        m_right_a.duty_u16(0)
        m_right_b.duty_u16(0)

def stop():
    drive(0, 0)

# --- 4. Κύριο Πρόγραμμα (Ατέρμονος Βρόχος) ---

integral = 0
last_error = 0
last_sensor = "center" 
running = False

# Μεταβλητή για το χρονόμετρο απώλειας γραμμής
lost_line_time = None 

stop() 
print("Έτοιμο! Πάτα το κουμπί για εκκίνηση...")

try:
    while True:
        # Έλεγχος του κουμπιού
        if button.value() == 0:
            running = not running
            if running:
                print("Εκκίνηση PID Line Follower!")
                integral = 0
                last_error = 0
                lost_line_time = None # Μηδενίζουμε το χρονόμετρο στην εκκίνηση
            else:
                print("Στοπ από το κουμπί.")
                stop()
            time.sleep(0.5) 

        # Αν είναι σε κατάσταση λειτουργίας (running == True)
        if running:
            # Διαβάζουμε τους αισθητήρες
            val_L = sensor_left.read_u16()
            val_C = sensor_center.read_u16()
            val_R = sensor_right.read_u16()

            is_black_L = val_L > TH_LEFT
            is_black_C = val_C > TH_CENTER
            is_black_R = val_R > TH_RIGHT

            # 1. Ενημέρωση μνήμης για την αναζήτηση
            if is_black_L:
                last_sensor = "left"
            elif is_black_R:
                last_sensor = "right"
            elif is_black_C:
                last_sensor = "center"


            # --- Ο ΑΛΓΟΡΙΘΜΟΣ ΠΛΟΗΓΗΣΗΣ ---
            
            # 2. ΧΑΣΑΜΕ ΤΗ ΓΡΑΜΜΗ ΕΝΤΕΛΩΣ
            if not is_black_L and not is_black_C and not is_black_R:
                
                # Αν μόλις την χάσαμε (το lost_line_time είναι None), καταγράφουμε την ώρα
                if lost_line_time is None:
                    lost_line_time = time.ticks_ms()
                
                # Ελέγχουμε αν πέρασαν 1500 milliseconds (1.5 δευτερόλεπτο)
                if time.ticks_diff(time.ticks_ms(), lost_line_time) > 900:
                    print("Σφάλμα: Η γραμμή χάθηκε για πάνω από 0.9s. Σταμάτημα!")
                    stop()
                    running = False # Απενεργοποιεί το ρομπότ, περιμένει πάτημα κουμπιού
                
                # Αν δεν έχουν περάσει 1.5s, συνεχίζει το Ζικ-Ζακ αναζήτησης
                else:
                    if last_sensor == "left":
                        drive(-20000, SPEED_SEARCH) #20000
                    elif last_sensor == "right":
                        drive(SPEED_SEARCH, -20000)  #20000
                    else:
                        drive(SPEED_BASE, SPEED_BASE)

            # 3. ΕΙΜΑΣΤΕ ΠΑΝΩ Ή ΚΟΝΤΑ ΣΤΗ ΓΡΑΜΜΗ -> PID Control
            else:
                # Αφού βρήκαμε τη γραμμή, σβήνουμε το χρονόμετρο
                lost_line_time = None
                
                # Υπολογισμός συνεχούς σφάλματος
                error = (val_L - val_R) / 1000.0
                
                # Εξίσωση PID
                P = Kp * error
                integral += error
                I = Ki * integral
                D = Kd * (error - last_error)
                
                correction = P + I + D
                last_error = error
                
                # Εφαρμογή διόρθωσης
                speed_L = SPEED_BASE - correction
                speed_R = SPEED_BASE + correction
                
                # Clamping: Αποτροπή υπέρβασης ορίων του PWM (-65535 έως 65535)
                speed_L = max(-65535, min(65535, speed_L))
                speed_R = max(-65535, min(65535, speed_R))
                
                drive(speed_L, speed_R)
            
            # Ελάχιστη παύση για σταθερό υπολογισμό
            time.sleep(0.005)

except KeyboardInterrupt:
    print("Διακοπή από το χρήστη.")
    stop()
