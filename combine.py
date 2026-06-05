import time
import spidev
import RPi.GPIO as GPIO
import mfrc522
from mfrc522 import SimpleMFRC522
import pymysql
from RPLCD.i2c import CharLCD
import threading

lcd = CharLCD('PCF8574', 0x27)

# =========================
# LCD Helper Functions
# =========================
def update_lcd(line1, line2):
    """Safely update the 16x2 LCD screen. Keep content in English to avoid mojibake."""
    try:
        lcd.clear()
        lcd.cursor_pos = (0, 0)
        lcd.write_string(str(line1)[:16])

        lcd.cursor_pos = (1, 0)
        lcd.write_string(str(line2)[:16])
    except Exception as e:
        print(f"[LCD ERROR] {e}")


# =========================
# Database Connection
# =========================
db = pymysql.connect(
    host="10.23.201.16",
    user="student",
    password="student",
    database="clothing_care",
    autocommit=True
)
cursor = db.cursor()

# =========================
# Basic Configuration
# =========================
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

TOTAL_BASKETS = 10

# Logical baskets: 1..10
# Baskets currently with physical servos: 1..3
SERVO_PINS = {
    1: 18,
    2: 23,
    3: 24
}

SERVO_FREQ_HZ = 50

SERVO_CLOSE_ANGLES = {
    1: 45,
    2: 0,
    3: 0
}

SERVO_OPEN_ANGLES = {
    1: 135,
    2: 0,    # Basket 2 specially calibrated to avoid over-rotation
    3: 90
}

# =========================
# Stepper Motor Settings
# =========================
STEPPER_PINS = [12, 16, 20, 21]
STEPPER2_PINS = [26, 19, 13, 6]

STEPPER_SEQ = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

# =========================
# LED Pin Settings
# =========================
LED_RFID_PIN = 17
LED_STEPPER2_PIN = 27
LED_SERVO_PIN = 22

# =========================
# Button Settings
# =========================
BUTTON_RELEASE_PIN = 14
BUTTON_SELECT_PIN = 25

# SPI CS pins (SPI0)
RFID_CS_PIN = 7
ADC_CS_PIN = 8

# RFID Reset pin moved to GPIO 4
RFID_RST_PIN = 4

# ADC Settings
VREF = 3.3
ADC_CHANNEL = 1

# Detection Timing Parameters
SAMPLE_PERIOD_S = 0.2
AVG_DELAY_S = 0.2

# Stability Parameters
STABLE_TIME_S = 2.0
RETRY_DELAY_S = 0.3
READ_DELAY_S = 1.0

# Open time during release
RELEASE_OPEN_TIME_S = 3.0

# =========================
# Weight Calibration (Corrected)
# =========================
ZERO_VOLTAGE = 0.9045       # This value will be re-measured and overwritten at startup
FULL_WEIGHT_VOLTAGE = 0.5   # Absolute voltage reference for 830g
FULL_WEIGHT_GRAMS = 830.0

# --- NEW: Weight Calibration Factor ---
# Corrected based on the ratio of true weight (240g) to displayed weight (300g)
CALIBRATION_FACTOR = 240.0 / 300.0 

# Multiply calibration factor into the multiplier
GRAMS_PER_VOLT = (FULL_WEIGHT_GRAMS / (ZERO_VOLTAGE - FULL_WEIGHT_VOLTAGE)) * CALIBRATION_FACTOR

# Weight Logic Thresholds
CHANGE_THRESHOLD_G = 50.0    # Item detected if net change exceeds 50g
STABLE_RANGE_G = 50.0        # Reading is stable if fluctuation is within 50g
HEAVY_THRESHOLD_G = 500.0    # If basket total > 500g, rotate stepper twice
ERROR_THRESHOLD_G = 1000.0   # If basket total >= 1000g, trigger overload error

# =========================
# Global Runtime State
# =========================
next_release_basket = 1

# Track estimated total weight for each basket
basket_estimated_weight = {basket_no: 0.0 for basket_no in range(1, TOTAL_BASKETS + 1)}

# Global button flags for the polling thread
button_pressed_flags = {
    'select': False,
    'release': False
}

# =========================
# Button Polling Thread (Debounce Logic Added)
# =========================
def button_poller_thread():
    prev_select = GPIO.LOW
    prev_release = GPIO.LOW
    
    # Record timestamp of last press
    last_select_time = 0
    last_release_time = 0
    debounce_delay = 0.2  # 200ms debounce delay

    while True:
        curr_select = GPIO.input(BUTTON_SELECT_PIN)
        curr_release = GPIO.input(BUTTON_RELEASE_PIN)
        current_time = time.time()

        # Detect rising edge: LOW -> HIGH, and elapsed time exceeds debounce delay
        if curr_select == GPIO.HIGH and prev_select == GPIO.LOW:
            if current_time - last_select_time > debounce_delay:
                button_pressed_flags['select'] = True
                last_select_time = current_time

        if curr_release == GPIO.HIGH and prev_release == GPIO.LOW:
            if current_time - last_release_time > debounce_delay:
                button_pressed_flags['release'] = True
                last_release_time = current_time

        prev_select = curr_select
        prev_release = curr_release
        time.sleep(0.02)

# =========================
# Weight Conversion Functions
# =========================
def voltage_to_grams(voltage):
    """Convert voltage to absolute weight (based on global zero point set at startup)"""
    grams = (ZERO_VOLTAGE - voltage) * GRAMS_PER_VOLT
    if grams < 0:
        grams = 0
    return grams


def voltage_to_net_grams(current_voltage, baseline_voltage):
    """
    Convert voltage to relative net weight (to detect weight of newly added item)
    Baseline voltage is treated as 0g (temporary tare)
    """
    grams = (baseline_voltage - current_voltage) * GRAMS_PER_VOLT
    if grams < 0:
        grams = 0
    return grams


# =========================
# GPIO Setup
# =========================
GPIO.setup(RFID_CS_PIN, GPIO.OUT)
GPIO.setup(ADC_CS_PIN, GPIO.OUT)

GPIO.output(RFID_CS_PIN, GPIO.HIGH)
GPIO.output(ADC_CS_PIN, GPIO.HIGH)

GPIO.setup(BUTTON_RELEASE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(BUTTON_SELECT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(LED_RFID_PIN, GPIO.OUT)
GPIO.setup(LED_STEPPER2_PIN, GPIO.OUT)
GPIO.setup(LED_SERVO_PIN, GPIO.OUT)

GPIO.output(LED_RFID_PIN, GPIO.LOW)
GPIO.output(LED_STEPPER2_PIN, GPIO.LOW)
GPIO.output(LED_SERVO_PIN, GPIO.LOW)

for pin in STEPPER_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

for pin in STEPPER2_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

servo_pwms = {}

for basket_id, pin in SERVO_PINS.items():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, SERVO_FREQ_HZ)
    pwm.start(0)
    servo_pwms[basket_id] = pwm


# =========================
# SPI Setup
# =========================
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000


# =========================
# LED Utility Functions
# =========================
def all_leds_on():
    GPIO.output(LED_RFID_PIN, GPIO.HIGH)
    GPIO.output(LED_STEPPER2_PIN, GPIO.HIGH)
    GPIO.output(LED_SERVO_PIN, GPIO.HIGH)

def all_leds_off():
    GPIO.output(LED_RFID_PIN, GPIO.LOW)
    GPIO.output(LED_STEPPER2_PIN, GPIO.LOW)
    GPIO.output(LED_SERVO_PIN, GPIO.LOW)


# =========================
# Database Utility Functions
# =========================
def get_clothing_info(clothing_id):
    sql = """
    SELECT id, clothing_type, color, color_type, material,
           washing_method, max_wash_temperature, drying_method,
           suggested_temp, user_id
    FROM clothes
    WHERE id = %s
    """
    cursor.execute(sql, (clothing_id,))
    return cursor.fetchone()

def ensure_basket_state_rows():
    for basket_number in range(1, TOTAL_BASKETS + 1):
        sql = """
        INSERT INTO basket_state (
            basket_number, washing_method, suggested_temp, color_type, status
        )
        SELECT %s, NULL, NULL, NULL, 'EMPTY'
        WHERE NOT EXISTS (
            SELECT 1 FROM basket_state WHERE basket_number = %s
        )
        """
        cursor.execute(sql, (basket_number, basket_number))

def find_active_basket_for_group(washing_method, suggested_temp, color_type):
    sql = """
    SELECT basket_number
    FROM basket_state
    WHERE status = 'ACTIVE'
      AND washing_method = %s
      AND suggested_temp = %s
      AND color_type = %s
    LIMIT 1
    """
    cursor.execute(sql, (washing_method, suggested_temp, color_type))
    row = cursor.fetchone()
    if row:
        return row[0]
    return None

def find_free_active_basket():
    sql = """
    SELECT basket_number
    FROM basket_state
    WHERE status = 'EMPTY'
      AND basket_number BETWEEN 1 AND %s
    ORDER BY basket_number
    LIMIT 1
    """
    cursor.execute(sql, (TOTAL_BASKETS,))
    row = cursor.fetchone()
    if row:
        return row[0]
    return None

def activate_basket_group(basket_number, washing_method, suggested_temp, color_type):
    sql = """
    UPDATE basket_state
    SET washing_method = %s,
        suggested_temp = %s,
        color_type = %s,
        status = 'ACTIVE'
    WHERE basket_number = %s
    """
    cursor.execute(sql, (washing_method, suggested_temp, color_type, basket_number))

def insert_basket_item(clothing_id, basket_number, item_status):
    sql = """
    INSERT INTO basket_items (clothing_id, basket_number, item_status)
    VALUES (%s, %s, %s)
    """
    cursor.execute(sql, (clothing_id, basket_number, item_status))

def insert_waiting_item(clothing_id):
    sql = """
    INSERT INTO basket_items (clothing_id, basket_number, item_status)
    VALUES (%s, NULL, 'WAITING')
    """
    cursor.execute(sql, (clothing_id,))

def clothing_already_exists_in_runtime(clothing_id):
    sql = """
    SELECT id
    FROM basket_items
    WHERE clothing_id = %s
      AND item_status IN ('ACTIVE', 'WAITING')
    LIMIT 1
    """
    cursor.execute(sql, (clothing_id,))
    row = cursor.fetchone()
    return row is not None

def assign_basket_by_group(washing_method, suggested_temp, color_type, clothing_id):
    if clothing_already_exists_in_runtime(clothing_id):
        sql = """
        SELECT basket_number, item_status
        FROM basket_items
        WHERE clothing_id = %s
          AND item_status IN ('ACTIVE', 'WAITING')
        ORDER BY id DESC
        LIMIT 1
        """
        cursor.execute(sql, (clothing_id,))
        row = cursor.fetchone()
        if row:
            return row[0], row[1]

    active_basket = find_active_basket_for_group(
        washing_method, suggested_temp, color_type
    )
    if active_basket is not None:
        insert_basket_item(clothing_id, active_basket, 'ACTIVE')
        return active_basket, 'ACTIVE'

    free_basket = find_free_active_basket()
    if free_basket is not None:
        activate_basket_group(free_basket, washing_method, suggested_temp, color_type)
        insert_basket_item(clothing_id, free_basket, 'ACTIVE')
        return free_basket, 'ACTIVE'

    insert_waiting_item(clothing_id)
    return None, 'WAITING'

def clear_active_items_in_basket(basket_number):
    sql = """
    DELETE FROM basket_items
    WHERE basket_number = %s
      AND item_status = 'ACTIVE'
    """
    cursor.execute(sql, (basket_number,))

def clear_basket_state_only(basket_number):
    sql = """
    UPDATE basket_state
    SET washing_method = NULL,
        suggested_temp = NULL,
        color_type = NULL,
        status = 'EMPTY'
    WHERE basket_number = %s
    """
    cursor.execute(sql, (basket_number,))

def record_washing_items(basket_number):
    """Record items ready for washing in the current basket to the washing table"""
    sql = """
    INSERT INTO washing (clothes_id, washing_method, suggest_temp, color_type)
    SELECT bi.clothing_id, c.washing_method, c.suggested_temp, c.color_type
    FROM basket_items bi
    JOIN clothes c ON bi.clothing_id = c.id
    WHERE bi.basket_number = %s AND bi.item_status = 'ACTIVE'
    """
    try:
        cursor.execute(sql, (basket_number,))
        print(f"[DB] Successfully synced washing records for basket {basket_number} to washing table.")
    except Exception as e:
        print(f"[DB ERROR] Failed to write to washing table: {e}")

def check_washing_table_has_data():
    """Check if there is data in the washing table"""
    sql = "SELECT COUNT(*) FROM washing"
    try:
        cursor.execute(sql)
        result = cursor.fetchone()
        return result[0] > 0
    except Exception as e:
        print(f"[DB ERROR] Failed to check washing table: {e}")
        return False

def clear_washing_table():
    """Clear the washing table"""
    sql = "DELETE FROM washing"
    try:
        cursor.execute(sql)
        print("[DB] Washing complete. Washing table cleared.")
    except Exception as e:
        print(f"[DB ERROR] Failed to clear washing table: {e}")

def get_oldest_logical_basket():
    max_physical = max(SERVO_PINS.keys()) if SERVO_PINS else 3
    sql = """
    SELECT basket_number, washing_method, suggested_temp, color_type
    FROM basket_state
    WHERE status = 'ACTIVE' AND basket_number > %s
    ORDER BY basket_number
    LIMIT 1
    """
    cursor.execute(sql, (max_physical,))
    return cursor.fetchone()

def get_oldest_waiting_group():
    sql = """
    SELECT c.washing_method, c.suggested_temp, c.color_type
    FROM basket_items bi
    JOIN clothes c ON bi.clothing_id = c.id
    WHERE bi.item_status = 'WAITING'
    ORDER BY bi.added_at
    LIMIT 1
    """
    cursor.execute(sql)
    return cursor.fetchone()

def get_basket_current_group(basket_number):
    sql = """
    SELECT washing_method, suggested_temp
    FROM basket_state
    WHERE basket_number = %s AND status = 'ACTIVE'
    """
    cursor.execute(sql, (basket_number,))
    return cursor.fetchone()

def promote_waiting_group_to_basket(basket_number, washing_method, suggested_temp, color_type):
    activate_basket_group(
        basket_number,
        washing_method,
        suggested_temp,
        color_type
    )
    sql = """
    UPDATE basket_items bi
    JOIN clothes c ON bi.clothing_id = c.id
    SET bi.basket_number = %s,
        bi.item_status = 'ACTIVE'
    WHERE bi.item_status = 'WAITING'
      AND c.washing_method = %s
      AND c.suggested_temp = %s
      AND c.color_type = %s
    """
    cursor.execute(sql, (
        basket_number,
        washing_method,
        suggested_temp,
        color_type
    ))

def release_basket_and_promote(basket_number):
    clear_active_items_in_basket(basket_number)
    clear_basket_state_only(basket_number)

    logical_basket = get_oldest_logical_basket()
    if logical_basket:
        lb_num, w_m, s_t, c_t = logical_basket
        sql_move = """
        UPDATE basket_items
        SET basket_number = %s
        WHERE basket_number = %s AND item_status = 'ACTIVE'
        """
        cursor.execute(sql_move, (basket_number, lb_num))

        activate_basket_group(basket_number, w_m, s_t, c_t)
        clear_basket_state_only(lb_num)

        return (basket_number, w_m, s_t, c_t, f"(Moved from logical basket {lb_num})")

    waiting_group = get_oldest_waiting_group()
    if waiting_group is None:
        return None

    waiting_washing_method, waiting_temp, waiting_color_type = waiting_group

    promote_waiting_group_to_basket(
        basket_number,
        waiting_washing_method,
        waiting_temp,
        waiting_color_type
    )

    return (
        basket_number,
        waiting_washing_method,
        waiting_temp,
        waiting_color_type,
        "(Promoted from WAITING queue)"
    )

def clear_all_basket_runtime_state():
    sql1 = "DELETE FROM basket_items"
    sql2 = """
    UPDATE basket_state
    SET washing_method = NULL,
        suggested_temp = NULL,
        color_type = NULL,
        status = 'EMPTY'
    """
    cursor.execute(sql1)
    cursor.execute(sql2)

# =========================
# CS Settings
# =========================
def deselect_all():
    GPIO.output(RFID_CS_PIN, GPIO.HIGH)
    GPIO.output(ADC_CS_PIN, GPIO.HIGH)

def select_adc():
    GPIO.output(RFID_CS_PIN, GPIO.HIGH)
    GPIO.output(ADC_CS_PIN, GPIO.LOW)

def select_rfid():
    GPIO.output(ADC_CS_PIN, GPIO.HIGH)
    GPIO.output(RFID_CS_PIN, GPIO.LOW)


# =========================
# Servo Control
# =========================
def servo_set_angle(basket_id, angle_deg):
    if basket_id not in servo_pwms:
        return

    if angle_deg < 0:
        angle_deg = 0
    if angle_deg > 180:
        angle_deg = 180

    GPIO.output(LED_SERVO_PIN, GPIO.HIGH)

    duty = 2.5 + (angle_deg / 18.0)
    servo_pwms[basket_id].ChangeDutyCycle(duty)
    time.sleep(0.35)
    servo_pwms[basket_id].ChangeDutyCycle(0)

    GPIO.output(LED_SERVO_PIN, GPIO.LOW)

def open_wait_close_basket(basket_number, hold_time_s=3.0):
    if basket_number not in SERVO_PINS:
        return
    servo_set_angle(basket_number, SERVO_OPEN_ANGLES[basket_number])
    time.sleep(hold_time_s)
    servo_set_angle(basket_number, SERVO_CLOSE_ANGLES[basket_number])


# =========================
# MCP3008 ADC Load Cell Reading
# =========================
def read_adc_raw(channel=0):
    if channel < 0 or channel > 7:
        raise ValueError("MCP3008 channel must be 0..7")

    select_adc()
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    deselect_all()

    return ((r[1] & 0x03) << 8) | r[2]

def adc_to_voltage(adc, vref=3.3):
    return adc * vref / 1023.0

def average_voltage(samples=10):
    total = 0.0
    for _ in range(samples):
        adc = read_adc_raw(ADC_CHANNEL)
        total += adc_to_voltage(adc, VREF)
        time.sleep(AVG_DELAY_S)
    return total / samples

def wait_for_weight_change(v0):
    while True:
        v = adc_to_voltage(read_adc_raw(ADC_CHANNEL), VREF)
        net_g = voltage_to_net_grams(v, v0)
        abs_g = voltage_to_grams(v)

        # Turn on all 3 LEDs if total weight exceeds 1kg
        if abs_g >= ERROR_THRESHOLD_G:
            all_leds_on()
        else:
            all_leds_off()

        print(f"\r[MONITOR] Current net weight change: {net_g:.1f} g |Δg|={net_g:.1f} g", end="", flush=True)

        if net_g >= CHANGE_THRESHOLD_G:
            print(f"\n[DETECT] Sensor detected weight change: {net_g:.1f} g")
            return True

        time.sleep(SAMPLE_PERIOD_S)

def wait_until_stable(v0):
    window_n = max(1, int(STABLE_TIME_S / SAMPLE_PERIOD_S))
    buf_g = []

    while True:
        v = adc_to_voltage(read_adc_raw(ADC_CHANNEL), VREF)
        net_g = voltage_to_net_grams(v, v0)
        abs_g = voltage_to_grams(v)

        # Turn on all 3 LEDs if total weight exceeds 1kg
        if abs_g >= ERROR_THRESHOLD_G:
            all_leds_on()
        else:
            all_leds_off()

        buf_g.append(net_g)

        if len(buf_g) > window_n:
            buf_g.pop(0)

        if len(buf_g) == window_n:
            g_range = max(buf_g) - min(buf_g)

            print(f"\r[STABILIZING] Net weight... {net_g:.1f} g (Range={g_range:.1f} g)", end="", flush=True)

            if g_range <= STABLE_RANGE_G:
                print()
                return net_g, g_range

        time.sleep(SAMPLE_PERIOD_S)


# =========================
# Stepper Motor Control
# =========================
def rotate_stepper_75_degrees(turns=1):
    print(f"[ACTION] Starting Stepper 1 (75 degrees) x {turns}...")
    update_lcd("Starting Wash...", f"Motor1 x{turns}")

    for _ in range(turns):
        for _ in range(107):  # Changed from 85 to 107 for ~75 degrees
            for halfstep in range(8):
                for pin in range(4):
                    GPIO.output(STEPPER_PINS[pin], STEPPER_SEQ[halfstep][pin])
                time.sleep(0.002)
        time.sleep(0.2)

    for pin in STEPPER_PINS:
        GPIO.output(pin, 0)

    print("[ACTION] Stepper 1 rotation complete.")

def rotate_stepper2_for_15_seconds():
    print("[ACTION] Starting Stepper 2 (for 15 seconds)...")
    update_lcd("Washing...", "Time left: 15s")

    GPIO.output(LED_STEPPER2_PIN, GPIO.HIGH)

    end_time = time.time() + 15.0  # 15 seconds total
    last_lcd_update = time.time()

    while time.time() < end_time:
        current_time = time.time()
        
        # Update LCD every 1 second to show progress without lagging the motor
        if current_time - last_lcd_update >= 1.0:
            time_left = int(end_time - current_time)
            update_lcd("Washing...", f"Time left: {time_left}s")
            last_lcd_update = current_time

        for halfstep in range(8):
            for pin in range(4):
                GPIO.output(STEPPER2_PINS[pin], STEPPER_SEQ[halfstep][pin])
            time.sleep(0.002)

    for pin in STEPPER2_PINS:
        GPIO.output(pin, 0)

    GPIO.output(LED_STEPPER2_PIN, GPIO.LOW)
    print("[ACTION] Stepper 2 rotation complete.")
    
    # --- NEW: Clear washing table after washing finishes ---
    clear_washing_table()
    update_lcd("Wash Complete", "Table Cleared")
    time.sleep(1)


# =========================
# Button Action Processing
# =========================
def process_release_action():
    global next_release_basket
    global basket_estimated_weight

    basket_to_release = next_release_basket

    print(f"\n[BUTTON] Execute/Confirm button (GPIO 14) pressed. Processing basket {basket_to_release}")

    basket_weight = basket_estimated_weight.get(basket_to_release, 0.0)
    print(f"[INFO] Estimated total weight of basket {basket_to_release} = {basket_weight:.1f} g")

    if basket_weight >= ERROR_THRESHOLD_G:
        print(f"[ERROR] Basket overweight: {basket_weight:.1f} g (>= 1kg threshold)")
        print("[ERROR] All indicator LEDs turned on. Release operation aborted.")
        update_lcd("ERROR OVERLOAD", f"{basket_weight:.0f}g >=1kg")
        all_leds_on()
        time.sleep(5)
        all_leds_off()
        return

    stepper_turns = 2 if basket_weight > HEAVY_THRESHOLD_G else 1

    rotate_stepper_75_degrees(turns=stepper_turns)

    print("[ACTION] Waiting 2 seconds...")
    time.sleep(2)

    # --- MODIFIED: Record items to washing table before starting wash (Stepper 2) ---
    record_washing_items(basket_to_release)

    rotate_stepper2_for_15_seconds()

    print(f"[ACTION] Releasing clothes in basket {basket_to_release}...")

    current_group = get_basket_current_group(basket_to_release)

    if basket_to_release in SERVO_PINS:
        print(f"[ACTION] Opening basket {basket_to_release} servo for {RELEASE_OPEN_TIME_S:.1f} seconds...")
        open_wait_close_basket(basket_to_release, RELEASE_OPEN_TIME_S)
        print(f"[ACTION] Basket {basket_to_release} re-closed.")
    else:
        print(f"[INFO] Basket {basket_to_release} is a logical basket, no corresponding physical servo.")

    result = release_basket_and_promote(basket_to_release)
    basket_estimated_weight[basket_to_release] = 0.0

    if current_group:
        w_m, s_t = current_group
        print(f"[LCD] Basket {basket_to_release} washing started: Mode {w_m}, Temp {s_t}C")
        update_lcd(f"Wash B{basket_to_release}: {s_t}C", f"Mode:{w_m}")
    else:
        print(f"[LCD] Basket {basket_to_release} is empty, no wash needed.")
        update_lcd(f"B{basket_to_release} Empty", "No Wash Needed")

    if result is None:
        print(f"[DB] Database cleared for basket {basket_to_release}.")
        print("[QUEUE] No waiting clothes or occupied logical baskets.")
    else:
        promoted_basket, washing_method, suggested_temp, color_type, source_msg = result
        print(f"[DB] Old data cleared for promoted basket {promoted_basket}.")
        print(
            f"[QUEUE] Promoted queued clothes "
            f"({washing_method}, {suggested_temp}, {color_type}) "
            f"to physical basket {promoted_basket} {source_msg}."
        )

    print(f"[INFO] Process finished for basket {basket_to_release}.")


# =========================
# RFID Setup & Core Logic
# =========================
def build_rfid(bus=0, device=1, rst_pin=4):
    obj = SimpleMFRC522()
    obj.READER = mfrc522.MFRC522(bus=bus, device=device, pin_rst=rst_pin)
    return obj

rfid = build_rfid(bus=0, device=1, rst_pin=RFID_RST_PIN)

# -------------------------
# Mode 1: Washing/Sorting Mode
# -------------------------
def wait_for_rfid_and_poll_buttons():
    global rfid
    global next_release_basket

    # Turn off all LEDs to ensure a clean state before next scan
    all_leds_off()

    button_pressed_flags['select'] = False
    button_pressed_flags['release'] = False

    try:
        rfid.READER.spi.close()
    except Exception:
        pass

    GPIO.setup(RFID_RST_PIN, GPIO.OUT)
    GPIO.output(RFID_RST_PIN, GPIO.LOW)
    time.sleep(0.05)
    GPIO.output(RFID_RST_PIN, GPIO.HIGH)
    time.sleep(0.05)

    rfid = build_rfid(bus=0, device=1, rst_pin=RFID_RST_PIN)

    GPIO.setup(RFID_CS_PIN, GPIO.OUT)
    GPIO.setup(ADC_CS_PIN, GPIO.OUT)
    GPIO.output(RFID_CS_PIN, GPIO.HIGH)
    GPIO.output(ADC_CS_PIN, GPIO.HIGH)

    print("\n==============================")
    print("Entered RFID scan mode")
    print(f"Currently selected target basket: {next_release_basket}")
    print("Please tap the clothing tag on the reader...")
    print("[INFO] Waiting for RFID... (You can also press buttons now)")

    update_lcd("System Ready", f"Sel: Basket {next_release_basket}")

    time.sleep(0.1)

    # --- NEW: Record time of last washing table check ---
    last_washing_check_time = time.time()

    while True:
        # --- NEW: Check washing table every 5 seconds ---
        current_time = time.time()
        if current_time - last_washing_check_time >= 5.0:
            last_washing_check_time = current_time
            if check_washing_table_has_data():
                print("\n[DB EVENT] New external data detected in washing table, starting auto-wash...")
                update_lcd("Auto Washing", "Data detected")
                time.sleep(1)
                
                # Start washing (rotates motor and auto-clears washing table at the end)
                rotate_stepper2_for_15_seconds()
                
                print("[INFO] Auto-wash complete, returning to scan mode...")
                # Restore standby display after washing
                update_lcd("System Ready", f"Sel: Basket {next_release_basket}")
                last_washing_check_time = time.time() # Realign timer

        if button_pressed_flags['select']:
            button_pressed_flags['select'] = False

            max_physical = max(SERVO_PINS.keys()) if SERVO_PINS else 3
            next_release_basket += 1
            if next_release_basket > max_physical:
                next_release_basket = 1

            print(f"[BUTTON] Select button pressed. Target switched to basket {next_release_basket}")
            update_lcd("Select Target:", f"Basket {next_release_basket}")

            time.sleep(0.5)
            update_lcd("System Ready", f"Sel: Basket {next_release_basket}")

        if button_pressed_flags['release']:
            button_pressed_flags['release'] = False

            process_release_action()
            print("\n[INFO] Returning to RFID scan mode...")

            time.sleep(2)
            update_lcd("System Ready", f"Sel: Basket {next_release_basket}")

        try:
            select_rfid()
            tag_id, text = rfid.read_no_block()
            if tag_id is not None:
                return tag_id, text
        except Exception:
            pass
        finally:
            deselect_all()

        time.sleep(0.02)


def running_washing_mode():
    global basket_estimated_weight

    tag_id, text = wait_for_rfid_and_poll_buttons()

    GPIO.output(LED_RFID_PIN, GPIO.HIGH)
    time.sleep(0.3)
    GPIO.output(LED_RFID_PIN, GPIO.LOW)

    clothing_id_raw = text.strip()

    print(f"[RFID] Read Tag ID = {tag_id}")
    print(f"[RFID] Stored Clothing ID on tag = '{clothing_id_raw}'")

    try:
        clothing_id = int(clothing_id_raw)
    except ValueError:
        print("[ERROR] Invalid tag data. Stored data must be an integer ID.")
        return

    info = get_clothing_info(clothing_id)
    if not info:
        print("[ERROR] Clothing ID not found in database.")
        return

    (
        row_id, clothing_type, color, color_type, material,
        washing_method, temp, drying_method, suggested_temp, user_id
    ) = info

    print("=== Clothing Details ===")
    print(f"ID                 : {row_id}")
    print(f"Type               : {clothing_type}")
    print(f"Color              : {color}")
    print(f"Color Type         : {color_type}")
    print(f"Material           : {material}")
    print(f"Washing Method     : {washing_method}")
    print(f"Max Temp           : {temp} °C")
    print(f"Drying Method      : {drying_method}")
    print(f"Suggested Temp     : {suggested_temp} °C")
    print(f"User ID            : {user_id}")

    # --- NEW: Check for Professional Dry Clean Only ---
    if washing_method and "Professional Dry Clean Only" in washing_method:
        print("[WARNING] Item requires Professional Dry Clean Only!")
        update_lcd("Dry Clean Only", "Special Care!")
        time.sleep(3)
        print("[INFO] Sequence aborted for dry clean item.")
        print("==============================\n")
        time.sleep(READ_DELAY_S)
        return
    # --------------------------------------------------

    target_basket_id, state = assign_basket_by_group(
        washing_method,
        suggested_temp,
        color_type,
        clothing_id
    )

    print(
        f"[LOGIC] Wash Group = ({washing_method}, {suggested_temp}, {color_type}) "
        f"-> Assigned to basket {target_basket_id}, Current state = {state}"
    )

    if state == 'WAITING':
        update_lcd("Baskets Full", "Added to Queue")
        print("[QUEUE] All 10 baskets are full.")
        print("[QUEUE] This clothing item was added to the virtual waiting queue.")
        print("[INFO] It will be automatically promoted and prompted once a basket is free.")
        print("==============================\n")
        time.sleep(READ_DELAY_S)
        return

    print(f"\n[ACTION] Please place the clothing item into basket {target_basket_id}.")

    if target_basket_id in SERVO_PINS:
        update_lcd(f"Opening B{target_basket_id}...", "Reading scale")
        print(f"[ACTION] Opening lid of basket {target_basket_id}...")
        servo_set_angle(target_basket_id, SERVO_OPEN_ANGLES[target_basket_id])

        # --- MODIFIED: Read initial resting voltage to monitor weight change, without overwriting global 0g baseline ---
        print("[INFO] Reading current resting sensor voltage, waiting for item to be added...")
        resting_v = average_voltage(samples=10) 
        
        # Print current absolute total weight on the scale (based on ZERO_VOLTAGE from startup)
        absolute_current_g = voltage_to_grams(resting_v)
        print(f"[BASE] Current absolute total weight before adding = {absolute_current_g:.1f} g")

        update_lcd(f"B{target_basket_id} Open", "Put item inside")
        print("[INFO] Please add the item, waiting for weight change...")
        
        changed = wait_for_weight_change(resting_v)
        if not changed:
            return

        update_lcd("Detected!", "Stabilizing...")
        print(f"[INFO] Item detected, waiting for weight to stabilize (approx {STABLE_TIME_S:.1f}s)...")
        
        g_added_net, g_range = wait_until_stable(resting_v)
        
        # Get total weight voltage after item is added
        stable_v = average_voltage(samples=10)
        absolute_total_g = voltage_to_grams(stable_v)

        # Update LED warning based on final absolute weight
        if absolute_total_g >= ERROR_THRESHOLD_G:
            all_leds_on()
        else:
            all_leds_off()

        print(f"[STABLE] [Net Weight] of this added item = {g_added_net:.1f} g")
        print(f"[STABLE] Weight fluctuation range = {g_range:.1f} g")
        print(f"[STABLE] [Absolute Total Weight] after adding = {absolute_total_g:.1f} g")
        update_lcd(f"Added:{g_added_net:.1f}g", "Closing...")

        basket_estimated_weight[target_basket_id] += g_added_net
        print(f"[INFO] DB Record: Estimated total weight of basket {target_basket_id} is now = {basket_estimated_weight[target_basket_id]:.1f} g")

        print(f"[ACTION] Closing lid of basket {target_basket_id}...")
        servo_set_angle(target_basket_id, SERVO_CLOSE_ANGLES[target_basket_id])

        time.sleep(1)
        update_lcd("Item Added!", "Waiting RFID...")
    else:
        update_lcd(f"Put in B{target_basket_id}", "(Manual Basket)")
        print(
            f"[INFO] Basket {target_basket_id} is a virtual basket without physical servo control."
        )
        print(f"[INFO] Please manually place the item into designated area for basket {target_basket_id}.")
        time.sleep(2)
        update_lcd("Item Added!", "Waiting RFID...")

    print("[INFO] Single item addition process complete.")
    print("==============================\n")
    time.sleep(READ_DELAY_S)


# -------------------------
# Mode 2: RFID Write Mode
# -------------------------
def write_rfid_blocking(text):
    """Blocking RFID write function"""
    while True:
        try:
            select_rfid()
            rfid.write(text)
            return
        except Exception:
            time.sleep(RETRY_DELAY_S)
        finally:
            deselect_all()

def rfid_write_mode_loop():
    global rfid

    update_lcd("RFID Write Mode", "Check Terminal")
    print("\n" + "=" * 30)
    print("RFID Tag Write Mode Started")
    print("Enter 'q' to quit and return to main menu")
    print("=" * 30)

    while True:
        text_to_write = input("\nPlease enter the Data ID to write (or 'q' to quit): ").strip()

        if text_to_write.lower() == "q":
            print("\nReturning to main menu...")
            break

        update_lcd("Tap tag to write", f"Data: {text_to_write[:16]}")
        print(f"Please hold a blank or overwritable tag near the reader to write: {text_to_write}")

        try:
            rfid.READER.spi.close()
        except Exception:
            pass

        GPIO.setup(RFID_RST_PIN, GPIO.OUT)
        GPIO.output(RFID_RST_PIN, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(RFID_RST_PIN, GPIO.HIGH)
        time.sleep(0.05)
        rfid = build_rfid(bus=0, device=1, rst_pin=RFID_RST_PIN)

        write_rfid_blocking(text_to_write)

        print(f"Write successful! Content: {text_to_write}")
        update_lcd("Write Success!", f"Data:{text_to_write[:11]}")

        GPIO.output(LED_RFID_PIN, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(LED_RFID_PIN, GPIO.LOW)

        time.sleep(1)
        update_lcd("RFID Write Mode", "Check Terminal")


# =========================
# Program Entry (Boot Menu)
# =========================
try:
    ensure_basket_state_rows()

    for basket_id in SERVO_PINS.keys():
        servo_set_angle(basket_id, SERVO_CLOSE_ANGLES[basket_id])

    print("[INFO] System Started.")
    
    # --- NEW: Global tare executed only once at startup ---
    print("[INFO] Initializing scale and performing first tare, please ensure the scale is empty...")
    update_lcd("Zeroing Scale...", "Please wait")
    # Overwrite top ZERO_VOLTAGE, setting global 0g zero point
    ZERO_VOLTAGE = average_voltage(samples=20)  
    print(f"[BASE] Initial global tare baseline voltage set to: {ZERO_VOLTAGE:.4f} V")
    update_lcd("Scale Ready", "Zero Completed")
    time.sleep(1)
    # ----------------------------------------

    print("[INFO] GPIO 25 = Select/Toggle Mode")
    print("[INFO] GPIO 14 = Confirm/Execute Action")
    print("[INFO] Starting button background polling thread...")

    poller = threading.Thread(target=button_poller_thread, daemon=True)
    poller.start()

    while True:
        modes = ["Washing Mode", "RFID Write"]
        current_mode_idx = 0

        button_pressed_flags['select'] = False
        button_pressed_flags['release'] = False

        update_lcd("Select Mode:", f"> {modes[current_mode_idx]}")
        print("\n" + "=" * 30)
        print("[BOOT MENU] Please select system mode:")
        print(" -> Press GPIO 25 to toggle mode")
        print(" -> Press GPIO 14 to confirm mode")
        print("=" * 30)

        selected_mode = None

        while True:
            if button_pressed_flags['select']:
                button_pressed_flags['select'] = False
                current_mode_idx = (current_mode_idx + 1) % len(modes)
                update_lcd("Select Mode:", f"> {modes[current_mode_idx]}")
                print(f"[MENU] Cursor switched to: {modes[current_mode_idx]}")
                time.sleep(0.2)

            if button_pressed_flags['release']:
                button_pressed_flags['release'] = False
                selected_mode = modes[current_mode_idx]
                print(f"\n[MENU] Confirmed mode: {selected_mode}")
                update_lcd("Starting...", f"{selected_mode}")
                time.sleep(1)
                break

            time.sleep(0.05)

        if selected_mode == "Washing Mode":
            print("\n[INFO] Entered Washing/Sorting Mode.")
            while True:
                running_washing_mode()

        elif selected_mode == "RFID Write":
            rfid_write_mode_loop()

except KeyboardInterrupt:
    print("\nProgram manually terminated by user.")

finally:
    update_lcd("System Stopped", "Goodbye!")
    deselect_all()

    GPIO.output(LED_RFID_PIN, GPIO.LOW)
    GPIO.output(LED_STEPPER2_PIN, GPIO.LOW)
    GPIO.output(LED_SERVO_PIN, GPIO.LOW)

    for pin in STEPPER_PINS:
        GPIO.output(pin, 0)
    for pin in STEPPER2_PINS:
        GPIO.output(pin, 0)

    for pwm in servo_pwms.values():
        pwm.stop()

    spi.close()
    cursor.close()
    db.close()
    GPIO.cleanup()