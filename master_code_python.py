import os
import sys
import time
import json
import csv
import threading
from datetime import datetime
from collections import deque
import paho.mqtt.client as mqtt
import joblib
import numpy as np
import cv2
from ultralytics import YOLO


print("="*70)
print("Initializing SCADA Edge-AI Supervisor...")
print("="*70)


# =======================================================================
# 1. LOAD AI MODELS
# =======================================================================
print("[AI] Loading YOLOv8 Model (best.pt)...")
try:
    yolo_model = YOLO('best.pt')
except Exception as e:
    print(f"[ERROR] Could not load best.pt: {e}")
    sys.exit(1)


print("[AI] Loading Yield Prophet Model (yield_prediction_model.pkl)...")
try:
    yield_model = joblib.load('yield_prediction_model.pkl')
except Exception as e:
    print(f"[ERROR] Could not load yield_prediction_model.pkl: {e}")
    sys.exit(1)


# =======================================================================
# 2. CONSTANTS, GAINS & SETUP
# =======================================================================
THINGSBOARD_HOST = '127.0.0.1'
ACCESS_TOKEN = 'thingsboard key id'
LOCAL_LOG_FILE = "final_log.csv"
PLANTING_DATE_FILE = 'planting_date.txt'


# PI Controller Gains & Settings
Kp_ec_up = 0.4500; Ki_ec_up = 0.0176
Kp_ph_up = 0.8737; Ki_ph_up = 0.0069
Kp_ph_dn = 0.5825; Ki_ph_dn = 0.0046
Ts = 1


INTEGRAL_EC_LIMIT = 20.0
INTEGRAL_PH_LIMIT = 20.0
# EC and pH output thresholds are separated so each controller can be
# tuned independently without affecting the other.
OUTPUT_THRESHOLD = 0.3       # EC controller pump fire threshold — unchanged
OUTPUT_THRESHOLD_PH = 0.3    # pH controller pump fire threshold — independently tunable
MIN_ON_CYCLES = 3


# Default Control Setpoints
nominal_ec_sp = 1.2
setpoint_ph = 7.0            # Updated setpoint to pH 7.00
vision_offset = 0.0
active_setpoint_ec = nominal_ec_sp + vision_offset


# pH Dead-band: pump will not fire if pH is within this range of setpoint.
# With setpoint_ph = 7.0 and PH_DEAD_BAND = 1.0, the no-fire zone is 6.0–8.0.
# Adjust PH_DEAD_BAND to widen or narrow the tolerance window.
PH_DEAD_BAND = 1.0


# Dynamic Dosing Ratios & State Variables
integral_ec = 0.0; integral_ph = 0.0
ec_on_count = 0
# [FIX — pH Sign Convention Fix B] Replaced the single signed ph_on_count
# integer (positive = UP active, negative = DOWN active) with two independent
# counters. Each counter only tracks its own pump direction, eliminating the
# stranded-counter bug that occurred when the pH error changed sign mid-sequence.
ph_up_count = 0   # counts down from MIN_ON_CYCLES while pH UP pump is mid-sequence
ph_dn_count = 0   # counts down from MIN_ON_CYCLES while pH DOWN pump is mid-sequence
ratio_counter = 0


current_ratio_a = 1  
current_ratio_b = 1  


# Rolling Windows (10 seconds)
ec_window = deque(maxlen=10)
ph_window = deque(maxlen=10)


# Data Buffers
daily_buffer = {'temp': [], 'hum': [], 'tds': [], 'ph': [], 'weight': []}
csv_batch_buffer = []


# AI Status States
current_yolo_status = "healthy"
yield_days_left = 0
vision_thread_active = False


# Pump states (For CSV Logging)
pump_states = {'A': 0, 'B': 0, 'U': 0, 'D': 0, 'W': 0}


# Timers & Lockouts
EC_COOLDOWN = 3600
PH_COOLDOWN = 3600
last_ec_action_time = 0.0
last_ph_action_time = 0.0


last_csv_flush_time = time.time()
last_tb_publish_time = time.time()
last_yolo_time = time.time()
last_yield_time = time.time()


# [FIX — Vision Offset Timeout] Track last SUCCESSFUL vision update time.
# If no successful inference occurs within 3 hours, offsets are reset to
# safe defaults to prevent stale deficiency overrides from persisting.
VISION_STALENESS_LIMIT = 10800  # 3 hours in seconds
last_vision_success_time = time.time()


# [FIX — Vision Confidence Threshold] Minimum confidence score (0.0–1.0)
# required before a detection is acted upon. Detections below this
# threshold are discarded and the system defaults to the healthy state.
YOLO_CONFIDENCE_THRESHOLD = 0.50


# =======================================================================
# 3. HELPER FUNCTIONS & ACTUATION MECHANICS
# =======================================================================
def load_planting_date():
    if os.path.exists(PLANTING_DATE_FILE):
        with open(PLANTING_DATE_FILE, 'r') as f:
            date_str = f.read().strip()
        return datetime.strptime(date_str, '%Y-%m-%d')
    else:
        now = datetime.now()
        with open(PLANTING_DATE_FILE, 'w') as f:
            f.write(now.strftime('%Y-%m-%d'))
        return now


PLANTING_DATE = load_planting_date()


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def send_mqtt_cmd(pump_id, state):
    """Sends command to ESP32 and updates local dictionary for the CSV logger."""
    local_client.publish("esp32/commands", f"{pump_id}{int(state)}")
    pump_states[pump_id] = int(state)


def fire_nutrient_pumps():
    """Executes precision alternating pump fires based on AI ratios."""
    global ratio_counter
    ratio_counter += 1
    
    if ratio_counter % current_ratio_a == 0:
        send_mqtt_cmd('A', 1)
    else:
        send_mqtt_cmd('A', 0)
        
    if ratio_counter % (current_ratio_a * current_ratio_b) == 0:
        send_mqtt_cmd('B', 1)
        ratio_counter = 0  # Reset counter once the complete sequence finishes
    else:
        send_mqtt_cmd('B', 0)
    
    send_mqtt_cmd('W', 0)  # Safety interlock: shut off dilution if dosing nutrients


def stop_nutrient_pumps():
    send_mqtt_cmd('A', 0)
    send_mqtt_cmd('B', 0)


def flush_csv_batch():
    """Writes 60 seconds of 1Hz data to the SD card at once to prevent burnout."""
    global csv_batch_buffer
    if not csv_batch_buffer:
        return
        
    file_exists = os.path.isfile(LOCAL_LOG_FILE)
    try:
        with open(LOCAL_LOG_FILE, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=csv_batch_buffer[0].keys())
            if not file_exists:
                writer.writeheader()
            writer.writerows(csv_batch_buffer)
        csv_batch_buffer.clear()
        print("[DISK] Successfully flushed 60-second batch to CSV.")
    except Exception as e:
        print(f"[DISK ERROR] Failed to write CSV batch: {e}")


def run_vision_inference():
    """Runs YOLOv8 leaf analysis inside a distinct non-blocking execution block."""
    global vision_offset, current_yolo_status
    global current_ratio_a, current_ratio_b
    global vision_thread_active
    global last_vision_success_time  # [FIX — Vision Offset Timeout]


    try:
        print("[VISION] Starting 1-Hour Plant Health Scan...")
        cap = cv2.VideoCapture(0)


        # [FIX — Camera Fault Handling] Validate that the camera device opened
        # successfully before attempting to capture a frame. A missing or busy
        # camera raises a RuntimeError instead of silently passing bad data to
        # the YOLO model.
        if not cap.isOpened():
            raise RuntimeError("Camera device could not be opened (device index 0 unavailable).")


        ret, frame = cap.read()
        cap.release()


        # [FIX — Camera Fault Handling] Validate the captured frame itself.
        # ret=True with a blank/black frame (sum == 0) indicates a hardware
        # fault; treat it the same as a failed capture to avoid inference on
        # empty data.
        if not ret or frame is None or frame.sum() == 0:
            raise RuntimeError("Camera returned an empty or blank frame - possible hardware fault.")


        if ret:
            results = yolo_model(frame, verbose=False)
            found_deficiency = False
            
            if len(results[0].boxes) > 0:
                # [FIX — Multi-Detection: Use Highest-Confidence Box]
                # Original code always used boxes[0] (first detected box).
                # Now we select the box with the highest confidence score so
                # the most certain detection drives the control decision,
                # regardless of bounding-box ordering.
                best_box = max(results[0].boxes, key=lambda b: b.conf[0].item())


                # [FIX — Confidence Threshold Filter]
                # Only act on the detection if its confidence meets the minimum
                # threshold defined by YOLO_CONFIDENCE_THRESHOLD (default 0.50).
                # Detections below this value are discarded and the system
                # falls through to the healthy-state reset below.
                best_conf = best_box.conf[0].item()
                if best_conf >= YOLO_CONFIDENCE_THRESHOLD:
                    class_id = int(best_box.cls[0].item())
                    label = yolo_model.names[class_id]
                    print(f"[VISION] Best detection: '{label}' at {best_conf:.2f} confidence.")
                    
                    if label == 'N':
                        current_ratio_a = 1
                        current_ratio_b = 2
                        vision_offset = 0.4
                        current_yolo_status = "N"
                        found_deficiency = True
                        print("[AI-OVERRIDE] Nitrogen deficiency identified - Adjusting Target EC +0.4, ratio A:B = 2:1")
                        
                    elif label == 'P':
                        current_ratio_a = 2
                        current_ratio_b = 1
                        vision_offset = 0.2
                        current_yolo_status = "P"
                        found_deficiency = True
                        print("[AI-OVERRIDE] Phosphorus deficiency identified - Adjusting Target EC +0.2, ratio A:B = 1:2")
                        
                    elif label == 'K':
                        current_ratio_a = 1
                        current_ratio_b = 1
                        vision_offset = 0.3
                        current_yolo_status = "K"
                        found_deficiency = True
                        print("[AI-OVERRIDE] Potassium deficiency identified - Adjusting Target EC +0.3, ratio A:B = 1:1")


                else:
                    # [FIX — Confidence Threshold Filter] Detection exists but
                    # confidence is too low — log and treat as no finding.
                    print(f"[VISION] Detection confidence {best_conf:.2f} below threshold {YOLO_CONFIDENCE_THRESHOLD:.2f} - discarding, treating as healthy.")


            # False-Positive validation safeguard
            if not found_deficiency:
                current_ratio_a = 1
                current_ratio_b = 1
                vision_offset = 0.0
                current_yolo_status = "healthy"
                print("[VISION] Healthy nominal state maintained.")


            # [FIX — Vision Offset Timeout] Mark the timestamp of this
            # successful inference so the staleness watchdog in the control
            # loop knows vision data is fresh.
            last_vision_success_time = time.time()


    except Exception as e:
        print(f"[VISION ERROR] Inference failed: {e}")
    finally:
        vision_thread_active = False


# =======================================================================
# 4. MAIN PI CONTROL LOOP (Triggered at 1Hz by ESP32 Telemetry)
# =======================================================================
def on_local_message(client, userdata, msg):
    global integral_ec, integral_ph, ec_on_count, ph_up_count, ph_dn_count, ratio_counter
    global last_yield_time, last_csv_flush_time, last_tb_publish_time, last_yolo_time
    global last_ec_action_time, last_ph_action_time, yield_days_left
    global active_setpoint_ec
    global ec_window, ph_window
    global vision_offset, current_yolo_status, current_ratio_a, current_ratio_b
    global vision_thread_active  # [FIX — Vision Offset Timeout]


    now = time.time()
    
    # ---------------------------------------------------------
    # A. Parse Raw ESP32 Data & Calculate Single Plant Mass
    # ---------------------------------------------------------
    try:
        payload = json.loads(msg.payload.decode())
        temp_ambient  = float(payload.get('temp', 25.0))
        hum_ambient   = float(payload.get('hum', 50.0))
        raw_ec        = float(payload.get('tds', 0.0))
        raw_ph        = float(payload.get('ph', 7.0))
        weight_raw    = float(payload.get('weight', 0.0))
        
        pump_states['W'] = int(payload.get('water_pump', 0))
        is_diluting_hw   = int(payload.get('diluting', 0))


        # [FIX — Sensor Sanity Checks] Validate all sensor readings against
        # physically plausible operating ranges before passing them to the PI
        # controllers. A reading outside these bounds indicates a probe fault,
        # disconnected wire, or ADC glitch. The entire control cycle is skipped
        # via return so no dosing decision is made on bad data.
        # Ranges are conservative for leafy-green hydroponics — adjust to your
        # specific crop and hardware if needed.
        if not (0.5 < raw_ph < 14.0):
            print(f"[SENSOR FAULT] pH reading {raw_ph:.2f} out of plausible range (0.5–14.0). Skipping cycle.")
            return
        if not (0.0 <= raw_ec < 10.0):
            print(f"[SENSOR FAULT] EC/TDS reading {raw_ec:.3f} out of plausible range (0.0–5.0). Skipping cycle.")
            return
        if not (-2000.0 <= weight_raw < 50000.0):
            print(f"[SENSOR FAULT] Weight reading {weight_raw:.1f}g out of plausible range (0–50000g). Skipping cycle.")
            return


        # ISOLATE 1 PLANT FOR THE AI PREDICTION
        single_plant_weight = max(0.0, weight_raw / 5.0)
        
    except Exception as e:
        return


    # ---------------------------------------------------------
    # B. Signal Smoothing (10-Second Moving Average)
    # ---------------------------------------------------------
    ec_window.append(raw_ec)
    ph_window.append(raw_ph)
    ec_measured = np.mean(ec_window)
    ph_measured = np.mean(ph_window)


    ec_cooldown_elapsed = now - last_ec_action_time
    ph_cooldown_elapsed = now - last_ph_action_time


    # [FIX — Vision Offset Timeout] If the last successful vision inference
    # is older than VISION_STALENESS_LIMIT (3 hours), reset all vision-driven
    # overrides to safe defaults. This prevents a camera failure from leaving
    # a stale EC offset active indefinitely.
    if (now - last_vision_success_time) > VISION_STALENESS_LIMIT:
        if vision_offset != 0.0 or current_yolo_status != "healthy":
            print("[VISION WATCHDOG] Vision data is stale (>3 hrs). Resetting offsets to safe defaults.")
            vision_offset = 0.0
            current_yolo_status = "healthy"
            current_ratio_a = 1
            current_ratio_b = 1


    # Apply YOLO offset to baseline EC
    previous_setpoint_ec = active_setpoint_ec
    active_setpoint_ec = nominal_ec_sp + vision_offset


    # If the YOLO supervisory layer has raised the EC setpoint since the last
    # cycle (e.g. a new deficiency was just detected), reset the EC cooldown
    # timer immediately so the controller does not wait out the remaining
    # lockout period against the old, lower setpoint. This ensures a new
    # corrective dose fires at the very next cycle rather than up to 1 hour
    # later. The reset only triggers on a setpoint INCREASE to avoid
    # prematurely unlocking the cooldown during a healthy→healthy transition.
    if active_setpoint_ec > previous_setpoint_ec:
        last_ec_action_time = 0.0
        print(f"[EC OVERRIDE] Setpoint raised {previous_setpoint_ec:.2f} -> {active_setpoint_ec:.2f}. EC cooldown reset for immediate correction.")


    # -------------------------------------------------------------------
    # CONTROLLER BLOCK A: EC PI ACTUATION PIPELINE
    # -------------------------------------------------------------------
    error_ec = active_setpoint_ec - ec_measured


    if error_ec > 0:
        integral_ec = clamp(integral_ec + (error_ec * Ts), 0.0, INTEGRAL_EC_LIMIT)
        output_ec = clamp((Kp_ec_up * error_ec) + (Ki_ec_up * integral_ec), 0.0, 1.0)
    else:
        integral_ec = 0.0
        output_ec = 0.0


    if ec_cooldown_elapsed >= EC_COOLDOWN:
        if output_ec > OUTPUT_THRESHOLD or ec_on_count > 0:
            fire_nutrient_pumps()
            if ec_on_count <= 0:
                ec_on_count = MIN_ON_CYCLES
            else:
                ec_on_count -= 1
            
            if ec_on_count == 0:
                last_ec_action_time = now
                print(f"[EC] Dosed. Smoothed EC={ec_measured:.3f}, SP={active_setpoint_ec:.2f}")
        else:
            stop_nutrient_pumps()
    else:
        stop_nutrient_pumps()
        if ec_on_count > 0:
            ec_on_count = 0  


    # -------------------------------------------------------------------
    # CONTROLLER BLOCK B: pH PI ACTUATION PIPELINE
    # -------------------------------------------------------------------
    error_ph = setpoint_ph - ph_measured
    dosed_ph = False


    if ph_cooldown_elapsed >= PH_COOLDOWN:
        if abs(error_ph) > PH_DEAD_BAND:
            integral_ph = clamp(integral_ph + (error_ph * Ts), 0.0, INTEGRAL_PH_LIMIT)
        else:
            integral_ph = 0.0


        if error_ph > 0:  # pH too low — fire pH UP
            # [FIX — pH Sign Convention Fix B] ph_up_count is an independent
            # counter that only manages the pH UP pump. It counts down from
            # MIN_ON_CYCLES to 0, enforcing the minimum on-duration. It is
            # never modified by the pH DOWN path, so a direction change mid-
            # sequence cannot leave it stranded at a non-zero value.
            out_ph = clamp((Kp_ph_up * error_ph) + (Ki_ph_up * integral_ph), 0.0, 1.0)
            if out_ph > OUTPUT_THRESHOLD_PH or ph_up_count > 0:
                send_mqtt_cmd('U', 1)
                send_mqtt_cmd('D', 0)
                if ph_up_count <= 0:
                    ph_up_count = MIN_ON_CYCLES
                else:
                    ph_up_count -= 1
                # [FIX — pH Sign Convention Fix B] Ensure the DOWN counter is
                # always cleared when the UP path is active, so any residual
                # count from a previous DOWN sequence cannot re-trigger dosing.
                ph_dn_count = 0
                dosed_ph = True
            else:
                send_mqtt_cmd('U', 0)
                if ph_up_count > 0: ph_up_count -= 1


        else:  # pH too high — fire pH DOWN
            # [FIX — pH Sign Convention Fix B] ph_dn_count is an independent
            # counter that only manages the pH DOWN pump. It counts down from
            # MIN_ON_CYCLES to 0. It is never touched by the pH UP path,
            # eliminating the cross-contamination bug from the original single
            # signed ph_on_count variable.
            out_ph = clamp((Kp_ph_dn * abs(error_ph)) + (Ki_ph_dn * abs(integral_ph)), 0.0, 1.0)
            if out_ph > OUTPUT_THRESHOLD_PH or ph_dn_count > 0:
                send_mqtt_cmd('D', 1)
                send_mqtt_cmd('U', 0)
                if ph_dn_count <= 0:
                    ph_dn_count = MIN_ON_CYCLES
                else:
                    ph_dn_count -= 1
                # [FIX — pH Sign Convention Fix B] Ensure the UP counter is
                # always cleared when the DOWN path is active.
                ph_up_count = 0
                dosed_ph = True
            else:
                send_mqtt_cmd('D', 0)
                if ph_dn_count > 0: ph_dn_count -= 1


        if dosed_ph and ph_up_count == 0 and ph_dn_count == 0:
            last_ph_action_time = now
            print(f"[pH] Dosed. Smoothed pH={ph_measured:.2f}, SP={setpoint_ph:.2f}")
    else:
        send_mqtt_cmd('U', 0)
        send_mqtt_cmd('D', 0)
        # [FIX — pH Sign Convention Fix B] Both counters reset together when
        # the cooldown is still active, ensuring a clean state for the next
        # dosing window regardless of which pump was last active.
        ph_up_count = 0
        ph_dn_count = 0


    # ---------------------------------------------------------
    # D. 1Hz CSV Memory Batching
    # ---------------------------------------------------------
    days_grown = (datetime.now() - PLANTING_DATE).days
    
    csv_batch_buffer.append({
        "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "EC_Measured": round(ec_measured, 3),
        "EC_Setpoint": active_setpoint_ec,
        "pH_Measured": round(ph_measured, 2),
        "pH_Setpoint": setpoint_ph,
        "Weight_Total_g": round(weight_raw, 1),
        "Weight_Per_Plant_g": round(single_plant_weight, 1),
        "Pump_A": pump_states['A'],
        "Pump_B": pump_states['B'],
        "Pump_U": pump_states['U'],
        "Pump_D": pump_states['D'],
        "Pump_W": pump_states['W'],
        "Ratio_A": current_ratio_a,
        "Ratio_B": current_ratio_b,
        "YOLO_Condition": current_yolo_status,
        "Harvest_Days_Left": yield_days_left
    })


    # Buffer daily ML variables
    daily_buffer['temp'].append(temp_ambient)
    daily_buffer['hum'].append(hum_ambient)
    daily_buffer['tds'].append(ec_measured)
    daily_buffer['ph'].append(ph_measured)
    daily_buffer['weight'].append(single_plant_weight)


    # ---------------------------------------------------------
    # E. 60-Second Loop: CSV SD Card Flush
    # ---------------------------------------------------------
    if now - last_csv_flush_time >= 60:
        flush_csv_batch()
        last_csv_flush_time = now


    # ---------------------------------------------------------
    # F. 30-Minute Loop: ThingsBoard Telemetry Push (1800s)
    # ---------------------------------------------------------
    if now - last_tb_publish_time >= 1800:
        scada_payload = {
            "pH": round(ph_measured, 2),
            "EC": round(ec_measured, 2),
            "temperature": round(temp_ambient, 1),
            "humidity": round(hum_ambient, 1),
            "total_biomass": round(weight_raw, 1),
            "single_plant_weight": round(single_plant_weight, 1),
            "ai_status": current_yolo_status,
            "plant_day": days_grown,
            "yield_days_left": yield_days_left,
            "pump_A_state": pump_states['A'],
            "pump_B_state": pump_states['B'],
            "pump_U_state": pump_states['U'],
            "pump_D_state": pump_states['D'],
            "pump_W_state": pump_states['W']
        }
        try:
            tb_client.publish('v1/devices/me/telemetry', json.dumps(scada_payload))
            print(f"[CLOUD] 30-Min Telemetry pushed. YOLO: {current_yolo_status} | Day {days_grown}")
        except Exception as e:
            print(f"[CLOUD ERROR] {e}")
        last_tb_publish_time = now


    # ---------------------------------------------------------
    # G. 1-Hour Loop: YOLO Vision Diagnostics (3600s)
    # ---------------------------------------------------------
    if now - last_yolo_time >= 3600:
        if not vision_thread_active:
            vision_thread_active = True
            threading.Thread(target=run_vision_inference, daemon=True).start()
        last_yolo_time = now


    # ---------------------------------------------------------
    # H. 6-Hour Loop: Yield Prophet Prediction (21600s)
    # ---------------------------------------------------------
    if now - last_yield_time >= 21600:
        if len(daily_buffer['temp']) > 0:
            avg_data = [
                np.mean(daily_buffer['temp']),
                np.mean(daily_buffer['hum']),
                np.mean(daily_buffer['tds']),
                np.mean(daily_buffer['ph']),
                np.mean(daily_buffer['weight'])
            ]
            try:
                # AI predicts the CURRENT age of the plant based on its weight and environment
                predicted_current_age = yield_model.predict([avg_data])[0]
                
                # Assume Harvest is at Day 30 (Change this 30 to your actual crop cycle length!)
                TARGET_HARVEST_DAY = 30
                
                yield_days_left = max(0, int(TARGET_HARVEST_DAY - predicted_current_age))
                print(f"[AI] Yield Prophet Complete. Estimated Days Left: {yield_days_left}")
            except Exception as e:
                print(f"[PREDICT ERROR] {e}")

            for key in daily_buffer:
                daily_buffer[key].clear()
                
        last_yield_time = now




# =======================================================================
# ENTRY POINT RUN TIME EXECUTOR
# =======================================================================
if __name__ == '__main__':
    
    # Callback registration strictly precedes service subscription windows
    local_client = mqtt.Client()
    local_client.on_message = on_local_message


    # [FIX — MQTT Reconnect] Register a disconnect callback for the local
    # broker (ESP32 sensor feed). On any unexpected disconnect (rc != 0),
    # the client will automatically attempt to reconnect so the 1Hz control
    # loop is not silently interrupted.
    def on_local_disconnect(client, userdata, rc):
        if rc != 0:
            print(f"[MQTT LOCAL] Unexpected disconnect (rc={rc}). Attempting reconnect...")
            try:
                client.reconnect()
                print("[MQTT LOCAL] Reconnected to local broker successfully.")
            except Exception as e:
                print(f"[MQTT LOCAL] Reconnect failed: {e}")


    local_client.on_disconnect = on_local_disconnect
    
    tb_client = mqtt.Client()
    tb_client.username_pw_set(ACCESS_TOKEN)


    # [FIX — MQTT Reconnect] Register a disconnect callback for the
    # ThingsBoard cloud broker. Reconnects automatically on unexpected drops
    # so 30-minute telemetry pushes resume without manual intervention.
    def on_tb_disconnect(client, userdata, rc):
        if rc != 0:
            print(f"[MQTT CLOUD] Unexpected disconnect (rc={rc}). Attempting reconnect...")
            try:
                client.reconnect()
                print("[MQTT CLOUD] Reconnected to ThingsBoard successfully.")
            except Exception as e:
                print(f"[MQTT CLOUD] Reconnect failed: {e}")


    tb_client.on_disconnect = on_tb_disconnect
    
    print("Establishing Network Connections...")
    try:
        # Using your local mosquitto broker port
        local_client.connect("127.0.0.1", 1884, 60)
        local_client.subscribe("esp32/sensors")
        
        # Connect ThingsBoard and start its background network thread
        tb_client.connect(THINGSBOARD_HOST, 1883, 60)
        tb_client.loop_start()


        print("\n" + "="*70)
        print("SCADA Digital Twin Online. Running Control & Edge-AI Supervisors...")
        print(f"Planting date    : {PLANTING_DATE.strftime('%Y-%m-%d')}")
        print("CSV Data Logging : Memory Batching Active (Flushing every 60s)")
        print("Cloud Telemetry  : Throttled to 30-Minute Updates")
        print("AI Vision Scan   : 1-Hour Cadence")
        print("Yield Prophet    : 12-Hour Cadence")
        print("Moving Window    : 10-Second Signal Smoothing Active")
        print("="*70 + "\n")


        # Fire an immediate baseline YOLO vision scan on startup
        vision_thread_active = True
        threading.Thread(target=run_vision_inference, daemon=True).start()


        # Start the blocking local 1Hz control loop
        local_client.loop_forever()
        
    except KeyboardInterrupt:
        print("\n[SYSTEM] Terminating SCADA monitor gracefully...")
        
        # CRITICAL: Save any pending 1Hz data from RAM to the SD card before dying
        print("[SYSTEM] Flushing final memory batch to SD Card CSV...")
        flush_csv_batch() 
        
        print("[SYSTEM] Shutting down telemetry threads...")
        tb_client.loop_stop()
        sys.exit(0)
