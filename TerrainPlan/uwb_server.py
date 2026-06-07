#!/usr/bin/env python3
"""
UWB Server - Up to 8 Anchors with 3D Position Calculation
Reads positions 5, 7, 9, 11, 13, 15, 17, 19 for Anchor IDs 0-7
Calculates 3D position using multilateration and Kalman filtering
"""
import serial
import serial.tools.list_ports
import struct
import time
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
from position_solver import PositionSolver
import numpy as np
import paho.mqtt.client as mqtt

try:
    import bridge_control
except ImportError:
    bridge_control = None

latest_data = {
    'timestamp': 0,
    'anchors': {},
    'packet_count': 0,
    'device_port': '',
    'format': 'Waiting for data...',
    'position': {
        'x': None,
        'y': None,
        'z': None,
        'success': False,
        'num_anchors': 0,
        'residual': None,
        'velocity': {'x': 0, 'y': 0, 'z': 0}
    },
    'imu': {
        'acc_x': 0,
        'acc_y': 0,
        'acc_z': 0,
        'angle': 0
    }
}

# Global position solver
position_solver = None

# Track last seen time for each anchor
anchor_last_seen = {}
ANCHOR_TIMEOUT = 2.0  # Seconds before considering anchor disconnected

# Calibration data
calibration_config = {}

def load_calibration():
    """Load calibration data from anchor_config.json"""
    global calibration_config
    try:
        with open('anchor_config.json', 'r') as f:
            config = json.load(f)
            calibration_config = config.get('calibration', {
                'scale_factor': 1.0,
                'offset_mm': 0.0,
                'measurements': []
            })
    except Exception as e:
        print(f"Warning: Could not load calibration config: {e}")
        calibration_config = {
            'scale_factor': 1.0,
            'offset_mm': 0.0,
            'measurements': []
        }

def save_calibration():
    """Save calibration data to anchor_config.json"""
    try:
        with open('anchor_config.json', 'r') as f:
            config = json.load(f)

        config['calibration'] = calibration_config

        with open('anchor_config.json', 'w') as f:
            json.dump(config, f, indent=2)

        return True
    except Exception as e:
        print(f"Error saving calibration: {e}")
        return False

def apply_calibration(anchor_id, raw_distance):
    """Apply global calibration correction to all anchors"""
    if not calibration_config:
        return raw_distance

    scale = calibration_config.get('scale_factor', 1.0)
    offset = calibration_config.get('offset_mm', 0.0)

    # Apply correction: corrected = scale * raw + offset_meters
    corrected = scale * raw_distance + (offset / 1000.0)

    return corrected

def fit_calibration():
    """
    Fit linear calibration model using all collected measurements
    Returns (scale_factor, offset_mm, r_squared, error_message)
    """
    measurements = calibration_config.get('measurements', [])

    if len(measurements) < 2:
        return None, None, None, "Need at least 2 measurements"

    # Extract true and measured distances
    true_distances = np.array([m['true_distance_m'] for m in measurements])
    measured_distances = np.array([m['measured_distance_m'] for m in measurements])

    # Perform linear regression: true = scale * measured + offset
    # Solve: [measured, 1] * [scale, offset] = true
    A = np.column_stack([measured_distances, np.ones(len(measured_distances))])
    params, residuals, rank, s = np.linalg.lstsq(A, true_distances, rcond=None)

    scale_factor = params[0]
    offset_m = params[1]
    offset_mm = offset_m * 1000.0

    # Calculate R-squared
    ss_res = np.sum((true_distances - (scale_factor * measured_distances + offset_m)) ** 2)
    ss_tot = np.sum((true_distances - np.mean(true_distances)) ** 2)
    r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

    return scale_factor, offset_mm, r_squared, None

def get_all_anchors_status(current_anchors):
    """
    Build status for all 8 anchors (IDs 0-7)
    Returns dict with anchor data and connection status
    """
    global anchor_last_seen
    current_time = time.time()

    # Update last seen times for anchors with current data
    for anchor_id, distance in current_anchors.items():
        anchor_last_seen[anchor_id] = current_time

    # Build complete anchor status
    all_anchors = {}
    for anchor_id in ['0', '1', '2', '3', '4', '5', '6', '7']:
        if anchor_id in anchor_last_seen:
            time_since_seen = current_time - anchor_last_seen[anchor_id]
            is_connected = time_since_seen < ANCHOR_TIMEOUT

            all_anchors[anchor_id] = {
                'distance': current_anchors.get(anchor_id, None),
                'connected': is_connected,
                'last_seen': anchor_last_seen[anchor_id]
            }
        else:
            # Never seen this anchor
            all_anchors[anchor_id] = {
                'distance': None,
                'connected': False,
                'last_seen': None
            }

    return all_anchors

def parse_uwb_packet(data):
    """Parse CmdM:4[ format - extract up to 8 anchor distances"""
    header = b'CmdM:4['
    if header not in data:
        return None

    start = data.find(header)
    end = data.find(b'\r\n', start)
    if end == -1:
        return None

    packet = data[start:end]
    payload = packet[7:]

    # Parse 16-bit LE values
    values = []
    for i in range(0, len(payload) - 1, 2):
        if i + 1 < len(payload):
            value = struct.unpack('<H', payload[i:i+2])[0]
            values.append(value)

    # Extract anchor distances based on pattern
    # Position 4 is counter/timestamp (SKIP)
    # Anchor positions increment by 2: 5, 7, 9, 11, 13, 15, 17, 19
    anchors = {}

    anchor_positions = {
        '0': 5,
        '1': 7,
        '2': 9,
        '3': 11,
        '4': 13,
        '5': 15,
        '6': 17,
        '7': 19
    }

    for anchor_id, pos in anchor_positions.items():
        if len(values) > pos and 100 < values[pos] < 10000:
            raw_distance = values[pos] / 1000.0
            # Apply per-anchor software calibration (device onboard calibration not working as expected)
            calibrated_distance = apply_calibration(anchor_id, raw_distance)
            anchors[anchor_id] = calibrated_distance

    return anchors if anchors else None

# Thread lock to protect concurrent updates to latest_data/solver from Serial and MQTT threads
data_lock = threading.Lock()
detected_anchors = set()

def process_uwb_packet(packet, source_name):
    """
    Process a raw UWB packet from a source (Serial or MQTT).
    Parses the packet, solves the 3D position, runs the motor control loop,
    and updates the latest_data dictionary.
    """
    global latest_data, position_solver, detected_anchors

    anchors = parse_uwb_packet(packet)
    if not anchors:
        return False

    with data_lock:
        latest_data['packet_count'] += 1

        # Track newly detected anchors
        new_anchors = set(anchors.keys()) - detected_anchors
        if new_anchors:
            detected_anchors.update(new_anchors)
            print(f"✓ Detected anchors via {source_name}: {sorted(detected_anchors, key=int)}")

        # Get full status for all 8 anchors
        all_anchors_status = get_all_anchors_status(anchors)

        # Calculate 3D position if solver is available
        position_data = {
            'x': None,
            'y': None,
            'z': None,
            'success': False,
            'num_anchors': 0,
            'residual': None,
            'velocity': {'x': 0, 'y': 0, 'z': 0},
            'failure_reason': None
        }

        if position_solver:
            result = position_solver.solve(anchors)
            position_data['success'] = result['success']
            position_data['num_anchors'] = result['num_anchors']
            position_data['residual'] = result['residual']
            position_data['failure_reason'] = result.get('failure_reason')

            if result['success'] and result['position']:
                position_data['x'] = result['position'][0]
                position_data['y'] = result['position'][1]
                position_data['z'] = result['position'][2]

                if result['velocity']:
                    position_data['velocity']['x'] = result['velocity'][0]
                    position_data['velocity']['y'] = result['velocity'][1]
                    position_data['velocity']['z'] = result['velocity'][2]

                # Run robot motor control loop
                if bridge_control is not None:
                    try:
                        bridge_control.process_new_position(position_data['x'], position_data['y'])
                    except Exception as e:
                        print(f"Error calling bridge_control.process_new_position: {e}")

        latest_data.update({
            'timestamp': time.time(),
            'anchors': all_anchors_status,
            'device_port': source_name,
            'format': f'{len(anchors)} Active Anchor{"s" if len(anchors) != 1 else ""}',
            'position': position_data
        })
    return True

def run_mqtt_subscriber():
    """Start MQTT subscriber to receive UWB packets wirelessly"""
    mqtt_broker = "broker.hivemq.com"
    mqtt_port = 1883
    mqtt_topic = "rover/uwb/raw"

    print(f"Starting MQTT UWB subscriber on {mqtt_broker}:{mqtt_port}, topic: {mqtt_topic}")

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("✓ Connected to HiveMQ MQTT Broker for UWB telemetry!")
            client.subscribe(mqtt_topic)
        else:
            print(f"❌ Failed to connect to MQTT broker, rc={rc}")

    def on_message(client, userdata, msg):
        try:
            # Check if payload contains the UWB packet signature
            if b'CmdM:4[' in msg.payload:
                process_uwb_packet(msg.payload, "MQTT (Wireless)")
        except Exception as e:
            print(f"Error in MQTT packet callback: {e}")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    # Keep trying to connect in a loop in case broker is down
    while True:
        try:
            client.connect(mqtt_broker, mqtt_port, 60)
            client.loop_forever()
        except Exception as e:
            print(f"MQTT subscriber connection error: {e}. Retrying in 5s...")
            time.sleep(5)


def read_uwb_data(port='/dev/ttyACM0'):
    """Read UWB data with auto-reconnect on disconnection"""
    global latest_data

    print(f"Starting UWB data reader on {port}")

    ser = None
    buffer = b''
    reconnect_delay = 2.0  # seconds
    failed_attempts = 0
    last_error_print = 0
    was_connected = False

    def connect_serial(silent=False):
        """Attempt to connect to serial port"""
        import os

        # Check if port device file exists first
        #if not os.path.exists(port):
            #if not silent:
                #print(f"Port {port} does not exist")
            #return None

        try:
            s = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(0.5)
            print(f"✓ Connected to {port}")
            return s
        except serial.SerialException as e:
            if not silent:
                print(f"Could not open {port}: {e}")
            return None

    # Initial connection (optional - will retry in loop if fails)
    ser = connect_serial()
    if ser is None:
        print(f"\n⚠️  Device not connected yet. Will keep trying to connect to {port}...")
        print("   Server is running - connect device anytime and it will auto-detect.\n")
    else:
        was_connected = True

    while True:
        try:
            # Check if port is still open
            if ser is None or not ser.is_open:
                if was_connected:
                    # Only print once when initially disconnected
                    print(f"Serial port disconnected. Attempting to reconnect...")
                    was_connected = False
                    failed_attempts = 0

                time.sleep(reconnect_delay)

                # Try to reconnect silently (don't spam errors)
                ser = connect_serial(silent=True)

                if ser is None:
                    failed_attempts += 1

                    # Only print status occasionally (every 30 seconds at 2 second intervals = 15 attempts)
                    #if failed_attempts % 15 == 0:
                        #import os
                        #if os.path.exists(port):
                            #print(f"Still trying to connect to {port}... ({failed_attempts} attempts)")
                        #else:
                            #print(f"Waiting for {port} to appear... ({failed_attempts} attempts)")

                    time.sleep(reconnect_delay)
                    continue
                else:
                    # Successfully reconnected
                    was_connected = True
                    failed_attempts = 0

                buffer = b''  # Clear buffer on reconnect

            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk

                # Process UWB packets
                while b'CmdM:4[' in buffer and b'\r\n' in buffer:
                    start = buffer.find(b'CmdM:4[')
                    end = buffer.find(b'\r\n', start)

                    if start != -1 and end != -1:
                        packet = buffer[start:end+2]
                        buffer = buffer[end+2:]

                        process_uwb_packet(packet, port)

                if len(buffer) > 5000:
                    buffer = buffer[-2000:]

            time.sleep(0.01)

        except (serial.SerialException, OSError) as e:
            # Handle both serial exceptions and I/O errors (e.g., errno 5 when cable unplugged)
            print(f"Serial/IO error: {e}")
            if ser:
                try:
                    ser.close()
                except:
                    pass
            ser = None
            buffer = b''  # Clear buffer on reconnect
            print(f"Will attempt reconnect in {reconnect_delay}s...")
            time.sleep(reconnect_delay)

        except Exception as e:
            print(f"Error reading UWB data: {e}")
            time.sleep(0.1)

class UWBRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(latest_data).encode())

        elif self.path == '/anchor_config.json':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                with open('anchor_config.json', 'rb') as f:
                    self.wfile.write(f.read())
            except FileNotFoundError:
                self.wfile.write(b'{}')

        elif self.path == '/calibration/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(calibration_config).encode())

        elif self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('uwb_dashboard.html', 'rb') as f:
                self.wfile.write(f.read())
        else:
            super().do_GET()

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)

        if self.path == '/update_params':
            try:
                params = json.loads(post_data.decode('utf-8'))

                # Update position solver parameters
                if position_solver:
                    if 'process_noise' in params:
                        position_solver.kalman.update_process_noise(params['process_noise'])
                        print(f"✓ Updated process_noise: {params['process_noise']}")

                    if 'measurement_noise' in params:
                        position_solver.kalman.update_measurement_noise(params['measurement_noise'])
                        print(f"✓ Updated measurement_noise: {params['measurement_noise']}")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok'}).encode())

            except Exception as e:
                print(f"Error updating parameters: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/add_measurement':
            try:
                data = json.loads(post_data.decode('utf-8'))
                anchor_id = str(data['anchor_id'])
                true_distance = float(data['true_distance_m'])

                # Get current measured distance from latest_data
                current_distance = latest_data['anchors'].get(anchor_id, {}).get('distance')

                if current_distance is None:
                    raise ValueError(f"No distance measurement available for anchor {anchor_id}")

                # Ensure calibration structure exists
                if 'measurements' not in calibration_config:
                    calibration_config['measurements'] = []

                # Add measurement
                measurement = {
                    'anchor_id': anchor_id,
                    'true_distance_m': true_distance,
                    'measured_distance_m': current_distance,
                    'timestamp': time.time()
                }
                calibration_config['measurements'].append(measurement)

                # Save to config file
                save_calibration()

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'measurement': measurement,
                    'total_measurements': len(calibration_config['measurements'])
                }).encode())

            except Exception as e:
                print(f"Error adding calibration measurement: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/fit':
            try:
                scale, offset_mm, r_squared, error = fit_calibration()

                if error:
                    raise ValueError(error)

                # Update calibration parameters
                calibration_config['scale_factor'] = float(scale)
                calibration_config['offset_mm'] = float(offset_mm)

                # Save to config
                save_calibration()

                print(f"✓ Fitted global calibration: scale={scale:.4f}, offset={offset_mm:.2f}mm, R²={r_squared:.4f}")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'scale_factor': scale,
                    'offset_mm': offset_mm,
                    'r_squared': r_squared
                }).encode())

            except Exception as e:
                print(f"Error fitting calibration: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/clear':
            try:
                calibration_config['measurements'] = []
                calibration_config['scale_factor'] = 1.0
                calibration_config['offset_mm'] = 0.0
                save_calibration()

                print(f"✓ Cleared global calibration")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok'}).encode())

            except Exception as e:
                print(f"Error clearing calibration: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/delete_measurement':
            try:
                data = json.loads(post_data.decode('utf-8'))
                measurement_index = int(data['index'])

                measurements = calibration_config.get('measurements', [])
                if 0 <= measurement_index < len(measurements):
                    deleted = measurements.pop(measurement_index)
                    save_calibration()
                    print(f"✓ Deleted measurement {measurement_index}")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok'}).encode())

            except Exception as e:
                print(f"Error deleting measurement: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/edit_measurement':
            try:
                data = json.loads(post_data.decode('utf-8'))
                measurement_index = int(data['index'])
                new_true_distance = float(data['true_distance_m'])

                measurements = calibration_config.get('measurements', [])
                if 0 <= measurement_index < len(measurements):
                    measurements[measurement_index]['true_distance_m'] = new_true_distance
                    save_calibration()
                    print(f"✓ Updated measurement {measurement_index} to {new_true_distance}m")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok'}).encode())

            except Exception as e:
                print(f"Error editing measurement: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/read_device':
            try:
                from bu03_util import BU03Device

                # Read current device parameters
                with BU03Device() as device:
                    response = device.get_device_params()

                # Parse the response if possible
                # Response format unclear from docs, so just return raw
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'response': response
                }).encode())

            except Exception as e:
                error_msg = str(e)
                print(f"Error reading device parameters: {e}")
                print(f"Exception type: {type(e).__name__}")

                # Provide more helpful error messages
                if "No TTL port found" in error_msg or "Could not find serial device" in error_msg:
                    error_msg = "TTL port not found. Please connect the serial cable and ensure the device is powered on."
                elif "Permission denied" in error_msg:
                    error_msg = "Permission denied accessing TTL port. Check port permissions."
                elif "timed out" in error_msg.lower():
                    error_msg = "Device not responding. Check TTL connection and power."

                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': error_msg}).encode())

        elif self.path == '/calibration/reset_device':
            try:
                from bu03_util import BU03Device

                # Reset to neutral calibration (raw measurements)
                with BU03Device() as device:
                    device.set_device_params(
                        label_rate=5,
                        antenna_delay=16336,
                        kalman_enable=1,
                        kalman_q=0.018,
                        kalman_r=0.642,
                        correction_a=1.0,    # No scale correction
                        correction_b=0.0,    # No offset correction
                        positioning_enable=0,
                        positioning_dim=0
                    )

                print(f"✓ Reset device calibration to a=1.0, b=0.0 (raw measurements)")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok'}).encode())

            except Exception as e:
                print(f"Error resetting device calibration: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/apply_device':
            try:
                from bu03_util import BU03Device

                # Calculate average calibration across all anchors with data
                per_anchor = calibration_config.get('per_anchor', {})

                # Collect all fitted calibrations
                scale_factors = []
                offsets = []

                for anchor_id in range(8):
                    anchor_cal = per_anchor.get(str(anchor_id), {})
                    measurements = anchor_cal.get('measurements', [])

                    if len(measurements) >= 2:  # Only use anchors with fitted calibration
                        scale = anchor_cal.get('scale_factor', 1.0)
                        offset = anchor_cal.get('offset_mm', 0.0)

                        # Only include if calibrated (not default values)
                        if scale != 1.0 or offset != 0.0:
                            scale_factors.append(scale)
                            offsets.append(offset)

                # Use average if we have calibrated anchors, otherwise defaults
                if scale_factors:
                    avg_scale = sum(scale_factors) / len(scale_factors)
                    avg_offset = sum(offsets) / len(offsets)
                    print(f"Using average calibration: scale={avg_scale:.4f}, offset={avg_offset:.2f}mm")
                    print(f"  (averaged across {len(scale_factors)} calibrated anchors)")
                else:
                    avg_scale = 1.0
                    avg_offset = 0.0
                    print("No calibrated anchors found, using defaults")

                # Connect to device and apply parameters
                with BU03Device() as device:
                    device.set_device_params(
                        label_rate=5,           # Default tag refresh rate
                        antenna_delay=16336,    # Default antenna delay
                        kalman_enable=1,        # Enable Kalman filter
                        kalman_q=0.018,         # Default process noise
                        kalman_r=0.642,         # Default measurement noise
                        correction_a=avg_scale, # Unitless scale factor
                        correction_b=avg_offset, # Offset in millimeters (not meters!)
                        positioning_enable=0,   # Disable on-device positioning
                        positioning_dim=0
                    )

                print(f"✓ Applied calibration to device: a={avg_scale:.4f}, b={avg_offset:.2f}mm")

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'scale_factor': avg_scale,
                    'offset_mm': avg_offset,
                    'num_anchors_used': len(scale_factors)
                }).encode())

            except Exception as e:
                print(f"Error applying device calibration: {e}")
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())

        elif self.path == '/calibration/write_device':
            try:
                # post_data was already read at the top of do_POST()
                if not post_data:
                    raise ValueError("No data received")

                params = json.loads(post_data.decode('utf-8'))

                from bu03_util import BU03Device

                # Extract all 9 parameters from request
                label_rate = params.get('label_rate', 5)
                antenna_delay = params.get('antenna_delay', 16336)
                kalman_enable = params.get('kalman_enable', 1)
                kalman_q = params.get('kalman_q', 0.018)
                kalman_r = params.get('kalman_r', 0.642)
                correction_a = params.get('correction_a', 1.0)
                correction_b = params.get('correction_b', 0.0)
                positioning_enable = params.get('positioning_enable', 0)
                positioning_dim = params.get('positioning_dim', 0)

                # Connect to device and write all parameters
                result = None
                with BU03Device() as device:
                    result = device.set_device_params(
                        label_rate=label_rate,
                        antenna_delay=antenna_delay,
                        kalman_enable=kalman_enable,
                        kalman_q=kalman_q,
                        kalman_r=kalman_r,
                        correction_a=correction_a,
                        correction_b=correction_b,
                        positioning_enable=positioning_enable,
                        positioning_dim=positioning_dim
                    )

                print(f"✓ Parameters written to device")

                # Send success response immediately (before device finishes rebooting)
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'ok',
                    'params': params,
                    'response': result
                }).encode())

            except Exception as e:
                error_msg = str(e)
                print(f"Error writing device parameters: {e}")
                print(f"Exception type: {type(e).__name__}")

                # Provide more helpful error messages
                if "No TTL port found" in error_msg or "Could not find serial device" in error_msg:
                    error_msg = "TTL port not found. Please connect the serial cable and ensure the device is powered on."
                elif "Permission denied" in error_msg:
                    error_msg = "Permission denied accessing TTL port. Check port permissions."
                elif "timed out" in error_msg.lower():
                    error_msg = "Device not responding. Check TTL connection and power."
                elif "Empty request body" in error_msg or "No data received" in error_msg:
                    error_msg = "Request was aborted or incomplete. Please try again."

                try:
                    self.send_response(500)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(json.dumps({'status': 'error', 'message': error_msg}).encode())
                except (BrokenPipeError, ConnectionResetError):
                    # Client already disconnected (timeout) - just log it
                    print(f"Client disconnected before response could be sent")

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

def run_server(port=8080):
    try:
        server = HTTPServer(('0.0.0.0', port), UWBRequestHandler)
        print(f"\nServer running on http://localhost:{port}")
        server.serve_forever()
    except OSError as e:
        if e.errno == 98:  # Address already in use
            print(f"\nError: Port {port} is already in use!")
            print("\nOptions:")
            print(f"  1. Kill the existing server:")
            print(f"     pkill -f 'uwb_server'")
            print(f"  2. Use a different port:")
            print(f"     python3 uwb_server.py /dev/ttyACM0 {port + 1}")
            import sys
            sys.exit(1)
        else:
            raise

if __name__ == "__main__":
    import sys
    import os

    # Parse command line arguments
    http_port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080

    # Auto-detect UWB port if not specified
    if len(sys.argv) > 1:
        uwb_port = sys.argv[1]
    else:
        uwb_port = None
        # Try using serial.tools.list_ports for cross-platform detection
        try:
            ports = list(serial.tools.list_ports.comports())
            candidates = []
            for p in ports:
                # Exclude standard Bluetooth incoming/outgoing ports on Windows
                desc = p.description.lower() if p.description else ""
                dev = p.device.lower() if p.device else ""
                if 'bluetooth' in desc or 'bth' in dev or 'incoming' in desc:
                    continue
                candidates.append(p.device)
                
            if candidates:
                # Prefer COM10 or COM8 if they exist, otherwise pick first
                pref_ports = ['COM10', 'COM8']
                selected = None
                for pref in pref_ports:
                    if pref in candidates:
                        selected = pref
                        break
                if not selected:
                    selected = candidates[0]
                uwb_port = selected
                print(f"✓ Auto-detected serial device at {uwb_port} ({next(p.description for p in ports if p.device == uwb_port)})")
        except Exception as e:
            print(f"Warning: Failed to list serial ports using pyserial: {e}")

        # Fallback to checking Linux paths if pyserial list fails or finds nothing
        if uwb_port is None:
            for port_candidate in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']:
                if os.path.exists(port_candidate):
                    uwb_port = port_candidate
                    print(f"✓ Auto-detected device at {uwb_port}")
                    break

        if uwb_port is None:
            if sys.platform == "win32":
                print("⚠️  No UWB device detected yet (scanned COM ports)")
                print("   Defaulting to COM10 - will auto-connect when device is plugged in")
                uwb_port = 'COM10'
            else:
                print("⚠️  No UWB device detected yet (searched /dev/ttyACM0, /dev/ttyACM1)")
                print("   Defaulting to /dev/ttyACM0 - will auto-connect when device is plugged in")
                uwb_port = '/dev/ttyACM0'

    # Initialize position solver
    print("\n" + "="*50)
    print("Initializing 3D Position Solver")
    print("="*50)
    try:
        position_solver = PositionSolver('anchor_config.json')
        print("✓ Position solver ready")
    except Exception as e:
        print(f"Warning: Could not initialize position solver: {e}")
        print("Continuing without 3D positioning...")
        position_solver = None

    # Load calibration data
    print("\n" + "="*50)
    print("Loading Calibration Data")
    print("="*50)
    load_calibration()
    print("✓ Calibration data loaded")

    # Start the MQTT subscriber thread
    mqtt_thread = threading.Thread(target=run_mqtt_subscriber, daemon=True)
    mqtt_thread.start()

    uwb_thread = threading.Thread(target=read_uwb_data, args=(uwb_port,), daemon=True)
    uwb_thread.start()

    time.sleep(1)
    run_server(http_port)
