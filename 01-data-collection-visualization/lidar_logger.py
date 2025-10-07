#--------------------------------------------------------------------------------------------------------------
# SF45/B Lidar Logger - Fixed Version
#--------------------------------------------------------------------------------------------------------------

import time
import serial
import csv
import os

#--------------------------------------------------------------------------------------------------------------
# LWNX helper functions
#--------------------------------------------------------------------------------------------------------------
_packet_parse_state = 0
_packet_payload_size = 0
_packet_size = 0
_packet_data = []

def create_crc(data):
    crc = 0
    for i in data:
        code = crc >> 8
        code ^= int(i)
        code ^= code >> 4
        crc = crc << 8
        crc ^= code
        code = code << 5
        crc ^= code
        code = code << 7
        crc ^= code
        crc &= 0xFFFF
    return crc

def build_packet(command, write, data=[]):
    payload_length = 1 + len(data)
    flags = (payload_length << 6) | (write & 0x1)
    packet_bytes = [0xAA, flags & 0xFF, (flags >> 8) & 0xFF, command]
    packet_bytes.extend(data)
    crc = create_crc(packet_bytes)
    packet_bytes.append(crc & 0xFF)
    packet_bytes.append((crc >> 8) & 0xFF)
    return bytearray(packet_bytes)

def parse_packet(byte):
    global _packet_parse_state, _packet_payload_size, _packet_size, _packet_data
    if _packet_parse_state == 0:
        if byte == 0xAA:
            _packet_parse_state = 1
            _packet_data = [0xAA]
    elif _packet_parse_state == 1:
        _packet_parse_state = 2
        _packet_data.append(byte)
    elif _packet_parse_state == 2:
        _packet_parse_state = 3
        _packet_data.append(byte)
        _packet_payload_size = (_packet_data[1] | (_packet_data[2] << 8)) >> 6
        _packet_payload_size += 2
        _packet_size = 3
        if _packet_payload_size > 1019:
            _packet_parse_state = 0
    elif _packet_parse_state == 3:
        _packet_data.append(byte)
        _packet_size += 1
        _packet_payload_size -= 1
        if _packet_payload_size == 0:
            _packet_parse_state = 0
            crc = _packet_data[_packet_size - 2] | (_packet_data[_packet_size - 1] << 8)
            verify_crc = create_crc(_packet_data[0:-2])
            if crc == verify_crc:
                return True
    return False

def wait_for_packet(port, command, timeout=1):
    global _packet_parse_state, _packet_payload_size, _packet_size, _packet_data
    _packet_parse_state = 0
    _packet_data = []
    _packet_payload_size = 0
    _packet_size = 0
    end_time = time.time() + timeout
    while True:
        if time.time() >= end_time:
            return None
        c = port.read(1)
        if len(c) != 0:
            b = c[0] if isinstance(c[0], int) else ord(c)
            if parse_packet(b):
                if _packet_data[3] == command:
                    return _packet_data

def execute_command(port, command, write, data=[], timeout=1):
    packet = build_packet(command, write, data)
    retries = 4
    while retries > 0:
        retries -= 1
        port.write(packet)
        response = wait_for_packet(port, command, timeout)
        if response is not None:
            return response
    raise Exception('LWNX command failed to receive a response.')

def get_str16_from_packet(packet):
    str16 = ''
    for i in range(0, 16):
        if packet[4 + i] == 0:
            break
        str16 += chr(packet[4 + i])
    return str16

def print_product_information(port):
    response = execute_command(port, 0, 0, timeout=0.1)
    print('Product: ' + get_str16_from_packet(response))
    response = execute_command(port, 2, 0, timeout=0.1)
    print('Firmware: {}.{}.{}'.format(response[6], response[5], response[4]))
    response = execute_command(port, 3, 0, timeout=0.1)
    print('Serial: ' + get_str16_from_packet(response))

def set_update_rate(port, value):
    if value < 1 or value > 12:
        raise Exception('Invalid update rate value.')
    execute_command(port, 66, 1, [12])

def set_distance_output_with_yaw(port):
    # first return + yaw angle
    execute_command(port, 27, 1, [8, 1, 0, 0])

def set_distance_stream_enable(port, enable):
    if enable:
        execute_command(port, 30, 1, [5, 0, 0, 0])
    else:
        execute_command(port, 30, 1, [0, 0, 0, 0])

def wait_for_reading(port, timeout=1):
    response = wait_for_packet(port, 44, timeout)
    if response is None or len(response) < 8:
        return -1, None
    distance = (response[4] | (response[5] << 8)) / 100.0
    yaw_angle = response[6] | (response[7] << 8)
    if yaw_angle > 32000:
        yaw_angle -= 65535
    yaw_angle /= 100.0
    return distance, yaw_angle

#--------------------------------------------------------------------------------------------------------------
# Main application
#--------------------------------------------------------------------------------------------------------------

# Save CSV inside 01-data-collection-visualization folder
log_folder = "01-data-collection-visualization"
os.makedirs(log_folder, exist_ok=True)  # create folder if it doesn't exist
csv_path = os.path.join(log_folder, "sf45b_log.csv")

print('Running SF45/B Lidar Logger...')

serial_port_name = '/dev/ttyUSB0'  # update for your Pi
serial_port_baudrate = 921600
sensor_port = serial.Serial(serial_port_name, serial_port_baudrate, timeout=0.1)

print_product_information(sensor_port)
set_update_rate(sensor_port, 1)  # 50 Hz
set_distance_output_with_yaw(sensor_port)
set_distance_stream_enable(sensor_port, True)

# CSV logging
with open("sf45b_log.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "distance_m", "yaw_deg"])
    
    print("Logging started... Press Ctrl+C to stop.")
    try:
        while True:
            distance, yaw_angle = wait_for_reading(sensor_port)
            timestamp = time.time()
            
            if distance != -1 and yaw_angle is not None:
                print(f"{timestamp:.3f}, {distance:.2f} m, {yaw_angle:.2f} deg")
                writer.writerow([timestamp, distance, yaw_angle])
                f.flush()
            else:
                print("No valid reading.")
            
            #time.sleep(0.05)  # 50 ms delay
    except KeyboardInterrupt:
        print("Logging stopped.")
        set_distance_stream_enable(sensor_port, False)
