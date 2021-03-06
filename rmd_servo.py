from enum import Enum
import struct
from typing import Union

import serial
import serial.rs485

# Struct format for movement response
# Used for multiple commands, so specify it here for berevity and execution speed,
# as struct.Struct precompiles the unpacking
movement_resp_struct = struct.Struct("<BhhH")


class RMD_Servo:
    ''' Base class for all RMD series servos. You should use your models subclass instead '''

    def __init__(self, serial_port: str, id: int = 1, baudrate: int = 115200, timeout_s: float = 0.5):
        self.id = id
        self.ser = serial.Serial(
            serial_port, baudrate=baudrate, timeout=timeout_s)
        #self.ser.rs485_mode = serial.rs485.RS485Settings(True,False)
        self.command_struct = struct.Struct("<ccccc")

    def _parse_response_header(self, resp: bytearray, expected_command: int) -> Union[int, None]:
        ''' Parses a response, returns the data length or None if response is invalid '''
        # Verify length
        if len(resp) != 5:
            return None
        # Verify checksum
        if sum(resp[0:4]) != resp[4]:
            return None
        # Verify expected command (should be same as sent)
        if resp[1] != expected_command:
            return None
        # Verify ID byte
        if resp[2] != self.id:
            return None  # TODO: Would we ever recieve this in practice?
        # Finally, return length
        return resp[3]

    def _send_raw_command(self, command: int, data: bytearray = None) -> bytes:
        # Work out lengths
        data_len = len(data) if data else 0
        total_len = 5 + data_len + 1 if data else 5
        # Create what to transmit and fill in
        to_send = bytearray(total_len)
        to_send[0] = 0x3E
        to_send[1] = command
        to_send[2] = self.id
        to_send[3] = data_len
        to_send[4] = sum(to_send[0:4]) % 255    # Fill header_checksum
        if data:
            to_send[5:-1] = data
            to_send[-1] = sum(to_send[5:-1]) % 255  # Data checksum
        # Transfer data
        self.ser.reset_input_buffer()
        self.ser.write(to_send)
        # Get response, at least 5 bytes
        resp_header = self.ser.read(5)
        response_len = self._parse_response_header(resp_header, command)
        if response_len is not None:
            response = self.ser.read(response_len)
            # TODO check response checksum
            return response
        elif response_len == 0:
            pass  # No response
        else:
            raise IOError("Recieved wrong response header")

    @staticmethod
    def _unpack_movement_response(response):
        ''' Unpacks a movement response into a dict '''
        temperature, power, speed, position = movement_resp_struct.unpack(
            response)
        # TODO: For RMD-X, RMD-L, power is actually Torque (iq)
        return {"temperature": temperature, "power": power,
                "speed": speed, "position": position}

    def read_encoder(self) -> int:
        encoder_data = self._send_raw_command(0x90)
        # Return encoder position minus offset
        return int.from_bytes(encoder_data[0:2], byteorder='little', signed=False)

    def read_model(self) -> dict:
        ''' Read model and version information '''
        response = self._send_raw_command(0x12)
        driver, motor, hw_ver, fw_ver = struct.unpack("<20s20sBB", response)
        # Convert bytes to string and remove null termination, as struct.unpack pads
        driver = driver.decode().rstrip('\x00')
        motor = motor.decode().rstrip('\x00')
        # Version is actually version / 10.0
        hw_ver = round(hw_ver / 10.0, 3)
        fw_ver = round(fw_ver / 10.0, 3)
        # Return as dict
        return {"driver": driver, "motor": motor, "hw_ver": hw_ver, "fw_ver": fw_ver}

    def read_status(self) -> int:
        ''' Read status command '''
        # FIXME: Does not yet work? Returned data is all zero?
        response = self._send_raw_command(0x9A)
        temperature, voltage, error = struct.unpack("<20s", response)
        return {"temperature": temperature, "voltage": voltage, "error": error}

    def shutdown(self):
        ''' Stop motion, clear state and previous instructions '''
        self._send_raw_command(0x80)

    def stop(self):
        ''' Stop motion, but do not clear state or previous instructions '''
        self._send_raw_command(0x81)

    def clear_errors(self):
        self._send_raw_command(0x9B)

    def enable_movement(self):
        ''' Restore operation after stop command '''
        self._send_raw_command(0x88)

    def move_closed_loop_speed(self, speed_dps: float):
        ''' Move closed loop at speed_dps degrees per second 
            Returns temperature, torque_current_IQ, speed, position
        '''
        speed = int(speed_dps * 0.01)  # Scale to 0.01 DPS/LSB
        data = struct.pack("<i", speed)
        response = self._send_raw_command(0xA2, data)
        return self._unpack_movement_response(response)


class RMD_S_Servo(RMD_Servo):
    ''' RMD-S Model Servos '''

    def __init__(self, serial_port, id=1, baudrate=115200, timeout_s=0.5):
        super().__init__(serial_port, id=id, baudrate=baudrate, timeout_s=timeout_s)

    def move_open_loop(self, power: int) -> dict:
        ''' Move open loop with set power.
            Returns temperature, (previously set) power, speed, position
        '''
        # FIXME: Does not work with negative power? Gets no response
        # Check input is in range
        if abs(power) > 1000:
            raise ValueError("Power must be in range -1000 to 1000")
        data = struct.pack("<h", power)  # Convert power to little endian short
        response = self._send_raw_command(0xA0, data)
        return self._unpack_movement_response(response)


if __name__ == "__main__":
    import time
    servo = RMD_S_Servo("COM5")
    print(servo.read_model())

    servo.shutdown()
    print("Warning, servo will start moving in 5 seconds!")
    time.sleep(5)
    # Clear errors so motor is in known state
    servo.clear_errors()
    servo.enable_movement()

    print("Testing open loop")
    print(servo.move_open_loop(200))
    time.sleep(1)
    print(servo.move_open_loop(100))
    time.sleep(1)
    servo.move_open_loop(0)

    print("Testing constant speed")
    print(servo.move_closed_loop_speed(100))
    time.sleep(1)
    print(servo.move_closed_loop_speed(200))
    time.sleep(1)
    servo.stop()

    print("Stopped, reading encoder value")
    while 1:
        print("Encoder: ", servo.read_encoder())
        time.sleep(0.25)
