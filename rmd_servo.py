from enum import Enum
import struct
from typing import Union

import serial
import serial.rs485


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
        self._send_raw_command(0x80)

    def clear_errors(self):
        self._send_raw_command(0x9B)

    def enable_movement(self):
        ''' Restore operation after stop command '''
        self._send_raw_command(0x88)


class RMD_S_Servo(RMD_Servo):
    ''' RMD-S Model Servos '''

    def __init__(self, serial_port, id=1, baudrate=115200, timeout_s=0.5):
        super().__init__(serial_port, id=id, baudrate=baudrate, timeout_s=timeout_s)

    def move_open_loop(self, power: int) -> dict:
        ''' Move using Torque Open Loop command '''
        # FIXME: Does not work with negative power?
        if abs(power) > 1000:
            raise ValueError("Power must be in range -1000 to 1000")
        data = struct.pack("<h", power)  # Convert power to little endian short
        response = self._send_raw_command(0xA0, data)

        temperature, power, speed, position = struct.unpack("<BhhH", response)
        return {"temperature": temperature, "power": power, 
                "speed": speed, "position": position}


if __name__ == "__main__":
    import time
    servo = RMD_S_Servo("COM5")
    print(servo.read_model())

    servo.shutdown()
    time.sleep(0.1)
    servo.clear_errors()
    servo.enable_movement()

    print("Testing open loop")
    servo.move_open_loop(200)
    time.sleep(1)
    servo.move_open_loop(100)
    time.sleep(1)
    servo.move_open_loop(0)

    print("Stopped, reading encoder value")
    while 1:
        print("Encoder: ", servo.read_encoder())
        time.sleep(0.25)
