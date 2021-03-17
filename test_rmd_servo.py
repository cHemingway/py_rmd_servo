import unittest
import unittest.mock as mock

# Class to be tested
import rmd_servo


class Test__parse_response_header(unittest.TestCase):
    def setUp(self):
        self.patcher = unittest.mock.patch(
            "rmd_servo.serial.Serial", autospec=True)
        self.mock_serial = self.patcher.start()            
        self.servo = rmd_servo.RMD_Servo("test", 2)

    def test_wrong_len(self):
        self.assertIsNone(self.servo._parse_response_header(bytearray(4), 1),
                          "Resp of length 4 should fail")
        self.assertIsNone(self.servo._parse_response_header(bytearray(6), 1),
                          "Resp of length 6 should fail")

    def test_checksum(self):
        # Test command 0x04, data length 0
        valid_data = bytearray([0x3E,0x04,0x02,0x00,0x44])
        length = self.servo._parse_response_header(valid_data,0x04)
        self.assertEqual(length, 0)

        # Test command 0xFF, data length 5
        valid_data = bytearray([0x3E,0xFF,0x02,0x05,0x45])
        length = self.servo._parse_response_header(valid_data,0xFF)
        self.assertEqual(length, 5)

        # Test invalid checksum
        invalid_data = bytearray([0x3E,0xFF,0x02,0x05,0x46])
        length = self.servo._parse_response_header(invalid_data,0xFF)
        self.assertIsNone(length)


    def tearDown(self):
        mock.patch.stopall() # Stop mocks, or else other tests are corrupted
        return super().tearDown()


class Test__send_raw_command(unittest.TestCase):
    ''' Verify that __send_raw_command passes correct data to serial port '''

    def setUp(self):
        self.patcher = unittest.mock.patch(
            "rmd_servo.serial.Serial", autospec=True)
        self.mock_serial = self.patcher.start()
        self.servo = rmd_servo.RMD_Servo("test", 2)
        self.mock_serial.assert_called_once_with(
            "test", baudrate=115200, timeout=0.5)
        # Mock individual serial functions
        self.servo.ser.read = mock.Mock()
        self.servo.ser.reset_input_buffer = mock.Mock()
        self.servo.ser.write = mock.Mock()

    def test_correct_tx_no_data(self):
        expected_tx = bytearray.fromhex("3E AA 02 00 EA")
        given_rx = expected_tx
        # Set a returned value
        self.servo.ser.read.return_value = given_rx
        # Call function and check it called mocks OK
        self.servo._send_raw_command(0xAA)
        self.servo.ser.reset_input_buffer.assert_called_once()
        self.servo.ser.write.assert_called_once_with(bytes(expected_tx))

    def tearDown(self):
        mock.patch.stopall() # Stop mocks, or else other tests are corrupted
        return super().tearDown()


class Test_move_closed_loop_speed(unittest.TestCase):
    ''' Tests that move_positive passes correct data to __send_raw_command '''

    def setUp(self):
        self.mock_serial = mock.patch(
            "rmd_servo.serial.Serial").start()
        self.mock_send_raw_command = mock.patch(
            "rmd_servo.RMD_Servo._send_raw_command").start()
        self.servo = rmd_servo.RMD_Servo("test")

    def test_move_positive(self):
        # Verified in RMD Motor Assistant
        speed_hex = bytes.fromhex("10 34 02 00")
        # Call function, check it returned right value
        self.mock_send_raw_command.return_value = bytes.fromhex(
            "22 2003 0000 b80b")
        resp = self.servo.move_closed_loop_speed(1444)
        self.mock_send_raw_command.assert_called_once_with(0xA2, speed_hex)
        # Check response parsed properly
        self.assertDictEqual(resp,
                             {
                                 "temperature": 34,
                                 "power":       800,
                                 "speed":       0,
                                 "position":    3000
                             }
                             )

    def tearDown(self):
        mock.patch.stopall()  # Stop mocks, or else other tests are corrupted
        return super().tearDown()

if __name__ == "__main__":
    unittest.main()
