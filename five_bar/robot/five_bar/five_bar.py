import time
import can
import threading

class five_bar:
    def __init__(self,
                 SERIAL_PORT="COM1",
                 torque_constant1=0.45,
                 torque_constant2=0.45,
                 voltage=24,
                 max_speed=1e3,
                 max_speed_allowed_overshoot=250,
                 motor1_ID=0x141,
                 motor2_ID=0x142):
        """
        Initializes the five_bar robot interface and sets up CAN communication, motor parameters,
        control mode, safety parameters, and threading infrastructure.

        Args:
            SERIAL_PORT (str): Serial port name to connect to.
            torque_constant1 (float): Torque constant for motor 1 (Nm/A).
            torque_constant2 (float): Torque constant for motor 2 (Nm/A).
            voltage (float): Supply voltage used for power computation (V).
            max_speed (float): Maximum allowable motor speed (deg/s).
            max_speed_allowed_overshoot (int): Tolerance over max_speed before triggering shutdown.
            motor1_ID (int): CAN ID for motor 1.
            motor2_ID (int): CAN ID for motor 2.
        """
        print("\nInitializing Robot...\n")

        self.SERIAL_PORT = SERIAL_PORT

        self.motors_data = {
            "voltage": [voltage, voltage],
            "current": [0, 0],
            "speed": [0, 0],
            "angle": [0, 0],
            "temperature": [0, 0],
            "power": [0, 0],
            "torque": [0, 0],
            "torque_constant": [torque_constant1, torque_constant2],
            "motor_ID": [motor1_ID, motor2_ID]
        }

        self.active = True
        self.threads_status = False
        self.changing_mode= False
        self.target = [0, 0]
        self.control_modes = ["speed", "position", "torque"]
        self.control_mode = self.control_modes[2]
        self.max_speed = int(max_speed)
        self.max_speed_allowed_overshoot = max_speed_allowed_overshoot

        if self.max_speed > 2**16 - 1:
            raise Exception(f"Maximum speed out of range. Value attempted was {self.max_speed} and maximum is {2**16 - 1}")

        print(f"Connecting to CAN bus {self.SERIAL_PORT}...")

        try:
            self.bus = can.interface.Bus(
                interface='slcan',
                channel=self.SERIAL_PORT,
                bitrate=1000000
            )
        except Exception as e:
            raise RuntimeError(f"CAN connection failed: {e}")

        self.buffer = can.BufferedReader()
        self.notifier = can.Notifier(self.bus, [self.buffer])

        print(f"Successfully connected to CAN bus {self.SERIAL_PORT}!\n")

        self.thread1 = threading.Thread(target=self._read_serial_thread, daemon=True)
        self.thread2 = threading.Thread(target=self._serial_write_thread, daemon=True)


        print(f"Robot Initialized. \nDefaulting to control mode: '{self.control_mode}'\nStart threads to begin using the robot\n")

    def get_motors_data(self):
        """
        Returns the dictionary containing the latest motor data.

        Returns:
            dict: Dictionary with keys: voltage, current, speed, angle, temperature, power,
                  torque, torque_constant, and motor_ID.
        """
        return self.motors_data

    def set_control_mode(self, mode, target1=0, target2=0, override=False):
        """
        Sets the control mode of the robot and optionally updates motor targets.
        Performs safety checks on speed/torque transitions.

        Args:
            mode (str or None): New control mode ('speed', 'position', 'torque') or None to disable.
            target1 (float): Target for motor 1.
            target2 (float): Target for motor 2.
            override (bool): If True, bypass safety check on non-zero target change.
        """
        
            
        self.changing_mode=True
        
        if self.active:
            if mode == None:
                self.active = False
                self.control_mode = "torque"
                self.target = [0, 0]
                print("Aborting: Motors reached maximum allowed speed, setting control mode to torque and setting targets to [0,0]")
            elif mode not in self.control_modes:
                raise Exception(f"Error: Mode '{mode}' is not a valid mode")
            else:
                self.control_mode = mode

                if (mode in ["torque", "speed"]) and (target1 != 0 or target2 != 0) and not override:
                    print(f"Attempted to set targets [{target1}, {target2}] while changing to control mode '{mode}'. To override, set override=True.")
                    self.target = [0, 0]
                else:
                    self.target = [target1, target2]

                print(f"Control Mode set to '{self.control_mode}'. Setting targets to {self.target}\n")
        else:
            print("Can not change mode when mechanism is inactive")
        self.changing_mode=False

    def set_target(self, target1, target2):
        """
        Updates the current motor targets if the robot is active.

        Args:
            target1 (float): Target value for motor 1.
            target2 (float): Target value for motor 2.
        """
        if self.active:
            self.target = [target1, target2]
        else:
            print("Robot is inactive. Setting target unavailable. Please reactivate.\n")

    def start(self):
        """
        Starts the background threads for reading from and writing to the CAN bus.
        """
        print(f"Starting Read/write threads...")
        self.thread1.start()
        self.thread2.start()
        self.threads_status = True
        time.sleep(0.5)
        print(f"Read/write threads started!\n")

    def reactivate(self):
        """
        Reactivates the robot after being deactivated due to a fault condition.
        """
        self.active = True

    def _read_serial_thread(self):
        """
        Background thread to continuously read CAN messages and process them.
        Also checks motor speed to trigger safety mode if it exceeds thresholds.
        """
        while True:
            data = self._read_serial()
            if data is not None:
                self._process_data(data)
                if (max(list(map(abs, self.motors_data["speed"]))) > self.max_speed + self.max_speed_allowed_overshoot) and self.active:
                    self.set_control_mode(None)

    def _serial_write_thread(self):
        """
        Background thread to periodically send motor control messages based on current mode and targets.
        """
        while True:
            if not self.changing_mode:
                for msg in self._get_control_messages():
                    self._write_serial(msg)
                time.sleep(0.01)

    def _write_serial(self, msg):
        """
        Sends a single CAN message to the motor.

        Args:
            msg (list): CAN message as list of integers.
        """
        try:
            send_msg = can.Message(arbitration_id=msg[0], data=msg[1:], is_extended_id=False)
            self.bus.send(send_msg)
        except can.CanError as e:
            print(f"Failed to send message: {e}")

    def _read_serial(self):
        """
        Reads a single CAN message from the buffer.

        Returns:
            list or None: Received CAN message formatted as list.
        """
        data = None
        try:
            received_msg = self.buffer.get_message()
            if received_msg:
                data = [received_msg.arbitration_id] + list(received_msg.data)
        except can.CanError as e:
            print(f"Failed to receive message: {e}")
        return data

    def _process_data(self, data):
        """
        Parses a received CAN message and updates the internal state of the robot.

        Args:
            data (list): Raw CAN message.
        """
        data_bytes = data[1:]
        sign_multiplier = 1

        if data[0] == (self.motors_data["motor_ID"][0] + 0x100):
            motor_id = 0
            sign_multiplier = -1
        elif data[0] == (self.motors_data["motor_ID"][1] + 0x100):
            motor_id = 1
            sign_multiplier = 1
        else:
            motor_id = None

        if (data_bytes[0] == 0xA1) or (data_bytes[0] == 0xA4) or (data_bytes[0] == 0xA2):
            self.motors_data["temperature"][motor_id] = int.from_bytes(data_bytes[1:2], byteorder="little", signed=True)
            self.motors_data["current"][motor_id] = int.from_bytes(data_bytes[2:4], byteorder="little", signed=True) * 0.01 * sign_multiplier
            self.motors_data["speed"][motor_id] = int.from_bytes(data_bytes[4:6], byteorder="little", signed=True) * sign_multiplier
            self.motors_data["angle"][motor_id] = int.from_bytes(data_bytes[6:8], byteorder="little", signed=True) * sign_multiplier
        else:
            print("Unknown Information Received:", data)

        voltages = self.motors_data["voltage"]
        currents = self.motors_data["current"]
        motor_constants = self.motors_data["torque_constant"]

        self.motors_data["power"] = [voltages[0] * currents[0], voltages[1] * currents[1]]
        self.motors_data["torque"] = [motor_constants[0] * currents[0], motor_constants[1] * currents[1]]

    def _get_control_messages(self):
        """
        Constructs appropriate control messages based on the current control mode.

        Returns:
            list: Two control messages, one for each motor.
        """
        if self.control_mode == self.control_modes[0]:
            target1 = -1 * int(self.target[0] / 0.01)
            target2 = int(self.target[1] / 0.01)
            message1 = [self.motors_data["motor_ID"][0], 0xA2, 0x00, 0x00, 0x00,
                        target1 & 0xFF, (target1 >> 8) & 0xFF, (target1 >> 16) & 0xFF, (target1 >> 24) & 0xFF]
            message2 = [self.motors_data["motor_ID"][1], 0xA2, 0x00, 0x00, 0x00,
                        target2 & 0xFF, (target2 >> 8) & 0xFF, (target2 >> 16) & 0xFF, (target2 >> 24) & 0xFF]
        elif self.control_mode == self.control_modes[1]:
            target1 = -1 * int(self.target[0] / 0.01)
            target2 = int(self.target[1] / 0.01)
            message1 = [self.motors_data["motor_ID"][0], 0xA4, 0x00, self.max_speed & 0xFF,
                        (self.max_speed >> 8) & 0xFF, target1 & 0xFF, (target1 >> 8) & 0xFF,
                        (target1 >> 16) & 0xFF, (target1 >> 24) & 0xFF]
            message2 = [self.motors_data["motor_ID"][1], 0xA4, 0x00, self.max_speed & 0xFF,
                        (self.max_speed >> 8) & 0xFF, target2 & 0xFF, (target2 >> 8) & 0xFF,
                        (target2 >> 16) & 0xFF, (target2 >> 24) & 0xFF]
        else:
            motor_constants = self.motors_data["torque_constant"]
            target1 = -1 * int((self.target[0] / 0.01) / motor_constants[0])
            target2 = int((self.target[1] / 0.01) / motor_constants[1])
            message1 = [self.motors_data["motor_ID"][0], 0xA1, 0x00, 0x00, 0x00,
                        target1 & 0xFF, (target1 >> 8) & 0xFF, 0x00, 0x00]
            message2 = [self.motors_data["motor_ID"][1], 0xA1, 0x00, 0x00, 0x00,
                        target2 & 0xFF, (target2 >> 8) & 0xFF, 0x00, 0x00]
        return [message1, message2]

    def write_motor_zero_command(self):
        """
        Sends encoder zeroing and reset messages to motors. Only allowed when threads are inactive.
        """
        if not self.threads_status:
            for msg in self._get_motor_zero_messages():
                self._write_serial(msg)
                time.sleep(0.5)
                print(self._read_serial())
            for msg in self._get_system_reset_message():
                self._write_serial(msg)
        else:
            print("Motor Zero can only be set when threads are not running.")

    def _get_motor_zero_messages(self):
        """
        Returns CAN messages to zero both motor encoders.

        Returns:
            list: Messages for encoder zeroing.
        """
        return [
            [self.motors_data["motor_ID"][0], 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [self.motors_data["motor_ID"][1], 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ]

    def _get_system_reset_message(self):
        """
        Returns CAN messages to reset both motor controllers.

        Returns:
            list: Reset messages for each motor.
        """
        return [
            [self.motors_data["motor_ID"][0], 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [self.motors_data["motor_ID"][1], 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ]
