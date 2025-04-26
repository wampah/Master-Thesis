from pySerialTransfer import pySerialTransfer as txfer
import time
import threading

class five_bar:
    def __init__(self,
                 SERIAL_PORT="COM1",
                 BAUD_RATE=115200, 
                 msg_size=int((8+1)*4), # 8 bytes for msg, 1 for id - times 4 since pytransfer interprets bytes as 4 byte ints
                 torque_constant1=0.45,
                 torque_constant2=0.45,
                 max_speed=1e3,
                 max_speed_allowed_overshoot=250,
                 motor1_ID=0x00,
                 motor2_ID=0x01):
        """
        Initializes the five_bar robot interface.

        Args:
            SERIAL_PORT (str): Serial port name to connect to.
            BAUD_RATE (int): Communication baud rate.
            msg_size (int): Size of each message in bytes.
            torque_constant1 (float): Torque constant for motor 1.
            torque_constant2 (float): Torque constant for motor 2.
            max_speed (float): Maximum speed allowed before triggering an abort.
            max_speed_allowed_overshoot (int): Additional overshoot allowed for maximum speed.
            motor1_ID (int): Motor 1 identifier.
            motor2_ID (int): Motor 2 identifier.
        """
        print("\nInitializing Robot...\n")
        
        self.SERIAL_PORT = SERIAL_PORT
        self.BAUD_RATE = BAUD_RATE
        
        self.motors_data = {
            "voltage": [0, 0],
            "current": [0, 0],
            "speed": [0, 0],
            "angle": [0, 0],
            "temperature": [0, 0],
            "brake_status": [0, 0],
            "power": [0, 0],
            "torque": [0, 0],
            "torque_constant": [torque_constant1, torque_constant2],
            "motor_ID": [motor1_ID, motor2_ID]
        }
        
        self.active = True
        
        self.threads_status = False
                 
        self.target = [0, 0]
        
        self.control_modes = ["speed", "position", "torque"]
        
        self.control_mode = self.control_modes[2]
        
        self.max_speed = int(max_speed)
        
        self.max_speed_allowed_overshoot = max_speed_allowed_overshoot
        
        if self.max_speed > 2**16 - 1:
            raise Exception(f"Maximum speed out of range. Value attempted was {self.max_speed} and maximum is {2**16 - 1}")
        
        print(f"Connecting to Serial Port {self.SERIAL_PORT}...")
    
        self.link = txfer.SerialTransfer(self.SERIAL_PORT, baud=self.BAUD_RATE)
        
        self.msg_size = msg_size
        
        self.link.open()
        
        time.sleep(2)
        
        print(f"Successfully connected to Serial Port {self.SERIAL_PORT}!\n")
                
        self.thread1 = threading.Thread(target=self._read_serial_thread, daemon=True)
        self.thread2 = threading.Thread(target=self._serial_write_thread, daemon=True)
        
        print(f"Robot Initialized. \nDefaulting to control mode: '{self.control_mode}'\nStart threads to begin using the robot\n")
    
    def get_motors_data(self):
        """
        Getter function for motor data.

        Returns:
            dict: Dictionary containing current values for voltage, current, speed, angle, temperature, brake status, power, torque, torque constants, and motor IDs.
        """
        return self.motors_data
    
    def set_control_mode(self, mode):
        """
        Changes the control mode of the motors.

        Args:
            mode (str): Control mode of the motors. Must be one of the valid control modes ('speed', 'position', 'torque').
        """
        if mode in self.control_modes:
            self.target = [0, 0]
            self.control_mode = mode
            if self.active:
                print(f"Control Mode set to '{self.control_mode}'. Setting targets to [0,0]\n")
        else:
            raise Exception("Error: Mode", mode, "is not a valid mode")
        
    def set_target(self, target1, target2):
        """
        Sets the target values for the motors.

        Args:
            target1 (float): Target value for motor 1.
            target2 (float): Target value for motor 2.
        """
        if self.active:
            self.target = [target1, target2]
        else:
            print("Robot is inactive. Setting target unavailable. Please restart.\n")
            self.set_control_mode("torque")
            self.target = [0, 0]
            
    def start(self):
        """
        Starts the serial read and write threads.
        """
        print(f"Starting Read/write threads...")
        self.thread1.start()
        self.thread2.start()
        self.threads_status = True
        time.sleep(0.5)
        print(f"Read/write threads started!\n")
        
    def _read_serial_thread(self):
        """
        Thread function to continuously read from the serial port, process incoming data,
        and monitor motor speed to trigger safety shutdown if necessary.
        """
        while True:
            data = self._read_serial()
            if data is not None:
                self._process_data(data)
                if (max(list(map(abs, self.motors_data["speed"]))) > self.max_speed + self.max_speed_allowed_overshoot) and self.active:
                    self.set_control_mode("torque")
                    self.set_target(0, 0)
                    self.active = False
                    print("Aborting: Motors reached maximum allowed speed, setting control mode to torque and setting targets to [0,0]")
        
    def _serial_write_thread(self):
        """
        Thread function to continuously send request and control messages over the serial link.
        """
        while True:
            for msg in self._get_request_messages():
                self._write_serial(msg)
                
            for msg in self._get_control_messages():
                self._write_serial(msg)
        
    def _write_serial(self, msg):
        """
        Writes a message to the serial link.

        Args:
            msg (list): Formatted list representing the message to send.
        """
        try:
            msg_size = self.link.tx_obj(msg)
            self.link.send(msg_size)
        except:
            import traceback
            traceback.print_exc()
            try:
                self.link.close()
                self.active = False
            except:
                pass
        
    def _read_serial(self):
        """
        Reads data from the serial link once.

        Returns:
            list or None: A list containing the received data if available, otherwise None.
        """
        data = None
        try:
            if self.link.available():
                data = self.link.rx_obj(obj_type=list, obj_byte_size=self.msg_size, list_format='i')
            else:
                if self.link.status.value < 0:
                    if self.link.status == txfer.Status.CRC_ERROR:
                        print('ERROR: CRC_ERROR')
                    elif self.link.status == txfer.Status.PAYLOAD_ERROR:
                        print('ERROR: PAYLOAD_ERROR')
                    elif self.link.status == txfer.Status.STOP_BYTE_ERROR:
                        print('ERROR: STOP_BYTE_ERROR')
                    else:
                        print('ERROR: {}'.format(self.link.status.name)) 
        except:
            import traceback
            traceback.print_exc()
            try:
                self.link.close()  
                self.active = False
            except:
                pass
        return data
           
    def _process_data(self, data):
        """
        Processes a data packet received from the serial link and updates motor attributes.

        Args:
            data (list): List containing the data packet received.
        """
        data_bytes = data[1:]
        
        if data[0] == self.motors_data["motor_ID"][0]:
            motor_id = 0
        elif data[0] == self.motors_data["motor_ID"][1]:
            motor_id = 1
        else:
            motor_id = None
        
        if data_bytes[0] == 0x9C:
            self.motors_data["temperature"][motor_id] = int.from_bytes(data_bytes[1:2], byteorder="little", signed=True)
            self.motors_data["current"][motor_id] = int.from_bytes(data_bytes[2:4], byteorder="little", signed=True) * 0.01  # 0.01A/LSB
            self.motors_data["speed"][motor_id] = int.from_bytes(data_bytes[4:6], byteorder="little", signed=True)  # 1dps/LSB
            self.motors_data["angle"][motor_id] = int.from_bytes(data_bytes[6:8], byteorder="little", signed=True)  # 1°/LSB
        elif data_bytes[0] == 0x9A:
            self.motors_data["brake_status"][motor_id] = int.from_bytes(data_bytes[3:4], byteorder="little", signed=True)   # 1/LSB
            self.motors_data["voltage"][motor_id] = int.from_bytes(data_bytes[4:6], byteorder="little", signed=True) * 0.1  # 0.1V/LSB
        elif data_bytes[0] == 0x92:
            self.motors_data["angle"][motor_id] = int.from_bytes(data_bytes[4:8], byteorder="little", signed=True) * 0.01 # 0.01°/LSB
        elif (data_bytes[0] == 0xA1) or (data_bytes[0] == 0xA4) or (data_bytes[0] == 0xA2):
            pass
        else:
            print("Unknown Information Received:", data)
            
        voltages = self.motors_data["voltage"]
        currents = self.motors_data["current"]
        motor_constants = self.motors_data["torque_constant"]
        
        self.motors_data["power"] = [voltages[0] * currents[0], voltages[1] * currents[1]]
        self.motors_data["torque"] = [motor_constants[0] * currents[0], motor_constants[1] * currents[1]]

    def _get_control_messages(self):
        """
        Constructs control messages based on the current control mode and target values.

        Returns:
            list: A list containing two control messages (one per motor).
        """
        target1 = int(self.target[0] / 0.01)
        target2 = int(self.target[1] / 0.01)
        
        if self.control_mode == self.control_modes[0]:
            message1 = [self.motors_data["motor_ID"][0], 0xA2, 0x00, 0x00, 0x00,
                        target1 & 0xFF, (target1 >> 8) & 0xFF, (target1 >> 16) & 0xFF, (target1 >> 24) & 0xFF]
            message2 = [self.motors_data["motor_ID"][1], 0xA2, 0x00, 0x00, 0x00,
                        target2 & 0xFF, (target2 >> 8) & 0xFF, (target2 >> 16) & 0xFF, (target2 >> 24) & 0xFF]
        elif self.control_mode == self.control_modes[1]:
            message1 = [self.motors_data["motor_ID"][0], 0xA4, 0x00, self.max_speed & 0xFF,
                        (self.max_speed >> 8) & 0xFF, target1 & 0xFF, (target1 >> 8) & 0xFF, (target1 >> 16) & 0xFF, (target1 >> 24) & 0xFF]
            message2 = [self.motors_data["motor_ID"][1], 0xA4, 0x00, self.max_speed & 0xFF,
                        (self.max_speed >> 8) & 0xFF, target2 & 0xFF, (target2 >> 8) & 0xFF, (target2 >> 16) & 0xFF, (target2 >> 24) & 0xFF]
        else:
            message1 = [self.motors_data["motor_ID"][0], 0xA1, 0x00, 0x00, 0x00,
                        target1 & 0xFF, (target1 >> 8) & 0xFF, 0x00, 0x00]
            message2 = [self.motors_data["motor_ID"][1], 0xA1, 0x00, 0x00, 0x00,
                        target2 & 0xFF, (target2 >> 8) & 0xFF, 0x00, 0x00]

        return [message1, message2]
            
    def _get_request_messages(self):
        """
        Constructs a set of request messages to poll the motor controllers for updated data.

        Returns:
            list: A list containing request messages for temperature, current, speed, angle, voltage, brake status, and angle update.
        """
        messages = [
            [self.motors_data["motor_ID"][0], 0x9c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [self.motors_data["motor_ID"][1], 0x9c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [self.motors_data["motor_ID"][0], 0x9a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [self.motors_data["motor_ID"][1], 0x9a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [self.motors_data["motor_ID"][0], 0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [self.motors_data["motor_ID"][1], 0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ]
        
        return messages
    
    def _write_motor_zero_command(self):
        """
        Sends a command to zero the motor encoders. This function can only be executed when the
        communication threads are not running.
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
        Constructs messages to zero the motors' encoders.

        Returns:
            list: A list of messages to send to each motor for zeroing.
        """
        return [
            [self.motors_data["motor_ID"][0], 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [self.motors_data["motor_ID"][1], 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ]
    
    def _get_system_reset_message(self):
        """
        Constructs messages to reset the system controllers.

        Returns:
            list: A list of system reset messages for each motor.
        """
        return [
            [self.motors_data["motor_ID"][0], 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [self.motors_data["motor_ID"][1], 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ]
