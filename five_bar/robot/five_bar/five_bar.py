import serial
import time
import threading

class five_bar:
    def __init__(self,
                 SERIAL_PORT="COM1",
                 BAUD_RATE=115200, 
                 START_BYTE=0xAB,
                 STOP_BYTE = 0xBA,
                 torque_constant1=0.45,
                 torque_constant2=0.45,
                 pos_max_speed=int(1e6),
                 motor1_CANID="0x241",
                 motor2_CANID="0x242"):
        
        self.SERIAL_PORT=SERIAL_PORT
        self.BAUD_RATE=BAUD_RATE
        self.START_BYTE=START_BYTE
        self.STOP_BYTE=STOP_BYTE
        
        self.motors_data={
            "voltage":[0,0],
            "current":[0,0],
            "speed":[0,0],
            "angle":[0,0],
            "temperature":[0,0],
            "brake_status":[0,0],
            "power":[0,0],
            "torque":[0,0],
            "torque_constant":[torque_constant1,torque_constant2],
            "motor_ID":[motor1_CANID,motor2_CANID]
            }
        
        self.target=[0,0]
        
        self.control_modes=["speed","position","torque"]
        
        self.control_mode=self.control_modes[2]
        
        self.pos_max_speed=pos_max_speed
        
        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=1)
        
        self.thread1=threading.Thread(target=self._read_serial, daemon=True)
        self.thread2=threading.Thread(target=self._write_serial, daemon=True)
        
        self.thread1.start()
        self.thread2.start()
        
        print(f"Initialized 5 bar mechanism \n ")
    
    def get_motors_data(self):
        """Getter function for motor data

        Returns:
           dict: Dictionary containing all of the motor data
        """
        return self.motors_data
    
    def change_mode(self, mode):
        """Changes the control mode of the motors

        Args:
            mode (str): Control mode of the motors. Must be a valid control mode.
        """
        if mode in self.control_modes:
            self.target=[0,0]
            self.control_mode=mode
        else:
            print("Error: Mode",mode,"is not a valid mode")
    
    def set_target(self,target1,target2):
        """Setter function for the target values

        Args:
            target1 (float): Target of motor 1
            target2 (float): Target of motor 2
        """
        self.target=[target1,target2]
    
    def _read_serial(self):
        """Reads the serial port and processes the data. This function is meant to be run as a thread.
        """
        while True:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if not line:
                        continue
                    self._process_line(line)

            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.ser.close()
                
    def _process_line(self,line):
        """Processes a line of data

        Args:
            line (str): String containing the data

        Raises:
            Exception: Motor ID received is not valid
            Exception: The information received is not recognized
        """
        parts = line.split()  
        
        if ("Data:" in parts) and ("ID:" in parts) and (len(parts)==14): 
            
            data_index = parts.index("Data:") + 1
            id_index=parts.index("ID:") + 1
            
            data_bytes = bytes(int(h, 16) for h in parts[data_index:data_index + 8]) 
            
            if parts[id_index]==self.motors_data["motor_ID"][0]:
                motor_id = 0
            elif parts[id_index]==self.motors_data["motor_ID"][1]:
                motor_id = 1
            else:
                raise Exception("Motor ID error")
            
            if data_bytes[0] == 0x9C:
                self.motors_data["temperature"][motor_id] = int.from_bytes(data_bytes[1:2], byteorder="little", signed=True)
                self.motors_data["current"][motor_id] = int.from_bytes(data_bytes[2:4], byteorder="little", signed=True) * 0.01  # 0.01A/LSB
                self.motors_data["speed"][motor_id] = int.from_bytes(data_bytes[4:6], byteorder="little", signed=True)  # 1dps/LSB
                self.motors_data["angle"][motor_id] = int.from_bytes(data_bytes[6:8], byteorder="little", signed=True)  # 1°/LSB
            elif data_bytes[0] == 0x9A:
                self.motors_data["brake_status"][motor_id] = int.from_bytes(data_bytes[3:4], byteorder="little", signed=True)   # 1/LSB
                self.motors_data["voltage"][motor_id] = int.from_bytes(data_bytes[4:6], byteorder="little", signed=True)*0.1  # 0.1V/LSB
            elif data_bytes[0] == 0x92:
                self.motors_data["angle"][motor_id] = int.from_bytes(data_bytes[4:8], byteorder="little", signed=True) * 0.01 # 0.01°/LSB
            elif (data_bytes[0] == 0xA1) or (data_bytes[0] == 0xA4) or (data_bytes[0] == 0xA2):
                pass
            else:
                print("Unknown Information Received:",data_bytes)
            
            voltages=self.motors_data["voltage"]
            currents=self.motors_data["current"]
            motor_constants=self.motors_data["torque_constant"]
            
            self.motors_data["power"]=[voltages[0]*currents[0],voltages[1]*currents[1]]
            self.motors_data["torque"]=[motor_constants[0]*currents[0],motor_constants[1]*currents[1]]

        else:
            print("Arduino message:",line)                
            
    def _write_serial(self):
        """Writes in the serial port the targets and the requests for motor information. This function is meant to be run as a thread.
        """
        while True:
            time.sleep(0.001)  
            try:
                target1=int(self.target[0]/0.01)
                target2=int(self.target[1]/0.01)
                
                if self.control_mode==self.control_modes[0]:
                    message1 = [0xA2, 0x00, 0x00, 0x00,target1 & 0xFF,(target1 >> 8) & 0xFF,(target1 >> 16) & 0xFF,(target1 >> 24) & 0xFF]
                    message2 = [0xA2, 0x00, 0x00, 0x00,target2 & 0xFF,(target2 >> 8) & 0xFF,(target2 >> 16) & 0xFF,(target2 >> 24) & 0xFF]
                elif self.control_mode==self.control_modes[1]:
                    message1 = [0xA4,0x00,self.pos_max_speed & 0xFF,(self.pos_max_speed >> 8) & 0xFF,target1 & 0xFF,(target1 >> 8) & 0xFF,(target1 >> 16) & 0xFF,(target1 >> 24) & 0xFF]
                    message2 = [0xA4,0x00,self.pos_max_speed & 0xFF,(self.pos_max_speed >> 8) & 0xFF,target2 & 0xFF,(target2 >> 8) & 0xFF,(target2 >> 16) & 0xFF,(target2 >> 24) & 0xFF]
                else:
                    message1 = [0xA1, 0x00, 0x00, 0x00,target1 & 0xFF,(target1 >> 8) & 0xFF,0x00,0x00]
                    message2 = [0xA1, 0x00, 0x00, 0x00,target2 & 0xFF,(target2 >> 8) & 0xFF,0x00,0x00]
                    
                packet = bytes([self.START_BYTE] + message1 + message2 + [self.STOP_BYTE])
                self.ser.write(packet)
                
                message_9c = [0x9c, 0x00, 0x00,0x00,0x00,0x00,0x00,0x00]
                
                packet = bytes([self.START_BYTE] + message_9c + message_9c + [self.STOP_BYTE])
                self.ser.write(packet)
                
                message_9a = [0x9a, 0x00, 0x00,0x00,0x00,0x00,0x00,0x00]

                packet = bytes([self.START_BYTE] + message_9a + message_9a + [self.STOP_BYTE])
                self.ser.write(packet)
                
                message_92 = [0x92, 0x00, 0x00,0x00,0x00,0x00,0x00,0x00]

                packet = bytes([self.START_BYTE] + message_92 + message_92 + [self.STOP_BYTE])
                self.ser.write(packet)

            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.ser.close()
        