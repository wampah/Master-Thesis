from pySerialTransfer import pySerialTransfer as txfer
import time
import threading

class five_bar:
    def __init__(self,
                 SERIAL_PORT="COM1",
                 BAUD_RATE=115200, 
                 msg_size=int((8+1)*4), #8 bytes for msg, 1 for id - times 4 since pytransfer interprets bytes as 4 byte ints
                 torque_constant1=0.45,
                 torque_constant2=0.45,
                 max_speed=1e3,
                 max_speed_allowed_overshoot=250,
                 motor1_ID=0x00,
                 motor2_ID=0x01):
        
        print("\nInitializing Robot...\n")
        
        self.SERIAL_PORT=SERIAL_PORT
        self.BAUD_RATE=BAUD_RATE
        
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
            "motor_ID":[motor1_ID,motor2_ID]
            }
                
        self.abort_flag=False
        
        self.target=[0,0]
        
        self.control_modes=["speed","position","torque"]
        
        self.control_mode=self.control_modes[2]
        
        self.max_speed=int(max_speed)
        
        self.max_speed_allowed_overshoot=max_speed_allowed_overshoot
        
        if self.max_speed>2**16-1:
            raise Exception(f"Maximum speed out of range. Value was {self.max_speed} and maximum is {2**16-1}")
        
        print(f"Connecting to Serial Port {self.SERIAL_PORT}...")

        self.link = txfer.SerialTransfer('COM15', baud=115200)
        
        self.msg_size=msg_size
        
        self.link.open()
        
        time.sleep(1)
        
        print(f"Successfully connected to Serial Port {self.SERIAL_PORT}!\n")

        print(f"Initializing read/write threads...")
        
        self.thread1=threading.Thread(target=self._read_serial, daemon=True)
        self.thread2=threading.Thread(target=self._write_serial, daemon=True)
        
        self.thread1.start()
        self.thread2.start()
        
        print(f"Read/write threads started!\n")
        
        print(f"Robot Ready!\n ")
    
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
            raise Exception("Error: Mode",mode,"is not a valid mode")
    
    def set_target(self,target1,target2):
        """Setter function for the target values

        Args:
            target1 (float): Target of motor 1
            target2 (float): Target of motor 2
        """
        if not self.abort_flag:
            self.target=[target1,target2]
    
    def _read_serial(self):
        """Reads the serial port and processes the data. This function is meant to be run as a thread.
        """
        while True:
            try:
                if self.link.available():
                    data=self.link.rx_obj(obj_type=list, obj_byte_size=self.msg_size, list_format='i')
                    self._process_data(data)
                else:
                    if self.link.status.value < 0:
                        if self.link.status == txfer.Status.CRC_ERROR:
                            print('ERROR: CRC_ERROR')
                        elif self.link.status == txfer.Status.PAYLOAD_ERROR:
                            self.print('ERROR: PAYLOAD_ERROR')
                        elif self.link.status == txfer.Status.STOP_BYTE_ERROR:
                            print('ERROR: STOP_BYTE_ERROR')
                        else:
                            print('ERROR: {}'.format(self.link.status.name))
            
            except:
                    import traceback
                    traceback.print_exc()
                    
                    try:
                        self.link.close()  
                    except:
                        pass

           
    def _process_data(self,data):
        """Processes a line of data

        Args:
            line (str): String containing the data

        Raises:
            Exception: Motor ID received is not valid
            Exception: The information received is not recognized
        """
      
        data_bytes = data[1:]
        
        if data[0]==self.motors_data["motor_ID"][0]:
            motor_id = 0
        elif data[0]==self.motors_data["motor_ID"][1]:
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
            
        if (max(list(map(abs, self.motors_data["speed"])))>self.max_speed+self.max_speed_allowed_overshoot) and (not self.abort_flag):

            self.abort_flag=True
            self.change_mode("torque")
            self.set_target(0,0)
            print("Aborting: Motors reached maximum allowed speed, setting control mode to torque and setting targets to [0,0]")
        
        voltages=self.motors_data["voltage"]
        currents=self.motors_data["current"]
        motor_constants=self.motors_data["torque_constant"]
        
        self.motors_data["power"]=[voltages[0]*currents[0],voltages[1]*currents[1]]
        self.motors_data["torque"]=[motor_constants[0]*currents[0],motor_constants[1]*currents[1]]

            
            
    def _write_serial(self):
        """Writes in the serial port the targets and the requests for motor information. This function is meant to be run as a thread.
        """
        while True:
            try:
                target1=int(self.target[0]/0.01)
                target2=int(self.target[1]/0.01)
                
                if self.control_mode==self.control_modes[0]:
                    message1 = [self.motors_data["motor_ID"][0],0xA2, 0x00, 0x00, 0x00,target1 & 0xFF,(target1 >> 8) & 0xFF,(target1 >> 16) & 0xFF,(target1 >> 24) & 0xFF]
                    message2 = [self.motors_data["motor_ID"][1],0xA2, 0x00, 0x00, 0x00,target2 & 0xFF,(target2 >> 8) & 0xFF,(target2 >> 16) & 0xFF,(target2 >> 24) & 0xFF]
                elif self.control_mode==self.control_modes[1]:
                    message1 = [self.motors_data["motor_ID"][0],0xA4,0x00,self.max_speed & 0xFF,(self.max_speed >> 8) & 0xFF,target1 & 0xFF,(target1 >> 8) & 0xFF,(target1 >> 16) & 0xFF,(target1 >> 24) & 0xFF]
                    message2 = [self.motors_data["motor_ID"][1],0xA4,0x00,self.max_speed & 0xFF,(self.max_speed >> 8) & 0xFF,target2 & 0xFF,(target2 >> 8) & 0xFF,(target2 >> 16) & 0xFF,(target2 >> 24) & 0xFF]
                else:
                    message1 = [self.motors_data["motor_ID"][0],0xA1, 0x00, 0x00, 0x00,target1 & 0xFF,(target1 >> 8) & 0xFF,0x00,0x00]
                    message2 = [self.motors_data["motor_ID"][1],0xA1, 0x00, 0x00, 0x00,target2 & 0xFF,(target2 >> 8) & 0xFF,0x00,0x00]
                    
                messages=[[self.motors_data["motor_ID"][0],0x9c, 0x00, 0x00,0x00,0x00,0x00,0x00,0x00],[self.motors_data["motor_ID"][1],0x9c, 0x00, 0x00,0x00,0x00,0x00,0x00,0x00],
                          [self.motors_data["motor_ID"][0],0x9a, 0x00, 0x00,0x00,0x00,0x00,0x00,0x00],[self.motors_data["motor_ID"][1],0x9a, 0x00, 0x00,0x00,0x00,0x00,0x00,0x00],
                          [self.motors_data["motor_ID"][0],0x92, 0x00, 0x00,0x00,0x00,0x00,0x00,0x00],[self.motors_data["motor_ID"][1],0x92, 0x00, 0x00,0x00,0x00,0x00,0x00,0x00],
                          message1,message2]
                
                for msg in messages:
                    msg_size=self.link.tx_obj(msg)
                    self.link.send(msg_size)

            except:
                import traceback
                traceback.print_exc()
                
                try:
                    self.link.close()
                    
                except:
                    pass
        