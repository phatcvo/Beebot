
import serial
import time

class SerialComm1:
  
  def __init__(self, port, baud=115200, timeOut=0.1):
    self.ser = serial.Serial(port, baud, timeout=timeOut)

  def send_msg(self, msg_to_send):
    data = ""
    prev_time = time.time()
    while data=="":
      try:
        # self.ser.write(msg_to_send.encode())   # send a single or multiple byte    
        self.ser.write((msg_to_send + "\n").encode())
        data = self.ser.readline().decode().strip()
        if time.time()-prev_time > 2.0:
          raise Exception("Error getting response from arduino, wasted much time \n")
      except:
        raise Exception("Error getting response from arduino, wasted much time \n")
    return data

  
  def send(self, cmd_route, val1=0, val2=0, val3=0, val4=0):
    cmd_str = cmd_route+","+str(val1)+","+str(val2)+","+str(val3)+","+str(val4)
    data = self.send_msg(cmd_str)
    if data == "1":
      return True
    else:
      return False
  
  
  def get(self, cmd_route):
    data = self.send_msg(cmd_route).split(',')
    if len(data)==1:
      return float(data[0])
    elif len(data)==2:
      return float(data[0]), float(data[1])
    elif len(data)==3:
      return float(data[0]), float(data[1]), float(data[2])
    elif len(data)==4:
      return float(data[0]), float(data[1]), float(data[2]), float(data[3])

class SerialComm:
    def __init__(self, port, baud=115200, timeOut=0.1):
        try:
            self.ser = serial.Serial(port, baud, timeout=timeOut)
            print(f"[INFO] Serial port {port} opened at {baud} baud.")
        except serial.SerialException as e:
            raise Exception(f"[ERROR] Could not open serial port {port}: {str(e)}")

    def send_msg(self, msg_to_send):
        prev_time = time.time()
        self.ser.reset_input_buffer()  # Clear any old data
        self.ser.write((msg_to_send + "\n").encode())  # Ensure newline is sent

        while True:
            if time.time() - prev_time > 2.0:
                raise TimeoutError("Timeout: No response from Arduino.")

            try:
                line = self.ser.readline().decode().strip()
                if line:
                    return line
            except UnicodeDecodeError:
                continue  # Ignore bad characters and retry
            except Exception as e:
                raise Exception(f"Serial read error: {str(e)}")

    def send(self, cmd_route, val1=0, val2=0, val3=0, val4=0):
        cmd_str = f"{cmd_route},{val1},{val2},{val3},{val4}"
        data = self.send_msg(cmd_str)
        return data.strip() == "1"

    def get(self, cmd_route):
        while True:
            raw = self.send_msg(cmd_route)
            if raw == cmd_route:
                continue  # Skip echo line
            try:
                data = list(map(float, raw.split(',')))
                return data[0] if len(data) == 1 else tuple(data)
            except ValueError:
                raise ValueError(f"Non-numeric or unexpected response: '{raw}'")

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            print("[INFO] Serial port closed.")

    def __repr__(self):
        return f"<SerialComm port={self.ser.port} baudrate={self.ser.baudrate}>"