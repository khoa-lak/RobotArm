import serial
import threading
import queue
import time

class SerialDriver:
    def __init__(self, port="None", baudrate=9600, feedback_callback=None):
        self.port = port
        self.baudrate = baudrate
        self.feedback_callback = feedback_callback
        
        self.ser = None
        self.running = False
        self.write_queue = queue.Queue()
        
        self.thread_read = None
        self.thread_write = None

    def connect(self, port=None):
        """Connects to the microcontroller COM port."""
        if port is not None:
            self.port = port
            
        if self.port == "None" or not self.port:
            print("[INFO] Offline mode: No COM port selected.")
            return False
            
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.ser.flushInput()
            self.ser.flushOutput()
            
            self.running = True
            
            # Start background worker threads
            self.thread_read = threading.Thread(target=self._read_loop, daemon=True)
            self.thread_read.start()
            
            self.thread_write = threading.Thread(target=self._write_loop, daemon=True)
            self.thread_write.start()
            
            print(f"[INFO] Connected to serial port {self.port} at {self.baudrate} baud.")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to connect to serial port {self.port}: {e}")
            self.ser = None
            return False

    def disconnect(self):
        """Disconnects and clean up threads."""
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        print("[INFO] Serial connection closed.")

    def send_command(self, cmd_str):
        """Enqueues a command string to be sent to the robot."""
        if not cmd_str.endswith("\n"):
            cmd_str += "\n"
            
        if self.ser and self.running:
            self.write_queue.put(cmd_str)
        else:
            # Offline mock output
            print(f"[OFFLINE TX]: {cmd_str.strip()}")
            # Mock successful execution reply
            if self.feedback_callback:
                # Run callback after brief delay to simulate network latency
                threading.Timer(0.1, lambda: self.feedback_callback("Done")).start()

    def _write_loop(self):
        """Background thread writing queued commands to the serial port."""
        while self.running and self.ser:
            try:
                # Non-blocking check with timeout
                cmd = self.write_queue.get(timeout=0.1)
                self.ser.write(cmd.encode('utf-8'))
                self.ser.flush()
                print(f"[TX]: {cmd.strip()}")
                self.write_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[ERROR] Serial write error: {e}")
                self.running = False

    def _read_loop(self):
        """Background thread reading feedback responses from the serial port."""
        while self.running and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        print(f"[RX]: {line}")
                        if self.feedback_callback:
                            self.feedback_callback(line)
                else:
                    time.sleep(0.01) # Avoid burning CPU cycles
            except Exception as e:
                print(f"[ERROR] Serial read error: {e}")
                self.running = False
                
    def send_move_command(self, joints, speed=50, acc=50, dec=50):
        """Sends a standard Move Joint command (MJ) containing target coordinates and parameters.
        joints: list of 6 values [J1, J2, J3, J4, J5, J6]
        """
        # Format command matching AR4 protocol:
        # MJX...Y...Z...Rz...Ry...Rx...J7...S...Ac...Dc...Rm...W...Lm...
        cmd = f"MJJ1{joints[0]:.2f}J2{joints[1]:.2f}J3{joints[2]:.2f}J4{joints[3]:.2f}J5{joints[4]:.2f}J6{joints[5]:.2f}S{speed}Ac{acc}Dc{dec}\n"
        self.send_command(cmd)
