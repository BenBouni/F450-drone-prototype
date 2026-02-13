from PyQt6.QtCore import QThread, pyqtSignal
import serial
import numpy as np
import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QPushButton
import pyqtgraph as pg

# Initialize data buffer

class SerialWorker5(QThread):
    data_received = pyqtSignal(list)  # Signal to emit received data


    def __init__(self, port='com3', baudrate=115200): #adjust port and baudrate as needed for object self 
        super().__init__()
        self.running = True
        self.ser = None
        # GS connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Error connceting to GS: {e}")

        self.buffer_size = 100
        self.roll_buffer = np.zeros(self.buffer_size)
        self.pitch_buffer = np.zeros(self.buffer_size)
        self.yaw_buffer = np.zeros(self.buffer_size)
        self.data_buffer = np.zeros(self.buffer_size)

    def run(self):
        while True:
            if not self.ser : 
                return # Exit if serial connection failed
            break

        while self.running: #data acquisition loop
            if self.ser.in_waiting > 0:
                try:
                    #brut data aquisition
                    data = self.ser.readline().decode('utf-8').strip()
                    #process only GS data
                    if data.startswith('GS'):
                        self.split_data = data.split(',')
                    #process only if enough data    
                        if len(self.split_data) >= 5:
                           
                            try:
                                roll, pitch, yaw = float(self.split_data[1]), float(self.split_data[2]), float(self.split_data[3])
                                data = self.split_data[4]  # GS data is in the 5th
                                valeur = float(data)
                                #update buffers            
                                self.data_buffer = np.roll(self.data_buffer, -1)
                                self.data_buffer[-1] = valeur

                                self.roll_buffer = np.roll(self.roll_buffer, -1)
                                self.roll_buffer[-1] = roll

                                self.pitch_buffer = np.roll(self.pitch_buffer, -1)
                                self.pitch_buffer[-1] = pitch

                                self.yaw_buffer = np.roll(self.yaw_buffer, -1)
                                self.yaw_buffer[-1] = yaw

                                #calculate stats
                                #data
                                moyenne = np.mean(self.data_buffer)
                                moyenne_roll = np.mean(self.roll_buffer) #roll
                                moyenne_pitch = np.mean(self.pitch_buffer) #pitch
                                moyenne_yaw = np.mean(self.yaw_buffer) #yaw
                                #ecart type
                                ecart_type = np.std(self.data_buffer) 
                                ecart_type_roll = np.std(self.roll_buffer) #roll
                                ecart_type_pitch = np.std(self.pitch_buffer) #pitch
                                ecart_type_yaw = np.std(self.yaw_buffer) #yaw
                                #emit data
                                paquet = [
                                moyenne, ecart_type, valeur, 
                                roll, pitch, yaw,
                                moyenne_roll, moyenne_pitch, moyenne_yaw,
                                ecart_type_roll, ecart_type_pitch, ecart_type_yaw
                                ]
                                self.data_received.emit(paquet) # UN SEUL EMIT
                               
                            except ValueError:
                                print("corrupted data for roll, pitch, yaw")
                            
                except ValueError :
                    continue

                    
    def stop(self):
        self.running = False #BREAK LOOP
        if self.isRunning() : #true if thread is running
            self.wait() #wait for thread to finish
        if self.ser is not None: #close serial connection
            self.ser.close() #close serial connection

class GSMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GS Data Monitor")
        self.setGeometry(100, 100, 800, 600)

        self.central_widget = QWidget() 
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget) 
#Valeur
        self.label_moyenne = QLabel("Moyenne: N/A")
        self.label_ecart_type = QLabel("Ecart Type: N/A")
        self.label_valeur = QLabel("Valeur: N/A")

        self.layout.addWidget(self.label_moyenne)
        self.layout.addWidget(self.label_ecart_type)
        self.layout.addWidget(self.label_valeur)

        self.plot_widget = pg.PlotWidget(title="GS Data Plot")
        self.layout.addWidget(self.plot_widget)
        self.plot_data_curve = self.plot_widget.plot(pen='y')

#Roll
        self.label_moyenne_roll = QLabel("Moyenne Roll: N/A")
        self.label_ecart_type_roll = QLabel("Ecart Type Roll: N/A")
        self.label_roll = QLabel("Roll: N/A")

        self.layout.addWidget(self.label_moyenne_roll)
        self.layout.addWidget(self.label_ecart_type_roll)
        self.layout.addWidget(self.label_roll)

        self.curve_roll = self.plot_widget.plot(pen='r')  # roll
#Pitch
        self.label_moyenne_pitch = QLabel("Moyenne Pitch: N/A")
        self.label_ecart_type_pitch = QLabel("Ecart Type Pitch: N/A")
        self.label_pitch = QLabel("Pitch: N/A")

        self.layout.addWidget(self.label_moyenne_pitch)
        self.layout.addWidget(self.label_ecart_type_pitch)
        self.layout.addWidget(self.label_pitch)

        self.curve_pitch = self.plot_widget.plot(pen='g')  # pitch
#Yaw
        self.label_moyenne_yaw = QLabel("Moyenne Yaw: N/A")
        self.label_ecart_type_yaw = QLabel("Ecart Type Yaw: N/A")
        self.label_yaw = QLabel("Yaw: N/A")
        
        self.layout.addWidget(self.label_moyenne_yaw)
        self.layout.addWidget(self.label_ecart_type_yaw)
        self.layout.addWidget(self.label_yaw)

        self.curve_yaw = self.plot_widget.plot(pen='b')  # yaw

#controll buttons

        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start_acquisition)
        self.layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_acquisition)
        self.layout.addWidget(self.stop_button)

        self.worker = SerialWorker5()
        self.worker.data_received.connect(self.update_telemetry)


    def start_acquisition(self):
        self.worker.start()

    def stop_acquisition(self):
        self.worker.stop()

    def update_telemetry(self, data):
        #update labels
        self.label_moyenne.setText(f"Moyenne: {data[0]:.2f}")
        self.label_roll.setText(f"Roll: {data[3]:.2f}")
        self.label_pitch.setText(f"Pitch: {data[4]:.2f}")
        self.label_yaw.setText(f"Yaw: {data[5]:.2f}")
        #update plots
        self.plot_data_curve.setData(self.worker.data_buffer)
        self.curve_roll.setData(self.worker.roll_buffer)
        self.curve_pitch.setData(self.worker.pitch_buffer)
        self.curve_yaw.setData(self.worker.yaw_buffer)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GSMainWindow()
    window.show()
    sys.exit(app.exec())