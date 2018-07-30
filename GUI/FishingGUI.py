# !/bin/env/python
# The FishingGUI Software. Digital Forensic Data from heavy trucks.

from PyQt5.QtWidgets import (QMainWindow,
                             QWidget,
                             QTreeView,
                             QMessageBox,
                             QFileDialog,
                             QRadioButton,
                             QLabel,
                             QSlider,
                             QCheckBox,
                             QLineEdit,
                             QVBoxLayout,
                             QFormLayout,
                             QApplication,
                             QPushButton,
                             QButtonGroup,
                             QTableWidget,
                             QTableView,
                             QTableWidgetItem,
                             QScrollArea,
                             QAbstractScrollArea,
                             QAbstractItemView,
                             QSizePolicy,
                             QGridLayout,
                             QGroupBox,
                             QComboBox,
                             QAction,
                             QDockWidget,
                             QDialog,
                             QFrame,
                             QDialogButtonBox,
                             QInputDialog,
                             QProgressDialog,
                             QLCDNumber,
                             QTabWidget)
from PyQt5.QtCore import Qt, QTimer, QAbstractTableModel, QCoreApplication, QSize
from PyQt5.QtGui import QIcon

from ctypes import *
import threading
import queue
import time
import sys
import struct
import json
import humanize

from RP1210 import *
from RP1210Functions import *
from RP1210Select import *
from graphing import *

import logging
import logging.config

class FishingGUI(QMainWindow):
    def __init__(self):
        super(FishingGUI, self).__init__()
        
        os.system("TASKKILL /F /IM DGServer2.exe")
        os.system("TASKKILL /F /IM DGServer1.exe")  
        
        self.title = "Fishing GUI"
        self.setWindowTitle(self.title)
        self.top_line_text = "0123456789ABCDEF"
        self.bot_line_text = "0123456789ABCDEF"
        self.goalAngleData = []
        self.ekfYawData = []
        self.motorData = []
        self.GPSheadingData = []
        self.compassData = []
        self.update_rate = 250
        self.gpsSpeed = 0
        self.gpsCourse = 0
        self.network_connected = {"CAN": False}
        self.RP1210 = None
        self.init_ui()
        self.RP1210_toolbar = None
        self.selectRP1210(automatic=True)
        self.ok_to_send = False

        read_timer = QTimer(self)
        read_timer.timeout.connect(self.read_rp1210)
        read_timer.start(self.update_rate) #milliseconds
        
        connection_timer = QTimer(self)
        connection_timer.timeout.connect(self.check_connections)
        connection_timer.start(500) #milliseconds
        
        send_timer = QTimer(self)
        send_timer.timeout.connect(self.send_command)
        send_timer.start(50)

    def init_ui(self):
        # Builds GUI
        # Start with a status bar
        self.statusBar().showMessage("Welcome!")
        self.grid_layout = QGridLayout()
        
        # Build common menu options
        menubar = self.menuBar()
        
        # RP1210 Menu Items
        self.rp1210_menu = menubar.addMenu('&RP1210')
       
        self.graph_menu = menubar.addMenu('&Graph')
        
        graph_action = QAction(QIcon(r'icons/icons8_Line_Chart_48px.png'), 'Show G&raphs', self)
        graph_action.setShortcut('Alt+Shift+R')
        graph_action.setStatusTip('Show Available Graphs')
        graph_action.triggered.connect(self.show_graphs)
        self.graph_menu.addAction(graph_action)
            
        clear_voltage_action = QAction(QIcon(r'icons/icons8_Delete_Table_48px_1.png'), '&Clear Voltage Graphs', self)
        clear_voltage_action.setShortcut('Alt+Shift+C')
        clear_voltage_action.setStatusTip('Clear the time history shown in the vehicle voltage graph.')
        clear_voltage_action.triggered.connect(self.clear_voltage_graph)
        self.graph_menu.addAction(clear_voltage_action)
        
        # Setup the network status windows for logging
        info_box = {}
        info_box_area = {}
        info_layout = {}
        info_box_area_layout = {}
        self.previous_count = {}
        self.status_icon = {}
        self.previous_count = {}
        self.message_count_label = {}
        self.message_rate_label = {}
        self.message_duration_label = {}
        key = "CAN"
        info_box_CAN = QFrame()
        info_layout_CAN = QVBoxLayout()
        info_layout_CAN.setAlignment(Qt.AlignTop)
        info_box_CAN.setLayout(info_layout_CAN)
        self.status_icon_CAN = QLabel("<html><img src='icons/icons8_Unavailable_48px.png'><br>Network<br>Unavailable</html>")
        self.status_icon_CAN.setAlignment(Qt.AlignCenter)
        

        self.previous_count[key] = 0
        self.message_count_label[key] = QLabel("Count: 0")
        self.message_count_label[key].setAlignment(Qt.AlignCenter)
        
        #self.message_duration = 0
        self.message_duration_label[key] = QLabel("Duration: 0 sec.")
        self.message_duration_label[key].setAlignment(Qt.AlignCenter)
        
        #self.message_rate = 0
        self.message_rate_label[key] = QLabel("Rate: 0 msg/sec")
        self.message_rate_label[key].setAlignment(Qt.AlignCenter)
        
        
        info_layout_CAN.addWidget(QLabel("<html><h3>{} Status</h3></html>".format(key)))
        info_layout_CAN.addWidget(self.status_icon_CAN)
        info_layout_CAN.addWidget(self.message_count_label[key])
        info_layout_CAN.addWidget(self.message_rate_label[key])
        info_layout_CAN.addWidget(self.message_duration_label[key])

        self.topDisplayLine = QLabel()
        display_font = self.topDisplayLine.font()
        display_font.setPointSize(20)
        self.topDisplayLine.setFont(display_font)
        self.bottomDisplayLine = QLabel()
        self.bottomDisplayLine.setFont(display_font)
        info_layout_CAN.addWidget(QLabel("CAN Messages"))
        info_layout_CAN.addWidget(self.topDisplayLine)
        info_layout_CAN.addWidget(self.bottomDisplayLine)
        
        self.upButton = QPushButton("Up")
        button_font = self.upButton.font()
        button_font.setPointSize(36)
        self.upButton.setFont(button_font)
        
        self.downButton = QPushButton("Down")
        self.downButton.setFont(button_font)
        
        self.leftButton = QPushButton("  Left  ")
        self.leftButton.setFont(button_font)
        
        self.rightButton = QPushButton("  Right  ")
        self.rightButton.setFont(button_font)
        
        self.leftTurnButton = QPushButton("Left Turn")
        self.leftTurnButton.setFont(button_font)
        
        self.rightTurnButton = QPushButton("Right Turn")
        self.rightTurnButton.setFont(button_font)
        
        self.goStraightButton = QPushButton("Go Straight")
        self.goStraightButton.setFont(button_font)
        
        self.speedZeroButton = QPushButton("Zero Speed")
        self.speedZeroButton.setFont(button_font)
        
        self.startButton = QPushButton("Start")
        self.startButton.setFont(button_font)
        
        self.stopButton = QPushButton("Stop")
        self.stopButton.setFont(button_font)
        
        self.modeGroup = QButtonGroup()
        
        self.offButton = QRadioButton("Off")
        self.offButton.setFont(button_font)
        self.offButton.toggled.connect(self.setMode)
        self.modeGroup.addButton(self.offButton,0)

        self.normalButton = QRadioButton("Normal")
        self.normalButton.setFont(button_font)
        self.normalButton.toggled.connect(self.setMode)
        self.modeGroup.addButton(self.normalButton,1)

        self.fig8Button = QRadioButton("Figure 8")
        self.fig8Button.setFont(button_font)
        self.fig8Button.toggled.connect(self.setMode)
        self.modeGroup.addButton(self.fig8Button,2)

        self.fullButton = QRadioButton("Full On")
        self.fullButton.setFont(button_font)
        self.fullButton.toggled.connect(self.setMode)
        self.modeGroup.addButton(self.fullButton,3)

        self.calibrationButton = QRadioButton("Calibration")
        self.calibrationButton.setFont(button_font)
        self.calibrationButton.toggled.connect(self.setMode)
        self.modeGroup.addButton(self.calibrationButton,4)

        self.heading_graph = GraphDialog(self, title="Boat Heading")
        #self.heading_graph.set_yrange(0, 360)
        self.heading_graph.set_xlabel("Time")
        self.heading_graph.set_ylabel("Angle")
        self.heading_graph.set_title("Boat Heading and Goal")

        self.speed_graph = GraphDialog(self, title="Boat Speed")
        #self.speed_graph.set_yrange(0, 15)
        self.speed_graph.set_xlabel("Time")
        self.speed_graph.set_ylabel("Angle")
        self.speed_graph.set_title("Boat Heading and Goal")

        self.position_graph = GraphDialog(self, title="Boat Heading")
        self.position_graph.set_yrange(9, 15)
        self.position_graph.set_xlabel("Time")
        self.position_graph.set_ylabel("Angle")
        self.position_graph.set_title("Boat Heading and Goal")

        graph_box = QFrame()
        graph_box_layout = QGridLayout()
        graph_box.setLayout(graph_box_layout)
        graph_box_layout.addWidget(self.heading_graph,0,0,1,1)
        graph_box_layout.addWidget(self.speed_graph,0,1,1,1)
        graph_box_layout.addWidget(self.position_graph,0,2,1,1)
        #graph_box_layout.addWidget(self.sensor_graph,1,1,1,1)

        parameter_box = QFrame()
        parameter_box_layout = QFormLayout()
        parameter_box.setLayout(parameter_box_layout)
        self.GPScourseLabel = QLCDNumber()
        parameter_box_layout.addRow("GPS Course:", self.GPScourseLabel)
        
        parameter_box_layout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)


        self.grid_layout.addWidget(info_box_CAN,0,0,5,1)
        
        self.grid_layout.addWidget(self.startButton,0,1,1,1)
        self.grid_layout.addWidget(self.stopButton,4,1,1,1)
        self.grid_layout.addWidget(self.goStraightButton,0,3,1,1)
        self.grid_layout.addWidget(self.upButton,1,3,1,1)
        self.grid_layout.addWidget(self.downButton,3,3,1,1)
        self.grid_layout.addWidget(self.speedZeroButton,4,3,1,1)
        self.grid_layout.addWidget(self.rightTurnButton,2,5,1,1)
        self.grid_layout.addWidget(self.rightButton,2,4,1,1)
        self.grid_layout.addWidget(self.leftTurnButton,2,1,1,1)
        self.grid_layout.addWidget(self.leftButton,2,2,1,1)

        self.grid_layout.addWidget(self.offButton,0,6,1,1)
        self.grid_layout.addWidget(self.normalButton,1,6,1,1)
        self.grid_layout.addWidget(self.fig8Button,2,6,1,1)
        self.grid_layout.addWidget(self.fullButton,3,6,1,1)
        self.grid_layout.addWidget(self.calibrationButton,4,6,1,1)
        self.grid_layout.addWidget(graph_box,5,0,1,7)
        self.grid_layout.addWidget(parameter_box,0,7,6,1)


        main_widget = QWidget()
        main_widget.setLayout(self.grid_layout)
        self.setCentralWidget(main_widget)
        
        self.topDisplayLine.setText("0123456789ABCDEF")
        self.bottomDisplayLine.setText("0123456789ABCDEF")

        self.showMaximized()
    
    def setMode(self):
        print("Radio Button")
        if self.offButton.isChecked():
            print("Set Mode Off.") 
        else: 
            print("OFF mode is not selected.")


    def setup_RP1210_menus(self):
        connect_rp1210 = QAction(QIcon(r'icons/icons8_Connected_48px.png'), '&Client Connect', self)
        connect_rp1210.setShortcut('Ctrl+Shift+C')
        connect_rp1210.setStatusTip('Connect Vehicle Diagnostic Adapter')
        connect_rp1210.triggered.connect(self.selectRP1210)
        self.rp1210_menu.addAction(connect_rp1210)

        rp1210_version = QAction(QIcon(r'icons/icons8_Versions_48px.png'), '&Driver Version', self)
        rp1210_version.setShortcut('Ctrl+Shift+V')
        rp1210_version.setStatusTip('Show Vehicle Diagnostic Adapter Driver Version Information')
        rp1210_version.triggered.connect(self.display_version)
        self.rp1210_menu.addAction(rp1210_version)

        rp1210_detailed_version = QAction(QIcon(r'icons/icons8_More_Details_48px.png'), 'De&tailed Version', self)
        rp1210_detailed_version.setShortcut('Ctrl+Shift+T')
        rp1210_detailed_version.setStatusTip('Show Vehicle Diagnostic Adapter Detailed Version Information')
        rp1210_detailed_version.triggered.connect(self.display_detailed_version)
        self.rp1210_menu.addAction(rp1210_detailed_version)

        rp1210_get_hardware_status = QAction(QIcon(r'icons/icons8_Steam_48px.png'), 'Get &Hardware Status', self)
        rp1210_get_hardware_status.setShortcut('Ctrl+Shift+H')
        rp1210_get_hardware_status.setStatusTip('Determine details regarding the hardware interface status and its connections.')
        rp1210_get_hardware_status.triggered.connect(self.get_hardware_status)
        self.rp1210_menu.addAction(rp1210_get_hardware_status)

        rp1210_get_hardware_status_ex = QAction(QIcon(r'icons/icons8_System_Information_48px.png'), 'Get &Extended Hardware Status', self)
        rp1210_get_hardware_status_ex.setShortcut('Ctrl+Shift+E')
        rp1210_get_hardware_status_ex.setStatusTip('Determine the hardware interface status and whether the VDA device is physically connected.')
        rp1210_get_hardware_status_ex.triggered.connect(self.get_hardware_status_ex)
        self.rp1210_menu.addAction(rp1210_get_hardware_status_ex)

        disconnect_rp1210 = QAction(QIcon(r'icons/icons8_Disconnected_48px.png'), 'Client &Disconnect', self)
        disconnect_rp1210.setShortcut('Ctrl+Shift+D')
        disconnect_rp1210.setStatusTip('Disconnect all RP1210 Clients')
        disconnect_rp1210.triggered.connect(self.disconnectRP1210)
        self.rp1210_menu.addAction(disconnect_rp1210)

        self.RP1210_toolbar = self.addToolBar("RP1210")
        self.RP1210_toolbar.addAction(connect_rp1210)
        self.RP1210_toolbar.addAction(rp1210_version)
        self.RP1210_toolbar.addAction(rp1210_detailed_version)
        self.RP1210_toolbar.addAction(rp1210_get_hardware_status)
        self.RP1210_toolbar.addAction(rp1210_get_hardware_status_ex)
        self.RP1210_toolbar.addAction(disconnect_rp1210)

    
    def clear_voltage_graph(self):
        self.J1939.clear_voltage_history()
        self.J1587.clear_voltage_history()

    def show_graphs(self):
        self.voltage_graph.show()

    
    def confirm_quit(self):
        self.close()
    
    def closeEvent(self, event):
        result = QMessageBox.question(self, "Confirm Exit",
            "Are you sure you want to quit the program?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes)
        if result == QMessageBox.Yes:
            logger.debug("Quitting.")
            event.accept()
        else:
            event.ignore()

    def selectRP1210(self, automatic=False):
        logger.debug("Select RP1210 function called.")
        selection = SelectRP1210(self.title)
        logger.debug(selection.dll_name)
        if not automatic:
            selection.show_dialog()
        elif not selection.dll_name:
            selection.show_dialog()
        
        dll_name = selection.dll_name
        self.protocol = selection.protocol
        deviceID = selection.deviceID
        speed    = selection.speed

        if dll_name is None: #this is what happens when you hit cancel
            return
        #Close things down
        try:
            self.close_clients()
        except AttributeError:
            pass
        try:
            for thread in self.read_message_threads:
                thread.runSignal = False
        except AttributeError:
            pass
        
        # Once an RP1210 DLL is selected, we can connect to it using the RP1210 helper file.
        self.RP1210 = RP1210Class(dll_name)
    
        if self.RP1210_toolbar is None:
            self.setup_RP1210_menus()
        
        # We want to connect to multiple clients with different protocols.
        self.client_id = self.RP1210.get_client_id("CAN", deviceID, "{}".format(speed))
        self.rx_queue = queue.Queue(10000)
        self.extra_queue = queue.Queue(10000)
        if self.client_id is None:
            print("RP1210 is not available.")
            return

        # If there is a successful connection, save it.
        file_contents={ "dll_name":dll_name,
                        "protocol":self.protocol,
                        "deviceID":deviceID,
                        "speed":speed
                       }
        with open(selection.connections_file,"w") as rp1210_file:
            json.dump(file_contents, rp1210_file)

        # By turning on Echo Mode, our logger process can record sent messages as well as received.
        fpchClientCommand = (c_char*2000)()
        fpchClientCommand[0] = 1 #Echo mode on
        return_value = self.RP1210.SendCommand(c_short(RP1210_Echo_Transmitted_Messages), 
                                               c_short(self.client_id), 
                                               byref(fpchClientCommand), 1)
        logger.debug('RP1210_Echo_Transmitted_Messages returns {:d}: {}'.format(return_value,self.RP1210.get_error_code(return_value)))
        
         #Set all filters to pass
        return_value = self.RP1210.SendCommand(c_short(RP1210_Set_All_Filters_States_to_Pass), 
                                               c_short(self.client_id),
                                               None, 0)
        if return_value == 0:
            logger.debug("RP1210_Set_All_Filters_States_to_Pass for {} is successful.".format(self.protocol))
            #setup a Receive queue. This keeps the GUI responsive and enables messages to be received.
            self.read_message_thread = RP1210ReadMessageThread(self, 
                                                               self.rx_queue,
                                                               self.extra_queue,
                                                               self.RP1210.ReadMessage, 
                                                               self.client_id,
                                                               self.protocol, 
                                                               self.title)
            self.read_message_thread.setDaemon(True) #needed to close the thread when the application closes.
            self.read_message_thread.start()
            logger.debug("Started RP1210ReadMessage Thread.")

            self.statusBar().showMessage("{} connected using {}".format(self.protocol,dll_name))
        else :
            logger.debug('RP1210_Set_All_Filters_States_to_Pass returns {:d}: {}'.format(return_value,self.RP1210.get_error_code(return_value)))

    def check_connections(self):
        '''
        This function checks the VDA hardware status function to see if it has seen network traffic in the last second.
        
        '''    
        network_connection = {}

        for key in ["CAN"]:
            network_connection[key]=False            
            try:
                current_count = self.read_message_thread.message_count
                duration = time.time() - self.read_message_thread.start_time
                self.message_duration_label[key].setText("<html><img src='icons/icons8_Connected_48px.png'><br>Client Connected<br>{:0.0f} sec.</html>".format(duration))
                network_connection[key] = True
            except (KeyError, AttributeError) as e:
                current_count = 0
                duration = 0
                self.message_duration_label[key].setText("<html><img src='icons/icons8_Disconnected_48px.png'><br>Client Disconnected<br>{:0.0f} sec.</html>".format(duration))
                
            count_change = current_count - self.previous_count[key]
            self.previous_count[key] = current_count
            # See if messages come in. Change the 
            if count_change > 0 and not self.network_connected[key]: 
                self.status_icon_CAN.setText("<html><img src='icons/icons8_Ok_48px.png'><br>Network<br>Online</html>")
                self.network_connected[key] = True
            elif count_change == 0 and self.network_connected[key]:             
                self.status_icon_CAN.setText("<html><img src='icons/icons8_Unavailable_48px.png'><br>Network<br>Unavailable</html>")
                self.network_connected[key] = False

            self.message_count_label[key].setText("Message Count:\n{}".format(humanize.intcomma(current_count)))
            self.message_rate_label[key].setText("Message Rate:\n{} msg/sec".format(count_change))
        
        #return True if any connection is present.
        for key, val in network_connection.items():
            if val: 
                return True
        return False

    def get_hardware_status_ex(self):
        """
        Show a dialog box for valid connections for the extended get hardware status command implemented in the 
        vendor's RP1210 DLL.
        """
        logger.debug("get_hardware_status_ex")
        if self.client_id is not None:
            self.RP1210.get_hardware_status_ex(self.client_id)
            return
        QMessageBox.warning(self, 
                    "Connection Not Present",
                    "There were no Client IDs for an RP1210 device that support the extended hardware status command.",
                    QMessageBox.Cancel,
                    QMessageBox.Cancel)

    def get_hardware_status(self):
        """
        Show a dialog box for valid connections for the regular get hardware status command implemented in the 
        vendor's RP1210 DLL.
        """
        logger.debug("get_hardware_status")
        if self.client_id is not None:
            self.RP1210.get_hardware_status(self.client_id)
            return
        QMessageBox.warning(self, 
                    "Connection Not Present",
                    "There were no Client IDs for an RP1210 device that support the hardware status command.",
                    QMessageBox.Cancel,
                    QMessageBox.Cancel)
                
    def display_detailed_version(self):
        """
        Show a dialog box for valid connections for the detailed version command implemented in the 
        vendor's RP1210 DLL.
        """
        logger.debug("display_detailed_version")
        if self.client_id is not None:
            self.RP1210.display_detailed_version(self.client_id)
            return
        # otherwise show a dialog that there are no client IDs
        QMessageBox.warning(self, 
                    "Connection Not Present",
                    "There were no Client IDs for an RP1210 device.",
                    QMessageBox.Cancel,
                    QMessageBox.Cancel)
    
    def display_version(self):
        """
        Show a dialog box for valid connections for the extended get hardware status command implemented in the 
        vendor's RP1210 DLL. This does not require connection to a device, just a valid RP1210 DLL.
        """
        logger.debug("display_version")
        self.RP1210.display_version()

    def disconnectRP1210(self):
        """
        Close all the RP1210 read message threads and disconnect the client.
        """
        logger.debug("disconnectRP1210")
        del self.read_message_thread
        self.client_id = None
        for n in range(128):
            try:
                self.RP1210.ClientDisconnect(n)
            except:
                pass
        logger.debug("RP1210.ClientDisconnect() Finished.")

            
    def close_clients(self):
        logger.debug("close_clients Request")
        logger.debug("Closing {}".format(self.protocol))
        self.RP1210.disconnectRP1210(self.client_id)
        self.read_message_thread.runSignal = False
        logger.debug("Exiting.")
    
    def show_graphs(self):
        self.voltage_graph.show()        


    def send_command(self):
        if self.ok_to_send:
            pass

    def read_rp1210(self):
        start_time = time.time()
        while self.rx_queue.qsize():
            QCoreApplication.processEvents()
            if (time.time() - start_time + .8*self.update_rate) > self.update_rate: #give some time to process events
                logger.debug("Can't keep up with messages.")
                return
            (current_time, vda_timestamp, can_id, dlc, can_data) = self.rx_queue.get()
            #print("0x{:03X}".format(can_id))
            if can_id == 0x211:
                print(can_data)
                self.top_line_text = can_data.decode() + self.top_line_text[8:]
                self.topDisplayLine.setText(self.top_line_text)
            elif can_id == 0x212:
                self.top_line_text = self.top_line_text[:8] + can_data.decode()
                self.topDisplayLine.setText(self.top_line_text)
            elif can_id == 0x221:
                self.bot_line_text = can_data.decode() + self.bot_line_text[8:]
                self.bottomDisplayLine.setText(self.bot_line_text)
            elif can_id == 0x222:
                self.bot_line_text = self.bot_line_text[:8] + can_data.decode()
                self.bottomDisplayLine.setText(self.bot_line_text)
            elif can_id == 0x210:
                self.numberOfModes = can_data[0]
                self.currentMode = can_data[1]
                goalAngle = (struct.unpack(">H",can_data[2:4])[0] - 3600)/10
                ekfYawAngle = (struct.unpack(">H",can_data[4:6])[0] - 3600)/10
                leftMotor = can_data[6]
                rightMotor = can_data[7]

                self.goalAngleData.append((current_time, goalAngle))
                self.heading_graph.add_data(self.goalAngleData, 
                        marker = '-', 
                        label = "Goal Angle")
                self.ekfYawData.append((current_time, ekfYawAngle))
                self.heading_graph.add_data(self.ekfYawData, 
                        marker = 'o', 
                        label = "Yaw Angle")
                self.heading_graph.plot()

                self.motorData.append((current_time, goalAngle))
                self.heading_graph.add_data(self.goalAngleData, 
                        marker = 'o-', 
                        label = "Goal Angle")
                self.heading_graph.plot()
            elif can_id == 0x209:
                compassHeading = (struct.unpack(">H",can_data[0:2])[0] - 3600)/10 
                CANcompassHeading = (struct.unpack(">H",can_data[2:4])[0] - 3600)/10
                ekfYawRate = struct.unpack(">H",can_data[4:6])[0]/1000
                gpsCourse = struct.unpack(">H",can_data[6:8])[0]/10
              
                self.GPSheadingData.append((current_time,gpsCourse))
                self.heading_graph.add_data(self.GPSheadingData, 
                        marker = 'x', 
                        label = "GPS Course")

                self.compassData.append((current_time,compassHeading))
                self.heading_graph.add_data(self.compassData, 
                        marker = '.', 
                        label = "Raw Compass")

            elif can_id == 0x43c:
                CANheading = struct.unpack(">H",can_data[0:2])[0]/10
            elif can_id == 0x43d: 
                CANheading = struct.unpack(">H",can_data[6:8])[0]/10
            elif can_id == 0x43e:
                self.gpsSpeed = struct.unpack(">l",can_data[0:4])[0]/100 #Knots
                self.gpsCourse = struct.unpack(">l",can_data[0:4])[0]/100 #degrees

        
            elif can_id == 0x441:
                  headerValue = rxmsg.buf[0] * 256 + rxmsg.buf[1];
                  dist = rxmsg.buf[2] * 256 + rxmsg.buf[3];
                

if __name__ == '__main__':
    app = QCoreApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    else:
        app.close()
    execute = FishingGUI()
    sys.exit(app.exec_())
