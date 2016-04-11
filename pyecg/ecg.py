#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""
testing autodoc - this should be first line in doc
"""
#: Someone should clean these imports...
import random
import sys
import struct
import numpy as np
import serial
from scipy import signal
import time
import binascii
import ctypes
import scipy.fftpack
from PyQt4 import QtGui, QtCore, uic
import peakutils
from subprocess import PIPE, Popen
import shlex
import os.path
import matplotlib
matplotlib.use('TkAgg')  #: Choose backend
#: The following two statements must remain where they are or else our choosen backend will not be used.
from matplotlib.backends.backend_qt4agg import (FigureCanvasQTAgg, NavigationToolbar2QT)
from matplotlib.figure import Figure

Ui_MainWindow, QMainWindow = uic.loadUiType('ecgmonitor.ui')  #: Load the UI template


class MatplotlibWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        super(MatplotlibWidget, self).__init__(parent)

        self.figure = Figure()
        self.canvas = FigureCanvasQTAgg(self.figure)
        #: Initialize Plots
        self.axis1 = self.figure.add_subplot(211)
        self.axis2 = self.figure.add_subplot(212)
        self.axis1.set_autoscaley_on(False)  #: Disable Auto scale
        self.axis1.set_xlim([0, 2000])
        self.axis1.set_ylim([-1.5, 1.5])
        self.axis2.set_autoscaley_on(False)
        self.axis2.set_xlim([1, 60])
        self.axis2.set_ylim([0, 65000])
        self.axis1.set_xticks(np.arange(0, 2000, 100))
        self.axis1.set_yticks(np.arange(-1.5, 1.5, 0.3))
        self.axis1.grid(False)  #: Useless. True doesnt work either.
        #: Non dynamic parts are cached so that we can decrease the draw time
        self.background = self.canvas.copy_from_bbox(self.axis1.bbox)
        self.layoutVertical = QtGui.QVBoxLayout(self)
        self.layoutVertical.addWidget(self.canvas)


class SampleCollectorThread(QtCore.QThread):
    """Reads the data from the socket.
    """
    newSample = QtCore.pyqtSignal(list)
    finalSample = QtCore.pyqtSignal(list)

    def __init__(self, parent=None):
        # Initialisiere Thread
        super(SampleCollectorThread, self).__init__(parent)
        # Öffne Socket zu Bluetooth
        self.serial_interface = serial.Serial(port='/dev/rfcomm3', baudrate=19200)  # (port='/dev/ttyUSB0', baudrate=19200)#(port='/dev/rfcomm0',baudrate=19200,)
        self.serial_interface.isOpen()  #: Useless?
        self.serial_interface.flushInput()  #: Flush the socket Otherwise it clogs up
        self.sample_array = [[0] * 2000]  #: Initilize the array with zeroes
        self.position_sample_array = 0  #: Start position

    def start_transmitting_data(self):
        self.serial_interface.write(b'2')  #: Some arbitrary byte commands

    def stop_transmitting_data(self):
        self.serial_interface.write(b'5')  #: Some arbitrary byte commands

    def run(self):
        out = ''
        if self.serial_interface.inWaiting() > 0:  #: Do we have data?
            number_waiting_bytes = self.serial_interface.inWaiting()  #: How many bytes are waiting?
            read_bytes = self.ser.read(number_waiting_bytes)  #: Read everything
            self.serial_interface.flushInput()  #: Flush the rest
            for read_bytes_iter in range(len(read_bytes)):
                if binascii.hexlify(read_bytes[read_bytes_iter]) == "ff":  #: Wait for a sync byte
                    try:
                        byte_fields = struct.unpack('BB', read_bytes[read_bytes_iter + 1:read_bytes_iter + 3])  #: read the next two bytes
                    except Exception as e:
                        break
                    sample_entry_uint = byte_fields[0] << 8  #: bitshift to get the real 11bit uint
                    sample_entry_uint += byte_fields[1]
                    if sample_entry_uint > 1024:  #: Shouldnt happen. Means it is out of bound.
                        break  #: So we dont care and move on
                    self.sample_array[self.position_sample_array] = sample_entry_uint
                    if(self.position_sample_array == 1999):  #: If we have reached the end of our graph x axis we should restart at the beginning
                        self.finalSample.emit(self.sample_array)
                        self.position_sample_array = 0
                    else:
                        self.position_sample_array += 1
        self.newSample.emit(self.sample_array)


class EcgFrontend(QMainWindow, Ui_MainWindow):
    """Wrapper class which builds the graphical frontend.
    This is the "main" class of the program. Must be called for execution.
    """
    def __init__(self, ):
        super(EcgFrontend, self).__init__()
        self.process_command_bluetooth_socket = 0
        self.ecgmainwidget = EcgMainWidget(self)
        self.mplfigure = self.ecgmainwidget
        self.mplfigure.resize(1200, 400)
        self.setupUi(self)
        self.ecgmainwidget.qrs_pulse.connect(self.on_qrs_pulse)
        self.actionBeenden.setShortcut('Ctrl+Q')
        self.actionBeenden.setStatusTip('Exit application')
        self.actionBeenden.triggered.connect(self.close)
        self.actionDaten_Transfer_starten.setShortcut('Ctrl+D')
        self.actionDaten_Transfer_starten.setStatusTip('Start Data Transfer')
        self.actionDaten_Transfer_starten.triggered.connect(self.start_data_transfer)
        self.actionBluetooth_Starte.setShortcut('Ctrl+B')
        self.actionBluetooth_Starte.setStatusTip('Start Bluetooth')
        self.actionBluetooth_Starte.triggered.connect(self.start_bluetooth)
        self.actionPause.setShortcut('Ctrl+P')
        self.actionPause.setStatusTip('Pause Drawing')
        self.actionPause.triggered.connect(self.pause_draw)
        self.actionDaten_Aufzeichnen.setShortcut('Ctrl+A')
        self.actionDaten_Aufzeichnen.setStatusTip('Zeichne eine Minute auf und gebe das wieder')
        self.actionDaten_Aufzeichnen.triggered.connect(self.start_reading)

    def pause_draw(self):
        if (self.actionDaten_Transfer_starten.isChecked()):
            self.ecgmainwidget.draw_data = False
        else:
            self.ecgmainwidget.draw_data = True

    def start_reading(self):
        self.ecgmainwidget.sdata = np.zeros(2000)
        self.rtimer = QtCore.QTimer()
        self.rtimer.setSingleShot(True)
        self.rtimer.timeout.connect(lambda: self.collect_data())
        self.rtimer.start(60000)
        self.ecgmainwidget.scrape = True

    def collect_data(self):
        self.ecgmainwidget.scrape = False
        self.ecgmainwidget.draw_old = True

    def start_data_transfer(self):
        if (self.actionDaten_Transfer_starten.isChecked()):
            self.ecgmainwidget.samplecollectorthread.start_transmitting_data()
        else:
            self.ecgmainwidget.samplecollectorthread.stop_transmitting_data()

    def start_bluetooth(self):
        if (self.actionBluetooth_Starte.isChecked()):
            command_line = "sudo rfcomm connect /dev/rfcomm3 00:06:66:4C:CB:B7 1 &"  #: Dirty very dirty. Works only on Linux. Fails without Error. Apllication will not respond in this case.
            args = shlex.split(command_line)
            self.process_command_bluetooth_socket = Popen(args, stdin=PIPE, stdout=PIPE)
            while not os.path.exists("/dev/rfcomm3"):
                pass
            self.ecgmainwidget.init_thread()
            self.eventLog.append("Startup Completed.")
        else:
            self.stop_bluetooth()

    def stop_bluetooth(self):
        self.ecgmainwidget.destroy_thread()
        self.process_command_bluetooth_socket.kill()
        command_line = "sudo rfcomm release /dev/rfcomm3"
        args = shlex.split(command_line)
        self.process_command_bluetooth_socket = Popen(args, stdin=PIPE, stdout=PIPE)

    def closeEvent(self, event):
        self.ecgmainwidget.samplecollectorthread.stoptran()
        self.stop_bluetooth()
        event.accept()

    @QtCore.pyqtSlot(list)
    def on_qrs_pulse(self, sample, use_blit=True):
        try:
            self.eventLog.append("Puls: " + str(reduce(lambda x, y: x + y, sample[0]) / float(len(sample[0]))))
            self.eventLog.append("Durschnittliche QRS Zeit: " + str(reduce(lambda x, y: x + y, sample[1]) / float(len(sample[1]))))
        except TypeError:
            pass


class EcgMainWidget(QtGui.QWidget):
    """Main Widget to show ECG Data as a graph.
    Args:
        QtGui.QWidget: This class must be called with an instance of its parent Container. In this case with an instance of EcgFrontend
    """
    qrs_pulse = QtCore.pyqtSignal(list)  #: Register signal for the QRS time and pulse computation. Is emmited when a new dataset is available.

    def __init__(self, parent=None):
        """
        Args:
            parent: In this case the wrapper class EcgFrontend
        """
        super(EcgMainWidget, self).__init__(parent)
        self.matplotlibWidget = MatplotlibWidget(self)
        self.layoutVertical = QtGui.QVBoxLayout(self)  #: Set the layout so that everything is vertical aligned
        self.layoutVertical.addWidget(self.matplotlibWidget)  #: Add the matplotlibWidget to the layout container -> graphs are rendered in the Widget

    def init_thread(self):
        """
        Initializes the thread which reads the incoming data from the socket.
        """
        self.samplecollectorthread = SampleCollectorThread(self)
        self.sdata = np.array([])  #: Initialize data list as numpy array and set it to be empty
        self.sfdata = np.zeros(2000)  #: Initialize data list for the replay function as numpy array. Needs to be filled in this case with zeroes
        #: Connect to the thread signals so that the appropriate functions are calleds
        self.samplecollectorthread.newSample.connect(self.on_SampleCollectorThread_newSample)
        self.samplecollectorthread.finalSample.connect(self.on_SampleCollectorThread_finalSample)

        self.line1, = self.matplotlibWidget.axis1.plot(self.samplecollectorthread.sample_array, '-', alpha=0.8, color="red", markerfacecolor="red")
        self.line2, = self.matplotlibWidget.axis2.plot(scipy.fftpack.fftfreq(np.array(self.samplecollectorthread.sample_array, dtype=np.uint16).size, d=float(1.0 / 1000.0)), abs(scipy.fftpack.fft(np.array(self.samplecollectorthread.sample_array, dtype=np.uint16))), 'x-', alpha=0.8, color="blue", markerfacecolor="blue")
        self.line3, = self.matplotlibWidget.axis1.plot(self.sfdata, '-', alpha=0.8, color="blue", markerfacecolor="blue")

        self.qrs = []  #: Raw qrs data. Should update every 5 s
        self.draw_data = True  #: Should we draw the data on the graph
        self.qrs_f = []  #: Array from which we compute the mean qrs time
        self.pulse_arr = 0  #: Raw pulse data. Should update every 5 s
        self.pulse_f = []  #: Array from which we compute the mean pulse
        self.draw_old = False  #: Flag variable for the replay function
        self.old_data_iterator = 0  #: Iterator to compute the window to display at replay
        self.do1 = np.array([])
        self.do2 = np.array([])
        self.scrape = False  #: Flag variable. If true records the data for replay
        self.timer = QtCore.QTimer()  #: Initialize timer to read from the socket as fast as possible
        self.timer.setSingleShot(False)  #: Should be available foerver and not terminate after one run
        self.timer.timeout.connect(lambda: self.samplecollectorthread.start())  #: Which function should be called when the timer fires. Needs to be with this weird lambda call because of dragons
        self.timer.start(0)  #: WARNING: This timer can fire at any time when the program is not busy handling some other task
        self.pctimer = QtCore.QTimer()  #: Initialize timer to compute the pulse every 5 seconds
        self.pctimer.setSingleShot(False)  #: Should be available foerver and not terminate after one run
        self.pctimer.timeout.connect(lambda: self.compute_pulse())  #: Which function should be called when the timer fires. Needs to be with this weird lambda call because of dragons
        self.pctimer.start(5000)  #: Fires every 5000 ms

    def destroy_thread(self):
        """
        Garbage collector in case we want to exit or bad things happen. Otherwise we pollute our stack.
        WARNING: Does only some cleanup. Most flags and lesser variables are still floating around
        """
        del self.samplecollectorthread
        del self.line1
        del self.line2
        del self.qrs
        del self.qrs_f
        del self.pulse_arr
        del self.pulse_f
        del self.timer
        del self.pctimer

    def scrape_data(self):
        """
        Toggle the scraping flag
        """
        self.scrape = True

    def compute_pulse(self):
        """
        Calculates the pulse an mean qrs time and emits them as a signal. Signal is caught to update the Message Box
        """
        try:
            self.pulse_f.append(self.pulse_arr * 12)
            self.qrs_f.append(reduce(lambda x, y: x + y, self.qrs) / float(len(self.qrs)))  #: Just arithmetic mean in the short lambda notation. The float cast is necessary to preserve the data type.
            self.qrs = []  #: Reset Array
            self.pulse_arr = 0  #: Reset variable
            self.qrs_pulse.emit([self.pulse_f, self.qrs_f])
        except TypeError:
            #: Not the best method to catch exceptions. At least we are not disturbed. The missing data shouldnt bother us....
            pass

    def smooth(self, x, window_len=11, window='blackman'):
        """
        Smoothing filter. Does black magic with folding of the array. Produces nasty artifacts at the beginning of each array. Blackman window seem to work best for our case
        """
        s = np.r_[x[window_len - 1:0:-1], x, x[-1:-window_len:-1]]
        if window == 'flat':  #: means moving average
            w = np.ones(window_len, 'd')
        else:
            if window not in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
                raise ValueError("Window not available")  #: Thank you Cpt. Obvious
            w = eval('np.' + window + '(window_len)')
        y = np.convolve(w / w.sum(), s, mode='valid')  #: Yeah whatever. Changing mode doesnt seem to work.
        return y

    def butter_lowpass(self, cutoff, fs, order=5):
        """
        Computes filter coefficients.
        """
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def butter_lowpass_filter(self, data, cutoff=30.0, fs=1000.0, order=6):
        """
        Calls the computation of the filter coefficients and applies them. Best working variables hardcoded as defaults.
        If the sample rate is altered fs needs to be adjusted so that we have the right Nyquist frequency
        """
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = signal.lfilter(b, a, data)
        return y

    def movingaverage(self, data, window):
        """
        Computes moving average. Not reaaaaaly needed. But still used in the zero crossing computation
        """
        weights = np.repeat(1.0, window) / window
        sma = np.convolve(data, weights, 'valid')
        return sma

    @QtCore.pyqtSlot(list)
    def on_SampleCollectorThread_finalSample(self, sample, use_blit=True):
        """
        Is called when an array is "full" (has reached 2000 entrys). Computes the QRS Time for all QRS spikes in the array. Computes also the pulse.
        """
        data = np.array(sample, dtype=np.uint16)
        y = self.butter_lowpass_filter(data)
        z = self.smooth(y)
        z = z[10:]
        if (self.scrape):
            self.sdata = np.append(self.sdata, z)
        peaks = peakutils.indexes(z, thres=0.8, min_dist=100)  #: Only count peaks with a height of 80% of the max amplitude and 100 ticks apart.
        self.pulse_arr += len(peaks)  #: The pulse is nothing else then the number of QRS spikes
        zero_crossings = np.where(np.diff(np.sign(((self.movingaverage(z, 1) - 512) * 0.002929688))))[0]  #: Beware of the magic numbers. Chosen by fair dice role. Nah just kidding. The minus 512 is the half of the max and normalizes the scale to 0-512. The other is the conversion variable from discrete variables to mV
        for peak in peaks:
            try:
                qr = zero_crossings[zero_crossings < peak].flatten()[-2:]  #: Try to find the start point by finding the second last zero crossing before a peak
                t = zero_crossings[zero_crossings > peak].flatten()[0]  #: First Zero crossing behind a peak
                qrs_cycle = t - qr[0]
                self.qrs.append(qrs_cycle * (1.0 / 1000.0))
            except IndexError:
                continue

    @QtCore.pyqtSlot(list)
    """
    Redraws the graph everytime a new sample is emitted.
    """
    def on_SampleCollectorThread_newSample(self, sample, use_blit=True):
        if (self.draw_data):
            data = np.array(sample[0], dtype=np.uint16)
            self.do1 = data
        else:
            data = self.do1
        y = self.butter_lowpass_filter(data)
        z = self.smooth(y)
        z = z[10:]  #: Because of black magic we now habe 2010 data points. Drop the first 10. They are garbage
        self.matplotlibWidget.axis1.draw_artist(self.matplotlibWidget.axis1.patch)
        self.line1.set_ydata((z - 512) * 0.002929688)  #: Convert to a -1.5-1.5 mV scale
        self.matplotlibWidget.axis1.draw_artist(self.line1)
        if (self.draw_old):
            #: Builds a window of 2000 old data values to draw
            ug = self.old_data_iterator
            og = self.old_data_iterator + 2000
            self.sfdata = self.sdata[ug:og]
            self.line3.set_ydata(((self.sfdata) - 512) * 0.002929688)
            self.matplotlibWidget.axis1.draw_artist(self.line3)
            if (self.old_data_iterator + 2000 > len(self.sdata)):
                self.old_data_iterator = 0
            else:
                self.old_data_iterator += 1
        #: Set the data to the graphs
        self.line2.set_xdata(scipy.fftpack.fftfreq(z.size, d=float(1.0 / 1000.0)))
        self.line2.set_ydata(abs(scipy.fftpack.fft(z)))
        self.matplotlibWidget.axis2.draw_artist(self.matplotlibWidget.axis2.patch)
        self.matplotlibWidget.axis2.draw_artist(self.line2)
        #: Draw
        self.matplotlibWidget.canvas.update()
        self.matplotlibWidget.canvas.flush_events()

if __name__ == "__main__":
    import sys
    #: Erzeuge Applikation
    app = QtGui.QApplication(sys.argv)
    app.setApplicationName('MedInfSignals')
    main = EcgFrontend()
    main.resize(1400, 1000)  #: Yup resolution hardcoded for now
    main.show()
    sys.exit(app.exec_())
