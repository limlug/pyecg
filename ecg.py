#!/usr/bin/env python
# -*- coding:utf-8 -*-

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
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_qt4agg import (FigureCanvasQTAgg, NavigationToolbar2QT)
from matplotlib.figure import Figure
from PyQt4 import QtGui, QtCore, uic
import peakutils
from subprocess import PIPE, Popen
import shlex
import os.path

Ui_MainWindow, QMainWindow = uic.loadUiType('ecgmonitor.ui')


class MatplotlibWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        super(MatplotlibWidget, self).__init__(parent)

        self.figure = Figure()
        self.canvas = FigureCanvasQTAgg(self.figure)
        # Initialisiere Plots
        self.axis1 = self.figure.add_subplot(211)
        self.axis2 = self.figure.add_subplot(212)
        # Kein Autoscale
        self.axis1.set_autoscaley_on(False)
        self.axis1.set_xlim([0, 2000])
        self.axis1.set_ylim([-1.5, 1.5])
        self.axis2.set_autoscaley_on(False)
        self.axis2.set_xlim([1, 60])
        self.axis2.set_ylim([0, 65000])
        self.axis1.set_xticks(np.arange(0, 2000, 100))
        self.axis1.set_yticks(np.arange(-1.5, 1.5, 0.3))
        self.axis1.grid(False)
        # Cache nicht dynamische Anteile
        self.background = self.canvas.copy_from_bbox(self.axis1.bbox)
        self.layoutVertical = QtGui.QVBoxLayout(self)
        self.layoutVertical.addWidget(self.canvas)


class ThreadSample(QtCore.QThread):
    newSample = QtCore.pyqtSignal(list)
    finalSample = QtCore.pyqtSignal(list)

    def __init__(self, parent=None):
        # Initialisiere Thread
        super(ThreadSample, self).__init__(parent)
        # Öffne Socket zu Bluetooth
        self.ser = serial.Serial(port='/dev/rfcomm3', baudrate=19200)  # (port='/dev/ttyUSB0', baudrate=19200)#(port='/dev/rfcomm0',baudrate=19200,)
        self.ser.isOpen()
        self.ser.flushInput()
        # Setze Initialwerte auf 0
        self.sam = [[0] * 2000, [0] * 2000]
        self.pos = 0

    def starttran(self):
        self.ser.write(b'2')

    def stoptran(self):
        self.ser.write(b'5')

    def run(self):
        out = ''
        if self.ser.inWaiting() > 0:
            # bi = self.ser.read(1)
            # Finde Syncbyte
            wbytes = self.ser.inWaiting()
            read_byte = self.ser.read(wbytes)
            self.ser.flushInput()
            for i in range(len(read_byte)):
                if binascii.hexlify(read_byte[i]) == "ff":
                    try:
                        fields = struct.unpack('BB', read_byte[i + 1:i + 3])
                    except Exception as e:
                        break
                    out = fields[0] << 8
                    out += fields[1]
                    if out > 1024:
                        break
                    self.sam[0][self.pos] = out
                    if(self.pos == 1999):
                        self.finalSample.emit(self.sam[0])
                        self.pos = 0
                    else:
                        self.pos += 1
        self.newSample.emit(self.sam)


class Main(QMainWindow, Ui_MainWindow):
    def __init__(self, ):
        super(Main, self).__init__()
        self.p = 0
        self.mWd = MyWindow(self)
        self.mplfigure = self.mWd
        self.mplfigure.resize(1200, 400)
        self.setupUi(self)
        self.mWd.qrs_pulse.connect(self.on_qrs_pulse)
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
            self.mWd.dra = False
        else:
            self.mWd.dra = True

    def start_reading(self):
        self.mWd.sdata = np.zeros(2000)
        self.rtimer = QtCore.QTimer()
        self.rtimer.setSingleShot(True)
        self.rtimer.timeout.connect(lambda: self.collect_data())
        self.rtimer.start(60000)
        self.mWd.scrape = True

    def collect_data(self):
        self.mWd.scrape = False
        self.mWd.draw_old = True

    def start_data_transfer(self):
        if (self.actionDaten_Transfer_starten.isChecked()):
            self.mWd.threadSample.starttran()
        else:
            self.mWd.threadSample.stoptran()

    def start_bluetooth(self):
        if (self.actionBluetooth_Starte.isChecked()):
            command_line = "sudo rfcomm connect /dev/rfcomm3 00:06:66:4C:CB:B7 1 &"
            args = shlex.split(command_line)
            self.p = Popen(args, stdin=PIPE, stdout=PIPE)
            while not os.path.exists("/dev/rfcomm3"):
                pass
            self.mWd.init_thread()
            self.eventLog.append("Startup Completed.")
        else:
            self.stop_bluetooth()

    def stop_bluetooth(self):
        self.mWd.destroy_thread()
        self.p.kill()
        command_line = "sudo rfcomm release /dev/rfcomm3"
        args = shlex.split(command_line)
        self.p = Popen(args, stdin=PIPE, stdout=PIPE)

    def closeEvent(self, event):
        self.mWd.threadSample.stoptran()
        self.stop_bluetooth()
        event.accept()

    @QtCore.pyqtSlot(list)
    def on_qrs_pulse(self, sample, use_blit=True):
        try:
            self.eventLog.append("Puls: " + str(reduce(lambda x, y: x + y, sample[0]) / float(len(sample[0]))))
            self.eventLog.append("Durschnittliche QRS Zeit: " + str(reduce(lambda x, y: x + y, sample[1]) / float(len(sample[1]))))
        except TypeError:
            pass


class MyWindow(QtGui.QWidget):
    qrs_pulse = QtCore.pyqtSignal(list)

    def __init__(self, parent=None):
        super(MyWindow, self).__init__(parent)
        self.matplotlibWidget = MatplotlibWidget(self)
        # Definiere Layout
        self.layoutVertical = QtGui.QVBoxLayout(self)
        # Füge Graphen hinzu
        self.layoutVertical.addWidget(self.matplotlibWidget)
        # Erzeuge Thread

    def init_thread(self):
        self.threadSample = ThreadSample(self)
        # Verbinde Signal zu Threa
        self.sdata = np.array([])
        self.sfdata = np.zeros(2000)
        self.threadSample.newSample.connect(self.on_threadSample_newSample)
        self.threadSample.finalSample.connect(self.on_threadSample_finalSample)
        self.line1, = self.matplotlibWidget.axis1.plot(self.threadSample.sam[0], '-', alpha=0.8, color="red", markerfacecolor="red")
        self.line2, = self.matplotlibWidget.axis2.plot(scipy.fftpack.fftfreq(np.array(self.threadSample.sam[0], dtype=np.uint16).size, d=float(1.0 / 1000.0)), abs(scipy.fftpack.fft(np.array(self.threadSample.sam[0], dtype=np.uint16))), 'x-', alpha=0.8, color="blue", markerfacecolor="blue")
        self.line3, = self.matplotlibWidget.axis1.plot(self.sfdata, '-', alpha=0.8, color="blue", markerfacecolor="blue")
        # Initialisiere Timer
        self.qrs = []
        self.dra = True
        self.qrs_f = []
        self.pulse_arr = 0
        self.pulse_f = []
        self.draw_old = False
        self.o_i = 0
        self.do1 = np.array([])
        self.do2 = np.array([])
        self.scrape = False
        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(False)
        self.timer.timeout.connect(lambda: self.threadSample.start())
        # Setze Timer auf Laufe Nebenläufig Immer
        self.timer.start(0)
        self.pctimer = QtCore.QTimer()
        self.pctimer.setSingleShot(False)
        self.pctimer.timeout.connect(lambda: self.compute_pulse())
        self.pctimer.start(5000)

    def destroy_thread(self):
        del self.threadSample
        del self.line1
        del self.line2
        del self.qrs
        del self.qrs_f
        del self.pulse_arr
        del self.pulse_f
        del self.timer
        del self.pctimer

    def scrape_data(self):
        self.scrape = True

    def compute_pulse(self):
        try:
            self.pulse_f.append(self.pulse_arr * 12)
            self.qrs_f.append(reduce(lambda x, y: x + y, self.qrs) / float(len(self.qrs)))
            self.qrs = []
            self.pulse_arr = 0
            self.qrs_pulse.emit([self.pulse_f, self.qrs_f])
        except TypeError:
            pass

    def smooth(self, x, window_len=11, window='blackman'):
        s = np.r_[x[window_len - 1:0:-1], x, x[-1:-window_len:-1]]
        if window == 'flat':  # moving average
            w = np.ones(window_len, 'd')
        else:
            if window not in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
                raise ValueError("Window not available")
            w = eval('np.' + window + '(window_len)')
        y = np.convolve(w / w.sum(), s, mode='valid')
        return y

    def butter_lowpass(self, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def butter_lowpass_filter(self, data, cutoff=30.0, fs=1000.0, order=6):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = signal.lfilter(b, a, data)
        return y

    def movingaverage(self, data, window):
        weights = np.repeat(1.0, window) / window
        sma = np.convolve(data, weights, 'valid')
        return sma

    @QtCore.pyqtSlot(list)
    def on_threadSample_finalSample(self, sample, use_blit=True):
        data = np.array(sample, dtype=np.uint16)
        y = self.butter_lowpass_filter(data)
        z = self.smooth(y)
        z = z[10:]
        if (self.scrape):
            self.sdata = np.append(self.sdata, z)
        peaks = peakutils.indexes(z, thres=0.8, min_dist=100)
        self.pulse_arr += len(peaks)
        zero_crossings = np.where(np.diff(np.sign(((self.movingaverage(z, 1) - 512) * 0.002929688))))[0]
        for peak in peaks:
            try:
                qr = zero_crossings[zero_crossings < peak].flatten()[-2:]
                t = zero_crossings[zero_crossings > peak].flatten()[0]
                qrs_cycle = t - qr[0]
                self.qrs.append(qrs_cycle * (1.0 / 1000.0))
            except IndexError:
                continue

    @QtCore.pyqtSlot(list)
    def on_threadSample_newSample(self, sample, use_blit=True):
        if (self.dra):
            data = np.array(sample[0], dtype=np.uint16)
            self.do1 = data
        else:
            data = self.do1
        y = self.butter_lowpass_filter(data)
        z = self.smooth(y)
        z = z[10:]
        self.matplotlibWidget.axis1.draw_artist(self.matplotlibWidget.axis1.patch)
        self.line1.set_ydata((z - 512) * 0.002929688)
        self.matplotlibWidget.axis1.draw_artist(self.line1)
        if (self.draw_old):
            ug = self.o_i
            og = self.o_i + 2000
            self.sfdata = self.sdata[ug:og]
            self.line3.set_ydata(((self.sfdata) - 512) * 0.002929688)
            self.matplotlibWidget.axis1.draw_artist(self.line3)
            if (self.o_i + 2000 > len(self.sdata)):
                self.o_i = 0
            else:
                self.o_i += 1
        # Setze Daten
        self.line2.set_xdata(scipy.fftpack.fftfreq(z.size, d=float(1.0 / 1000.0)))
        self.line2.set_ydata(abs(scipy.fftpack.fft(z)))
        # Initialisiere Änderungen für Graph 2
        self.matplotlibWidget.axis2.draw_artist(self.matplotlibWidget.axis2.patch)
        self.matplotlibWidget.axis2.draw_artist(self.line2)
        # Zeichne
        self.matplotlibWidget.canvas.update()
        self.matplotlibWidget.canvas.flush_events()

if __name__ == "__main__":
    import sys
    # Erzeuge Applikation
    app = QtGui.QApplication(sys.argv)
    app.setApplicationName('MedInfSignals')
    main = Main()
    main.resize(1400, 1000)
    main.show()
    sys.exit(app.exec_())
