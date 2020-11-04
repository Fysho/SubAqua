from brping import Ping360
from brping import definitions
from brping import PingDevice
from brping import pingmessage

import subprocess #import Popen, PIPE, call
import numpy as np

class PingInterface():
    def __init__(self, csvWriter):
        print("Initialising a Ping360 Interface")

        self.csv_writer = csvWriter

        self.ping360 = Ping360()
        self.args = None

        #------Initialise device settings------#
        self.mode= 1
        self.num_points = 200       # Number of samples per reflected signal [200-1200]
        self.gain = 1                 	      # 0-2
        self.angle_sector = 200        # 200 (0 =0deg, 100= 90deg, 200=180deg, 300=270deg, 400=360deg)
        self.transmit_frequency = 800 #1000 # Default frequency (500kHz to 1000kHz)
        self.range_d= 2                   # e.g 2m range

        return


    def readArgs(self):
        print('Begin Main - Read Args')
        self.parser = argparse.ArgumentParser(description="Ping python library example.")
        self.parser.add_argument('--device', action="store", required=False, type=str, help="Ping device port. E.g: /dev/ttyUSB0")
        self.parser.add_argument('--baudrate', action="store", type=int, default=115200, help="Ping device baudrate. E.g: 115200")
        self.parser.add_argument('--udp', action="store", required=False, type=str, help="Ping UDP server. E.g: 192.168.2.2:9092")
        self.args = self.parser.parse_args()

        return


    def connectToPing(self):
        print('Connecting to the Ping360')
        subprocess.call('python -m serial.tools.list_ports -v', shell=True) #get list of all ports
        p = subprocess.Popen('python -m serial.tools.list_ports 0403:6015  -q', stdout=subprocess.PIPE,  shell=True)
        port_bin = p.stdout.readline()
        #port= str(port_bin.strip(), '/dev/ttyUSB0')    # original
        #port= str( str(port_bin.strip()) + '/dev/ttyUSB0')
        #port= str(port_bin.strip())
        port= str('/dev/ttyUSB0')
        #self.args.device = port #port identification changed
        #if self.args.device is None and self.args.udp is None:
        #    self.parser.print_help()
        #    exit(1)
        #Connect to device
  #      if self.args.device is not None:
   #         self.ping360.connect_serial(self.args.device, self.args.baudrate)
    #    elif self.args.udp is not None:
     #       (host, port) = self.args.udp.split(':')
        self.ping360.connect_udp('192.168.2.2', 9092)
        if(self.ping360.initialize() is False):
                print('Did not connect to ping360 on 192.168.2.2:9092')
        else:
                print('Successful connection to ping360 via UDP')
        self.orig_settings= self.ping360.get_device_data()

        return


    def autoTransmit(self):
        print('Begin auto transmit')
        print(self.ping360.set_transmit_frequency(800))
        print(self.ping360.set_sample_period(80))
        print(self.ping360.set_number_of_samples(200))
        self.ping360.control_auto_transmit(0, self.angle_sector-1 ,10,0)

        # turn on with 1 grad steps over num(was 400) gradians
        tstart_s = time.time()
        # wait for num(was 400) device_data messages to arrive
        data_vals={} #new dictionary to store sonar strength data array
        for grad in range( self.angle_sector ):
            print(grad)
            self.ping360.wait_message([definitions.PING360_DEVICE_DATA])
            self.ping360.transmitAngle(grad)
            data_vals[grad] = self.ping360._data #save all data values corresponding to each grad step
        tend_s = time.time()
        print("full scan in %dms, %dHz" % (1000*(tend_s - tstart_s), self.angle_sector/(tend_s - tstart_s)))
        # stop the auto-transmit process
        self.ping360.control_motor_off()
        self.ping360.control_reset(0, 0)

        return data_vals


    def updateDeviceSetting(self):
        #------Initialise device settings------#
        print('update device settings')
        errored = False
        try:
            with open('config.json') as configFile:
                configs = json.load(configFile)
                self.mode = configs[ 'mode' ]
                self.num_points  = configs[ 'number_of_points' ]
                self.gain = configs[ 'gain' ]
                self.angle_sector = configs[ 'angle_sector' ]
                self.transmit_frequency = configs[ 'transmit_frequency' ]
                self.range_d = configs[ 'range' ]
                print("mode     ", self.mode)
                print("noPoints", self.num_points)
                print("gain      ", self.gain)
                print("angle_sector", self.angle_sector)
                print("transFreq  ", self.transmit_frequency)
                print("range_d    ", self.range_d)
        except:
            print("@@@@@@ ------ Exception Caught - likely due to update of new parameters ------ @@@@@@")
            errored = True

        return

    def setSonarDistance(self, distance):
        self.range_d = distance
        self.setDeviceSetting()

    def setDeviceSetting(self):
        print('set device settings')
        angle_deg = self.angle_sector * 180/200
        angle_rad = self.angle_sector * 3.1415/200
        #-------Calculate the sample period and transmit duration for a specific range-----#
        self._sample_period = calculateSamplePeriod(self.range_d, self.num_points)
        #print(f'Sample period : {_sample_period}')
        self.transmit_duration = adjustTransmitDuration(self.range_d,self._sample_period)

        #----------Update device with set settings----------#
        #The desired gain [0-2]
        self.ping360.set_gain_setting(self.gain)
        #deafult mode for Ping360 is 1
        self.ping360.set_mode(self.mode)
        # Set the transducer position in gradian [0-400]
        self.ping360.set_angle(self.angle_sector)
        # Signal transmitted duration between 1-1000S
        self.ping360.set_transmit_duration(int(self.transmit_duration)) #requires integer value
        #Time interval between individual signal intensity samples in 25ns
        #increments (80 to 40000 == 2 microseconds to 1000 microseconds)
        self.ping360.set_sample_period(int(self._sample_period)) #requires integer value
        # Acoustic operating frequency, frequency range is 500kHz to 1000kHz,
        # but 740 is a good practical frequency for different scenarios
        self.ping360.set_transmit_frequency(self.transmit_frequency)
        #Number of samples per reflected signal [200-1200]
        self.ping360.set_number_of_samples(self.num_points)

        return

    def transmitAngle(self, targetAngle, data = None):  # checks a single angle in front of us
        print('transmitting single angle: ' , targetAngle);
        data_vals_trans={} #new dictionary to store sonar strength data array
        if(data is not None):
            data_vals_trans = data;
        if(targetAngle < 0):
            targetAngle = targetAngle + 360;
        targetStep = int (np.rint(targetAngle * 400.0 / 360.0))
        print("scanning this step: " + targetStep + " at this angle: " + targetAngle );

        self.ping360.transmitAngle(targetAngle)
        data = self.ping360._data
        count = 0;
        returndistrance = self.range_d;

        for reading in data:
            count+=1
            distance = (count / 200) * self.range_d
            if(distance > 0.4):
                if(reading > 0.65):
                    returndistrance = getDistance

        data_vals_trans[targetAngle];
        return data_vals_trans, returndistrance


    def transmitSweep(self, targetAngle1, targetAngle2, data = None): # checks a range of angles in front of us
        print('sweeping from ' , targetAngle1, ' to ', targetAngle2);
        if(targetAngle1 < 0):
            targetAngle1 = targetAngle1 + 360;
        if(targetAngle2 < 0):
            targetAngle2 = targetAngle2 + 360;

        targetStep1 = int (np.rint(targetAngle1 * 400.0 / 360.0))
        targetStep2 = int (np.rint(targetAngle2 * 400.0 / 360.0))

        print(targetStep1);
        print(targetStep2);

        data_vals_trans={} #new dictionary to store sonar strength data array
        if(data is not None):
            data_vals_trans = data;
        closestDistance = 10000;
        closestStep = -1;


        if(targetStep1 > targetStep2):
            for step in range(targetStep1,400):
                angle = (step / 400.0) * 360.0
                print('Step: ', step, '   Angle: ', angle)
                self.ping360.transmitAngle(step)
                data_vals_trans[step] = self.ping360._data #save all data values corresponding to each grad step
            for step in range(0,targetStep2):
                angle = (step / 400.0) * 360.0
                print('Step: ', step, '   Angle: ', angle)
                self.ping360.transmitAngle(step)
                data_vals_trans[step] = self.ping360._data #save all data values corresponding to each grad step
        else:
            for step in range(targetStep1,targetStep2):
                angle = (step / 400.0) * 360.0
                print('Step: ', step, '   Angle: ', angle)
                self.ping360.transmitAngle(step)
                data_vals_trans[step] = self.ping360._data #save all data values corresponding to each grad step
        print('full sweep complete');
        return data_vals_trans




    def autoTransmitTrans(self):
        print('auto transmit trans')
        if self.angle_sector == 0:
            self.ping360.control_transducer(self.mode, self.gain, 0, int(self.transmit_duration), int(self._sample_period), self.transmit_frequency, self.num_points,1, 0)
        else:
        #    self.ping360.control_transducer(self.mode, self.gain, self.angle_sector-1, int(self.transmit_duration), int(self._sample_period), self.transmit_frequency, self.num_points,1, 0)
             self.ping360.control_transducer(self.mode, self.gain, 1, int(self.transmit_duration), int(self._sample_period), self.transmit_frequency, self.num_points,1, 0)
        current_settings = self.ping360.get_device_data()
        #print(current_settings)

        #turn on auto-scan with 1 grad steps
        #self.ping360.control_auto_transmit(0,self.angle_sector-1,10,0)
        self.ping360.control_auto_transmit(0,1,10,0)
        tstart_s = time.time()
        data_vals_trans={} #new dictionary to store sonar strength data array
        for grad_steps in range(self.angle_sector):
            print('Step: ', grad_steps)
            self.ping360.transmitAngle(grad_steps)
            data_vals_trans[grad_steps] = self.ping360._data #save all data values corresponding to each grad step
        tend_s = time.time()
        print("full scan in %dms, %dHz" % (1000*(tend_s - tstart_s), self.angle_sector/(tend_s - tstart_s)))

        return data_vals_trans


def plotSonarStrength ( data ):
    print('plot sonar strength')
    #------Get sonar strength array and plot------#
    for idx, sonarStrength in enumerate( data ):
        # sonarStrength= [point for point in data_vals_trans.item]
        plt.plot(idx, sonarStrength,'ro')

        plt.ylabel('Normalised sonar strength')
        plt.xlabel('Number of points')

    plt.show()
    return


#-----Globals-----#
SPEED_OF_SOUND = 1500
_FW_MIN_TRANSMIT_DURATION = 5
_FW_MAX_TRANSMIT_DURATION = 500
_SAMPLE_PERIOD_TICK_DURATION = 25e-9    ##_sample_period is the number of timer ticks between each data point
                                        ##each timer tick has a duration of 25 nanoseconds

#@Calculate the sample period based in the new range
def calculateSamplePeriod(distance, num_points):
    return 2*distance/(num_points*SPEED_OF_SOUND*_SAMPLE_PERIOD_TICK_DURATION)

#Adjust the transmit duration for a specific range
 # Per firmware engineer:
 # 1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) / (Velocity of sound in metres
 # per second)
 # 2. Then check that TxPulse is wide enough for currently selected sample interval in usec, i.e.,
 #    if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
 # 3. Perform limit checking
 # @return Transmit duration
def adjustTransmitDuration(distance,_sample_period):
    # 1
    transmit_duration = 8000 * (distance / SPEED_OF_SOUND)
    # 2 (transmit duration is microseconds, samplePeriod() is nanoseconds)
    transmit_duration = np.maximum(2.5*_sample_period/1000, transmit_duration)
    # 3
    return np.maximum(_FW_MIN_TRANSMIT_DURATION, np.minimum(transmitDurationMax(_sample_period), transmit_duration))

#@The maximum transmit duration that will be applied is limited internally by the firmware to prevent damage to the hardware
 # The maximum transmit duration is equal to 64 * the sample period in microseconds
 # @return The maximum transmit duration possible
def transmitDurationMax(_sample_period):
    return min(_FW_MAX_TRANSMIT_DURATION, _sample_period * 64e6)

#@Sample period is in ns
# @return double
def samplePeriod(_sample_period):
    return _sample_period*_SAMPLE_PERIOD_TICK_DURATION
