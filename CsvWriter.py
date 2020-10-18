#-----Imports-----#
import serial
import time
import os
import csv
import shutil

#-----Globals-----#
_WRITE_CONFIG_TO_CSV = False
_FILE_NUMBER = 100
_USE_LONG_SAVE_FORMAT = True

class CsvWriter():
    def __init__(self, fileName=None):
        print('Init CsvWriter')
        self.writeConfig = _WRITE_CONFIG_TO_CSV
        self.fileNameNumber = _FILE_NUMBER

        if fileName is not None:
            self.overWrittenFileName = fileName
        else:
            self.overWrittenFileName = './sonar_strength.csv'

        return

    def saveToCSV(self, data_array, pingInterface=None):
        print('Saving to CSV')
        timeNow = time.localtime()
        timeStr = str(timeNow.tm_year) + '_' + str(timeNow.tm_mon) + '_' + str(timeNow.tm_mday) + '_' + str(self.fileNameNumber )
        if (_USE_LONG_SAVE_FORMAT):
            timeStr = str(timeNow.tm_year) + '_' + str(timeNow.tm_mon) + '_' + str(timeNow.tm_mday) + '_' + str(timeNow.tm_hour) + 'h' + str(timeNow.tm_min) + 'm' + str(timeNow.tm_sec) + 's'

        with open(self.overWrittenFileName, mode='w', newline ='') as csvfile:
            csv_writer = csv.writer(csvfile)
            if self.writeConfig is True and pingInterface is not None:
                csv_writer.writerow( [pingInterface.mode] )
                csv_writer.writerow( [pingInterface.gain] )
                csv_writer.writerow( [pingInterface.angle_sector] )
                csv_writer.writerow( [pingInterface.transmit_frequency] )
                csv_writer.writerow( [pingInterface.range_d] )
                csv_writer.writerow( [timeStr] )
            for data in data_array:
                csv_writer.writerow([point for point in data_array[data]])
            self.fileNameNumber += 1

        my_path = os.path.dirname(os.path.abspath(__file__)) # Figures out the absolute path for you in case your working directory moves around.
        if not os.path.exists(my_path + '/RawData'):
            os.mkdir(my_path + '/RawData')
        newFileName = my_path + '/RawData/' + timeStr + '.csv'
        newpath = shutil.copy2(self.overWrittenFileName, newFileName)

        print ("Saving to: " + newFileName)

        return

    def getEmptyCSV(self):
        emptyData = {}
        for i in range(400):
            emptyData[i] = [0] * 200;

        return emptyData
