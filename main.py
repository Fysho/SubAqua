import CsvWriter
import PingInterface
def main():
    csvWriter = CsvWriter.CsvWriter("sonarData.csv")
    pingInterface = PingInterface.PingInterface(csvWriter)
    #pingInterface.readArgs()
    pingInterface.connectToPing()
    pingInterface.setSonarDistance(5);
    data_vals = csvWriter.getEmptyCSV();
    data_vals = pingInterface.transmitSweep(0, 360, data_vals)

    csvWriter.saveToCSV(data_vals)

    #newData = {}
    #for i in range(20):
    #    newData[i] = [1,2,3] #save all data values corresponding to each grad step

    #csvWriter.saveToCSV(newData)

# PYTHON MAIN CALL
if __name__ == "__main__":
    main()
