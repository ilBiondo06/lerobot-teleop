
#  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#  
#  1.	Redistributions of source code must retain the above copyright notice,
#  	this list of conditions and the following disclaimer.
#  
#  2.	Redistributions in binary form must reproduce the above copyright notice,
#  	this list of conditions and the following disclaimer in the documentation
#  	and/or other materials provided with the distribution.
#  
#  3.	Neither the names of the copyright holders nor the names of their contributors
#  	may be used to endorse or promote products derived from this software without
#  	specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
#  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
#  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
#  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  

# Requires installation of the correct Movella DOT PC SDK wheel through pip
# For example, for Python 3.9 on Windows 64 bit run the following command
# pip install movelladot_pc_sdk-202x.x.x-cp39-none-win_amd64.whl

from xdpchandler import *


if __name__ == "__main__":
    xdpcHandler = XdpcHandler()

    if not xdpcHandler.initialize():
        xdpcHandler.cleanup()
        exit(-1)

    xdpcHandler.detectUsbDevices()
    if len(xdpcHandler.detectedDots()) == 0:
        print("No Movella DOT device(s) found. Aborting.")
        xdpcHandler.cleanup()
        exit(-1)

    xdpcHandler.connectDots()

    if len(xdpcHandler.connectedUsbDots()) == 0:
        print("Could not connect to any Movella DOT device(s). Aborting.")
        xdpcHandler.cleanup()
        exit(-1)

    exportData = movelladot_pc_sdk.XsIntArray()
    exportData.push_back(movelladot_pc_sdk.RecordingData_Timestamp);
    exportData.push_back(movelladot_pc_sdk.RecordingData_Euler);
    exportData.push_back(movelladot_pc_sdk.RecordingData_Acceleration);
    exportData.push_back(movelladot_pc_sdk.RecordingData_AngularVelocity);
    exportData.push_back(movelladot_pc_sdk.RecordingData_MagneticField);
    exportData.push_back(movelladot_pc_sdk.RecordingData_Status);

    device = xdpcHandler.connectedUsbDots()[0]

    recordingIndex = device.recordingCount()
    recInfo = device.getRecordingInfo(recordingIndex)
    if recInfo.empty():
        print(f'Could not get recording info. Reason: {device.lastResultText()}')
    else:
        print(f'Recording [{recordingIndex}], Storage Size: {recInfo.storageSize()} bytes')
        print(f'Recording [{recordingIndex}], Recording Time: {recInfo.totalRecordingTime()} seconds')

    csvFilename = f'device_{device.deviceId().toXsString()}_{recordingIndex}.csv'
    print(f'Exporting recording {recordingIndex} to file {csvFilename}')

    if not device.selectExportData(exportData):
        print(f'Could not select export data. Reason: {device.lastResultText()}')
    elif not device.enableLogging(csvFilename):
        print(f'Could not open logfile for data export. Reason: {device.lastResultText()}')
    elif not device.startExportRecording(recordingIndex):
        print(f'Could not export recording. Reason: {device.lastResultText()}')
    else:
        # Sleeping for max 10 seconds...
        startTime = movelladot_pc_sdk.XsTimeStamp_nowMs()
        while not xdpcHandler.exportDone() and movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime <= 10000:
            time.sleep(0.1)

        if xdpcHandler.exportDone():
            print('File export finished!')
        else:
            print('Done sleeping, aborting export for demonstration purposes.')
            if not device.stopExportRecording():
                print(f'Device stop export failed. Reason: {device.lastResultText()}')
            else:
                print('Device export stopped')

        print(f'Received {xdpcHandler.packetsReceived()} data packets from the recording.')

    device.disableLogging()

    xdpcHandler.cleanup()
