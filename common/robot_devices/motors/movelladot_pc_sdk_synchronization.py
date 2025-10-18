
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

    xdpcHandler.scanForDots()
    if len(xdpcHandler.detectedDots()) == 0:
        print("No Movella DOT device(s) found. Aborting.")
        xdpcHandler.cleanup()
        exit(-1)

    xdpcHandler.connectDots()

    if len(xdpcHandler.connectedDots()) == 0:
        print("Could not connect to any Movella DOT device(s). Aborting.")
        xdpcHandler.cleanup()
        exit(-1)

    for device in xdpcHandler.connectedDots():
        # Make sure all connected devices have the same filter profile and output rate
        if device.setOnboardFilterProfile("General"):
            print("Successfully set profile to General")
        else:
            print("Setting filter profile failed!")

        if device.setOutputRate(20):
            print("Successfully set output rate to 20 Hz")
        else:
            print("Setting output rate failed!")

    manager = xdpcHandler.manager()
    deviceList = xdpcHandler.connectedDots()
    print(f"\nStarting sync for connected devices... Root node: {deviceList[-1].bluetoothAddress()}")
    print("This takes at least 14 seconds")
    if not manager.startSync(deviceList[-1].bluetoothAddress()):
        print(f"Could not start sync. Reason: {manager.lastResultText()}")
        if manager.lastResult() != movelladot_pc_sdk.XRV_SYNC_COULD_NOT_START:
            print("Sync could not be started. Aborting.")
            xdpcHandler.cleanup()
            exit(-1)

        # If (some) devices are already in sync mode.Disable sync on all devices first.
        manager.stopSync()
        print(f"Retrying start sync after stopping sync")
        if not manager.startSync(deviceList[-1].bluetoothAddress()):
            print(f"Could not start sync. Reason: {manager.lastResultText()}. Aborting.")
            xdpcHandler.cleanup()
            exit(-1)

    # Start live data output. Make sure root node is last to go to measurement.
    print("Putting devices into measurement mode.")
    for device in xdpcHandler.connectedDots():
        if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_ExtendedEuler):
            print(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
            continue

    print("\nMain loop. Recording data for 2 seconds.")
    print("-----------------------------------------")

    # First printing some headers so we see which data belongs to which device
    s = ""
    for device in xdpcHandler.connectedDots():
        s += f"{device.portInfo().bluetoothAddress():27}"
    print("%s" % s, flush=True)

    startTime = movelladot_pc_sdk.XsTimeStamp_nowMs()
    while movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime <= 2000:
        if xdpcHandler.packetsAvailable():
            s = ""
            for device in xdpcHandler.connectedDots():
                # Retrieve a packet
                packet = xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())

                if packet.containsOrientation():
                    euler = packet.orientationEuler()
                    s += f"TS:{packet.sampleTimeFine():8d}, Roll:{euler.x():7.2f}| "

            print("%s" % s, flush=True)

    print("\n-----------------------------------------", end="", flush=True)

    print("\nStopping measurement...")
    for device in xdpcHandler.connectedDots():
        if not device.stopMeasurement():
            print("Failed to stop measurement.")

    print("Stopping sync...")
    if not manager.stopSync():
        print("Failed to stop sync.")

    print("Closing ports...")
    manager.close()

    print("Successful exit.")
