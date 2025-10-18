# hid is used to read inputs from the PS4 controller.
# pip install hid
# Requires installing hidapi library: https://github.com/libusb/hidapi
# The PS4 controller should be connected via USB
# Bluetooth connection can partially work but may require some changes
# So far only tested on Mac and Ubuntu
# When running on ubuntu make sure that the user has access to the USB device (joystick)

import logging
import math
import struct
import threading
import time
import pyttsx3 # Da installare manualmente
import enum

# TODO create an initial_connection function (iniziale) and a meantime_connection function (durante il test synch) 
# TODO Inoltre devono essere connessi un minimo di 2 dot.

from lerobot.common.robot_devices.motors.xdpchandler import *

class MovellaDotConfig:

    def __init__(self, motor_names, initial_position=None, *args, **kwargs):
        print("chiamo")
        self.motor_names = motor_names
        self.initial_position = initial_position if initial_position else [0, 20, 20, 0, 0, 10]
        self.current_positions = dict(zip(self.motor_names, self.initial_position, strict=False))
        self.new_positions = self.current_positions.copy()

        if False:    
            
            """
            MAPPING
            movella dot su spalla yaw --> shoulder_pan: [1, "sts3215"]
            movella dot su spalla pitch --> shoulder_lift: [2, "sts3215"] # mi sto chiedendo se può sostituire quella del gomito.
            movella dot su gomito pitch --> elbow_flex: [3, "sts3215"] # se non basta allora devi usare anche questa. dipende dalla estensione richiesta.
            movella dot su polso pitch --> wrist_flex: [4, "sts3215"] # forse non serve.
            movella dot su polso roll --> wrist_roll: [5, "sts3215"]
            movella dot su mano --> gripper: [6, "sts3215"]

            # però noi non ne abbiamo 6. --> uno o due vanno esclusi.
            """

            self.xdpcHandler = XdpcHandler()
            self.samplerate = 20
            self.dotProfile = "General"

            self.loop = False
            self.running = False
            # Connessione pregressa
            self.talk("initial connection")
            self.connect()
            
            self.rilascio = True

            self.running=True
            # Start the thread to read inputs
            self.lock = threading.Lock()
            self.eulerPerSensor = {
            "shoulder_pan": self.initial_position[0],
            "shoulder_lift": self.initial_position[1],
            "elbow_flex": self.initial_position[2],
            "wrist_flex": self.initial_position[3],
            "wrist_roll": self.initial_position[4],
            "gripper": self.initial_position[5]
            }
            self.thread = threading.Thread(target=self.read_loop, daemon=True)
            self.thread.start()

    def connect(self):
        try:
            # Chiama una funzione di una classe che inizializza l'ambiente per la connessione. Se non funziona esce.
            if not self.xdpcHandler.initialize():
                self.xdpcHandler.cleanup()
                raise RuntimeError("Failed to initialize xdpcHandler")

            # scansione dei dots. Se non lo trova esce.
            #contatore = 0
            #start_time = time.time()  # Salva l'orario di inizio

            self.xdpcHandler.scanForDots()
            
            if len(self.xdpcHandler.detectedDots()) == 0:
                self.xdpcHandler.cleanup()
                raise RuntimeError("No Movella DOT device(s) found. Aborting.")

            # effettiva connessione ai dots.
            self.xdpcHandler.connectDots()

            # Se non ci sono dots connessi allora esci.
            if len(self.xdpcHandler.connectedDots()) == 0:
                self.xdpcHandler.cleanup()
                raise RuntimeError("Could not connect to any Movella DOT device(s). Aborting.")

            # Imposta per ogni dot il profilo e ti mostra i profili disponibili
            dots = self.xdpcHandler.connectedDots()
            self.talk(f"{str(len(dots))} connessi")
            for device in dots:
                filterProfiles = device.getAvailableFilterProfiles()
                print("Available filter profiles:")
                for f in filterProfiles:
                    print(f.label())

                # Stampa il profilo attuale del dot e setta quello nuovo
                print(f"Current profile: {device.onboardFilterProfile().label()}")

                # Imposta a profilo General (movimenti lenti)
                if device.setOnboardFilterProfile(self.dotProfile):
                    print("Successfully set profile to "+self.dotProfile)
                else:
                    raise RuntimeError("Setting filter profile failed!")

                # SAMPLE FREQ
                if device.setOutputRate(self.samplerate): # possibili valori 1, 4, 10, 12, 15, 20, 30, 60, 120
                    print("Successfully set output rate to "+str(self.samplerate)+" Hz")
                else:
                    print("Setting output rate failed!")
                    # VALORE 50 Hz
                    # abbiamo investigato con 50 Hz e ci ha fallito l'impostazione dell'outputrate.
                    # Si è quindi impostato su 20 (che era quello impostato precedentemente - default)

                # modalità che salva dati in memoria interna.
                print("Setting quaternion output")
                device.setLogOptions(movelladot_pc_sdk.XsLogOptions_QuaternionAndEuler)

                # nome del file
                # VOGLIO EVITARE DI SALVARE I DATI IN MEMORIA...NON MI SERVE.
                
            
                    
            # dopo che tutti sono impostati e connessi allora li sincronizzo
            if len(self.xdpcHandler.connectedDots()) > 1:
                self.sync()
                self.loop=True
            elif len(self.xdpcHandler.connectedDots()) == 1: 
                self.oneDotSetupAvoidingSync()
                self.loop=True
            # ho tolto da qui sync
            
        except OSError as e:
            logging.error(f"Unable to open device: {e}")
            self.running = False
            self.loop=False
            
    def sync(self):
    	# Sync
        self.talk("init synchronization")
        try:
            # viene impostato un manager per poter sincronizzare i device.
            manager = self.xdpcHandler.manager()
            # chiamo attraverso l'istanza della classe la connessione dei device. Stabilisce quale tra i device è ROOT
            deviceList = self.xdpcHandler.connectedDots()
            print(f"\nStarting sync for connected devices... Root node: {deviceList[-1].bluetoothAddress()}")
            print("This takes at least 14 seconds")

            # Fa partire la sincronizzazione dal dot ROOT.
            # Se ne abbiamo uno solo non ha senso sincronizzare
            if not manager.startSync(deviceList[-1].bluetoothAddress()):
                print(f"Could not start sync. Reason: {manager.lastResultText()}")
                # controlla che riesce a sincronizzarli tutti
                if manager.lastResult() != movelladot_pc_sdk.XRV_SYNC_COULD_NOT_START:
                    self.xdpcHandler.cleanup()
                    raise RuntimeError("Sync could not be started. Aborting.")
                # stop della sincronizzazione
                manager.stopSync()
                print(f"Retrying start sync after stopping sync")
                if not manager.startSync(deviceList[-1].bluetoothAddress()):
                    self.xdpcHandler.cleanup()
                    raise RuntimeError(f"Could not start sync. Reason: {manager.lastResultText()}. Aborting.")
            
            # Payload mode
            # serve a dire quale sia l'ooutput che viene messo nella tabella.
            print("Putting devices into measurement mode.")
            for device in self.xdpcHandler.connectedDots():
                print(device)
                # prendo nello specifico la misurazione con il payload mode "XsPayloadMode_CustomMode4"
                if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_CustomMode4):
                    print(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
                    continue

        except OSError as e:
            self.talk(f"Unable to open device: {e}")
            logging.error(f"Unable to open device: {e}")
            self.running = False

    def oneDotSetupAvoidingSync(self):
    	# Sync
        self.talk("inizio syncronizzazione")
        try:
            # viene impostato un manager per poter sincronizzare i device.
            manager = self.xdpcHandler.manager()
            # chiamo attraverso l'istanza della classe la connessione dei device. Stabilisce quale tra i device è ROOT
            deviceList = self.xdpcHandler.connectedDots()

            # Payload mode
            # serve a dire quale sia l'ooutput che viene messo nella tabella.
            print("Putting devices into measurement mode.")
            for device in self.xdpcHandler.connectedDots():
                print(device)
                # prendo nello specifico la misurazione con il payload mode "XsPayloadMode_CustomMode4"
                if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_CustomMode4):
                    print(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
                    continue

            print(f"\nSetup manager as the only one dot available... Root node: {deviceList[-1].bluetoothAddress()}")
            
        except OSError as e:
            self.talk(f"Unable to open device: {e}")
            logging.error(f"Unable to open device: {e}")
            self.running = False
	

    def disconnect(self):
        # disconnettere i movella dot.
        self.running = False
        for device in self.xdpcHandler.connectedDots():
            print(f"\nResetting heading to default for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
            if device.resetOrientation(movelladot_pc_sdk.XRM_DefaultAlignment):
                print("OK", end="", flush=True)
            else:
                print(f"NOK: {device.lastResultText()}", end="", flush=True)
            print("\n", end="", flush=True)

        # Stoppa le misurazioni per ogni dot.
        print("\nStopping measurement...")
        for device in self.xdpcHandler.connectedDots():
            if not device.stopMeasurement():
                print("Failed to stop measurement.")
            if not device.disableLogging():
                print("Failed to disable logging.")

        logging.info("Controller disconnected.")

    def read_loop(self):
        dot_reference="DOT0"
        while self.running: # TODO problema in stanby perchè non troviamo una soluzione: Se questo diventa false non campiona più. questo accade se perde la sincronizzazione oppure per un qualunque altro motivo. dobbiamo trovare un altra soluzione con un altro flag e mantenere self.running fino a che l'esecuzione continua. se invece viene chiusa allora self.running va messo a false.
            if self.loop:
                try:
                    if self.xdpcHandler.packetsAvailable():
                        # prendi il valore attuale dei dot e lo stampi.
                        data = {dot_reference:[],"DOT1":[],"DOT2":[],"DOT3":[]}
                        available=[]
                        name = ""
                        for device in self.xdpcHandler.connectedDots():
                            packet = self.xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())
                            name = device.deviceTagName()[:4]
                            data[name]=packet
                            available.append(name)
                            # quando scelgo la payload mode non tutti hanno la misur dell'orientamento # TODO togliere in fase di test
                            """if packet.containsOrientation():
                                euler = packet.orientationEuler()"""
                        #self._process_input(data)
                        
                        
                        # se quel dot non è connesso, allora non far fallire il codice ma imposta un valore di default. 
                        # Se un sensore non funziona allora gli lascio l'ultimo valore registrato. se invece c'è allora glielo assegno
                        """eulerPerSensor = {
                            "shoulder_pan": data["DOT0"][7],
                            "shoulder_lift": data["DOT0"][6],
                            "elbow_flex": data["DOT1"][6],
                            "wrist_flex": data["DOT2"][6],
                            "wrist_roll": data["DOT2"][5],
                            "gripper": data["DOT3"][6]
                        }""" # questo dizionario aveva un problema di fondo: se non c'è il dato allora da errore.
                        if len(available)>=1:
                            for check in [dot_reference,"DOT1","DOT2","DOT3"]:
                                if check in available:
                                    if check==dot_reference:
                                        self.eulerPerSensor["shoulder_pan"] = data[check][6]
                                        self.eulerPerSensor["shoulder_lift"] = data[check][7] 
                                    elif check=="DOT1":
                                        self.eulerPerSensor["elbow_flex"] = data[check][7] 
                                    elif check=="DOT2":
                                        self.eulerPerSensor["wrist_flex"] = data[check][6]
                                        self.eulerPerSensor["wrist_roll"] = data[check][5]
                                    elif check=="DOT3":
                                        self.eulerPerSensor["gripper"] = data[check][6]
                            """elif len(available)==1:
                                self.eulerPerSensor["shoulder_pan"] = data[name][6]
                                self.eulerPerSensor["shoulder_lift"] = data[name][7]
                                self.eulerPerSensor["wrist_flex"] = data[name][6]
                                self.eulerPerSensor["wrist_roll"] = data[name][5]
                                if data[name][6] > 60:
                                    self.eulerPerSensor["gripper"] = data[name][6] // gripper chiude
                                elif data[name][6] < 20:
                                    self.eulerPerSensor["gripper"] = data[name][6] // gripper rilascia
                            else:
                                    self.eulerPerSensor["gripper"] = data[name][6] 
                                self.eulerPerSensor["elbow_flex"] = data[name][7]"""
                                
                            """eulerPerSensor = {
                                "shoulder_pan": data["DOT0"][6],
                                "shoulder_lift": data["DOT0"][7],
                                "elbow_flex": data["DOT1"][7],
                                "wrist_flex": data["DOT2"][6],
                                "wrist_roll": data["DOT2"][5],
                                "gripper": data["DOT3"][6]
                            }"""
                            self._update_positions(self.eulerPerSensor)
                        else:
                            #self.connect()
                            print("no data available because there are not movella dot connected!")
        
                        
                        


                except Exception as e:
                    logging.error(f"Error reading from device: {e}")
                    time.sleep(1)  # Wait before retrying
                    # TODO RICHIEDO LA CONNESSIONE.
                    #self.connect()
                    #self.sync()




    def _update_positions(self,data):
        # Compute new positions based on inputs
        speed = 0.3
        # TODO: speed can be different for different directions

        temp_positions = self.current_positions.copy()

        if True:

            temp_positions["wrist_roll"] = data["wrist_roll"]
            temp_positions["wrist_flex"] = data["wrist_flex"]

            temp_positions["gripper"] = data["gripper"]

            temp_positions["shoulder_pan"] = data["shoulder_pan"]
            temp_positions["shoulder_lift"] = data["shoulder_lift"]

            temp_positions["elbow_flex"] = data["elbow_flex"]

        # Perform eligibility check
        if self._is_position_valid(temp_positions):
            # Atomic update: all positions are valid, apply the changes
            self.current_positions = temp_positions
        else:
            # Invalid positions detected, do not update
            logging.warning("Invalid motor positions detected. Changes have been discarded.")
            self.indicate_error()


    def _is_position_valid(self, positions):

        allowed_ranges = {
            """"shoulder_pan": (-10, 190),
            "shoulder_lift": (-5, 185),
            "elbow_flex": (-5, 185),
            "wrist_flex": (-110, 110),
            "wrist_roll": (-110, 110),
            "gripper": (0, 100)"""
            # SOLO DI DEBUG
            "shoulder_pan": (-10, 40),
            "shoulder_lift": (-5, 170),
            "elbow_flex": (-5, 40),
            "wrist_flex": (-30, 30),
            "wrist_roll": (-30, 30),
            "gripper": (0, 50)
        }

        for motor, (min_val, max_val) in allowed_ranges.items():
            if motor in positions and not (min_val <= positions[motor] <= max_val):
                logging.error(
                    f"Motor '{motor}' position {positions[motor]} out of range [{min_val}, {max_val}]."
                )
                
                return False

        return True



    def stop(self):
        """
        Clean up resources.
        """
        self.loop =False
        self.disconnect()
        self.thread.join()
        print("Stopping sync...")
        if not manager.stopSync():
            print("Failed to stop sync.")

        # pulizia delle impostazioni.
        self.xdpcHandler.cleanup()

    def get_command(self):
        """
        Return the current motor positions after reading and processing inputs.
        """
        return self.current_positions.copy()
        
    def talk(self, message):
        engine=pyttsx3.init()
        engine.setProperty("rate",150)
        engine.setProperty("volume",1)

        engine.say(message)
        engine.runAndWait() 


class TorqueMode(enum.Enum):
    ENABLED = 1
    DISABLED = 0
