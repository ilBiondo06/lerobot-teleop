
import logging
import math
import struct
import traceback
from copy import deepcopy
import threading
import time
import pyttsx3 # Da installare manualmente
import enum
import numpy as np

from keras.models import load_model
from keras.metrics import MeanSquaredError

from lerobot.common.robot_devices.motors.configs import MovellaDotMotorsBusConfig

from lerobot.common.robot_devices.motors.xdpchandler import *

SCS_SERIES_CONTROL_TABLE = {
    "Model": (3, 2),
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay": (7, 1),
    "Response_Status_Level": (8, 1),
    "Min_Angle_Limit": (9, 2),
    "Max_Angle_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1),
    "Max_Voltage_Limit": (14, 1),
    "Min_Voltage_Limit": (15, 1),
    "Max_Torque_Limit": (16, 2),
    "Phase": (18, 1),
    "Unloading_Condition": (19, 1),
    "LED_Alarm_Condition": (20, 1),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Minimum_Startup_Force": (24, 2),
    "CW_Dead_Zone": (26, 1),
    "CCW_Dead_Zone": (27, 1),
    "Protection_Current": (28, 2),
    "Angular_Resolution": (30, 1),
    "Offset": (31, 2),
    "Mode": (33, 1),
    "Protective_Torque": (34, 1),
    "Protection_Time": (35, 1),
    "Overload_Torque": (36, 1),
    "Speed_closed_loop_P_proportional_coefficient": (37, 1),
    "Over_Current_Protection_Time": (38, 1),
    "Velocity_closed_loop_I_integral_coefficient": (39, 1),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Time": (44, 2),
    "Goal_Speed": (46, 2),
    "Torque_Limit": (48, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Speed": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
    # Not in the Memory Table
    "Maximum_Acceleration": (85, 2),
}

class TorqueMode(enum.Enum):
    ENABLED = 1
    DISABLED = 0

MODEL_CONTROL_TABLE = {
    "scs_series": SCS_SERIES_CONTROL_TABLE,
    "sts3215": SCS_SERIES_CONTROL_TABLE,
}

MODEL_RESOLUTION = {
    "scs_series": 4096,
    "sts3215": 4096,
}

PROTOCOL_VERSION = 0
BAUDRATE = 1_000_000
TIMEOUT_MS = 1000

class JointOutOfRangeError(Exception):
    def __init__(self, message="Joint is out of range"):
        self.message = message
        super().__init__(self.message)

class MovellaDotConfig:

    def __init__(
            self, 
            config: MovellaDotMotorsBusConfig
    ): # motor_names, initial_position=None, *args, **kwargs):

        self.port = config.port
        self.motors = config.motors
        self.mock = config.mock

        self.model_ctrl_table = deepcopy(MODEL_CONTROL_TABLE)
        self.model_resolution = deepcopy(MODEL_RESOLUTION)


        #self.motor_names = motor_names
        self.initial_position = [
            0, 
            170, # da -5 a 170
            165, # da -5 a 40
            40, 
            0, 
            10]
        self.current_positions = self.initial_position #dict(zip(self.motor_names, self.initial_position, strict=False))
        self.new_positions = self.current_positions.copy()

        self.initial_v2 = None
        self.initial_v3 = None

        self.packet_handler = None
        self.calibration = None
        self.is_connected = False
        self.group_readers = {}
        self.group_writers = {}
        self.logs = {}

        self.track_positions = {}

        self.xdpcHandler = XdpcHandler()
        self.samplerate = 20 #30
        self.dotProfile = "General"

        self.limit_shoulder_pan = 60

        model_path = "ML_models/regression_model_gripper_0_50_V1.h5"
        self.model = load_model(model_path, custom_objects={'mse': MeanSquaredError()})

        self._average = {"DOT0":[],"DOT1":[],"DOT2":[],"DOT3":[],"DOT4":[]}

        # gripper estimation params (quaternion-based)
        self._gripper_prev = None
        self._calibrated_max_angle = None
        self._yaw_offset = None

        self.GRIPPER_MAX = 90.0
        self.GRIPPER_LIMIT = 60.0
        self.JITTER_FRAC = 0.01
        self.DEFAULT_ALPHA = 0.5
        

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

            """   
            for device in self.xdpcHandler.connectedDots():
                print(f"\nResetting heading to default for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
                if device.resetOrientation(movelladot_pc_sdk.XRM_DefaultAlignment):
                    print("OK", end="", flush=True)
                else:
                    print(f"NOK: {device.lastResultText()}", end="", flush=True)
                print("\n", end="", flush=True)
            

            if self._yaw_offset is None or self._calibrated_max_angle is None:
                print("\nCalibrazione: tieni la mano APERTA e i sensori PARALLELI, poi premi INVIO per calibrare (o Ctrl-C per saltare).")
                try:
                    input("Premi INVIO per calibrare ora... ")
                    self.calibrate_yaw_and_max(samples=12, timeout_s=2.5)
                except KeyboardInterrupt:
                    print("Calibrazione saltata dall'utente.")
            """
                    
            # dopo che tutti sono impostati e connessi allora li sincronizzo
            if len(self.xdpcHandler.connectedDots()) > 1:
                #self.sync(self.xdpcHandler)
                print("init:Quanti dot connessi?",len(self.xdpcHandler.connectedDots()))
                self.loop=True
            elif len(self.xdpcHandler.connectedDots()) == 1: 
                #self.oneDotSetupAvoidingSync(self.xdpcHandler)
                self.loop=True
            # ho tolto da qui sync
            #self.start_background_reader()
            # Payload mode: serve a dire quale sia l'output che viene messo nella tabella.
            print("Putting devices into measurement mode.")
            for device in self.xdpcHandler.connectedDots():
                print(f"{device.deviceTagName()}",device)
                # prendo nello specifico la misurazione con il payload mode "XsPayloadMode_CustomMode4"
                if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_CustomMode4):
                    print(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
                    continue

            if self._yaw_offset is None or self._calibrated_max_angle is None:
                print("\nCalibrazione: tieni la mano APERTA e i sensori PARALLELI, poi premi INVIO per calibrare (o Ctrl-C per saltare).")
                try:
                    input("Premi INVIO per calibrare ora... ")
                    self.calibrate_yaw_and_max(samples=12, timeout_s=2.5)
                except KeyboardInterrupt:
                    print("Calibrazione saltata dall'utente.")            
            
        except OSError as e:
            logging.error(f"Unable to open device: {e}")
            self.running = False
            self.loop=False

        self.loop = False
        self.running = False
        # Connessione pregressa
        self.talk("initial connection")
        #self.connect()
        
        self.rilascio = True

        self.running=True

        self.eulerPerSensor = {
            "shoulder_pan": self.initial_position[0],
            "shoulder_lift": self.initial_position[1],
            "elbow_flex": self.initial_position[2],
            "wrist_flex": self.initial_position[3],
            "wrist_roll": self.initial_position[4],
            "gripper": self.initial_position[5]
            }
        
        import scservo_sdk as scs

        self.port_handler = scs.PortHandler(self.port)
        self.packet_handler = scs.PacketHandler(PROTOCOL_VERSION)

        try:
            if not self.port_handler.openPort():
                raise OSError(f"Failed to open port '{self.port}'.")
        except Exception:
            traceback.print_exc()
            print(
                "\nTry running `python lerobot/scripts/find_motors_bus_port.py` to make sure you are using the correct port.\n"
            )
            raise

        # Allow to read and write
        self.is_connected = True

        self.port_handler.setPacketTimeoutMillis(TIMEOUT_MS)

        

        if False:    
            
            """
            MAPPING
            movella dot su spalla yaw --> shoulder_pan: [1, "sts3215"]
            movella dot su spalla pitch --> shoulder_lift: [2, "sts3215"] # mi sto chiedendo se può sostituire quella del gomito.
            movella dot su gomito pitch --> elbow_flex: [3, "sts3215"] # se non basta allora devi usare anche questa. dipende dalla estensione richiesta.
            movella dot su polso pitch --> wrist_flex: [4, "sts3215"] # forse non serve.
            movella dot su polso roll --> wrist_roll: [5, "sts3215"]
            movella dot su mano --> gripper: [6, "sts3215"]
            """
            
            # Start the thread to read inputs
            self.lock = threading.Lock()
            self.thread = threading.Thread(target=self.read_loop, daemon=True)
            self.thread.start()

    # ---------- helper quaternion ----------
    @staticmethod
    def normalize_angle_deg(a):
        return ((a + 180) % 360) - 180

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        r = math.radians(roll) * 0.5
        p = math.radians(pitch) * 0.5
        y = math.radians(yaw) * 0.5
        cr = math.cos(r); sr = math.sin(r)
        cp = math.cos(p); sp = math.sin(p)
        cy = math.cos(y); sy = math.sin(y)
        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        return (qw, qx, qy, qz)

    @staticmethod
    def quat_conjugate(q):
        qw, qx, qy, qz = q
        return (qw, -qx, -qy, -qz)

    @staticmethod
    def quat_multiply(a, b):
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        w = aw*bw - ax*bx - ay*by - az*bz
        x = aw*bx + ax*bw + ay*bz - az*by
        y = aw*by - ax*bz + ay*bw + az*bx
        z = aw*bz + ax*by - ay*bx + az*bw
        return (w, x, y, z)

    @staticmethod
    def relative_angle_between_quaternions(q_ref, q_target):
        q_rel = MovellaDotConfig.quat_multiply(q_target, MovellaDotConfig.quat_conjugate(q_ref))
        qw = max(-1.0, min(1.0, q_rel[0]))
        angle_rad = 2.0 * math.acos(qw)
        angle_deg = math.degrees(angle_rad)
        if angle_deg > 180:
            angle_deg = 360 - angle_deg
        return angle_deg

    def gripper_from_euler(self, dot2_euler, dot3_euler, max_angle=None, yaw_offset=None, prev=None, alpha=None, auto_calibrate=True):
        """
        Restituisce (gripper_value, relative_angle_deg, used_max_angle).
        dot2_euler e dot3_euler sono tuple (roll,pitch,yaw) in gradi.
        """
        if alpha is None:
            alpha = self.DEFAULT_ALPHA

        if yaw_offset is None:
            yaw_offset = self._yaw_offset if self._yaw_offset is not None else 0.0

        roll2, pitch2, yaw2 = dot2_euler
        roll3, pitch3, yaw3 = dot3_euler

        yaw3_adj = self.normalize_angle_deg(yaw3 - yaw_offset)

        roll2_n = self.normalize_angle_deg(roll2)
        pitch2_n = self.normalize_angle_deg(pitch2)
        yaw2_n = self.normalize_angle_deg(yaw2)

        roll3_n = self.normalize_angle_deg(roll3)
        pitch3_n = self.normalize_angle_deg(pitch3)
        yaw3_n = yaw3_adj

        q2 = self.euler_to_quaternion(roll2_n, pitch2_n, yaw2_n)
        q3 = self.euler_to_quaternion(roll3_n, pitch3_n, yaw3_n)

        angle_deg = self.relative_angle_between_quaternions(q2, q3)

        # calibrazione automatica (se richiesta)
        used_max_angle = max_angle
        if used_max_angle is None:
            if self._calibrated_max_angle is None and auto_calibrate:
                if angle_deg > 8.0:
                    self._calibrated_max_angle = max(70, angle_deg * 1.05)
                    used_max_angle = self._calibrated_max_angle
                    print(f"[Calibration] impostato max_angle a {self._calibrated_max_angle:.1f} deg")
                else:
                    used_max_angle = 62.0
            else:
                used_max_angle = self._calibrated_max_angle if self._calibrated_max_angle is not None else 62.0

        a = max(0.0, min(used_max_angle, angle_deg))
        gripper_raw = self.GRIPPER_MAX * (1.0 - (a / used_max_angle))

        jitter_thresh = self.GRIPPER_MAX * self.JITTER_FRAC
        if gripper_raw < jitter_thresh:
            gripper_raw = 0.0

        if prev is None:
            gripper_f = gripper_raw if self._gripper_prev is None else (alpha * gripper_raw + (1 - alpha) * self._gripper_prev)
        else:
            gripper_f = alpha * gripper_raw + (1 - alpha) * prev

        gripper_f = min(gripper_f, self.GRIPPER_LIMIT)
        self._gripper_prev = gripper_f
        return gripper_f, angle_deg, used_max_angle

    def calibrate_yaw_and_max(self, samples=10, timeout_s=2.0):
        """
        Calibrazione non-bloccante: campiona fino a timeout_s o samples e imposta
        self._yaw_offset e self._calibrated_max_angle.
        """
        t0 = time.time()
        collected = []
        while time.time() - t0 < timeout_s and len(collected) < samples:
            if not self.xdpcHandler.packetsAvailable():
                time.sleep(0.02)
                continue
            vals = {}
            for device in self.xdpcHandler.connectedDots():
                packet = self.xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())
                name = device.deviceTagName()[8:12]
                if packet.containsOrientation() and name in ("DOT2", "DOT3"):
                    e = packet.orientationEuler()
                    vals[name] = (e.x(), e.y(), e.z())
            if "DOT2" in vals and "DOT3" in vals:
                r2,p2,y2 = vals["DOT2"]
                r3,p3,y3 = vals["DOT3"]
                yaw_diff = self.normalize_angle_deg(y3 - y2)
                q2 = self.euler_to_quaternion(self.normalize_angle_deg(r2), self.normalize_angle_deg(p2), self.normalize_angle_deg(y2))
                q3 = self.euler_to_quaternion(self.normalize_angle_deg(r3), self.normalize_angle_deg(p3), self.normalize_angle_deg(y3))
                ang = self.relative_angle_between_quaternions(q2, q3)
                collected.append((yaw_diff, ang))
        if not collected:
            print("[Calibration] nessun dato raccolto.")
            return None, None
        yaw_avg = sum(c[0] for c in collected) / len(collected)
        ang_avg = sum(c[1] for c in collected) / len(collected)
        self._yaw_offset = yaw_avg
        self._calibrated_max_angle = max(70.0, ang_avg * 1.05)
        print(f"[Calibration] yaw_offset impostato a {self._yaw_offset:.2f} deg, max_angle impostato a {self._calibrated_max_angle:.2f} deg (media su {len(collected)} campioni)")
        return self._yaw_offset, self._calibrated_max_angle
    # ---------- end helpers ----------

    def connect(self):
        print("spostato")

    def reconnect(self):
        self.connect()
            
    def sync(self,xdpcHandler):
    	# Sync
        self.talk("init synchronization")
        try:
            # viene impostato un manager per poter sincronizzare i device.
            manager = xdpcHandler.manager()
            # chiamo attraverso l'istanza della classe la connessione dei device. Stabilisce quale tra i device è ROOT
            deviceList = xdpcHandler.connectedDots()
            print(f"\nStarting sync for connected devices... Root node: {deviceList[-1].bluetoothAddress()}")
            print("This takes at least 14 seconds")

            # Fa partire la sincronizzazione dal dot ROOT.
            # Se ne abbiamo uno solo non ha senso sincronizzare
            if not manager.startSync(deviceList[-1].bluetoothAddress()):
                print(f"Could not start sync. Reason: {manager.lastResultText()}")
                # controlla che riesce a sincronizzarli tutti
                if manager.lastResult() != movelladot_pc_sdk.XRV_SYNC_COULD_NOT_START:
                    xdpcHandler.cleanup()
                    raise RuntimeError("Sync could not be started. Aborting.")
                # stop della sincronizzazione
                manager.stopSync()
                print(f"Retrying start sync after stopping sync")
                if not manager.startSync(deviceList[-1].bluetoothAddress()):
                    xdpcHandler.cleanup()
                    raise RuntimeError(f"Could not start sync. Reason: {manager.lastResultText()}. Aborting.")
        except OSError as e:
            self.talk(f"Unable to open device: {e}")
            logging.error(f"Unable to open device: {e}")
            self.running = False

    def oneDotSetupAvoidingSync(self,xdpcHandler):
    	# Sync
        self.talk("inizio syncronizzazione")
        try:
            # viene impostato un manager per poter sincronizzare i device.
            manager = xdpcHandler.manager()
            # chiamo attraverso l'istanza della classe la connessione dei device. Stabilisce quale tra i device è ROOT
            deviceList = xdpcHandler.connectedDots()

            # Payload mode
            # serve a dire quale sia l'ooutput che viene messo nella tabella.
            print("Putting devices into measurement mode.")
            for device in xdpcHandler.connectedDots():
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
    
    def cleanup(self):
        self.running = False
        for device in self.xdpcHandler.connectedDots():
            print(f"\nResetting heading to default for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
            if device.resetOrientation(movelladot_pc_sdk.XRM_DefaultAlignment):
                print("OK", end="", flush=True)
            else:
                print(f"NOK: {device.lastResultText()}", end="", flush=True)
            print("\n", end="", flush=True)

        print("\nStopping measurement...")
        for device in self.xdpcHandler.connectedDots():
            if not device.stopMeasurement():
                print("Failed to stop measurement.")
            if not device.disableLogging():
                print("Failed to disable logging.")
        
        if not self.xdpcHandler.manager().stopSync():
            print("Failed to stop sync.")

        self.xdpcHandler.cleanup()
        logging.info("Controller disconnected.")

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
        
    def are_motors_configured(self):
        # Only check the motor indices and not baudrate, since if the motor baudrates are incorrect,
        # a ConnectionError will be raised anyway.
        return self.loop
    
    def find_motor_indices(self):
        return self.xdpcHandler.connectedDots()
    
    #def set_bus_baudrate(self, baudrate): Data are in local using movella dot and there is not a port connected.

    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    def start_background_reader(self, interval=0.05):
        """Avvia un thread che aggiorna eulerPerSensor ogni `interval` secondi."""
        self._stop_reader = threading.Event()
        self._reader_thread = threading.Thread(target=self._background_read_loop, args=(interval,))
        self._reader_thread.daemon = True
        self._reader_thread.start()

    def stop_background_reader(self):
        """Ferma il thread di aggiornamento."""
        if hasattr(self, "_stop_reader"):
            self._stop_reader.set()
            self._reader_thread.join()

    def _background_read_loop(self, interval):
        """Loop eseguito nel thread per aggiornare eulerPerSensor."""
        while not self._stop_reader.is_set():
            try:
                self.updateDataFromSensors()
            except Exception as e:
                logging.error(f"Errore nel background reader: {e}")
            time.sleep(interval)

    
    def read(self):
        return self.read_with_motor_ids()
    
    def TODO_update(self,xdpcHandler):
        dot_reference="DOT0"
        print("update:Quanti dot connessi?",len(xdpcHandler.connectedDots()))
        if xdpcHandler.packetsAvailable():
            # prendi il valore attuale dei dot e lo stampi.
            #data = {dot_reference:[],"DOT1":[],"DOT2":[],"DOT3":[],"DOT4":[]}
            available=[]
            name = ""
            for device in xdpcHandler.connectedDots():
                packet = xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())
                print("DEBUG NOME",device.deviceTagName()[8:8+4])
                name = device.deviceTagName()[8:8+4]
                print(packet)
                #data[name]=packet
                euler = packet.orientationEuler()
                roll, pitch, yaw = euler.x(), euler.y(), euler.z()
                if len(self.weighted_average[name])>50:
                    self.weighted_average[name].pop(0)
                self.weighted_average[name].append([roll, pitch, yaw])
                available.append(name)
                # quando scelgo la payload mode non tutti hanno la misur dell'orientamento # TODO togliere in fase di test
                #if packet.containsOrientation():
                #    euler = packet.orientationEuler()
            #self._process_input(data)
            
            
            # se quel dot non è connesso, allora non far fallire il codice ma imposta un valore di default. 
            # Se un sensore non funziona allora gli lascio l'ultimo valore registrato. se invece c'è allora glielo assegno
            #eulerPerSensor = {"shoulder_pan": data["DOT0"][7],"shoulder_lift": data["DOT0"][6],"elbow_flex": data["DOT1"][6],"wrist_flex": data["DOT2"][6],"wrist_roll": data["DOT2"][5],"gripper": data["DOT3"][6]} # questo dizionario aveva un problema di fondo: se non c'è il dato allora da errore.
            if len(available)>=1:
                for check in [dot_reference,"DOT1","DOT2","DOT3","DOT4"]:
                    if check in available:
                        if check==dot_reference:
                            #self.eulerPerSensor["shoulder_pan"] = min(60,max(-60,-data[check][2]-70)) #6]  TODO FIX IT BASED ON ELBOW AND NOT SHOULDER.
                            #self.eulerPerSensor["shoulder_lift"] = min(60,max(-data[check][0]+110,0)) #7]
                            
                            """diff = -data[check][1]+110
                            angle=min(360-abs(diff),abs(diff))
                            self.eulerPerSensor["shoulder_lift"] = min(60,max(angle,0)) """

                            diff1 = np.mean(self.weighted_average[check][1])+180
                            angle1=min(360-abs(diff1),abs(diff1))
                            trend1 = 1 + (180 - angle1) / 180 * 2
                            #self.eulerPerSensor["shoulder_lift"] = min(170,max(angle/trend,0))
                            self.eulerPerSensor["shoulder_lift"] = min(170,max(angle1/trend1,75)) # in caso rimetti ad 80.
                            
                        elif check=="DOT1":
                            #self.eulerPerSensor["elbow_flex"] = min(data[check][2]-80,40) #7]
                            
                            """self.eulerPerSensor["elbow_flex"] = max(min(data[check][1]-80,40),0)"""

                            angle2 = 165-abs(np.mean(self.weighted_average[check][1]))
                            #angle=min(360-abs(diff),abs(diff)) 
                            #angle = abs(diff)
                            #self.eulerPerSensor["elbow_flex"] = min(160,max(angle,0))
                            self.eulerPerSensor["elbow_flex"] = min(140,max(angle2,40))

                        elif check=="DOT2":
                            self.eulerPerSensor["wrist_flex"] = max(0,min(-np.mean(self.weighted_average[check][0],110))) #6]
                            self.eulerPerSensor["wrist_roll"] = max(0,min(-np.mean(self.weighted_average[check][1],80))) #5]
                        elif check=="DOT3":
                            # angle3 = (data[check][1]+data[check][0])/2
                            
                            """diff3 = data[check][0] # TODO <-- FIX
                            angle3=min(360-abs(diff3),abs(diff3))
                            self.eulerPerSensor["gripper"] = min(max(angle3,0),50) #6]"""

                            input_data = np.array([[np.mean(self.weighted_average["DOT2"][0]),np.mean(self.weighted_average["DOT2"][1]),np.mean(self.weighted_average["DOT2"][2]),np.mean(self.weighted_average["DOT3"][0]),np.mean(self.weighted_average["DOT3"][1]),np.mean(self.weighted_average["DOT3"][2])]],dtype=np.float32)
                            prediction = self.model.predict(input_data)
                            angle3 = float(abs(prediction.squeeze()))  # squeeze rimuove dimensioni singole
                            self.eulerPerSensor["gripper"] = min(max(angle3,0),50) #6]
                            
                        elif check=="DOT4":
                            self.eulerPerSensor["shoulder_pan"] = min(self.limit_shoulder_pan,max(-self.limit_shoulder_pan,-np.mean(self.weighted_average[check][2]))*0.9) # Girato di 150. occorre definire l'algoritmo per evitare l'angolo giro.
                
                else:
                    # Invalid positions detected, do not update
                    logging.warning("Invalid motor positions detected. Changes have been discarded.")
                    #self.indicate_error()
        else:
            #self.connect()
            print("no data available because there are not movella dot connected!")
            self.current_positions =  [0,170,165,0,0,10] #[0, 30, 30, 0, 0, 10]
        #self.checkZposition(self.eulerPerSensor["shoulder_lift"],self.eulerPerSensor["elbow_flex"],self.eulerPerSensor["wrist_flex"])

    
    def updateDataFromSensors(self,xdpcHandler):
        dot_reference="DOT0"
        #print("update:Quanti dot connessi?",len(xdpcHandler.connectedDots()))
        if xdpcHandler.packetsAvailable():
            # prendi il valore attuale dei dot e lo stampi.
            data = {dot_reference:[],"DOT1":[],"DOT2":[],"DOT3":[],"DOT4":[]}
            available=[]
            name = ""
            for device in xdpcHandler.connectedDots():
                packet = xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())
                #print("DEBUG NOME",device.deviceTagName()[8:8+4])
                name = device.deviceTagName()[8:8+4]
                #print(packet)
                #data[name]=packet
                euler = packet.orientationEuler()
                roll, pitch, yaw = euler.x(), euler.y(), euler.z()
                data[name]=[roll, pitch, yaw]
                available.append(name)
                # quando scelgo la payload mode non tutti hanno la misur dell'orientamento # TODO togliere in fase di test
                #if packet.containsOrientation():
                #euler = packet.orientationEuler()
                #self._process_input(data)
            
            
            # se quel dot non è connesso, allora non far fallire il codice ma imposta un valore di default. 
            # Se un sensore non funziona allora gli lascio l'ultimo valore registrato. se invece c'è allora glielo assegno
            #eulerPerSensor = {"shoulder_pan": data["DOT0"][7],"shoulder_lift": data["DOT0"][6],"elbow_flex": data["DOT1"][6],"wrist_flex": data["DOT2"][6],"wrist_roll": data["DOT2"][5],"gripper": data["DOT3"][6]} # questo dizionario aveva un problema di fondo: se non c'è il dato allora da errore.
            if len(available)>=1:
                for check in [dot_reference,"DOT1","DOT2","DOT3","DOT4"]:
                    if check in available:
                        if check==dot_reference:
                            #self.eulerPerSensor["shoulder_pan"] = min(60,max(-60,-data[check][2]-70)) #6]  TODO FIX IT BASED ON ELBOW AND NOT SHOULDER.
                            #self.eulerPerSensor["shoulder_lift"] = min(60,max(-data[check][0]+110,0)) #7]
                            
                            """diff = -data[check][1]+110
                            angle=min(360-abs(diff),abs(diff))
                            self.eulerPerSensor["shoulder_lift"] = min(60,max(angle,0)) """

                            diff1 = data[check][1]+180
                            angle1=min(360-abs(diff1),abs(diff1))
                            trend1 = 1 + (180 - angle1) / 180 * 2
                            #self.eulerPerSensor["shoulder_lift"] = min(170,max(angle/trend,0))
                            self.eulerPerSensor["shoulder_lift"] = min(170,max(angle1/trend1,75)) # in caso rimetti ad 80.
                            
                        elif check=="DOT1":
                            #self.eulerPerSensor["elbow_flex"] = min(data[check][2]-80,40) #7]
                            
                            """self.eulerPerSensor["elbow_flex"] = max(min(data[check][1]-80,40),0)"""

                            angle2 = 165-abs(data[check][1])
                            #angle=min(360-abs(diff),abs(diff)) 
                            #angle = abs(diff)
                            #self.eulerPerSensor["elbow_flex"] = min(160,max(angle,0))
                            self.eulerPerSensor["elbow_flex"] = min(140,max(angle2,40))

                        elif check=="DOT2":
                            self.eulerPerSensor["wrist_flex"] = max(0,min(-data[check][0],110)) #6]
                            self.eulerPerSensor["wrist_roll"] = max(0,min(-data[check][1],80)) #5]
                        elif check=="DOT3":
                            if ("DOT2" in available and "DOT3" in available  and "DOT2" in data and "DOT3" in data and len(data["DOT2"]) >= 3 and len(data["DOT3"]) >= 3):
                                dot2 = (data["DOT2"][0], data["DOT2"][1], data["DOT2"][2])
                                dot3 = (data["DOT3"][0], data["DOT3"][1], data["DOT3"][2])
                                print("DOT2: ",dot2)
                                print("DOT3: ",dot3)
                                g_val, angle_rel, used_max = self.gripper_from_euler(dot2, dot3,
                                                                                    max_angle=None,
                                                                                    yaw_offset=None,
                                                                                    prev=self._gripper_prev,
                                                                                    alpha=self.DEFAULT_ALPHA,
                                                                                    auto_calibrate=True)

                                print(f"DEBUG gripper calc: angle_rel={angle_rel:.2f} used_max={used_max:.2f} => g_val={g_val:.2f}")
                                self._gripper_prev = g_val
                                # mappa finale e clamp al range motore
                                self.eulerPerSensor["gripper"] = min(max(g_val, 0.0), self.GRIPPER_LIMIT)
                                print("DEBUG gripper:", g_val, "clamped:", self.eulerPerSensor["gripper"])
                            
                        elif check=="DOT4":
                            self.eulerPerSensor["shoulder_pan"] = min(self.limit_shoulder_pan,max(-self.limit_shoulder_pan,-data[check][2])*0.9) # Girato di 150. occorre definire l'algoritmo per evitare l'angolo giro.
                
                else:
                    # Invalid positions detected, do not update
                    logging.warning("Invalid motor positions detected. Changes have been discarded.")
                    #self.indicate_error()
        else:
            #self.connect()
            #print("no data available because there are not movella dot connected!")
            self.current_positions =  [0,170,165,0,0,10] #[0, 30, 30, 0, 0, 10]
        #self.checkZposition(self.eulerPerSensor["shoulder_lift"],self.eulerPerSensor["elbow_flex"],self.eulerPerSensor["wrist_flex"])

    def read_with_motor_ids(self):
        try:
            self.updateDataFromSensors(self.xdpcHandler)
            
            self.temp_positions = self.current_positions.copy()
            #print("read_with_motor_ids",self.temp_positions)
            #print("self.eulerPerSensor",self.eulerPerSensor)
            
            self.temp_positions[0] = self.eulerPerSensor["shoulder_pan"]
            self.temp_positions[1] = self.eulerPerSensor["shoulder_lift"]

            self.temp_positions[2] = self.eulerPerSensor["elbow_flex"]

            self.temp_positions[3] = self.eulerPerSensor["wrist_roll"]
            self.temp_positions[4] = self.eulerPerSensor["wrist_flex"]

            self.temp_positions[5] = self.eulerPerSensor["gripper"]

            # TODO: Controllo sui salti perchè possono essere dannosi. se fa un salto di valore da un valore a un altro con più di tot allora non farlo. 90 gradi per gli altri e 40 per il gripper.

            
            # Perform eligibility check
            if self._is_position_valid(self.temp_positions):
                # Atomic update: all positions are valid, apply the changes
                self.current_positions = self.temp_positions.copy()
            else:
                # Invalid positions detected, d_is_posio not update
                logging.warning("Invalid motor positions detected. Changes have been discarded.")
                #self.indicate_error()

            return self.current_positions

        except KeyboardInterrupt:
            logging.info("Interruzione manuale ricevuta (Ctrl+C). Pulizia e uscita.")
            self.cleanup()  # Se hai una funzione di pulizia, altrimenti crea una
            
        except Exception as e:
            logging.error(f"Error reading from device: {e}")
            time.sleep(1)  # Wait before retrying
            # TODO RICHIEDO LA CONNESSIONE.
            self.current_positions =  [0, 170, 165, 0, 0, 0]
            return self.current_positions
            #self.connect()
            #self.sync()
    
    def write(self, data_name, values: int | float | np.ndarray, motor_names: str | list[str] | None = None):

        start_time = time.perf_counter()

        import scservo_sdk as scs

        if motor_names is None:
            motor_names = self.motor_names

        if isinstance(motor_names, str):
            motor_names = [motor_names]

        if isinstance(values, (int, float, np.integer)):
            values = [int(values)] * len(motor_names)

        values = np.array(values)

        motor_ids = []
        models = []
        for name in motor_names:
            motor_idx, model = self.motors[name]
            motor_ids.append(motor_idx)
            models.append(model)

        if data_name in CALIBRATION_REQUIRED and self.calibration is not None:
            values = self.revert_calibration(values, motor_names)

        values = values.tolist()

        assert_same_address(self.model_ctrl_table, models, data_name)
        addr, bytes = self.model_ctrl_table[model][data_name]
        group_key = get_group_sync_key(data_name, motor_names)

        init_group = data_name not in self.group_readers
        if init_group:
            self.group_writers[group_key] = scs.GroupSyncWrite(
                self.port_handler, self.packet_handler, addr, bytes
            )

        #print("values - write",values)

        for idx, value in zip(motor_ids, values, strict=True):
            data = convert_to_bytes(value, bytes, self.mock)
            if init_group:
                self.group_writers[group_key].addParam(idx, data)
            else:
                self.group_writers[group_key].changeParam(idx, data)

        comm = self.group_writers[group_key].txPacket()
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(
                f"Write failed due to communication error on port {self.port} for group_key {group_key}: "
                f"{self.packet_handler.getTxRxResult(comm)}"
            )

        # log the number of seconds it took to write the data to the motors
        delta_ts_name = get_log_name("delta_timestamp_s", "write", data_name, motor_names)
        self.logs[delta_ts_name] = time.perf_counter() - start_time

        # TODO(rcadene): should we log the time before sending the write command?
        # log the utc time when the write has been completed
        ts_utc_name = get_log_name("timestamp_utc", "write", data_name, motor_names)
        self.logs[ts_utc_name] = capture_timestamp_utc()

    
    def write_with_motor_ids(self):
        #return self.read_with_motor_ids()
        print("write with motor ids")
        return self.current_positions

    def checkZposition(self, braccio_angle,gomito_angle,polso_angle, braccio_length=14.2, gomito_length=16, polso_length=18, altezza_base=11):
        z = 0
        z_braccio = 0
        z_gomito = 0
        z_polso = 0
        print("braccio_angle",braccio_angle,"braccio_length",braccio_length)
        pi_greco = np.pi
        """if braccio_angle<90:
            z_braccio = braccio_length*np.sin(braccio_angle*pi_greco/180)
        else:
            z_braccio = braccio_length*np.sin(braccio_angle*pi_greco/180)
        if gomito_angle<90:
            z_gomito = gomito_length*np.sin(gomito_angle)
        else:
            z_gomito = gomito_length*np.sin(gomito_angle)
        if polso_angle<90:
            z_polso = polso_length*np.sin(polso_angle)
        else:
            z_polso = polso_length*np.sin(polso_angle)"""
        alpha3 = gomito_angle+braccio_angle-90
        alpha5 = alpha3-polso_angle
        z_braccio = braccio_length * np.cos(braccio_angle*pi_greco/180)
        z_gomito = gomito_length * np.sin(alpha3*pi_greco/180)
        z_polso = polso_length * np.sin(alpha5*pi_greco/180)
        z=z_braccio+z_gomito-z_polso-altezza_base
        print("z_braccio",z_braccio,"z_gomito",z_gomito,"z_polso",0)
        print("altezza",z)
        

    def _is_position_valid(self, positions):

        allowed_ranges = {
            """"shoulder_pan": (-10, 190),
            "shoulder_lift": (-5, 185),
            "elbow_flex": (-5, 185),
            "wrist_flex": (-110, 110),
            "wrist_roll": (-110, 110),
            "gripper": (0, 100)"""
            # SOLO DI DEBUG
            "shoulder_pan": (-60, 60),
            "shoulder_lift": (-5, 170),
            "elbow_flex": (-5, 40),
            "wrist_flex": (-30, 30),
            "wrist_roll": (-30, 30),
            "gripper": (0, 70)
        }

        for motor, (min_val, max_val) in allowed_ranges.items():
            if motor in positions and not (min_val <= positions[motor] <= max_val):
                logging.error(
                    f"Motor '{motor}' position {positions[motor]} out of range [{min_val}, {max_val}]."
                )
                
                return False

        return True

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