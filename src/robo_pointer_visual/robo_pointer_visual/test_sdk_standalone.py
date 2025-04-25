# test_sdk_syncwrite_torque_recycle.py
import time, json, sys
from pathlib import Path
import scservo_sdk as scs

# --- Config port & servos (noms complets) ---
PORT           = "/dev/robot_arm"
BAUDRATE       = 1_000_000
MOTORS         = {
    "shoulder_pan":   1,
    "shoulder_lift":  2,
    "elbow_flex":     3,
}
ADDR_GOAL, LEN_GOAL     = 42, 2   # Goal_Position
ADDR_TORQUE             = 40
ADDR_RET_LEVEL          = 16      # Return Status Level EEPROM
ADDR_RET_DELAY          = 17      # Return Delay      EEPROM

# --- Charger la calibration JSON ---
calib_path = Path.home() / ".cache/lerobot/calibrations/so100/main_follower.json"
calib      = json.loads(calib_path.read_text())
drv        = calib["drive_mode"]
off        = calib["homing_offset"]
steps180   = 4096 // 2

def deg2step(motor_name: str, angle_deg: float) -> int:
    idx = calib["motor_names"].index(motor_name)
    # Convertit l’angle en pas, applique drive_mode & homing_offset
    raw = (angle_deg / 180.0) * steps180 * (-1 if drv[idx] == 1 else 1) - off[idx]
    return int(round(raw))

# --- Init port & handler ---
port   = scs.PortHandler(PORT)
packet = scs.PacketHandler(1)
if not port.openPort():    sys.exit(f"❌ impossible d'ouvrir {PORT}")
if not port.setBaudRate(BAUDRATE): sys.exit("❌ impossible de régler le baudrate")

# --- Configurer EEPROM (pas d’ACK, pas de délai) + torque ON ---
for mid in MOTORS.values():
    packet.write1ByteTxRx(port, mid, ADDR_RET_LEVEL, 0)  # plus d’ACK
    packet.write1ByteTxRx(port, mid, ADDR_RET_DELAY, 0)  # pas de délai
    packet.write1ByteTxRx(port, mid, ADDR_TORQUE, 1)     # torque ON
time.sleep(0.1)

# --- Créer le GroupSyncWrite ---
gsw = scs.GroupSyncWrite(port, packet, ADDR_GOAL, LEN_GOAL)

# --- Boucle de broadcast 0/0/90 ---
pan, lift, elbow = 0.0, 0.0, 90.0
cycle, errors   = 0, 0
print("Ctrl+C pour arrêter…\n")
try:
    while True:
        cycle += 1
        # calcul des pas
        steps = [
            deg2step("shoulder_pan",  pan),
            deg2step("shoulder_lift", lift),
            deg2step("elbow_flex",    elbow),
        ]
        # prépare le broadcast
        gsw.clearParam()
        for name, stp in zip(MOTORS.keys(), steps):
            mid = MOTORS[name]
            gsw.addParam(mid, [scs.SCS_LOBYTE(stp), scs.SCS_HIBYTE(stp)])
        # envoi
        res = gsw.txPacket()
        if res != scs.COMM_SUCCESS:
            print(f"[Cycle {cycle}] TX_FAIL ({res}) → réactivation torque")
            errors += 1
            for mid in MOTORS.values():
                packet.write1ByteTxRx(port, mid, ADDR_TORQUE, 1)
        time.sleep(0.1)  # ≈10 Hz

except KeyboardInterrupt:
    print("\nArrêt manuel…")

# --- Cleanup ---
for mid in MOTORS.values():
    packet.write1ByteTxRx(port, mid, ADDR_TORQUE, 0)
port.closePort()
print(f"Cycles: {cycle}, erreurs tx: {errors}")
