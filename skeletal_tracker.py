import socket
import tkinter as tk
import math
import struct
import time
import openvr

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0" 
UDP_PORT = 4242
BASELINE = 200     
MAX_DELTA = 185    
PIPE_NAME = r'\\.\pipe\vrapplication\input\glove\v2\right'

# --- UDP SETUP ---
print(f"Starting Pro-Tier Skeletal Tracker on port {UDP_PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.setblocking(False)

# --- STEAMVR SETUP ---
try:
    vr_sys = openvr.init(openvr.VRApplication_Background)
    vr_enabled = True
    print("Successfully hooked into SteamVR. Thumb and Index are alive.")
except openvr.OpenVRError as e:
    vr_sys = None
    vr_enabled = False
    print(f"[WARNING] SteamVR not running. Thumb and Index will be paralyzed. ({e})")

def get_vr_flex():
    if not vr_enabled:
        return 0.0, ("None", 0.0, 0.0, 0.0), 0.0
        
    right_id = None
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if vr_sys.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_Controller:
            if vr_sys.getControllerRoleForTrackedDeviceIndex(i) == openvr.TrackedControllerRole_RightHand:
                right_id = i
                break
                
    if right_id is not None:
        result, state = vr_sys.getControllerState(right_id)
        if result:
            trig_val = state.rAxis[1].x
            trig_touched = bool(state.ulButtonTouched & (1 << openvr.k_EButton_Axis1))
            index_flex = trig_val if trig_val > 0.05 else (0.2 if trig_touched else 0.0)

            thumb_btn = "None"
            thumb_flex = 0.0
            stick_x = state.rAxis[0].x
            stick_y = state.rAxis[0].y
            
            if bool(state.ulButtonPressed & (1 << openvr.k_EButton_A)):
                thumb_btn, thumb_flex = "A", 1.0
            elif bool(state.ulButtonTouched & (1 << openvr.k_EButton_A)):
                thumb_btn, thumb_flex = "A", 0.5
            elif bool(state.ulButtonPressed & (1 << openvr.k_EButton_ApplicationMenu)):
                thumb_btn, thumb_flex = "B", 1.0
            elif bool(state.ulButtonTouched & (1 << openvr.k_EButton_ApplicationMenu)):
                thumb_btn, thumb_flex = "B", 0.5
            elif bool(state.ulButtonPressed & (1 << openvr.k_EButton_SteamVR_Touchpad)):
                thumb_btn, thumb_flex = "Stick", 1.0
            elif bool(state.ulButtonTouched & (1 << openvr.k_EButton_SteamVR_Touchpad)):
                thumb_btn, thumb_flex = "Stick", 0.5
            
            grip_pressed = bool(state.ulButtonPressed & (1 << openvr.k_EButton_Grip))
            grip_touched = bool(state.ulButtonTouched & (1 << openvr.k_EButton_Grip))
            grip_flex = 1.0 if grip_pressed else (0.5 if grip_touched else 0.0)
            
            return index_flex, (thumb_btn, thumb_flex, stick_x, stick_y), grip_flex
            
    return 0.0, ("None", 0.0, 0.0, 0.0), 0.0

def normalize_pads(raw_vals):
    pcts = []
    for raw in raw_vals:
        delta = max(0, BASELINE - raw)
        pct = min(1.0, delta / MAX_DELTA)
        pcts.append(pct)
    return pcts

# --- GUI SETUP ---
root = tk.Tk()
root.title("DIY Knuckles: Full Sensor Fusion")
root.geometry("400x620")
root.configure(bg="#222")

canvas = tk.Canvas(root, width=400, height=450, bg="#222", highlightthickness=0)
canvas.pack(pady=5)

diag_text = tk.StringVar()
diag_text.set("Waiting for ESP32 packets...")
diag_label = tk.Label(root, textvariable=diag_text, bg="#222", fg="#00FF00", font=("Consolas", 10))
diag_label.pack(pady=5)

coupling_var = tk.DoubleVar(value=0.4)
coupling_slider = tk.Scale(root, from_=0.0, to=1.0, resolution=0.05, orient=tk.HORIZONTAL, 
                           variable=coupling_var, label="Distal Tendon Coupling", 
                           bg="#222", fg="#00FF00", highlightthickness=0, length=250)
coupling_slider.pack(pady=5)

def draw_skeleton(vr_data, flex_data):
    canvas.delete("skeleton")
    wrist = (200, 400)
    mcp = {"thumb": (120, 310), "index": (145, 230), "mid": (185, 210), "ring": (225, 220), "pinky": (265, 250)}
    
    palm_pts = [wrist, mcp["thumb"], mcp["index"], mcp["mid"], mcp["ring"], mcp["pinky"], wrist]
    for i in range(len(palm_pts)-1):
        canvas.create_line(palm_pts[i][0], palm_pts[i][1], palm_pts[i+1][0], palm_pts[i+1][1], fill="#00FF00", width=2, tags="skeleton")
        
    def draw_finger(start_pt, base_angle_deg, prox_flex, distal_flex, lengths):
        max_bend = math.radians(80) 
        coupling = coupling_var.get()
        effective_distal = min(1.0, max(distal_flex, prox_flex * coupling))
        
        a1 = math.radians(base_angle_deg) + (prox_flex * max_bend)
        a2 = a1 + (effective_distal * max_bend)
        a3 = a2 + (effective_distal * max_bend)
        
        j1 = (start_pt[0] + lengths[0]*math.cos(a1), start_pt[1] + lengths[0]*math.sin(a1))
        j2 = (j1[0] + lengths[1]*math.cos(a2), j1[1] + lengths[1]*math.sin(a2))
        j3 = (j2[0] + lengths[2]*math.cos(a3), j2[1] + lengths[2]*math.sin(a3))
        
        pts = [start_pt, j1, j2, j3]
        for i in range(3):
            canvas.create_line(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1], fill="#00FF00", width=3, tags="skeleton")
        for pt in pts:
            canvas.create_oval(pt[0]-4, pt[1]-4, pt[0]+4, pt[1]+4, fill="red", outline="red", tags="skeleton")

    index_flex, (thumb_btn, thumb_raw_flex, stick_x, stick_y), grip_flex = vr_data

    if thumb_btn == "A":
        thumb_angle, thumb_f = -100, thumb_raw_flex * 0.8
    elif thumb_btn == "B":
        thumb_angle, thumb_f = -120, thumb_raw_flex * 0.6
    elif thumb_btn == "Stick":
        thumb_angle = -140 + (stick_x * 30)
        thumb_f = 1.0 if thumb_raw_flex == 1.0 else min(1.0, max(0.0, 0.4 - (stick_y * 0.4)))
    else:
        thumb_angle, thumb_f = -150, 0.0

    draw_finger(mcp["thumb"], thumb_angle, thumb_f, thumb_f, [40, 30, 25])
    draw_finger(mcp["index"], -105, index_flex, index_flex, [55, 35, 25])
    draw_finger(mcp["mid"],   -90, flex_data[0], grip_flex, [60, 40, 25])
    draw_finger(mcp["ring"],  -75, flex_data[2], flex_data[3], [55, 35, 25])
    draw_finger(mcp["pinky"], -60, flex_data[4], flex_data[5], [40, 25, 20])
    
    return thumb_f # Return this to use it in the pipe payload

last_packet_id = 0 
last_ping_time = 0
last_flex_data = [0.0] * 6
opengloves_pipe = None

def update_gui():
    global last_packet_id, last_ping_time, last_flex_data, opengloves_pipe
    
    # 1. ESP32 Auto-Discovery Broadcast
    if last_packet_id == 0 and time.time() - last_ping_time > 1.0:
        try:
            sock.sendto(b'PING', ('192.168.1.255', UDP_PORT)) 
            last_ping_time = time.time()
            diag_text.set("Broadcasting discovery ping...")
        except OSError:
            pass

    # 2. Drain UDP Buffer
    data = None
    try:
        while True:
            raw_bytes, _ = sock.recvfrom(1024)
            data = raw_bytes
    except BlockingIOError:
        pass 

    if data and len(data) == 20:
        unpacked = struct.unpack('<IHHHHHHHH', data)
        packet_id = unpacked[0]
        if packet_id > last_packet_id:
            last_packet_id = packet_id
            last_flex_data = normalize_pads(unpacked[1:7])
            diag_text.set(f"ID: {packet_id} | I2C: {unpacked[7]}ms | Loop: {unpacked[8]}ms")

    # 3. Poll SteamVR Data
    vr_flex = get_vr_flex()
    index_flex, (thumb_btn, thumb_raw_flex, stick_x, stick_y), grip_flex = vr_flex

    # 4. Render Frame
    thumb_f = draw_skeleton(vr_flex, last_flex_data)

    # 5. Inject binary struct into OpenGloves Named Pipe
    if not opengloves_pipe:
        try:
            opengloves_pipe = open(PIPE_NAME, 'wb')
        except FileNotFoundError:
            pass # OpenGloves overlay isn't running or driver isn't loaded

    if opengloves_pipe:
        # Construct the OpenGloves v2 binary struct
        t = float(thumb_f)
        i = float(index_flex)
        m = float(max(last_flex_data[0], grip_flex))
        r = float(max(last_flex_data[2], last_flex_data[3]))
        p = float(max(last_flex_data[4], last_flex_data[5]))

        # Flexions (5 fingers x 4 joints)
        flexions = [
            t, t, t, t, # Thumb
            i, i, i, i, # Index
            m, m, m, m, # Middle
            r, r, r, r, # Ring
            p, p, p, p  # Pinky
        ]
        
        # Splays (5 fingers) - Set to 0.0 (neutral)
        splays = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Pack exactly 120 bytes: 25 floats, 2 floats, 8 booleans, 1 float
        # '<' ensures little-endian without implicit C-struct padding breaking the alignment
        payload = struct.pack('<25f 2f 8? f',
            *flexions, *splays,
            float(stick_x), float(stick_y),                # joyX, joyY
            False,                                         # joyButton
            bool(index_flex > 0.5),                        # trgButton
            bool(thumb_btn == "A" and thumb_raw_flex == 1.0), # aButton
            bool(thumb_btn == "B" and thumb_raw_flex == 1.0), # bButton
            bool(m > 0.8 and r > 0.8),                     # grab
            False,                                         # pinch
            False,                                         # menu
            False,                                         # calibrate
            float(index_flex)                              # trgValue
        )
        
        try:
            opengloves_pipe.write(payload)
            opengloves_pipe.flush()
        except IOError:
            # If SteamVR crashes or the pipe breaks, drop it and wait for next frame
            opengloves_pipe.close()
            opengloves_pipe = None

    root.after(10, update_gui)

def on_closing():
    print("\nShutting down...")
    sock.close()
    if opengloves_pipe:
        opengloves_pipe.close()
    if vr_enabled:
        openvr.shutdown()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)
draw_skeleton([0.0, ("None", 0.0, 0.0, 0.0), 0.0], last_flex_data)
root.after(10, update_gui)
root.mainloop()