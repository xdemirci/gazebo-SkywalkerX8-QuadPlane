import math
import os
import signal
import time
import tkinter as tk
import sys
import pygame
from pymavlink import mavutil
TARGET_ID = int(sys.argv[1])

# ArduPlane custom mode numbers (QuadPlane Q-modes are 17+)
MODES = {
    "MANUAL": 0,
    "FBWA": 5,
    "AUTO": 10,
    "RTL": 11,
    "LOITER": 12,
    "GUIDED": 15,
    "QHOVER": 18,
    "QLOITER": 19,
    "QLAND": 20,
    "QRTL": 21,
}
MODE_NAMES = {v: k for k, v in MODES.items()}
# modes where RC3 is a climb-rate stick (1500 = hold). Everything else = plane style.
VTOL_MODES = {18, 19, 20, 21}
GUIDED_ID = 15
# otonom modlar: joystick doğrudan sürmez. GUIDED'de hedefleri kaydırır,
# AUTO/RTL/LOITER'da sadece nötr "keep-alive" RC gönderilir.
NAV_MODES = {10, 11, 12}

# TARGET_ID = 0          # commands and RC override go only to this plane
THROTTLE_MIN = 1000
THROTTLE_MAX = 2000
THROTTLE_RATE = 1200     # plane style: PWM per second at full stick
TRANSITION_THROTTLE = 1600   # throttle when switching VTOL -> plane mode
VTOL_CLIMB_SCALE = 500   # was 400, stick bottom now reaches 1000
DEADZONE = 0.08

# ---------- GUIDED (plane otonom) ayarları ----------
# Parametre dosyasından: AIRSPEED_MIN 10 / CRUISE 22 / MAX 30,
# TECS_CLMB_MAX 5 / TECS_SINK_MAX 5, Q_ASSIST_SPEED 10
AIRSPEED_CRUISE = 22.0
SPEED_MIN, SPEED_MAX = 12.0, 28.0   # m/s, stall (10) ve max (30) ile arada pay
SPEED_STICK_RATE = 3.0      # m/s per s, sol stick tam ileri/geri
SPEED_ACCEL = 2.0           # m/s^2, uçağa verilen hızlanma limiti
ALT_MIN_REL, ALT_MAX_REL = 25.0, 150.0   # m, home'a göre (stick ile aşılamaz)
CLIMB_STICK_RATE = 3.0      # m/s, sağ stick tam (TECS_CLMB_MAX=5'in altında)
CLIMB_SIGN = 1.0            # +1: stick geri = tırman (uçak gibi), -1: ters
HEADING_STICK_RATE = 40.0   # derece/s, sağ stick tam sağ/sol
HEADING_MAX_LEAD = 45.0     # hedef, mevcut rotanın en fazla bu kadar önünde olur
HEADING_ACCEL = 5.0         # m/s^2 yatay ivme limiti (~27 derece bank)
HEADING_TYPE = 0            # 0 = course over ground, 1 = burun yönü (yaw)
GUIDED_EXPO = 0.4
GUIDED_CMD_PERIOD = 0.1     # s, hedefleri 10 Hz tekrar gönder
NAV_KEEPALIVE_THROTTLE = 1450   # otonom modda gönderilen nötr gaz (nudge yok)
AX_TURN, AX_CLIMB, AX_SPEED = 3, 4, 1   # sağ X, sağ Y, sol Y
CMD_GUIDED_SPEED, CMD_GUIDED_ALT, CMD_GUIDED_HEADING = 43000, 43001, 43002

# per-channel control settings, one set per regime
PLANE_CTRL = {
    1: {"name": "Roll",  "axis": 3, "scale": 220, "expo": 0.6, "alpha": 0.2},
    2: {"name": "Pitch", "axis": 4, "scale": 320, "expo": 0.2, "alpha": 0.3},
}
VTOL_CTRL = {
    1: {"name": "Roll",  "axis": 3, "scale": 350, "expo": 0.3, "alpha": 0.3},
    2: {"name": "Pitch", "axis": 4, "scale": 350, "expo": 0.3, "alpha": 0.3},
    4: {"name": "Yaw",   "axis": 0, "scale": 300, "expo": 0.3, "alpha": 0.3},
}
THROTTLE_AXIS = 1        # left stick Y (up = more)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def expo(x, e):
    """e=0 linear, e=1 fully cubic (very soft around center)."""
    return (1 - e) * x + e * x ** 3


class X8QuadGCS:

    def __init__(self, root):
        self.root = root
        self.root.title("YER KONTROL İSTASYONU - X8 QUADPLANE")
        self.root.geometry("600x800")
        self.root.configure(padx=15, pady=15)

        self.running = True
        self.cleaned = False

        # --- MAVLink (SITL / Gazebo multicast) ---
        print("SITL (Gazebo) multicast bağlantısı dinleniyor...")
        # self.master = mavutil.mavlink_connection("mcast:239.255.145.50:14550")
        self.master = mavutil.mavlink_connection("udp:localhost:14550")

        self.master.wait_heartbeat()
        print(f"İlk Heartbeat alındı! Komut hedefi SYS_ID: {TARGET_ID}")
        self.request_streams()

        signal.signal(signal.SIGINT, lambda signum, frame: self.request_exit())
        self.root.protocol("WM_DELETE_WINDOW", self.request_exit)

        # --- Pygame & Mixer ---
        pygame.init()
        pygame.mixer.init()
        pygame.joystick.init()

        self.kill_sound = None
        sound_filename = "intestaller_killing_main_engine.mp3"
        if os.path.exists(sound_filename):
            try:
                self.kill_sound = pygame.mixer.Sound(sound_filename)
                print("Ses efekti başarıyla yüklendi!")
            except Exception as e:
                print(f"Ses yükleme hatası: {e}")
        else:
            print(f"UYARI: '{sound_filename}' bulunamadı.")

        self.joystick = None
        self.setup_joystick()

        # --- State ---
        self.current_pwm = {1: 1500, 2: 1500, 3: THROTTLE_MIN, 4: 1500}
        self.last_loop_time = time.monotonic()

        self.mode_id = None            # last known / commanded mode number
        self.mode_hold_until = 0.0     # ignore heartbeats briefly after a mode command
        self.armed = False
        self.arm_low_until = 0.0
        self.prev_regime = None        # regime of the previous loop

        # telemetry + GUIDED targets
        self.tel = {}
        self.tel_time = 0.0
        self.ack_warned = set()
        self.guided_synced = False
        self.tgt_heading = 0.0
        self.tgt_alt = 0.0
        self.tgt_speed = AIRSPEED_CRUISE
        self.next_guided_send = 0.0

        self.prev_button_states = {}
        self.bars = {}
        self.pwm_labels = {}

        self.create_widgets()
        self.update_and_send_loop()

    def setup_joystick(self):
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Logitech F310 Aktif: {self.joystick.get_name()}")
        else:
            print("HATA: F310 kolu bulunamadı!")

    # ==========================================
    # EXIT
    # ==========================================
    def request_exit(self):
        self.running = False

    def release_rc_override(self):
        release = [0, 0, 0, 0] + [65535] * 14   # 0 = give channel back
        for _ in range(3):
            try:
                self.master.mav.rc_channels_override_send(TARGET_ID, 0, *release)
            except Exception:
                pass
            time.sleep(0.02)

    def cleanup(self):
        if self.cleaned:
            return
        self.cleaned = True
        print("\nÇıkılıyor: RC override bırakılıyor...")
        self.release_rc_override()
        try:
            pygame.quit()
        except Exception:
            pass
        try:
            self.root.destroy()
        except tk.TclError:
            pass
        print("Kapatıldı.")

    # ==========================================
    # REGIME
    # ==========================================
    def regime(self):
        if self.mode_id in VTOL_MODES:
            return "VTOL"
        if self.mode_id == GUIDED_ID:
            return "GUIDED"
        if self.mode_id in NAV_MODES:
            return "NAV"
        return "PLANE"

    def is_vtol(self):
        return self.mode_id in VTOL_MODES

    def request_streams(self):
        """VFR_HUD (74) ve GLOBAL_POSITION_INT (33) mesajlarını 10 Hz iste."""
        for msg_id in (74, 33):
            try:
                self.master.mav.command_long_send(
                    TARGET_ID, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, msg_id, 100000, 0, 0, 0, 0, 0,
                )
            except Exception:
                pass

    def poll_mavlink(self, now):
        """Tek noktadan oku: heartbeat (mod/arm), telemetri, komut yanıtları."""
        for _ in range(300):
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "BAD_DATA" or msg.get_srcSystem() != TARGET_ID:
                continue
            if t == "HEARTBEAT":
                if msg.get_srcComponent() != 1:
                    continue
                self.armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                if now > self.mode_hold_until:
                    self.mode_id = msg.custom_mode
            elif t == "VFR_HUD":
                self.tel["airspeed"] = msg.airspeed
                self.tel["groundspeed"] = msg.groundspeed
            elif t == "GLOBAL_POSITION_INT":
                self.tel["rel_alt"] = msg.relative_alt / 1000.0
                gs = math.hypot(msg.vx, msg.vy) / 100.0
                if gs > 2.0:    # yeterince hızlıysa gerçek rota (COG)
                    self.tel["course"] = math.degrees(math.atan2(msg.vy, msg.vx)) % 360.0
                elif msg.hdg != 65535:
                    self.tel["course"] = msg.hdg / 100.0
                self.tel_time = now
            elif t == "COMMAND_ACK":
                self.handle_ack(msg)

    def handle_ack(self, msg):
        names = {CMD_GUIDED_SPEED: "GUIDED_CHANGE_SPEED",
                 CMD_GUIDED_ALT: "GUIDED_CHANGE_ALTITUDE",
                 CMD_GUIDED_HEADING: "GUIDED_CHANGE_HEADING"}
        if msg.command in names and msg.result != 0:
            key = (msg.command, msg.result)
            if key not in self.ack_warned:
                self.ack_warned.add(key)
                print(f" UYARI: {names[msg.command]} reddedildi (result={msg.result}). "
                      f"1=geçici red 2=denied 3=desteklenmiyor 4=failed. "
                      f"GUIDED modunda ve uçak havada mı?")

    def telemetry_ok(self, now):
        return ("rel_alt" in self.tel and "course" in self.tel
                and now - self.tel_time < 2.0)

    # ==========================================
    # MAVLINK COMMANDS (TARGET_ID only)
    # ==========================================
    def set_mode(self, mode_name):
        mode_id = MODES[mode_name]
        try:
            self.master.mav.command_long_send(
                TARGET_ID, 0, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0, 1, mode_id, 0, 0, 0, 0, 0,
            )
            # assume it works for now, heartbeat corrects us after 1.5 s
            self.mode_id = mode_id
            self.mode_hold_until = time.monotonic() + 1.5
            print(f" Mod: {mode_name}")
        except Exception as e:
            print(f"Mod değiştirme hatası: {e}")

    def reset_throttle(self):
        self.current_pwm[3] = THROTTLE_MIN

    def arm(self):
        try:
            self.reset_throttle()
            self.arm_low_until = time.monotonic() + 1.0
            self.master.mav.command_long_send(
                TARGET_ID, 0, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
            print(" ARM Komutu Gönderildi.")
        except Exception as e:
            print(f"ARM hatası: {e}")

    def disarm(self):
        try:
            self.reset_throttle()
            self.master.mav.command_long_send(
                TARGET_ID, 0, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
            print(" DISARM Komutu Gönderildi.")
        except Exception as e:
            print(f"DISARM hatası: {e}")

    def kill_switch(self):
        """EMERGENCY MOTOR CUT + MEME SOUND EFFECT"""
        if self.kill_sound:
            self.kill_sound.play()
        self.reset_throttle()
        try:
            self.master.mav.command_long_send(
                TARGET_ID, 0, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0,
            )
            print("🚨 KILL SWITCH TETİKLENDİ! Motorlar Zorla Kapatıldı!")
        except Exception as e:
            print(f"Kill Switch hatası: {e}")

    # ==========================================
    # GUIDED (plane): hedef kaydırma
    # ==========================================
    def sync_guided_targets(self):
        """Hedefleri uçağın şu anki durumuna eşitle (GUIDED'e girerken / R3)."""
        t = self.tel
        self.tgt_heading = t["course"]
        self.tgt_alt = clamp(t["rel_alt"], ALT_MIN_REL, ALT_MAX_REL)
        asp = t.get("airspeed", 0.0)
        self.tgt_speed = clamp(asp if asp >= SPEED_MIN else AIRSPEED_CRUISE,
                               SPEED_MIN, SPEED_MAX)
        self.guided_synced = True
        print(f" GUIDED hedefleri eşitlendi: rota {self.tgt_heading:.0f} derece, "
              f"irtifa {self.tgt_alt:.0f} m, hız {self.tgt_speed:.1f} m/s")

    def send_guided_targets(self):
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        try:
            # hız: p1 tip (0=airspeed), p2 hız, p3 ivme
            self.master.mav.command_int_send(
                TARGET_ID, 1, frame, CMD_GUIDED_SPEED, 0, 0,
                0, self.tgt_speed, SPEED_ACCEL, 0, 0, 0, 0)
            # irtifa: z = home'a göre metre
            self.master.mav.command_int_send(
                TARGET_ID, 1, frame, CMD_GUIDED_ALT, 0, 0,
                0, 0, 0, 0, 0, 0, self.tgt_alt)
            # yön: p1 tip, p2 derece, p3 yatay ivme limiti
            self.master.mav.command_int_send(
                TARGET_ID, 1, frame, CMD_GUIDED_HEADING, 0, 0,
                HEADING_TYPE, self.tgt_heading, HEADING_ACCEL, 0, 0, 0, 0)
        except Exception as e:
            print(f"GUIDED komut hatası: {e}")

    def update_guided(self, now, dt, num_axes):
        if not self.guided_synced:
            if not self.telemetry_ok(now):
                self.guided_var.set("GUIDED: telemetri bekleniyor...")
                return
            self.sync_guided_targets()

        dt = min(dt, 0.2)
        turn = expo(self.axis(AX_TURN, num_axes), GUIDED_EXPO)
        climb = expo(self.axis(AX_CLIMB, num_axes), GUIDED_EXPO) * CLIMB_SIGN
        spd = -expo(self.axis(AX_SPEED, num_axes), GUIDED_EXPO)   # stick yukarı = hızlan

        self.tgt_heading = (self.tgt_heading + turn * HEADING_STICK_RATE * dt) % 360.0
        if self.telemetry_ok(now):      # hedef rotadan çok uzağa kaçmasın
            err = wrap180(self.tgt_heading - self.tel["course"])
            if abs(err) > HEADING_MAX_LEAD:
                self.tgt_heading = (self.tel["course"]
                                    + math.copysign(HEADING_MAX_LEAD, err)) % 360.0
        self.tgt_alt = clamp(self.tgt_alt + climb * CLIMB_STICK_RATE * dt,
                             ALT_MIN_REL, ALT_MAX_REL)
        self.tgt_speed = clamp(self.tgt_speed + spd * SPEED_STICK_RATE * dt,
                               SPEED_MIN, SPEED_MAX)

        if now >= self.next_guided_send:
            self.send_guided_targets()
            self.next_guided_send = now + GUIDED_CMD_PERIOD

        t = self.tel
        self.guided_var.set(
            f"ROTA {self.tgt_heading:03.0f}° (şu an {t.get('course', 0):03.0f}°)   "
            f"İRTİFA {self.tgt_alt:.0f} m (şu an {t.get('rel_alt', 0):.0f})   "
            f"HIZ {self.tgt_speed:.1f} m/s (şu an {t.get('airspeed', 0):.1f})"
        )

    def on_regime_change(self, prev, new):
        self.current_pwm[1] = 1500
        self.current_pwm[2] = 1500
        if new == "VTOL":
            self.current_pwm[3] = 1500      # climb-rate stick, hold
            self.current_pwm[4] = 1500
            print(" Kontrol: VTOL (gaz = tırmanma hızı)")
        elif new == "PLANE":
            self.current_pwm[3] = TRANSITION_THROTTLE
            print(" Kontrol: UÇAK (gaz = kademeli)")
        elif new == "GUIDED":
            self.current_pwm[3] = NAV_KEEPALIVE_THROTTLE
            self.guided_synced = False
            print(" Kontrol: GUIDED (stick = rota/irtifa/hız hedefi)")
        else:
            self.current_pwm[3] = NAV_KEEPALIVE_THROTTLE
            print(" Kontrol: OTONOM (AUTO/RTL/LOITER, stick devre dışı)")

    # ==========================================
    # F310 BUTTONS
    # ==========================================
    def edge(self, name, pressed):
        """True only on the press."""
        rising = pressed and not self.prev_button_states.get(name, False)
        self.prev_button_states[name] = pressed
        return rising

    def dpad(self):
        j = self.joystick
        if j.get_numhats() > 0:
            return j.get_hat(0)
        if j.get_numaxes() > 7:         # some drivers expose the D-pad as axes
            return (int(round(j.get_axis(6))), -int(round(j.get_axis(7))))
        return (0, 0)

    def read_physical_buttons(self):
        if not self.joystick:
            return

        num_buttons = self.joystick.get_numbuttons()

        def btn(i):
            return bool(self.joystick.get_button(i)) if i < num_buttons else False

        num_axes = self.joystick.get_numaxes()
        lt_axis = self.joystick.get_axis(2) if num_axes > 2 else -1.0
        dx, dy = self.dpad()

        if self.edge("B", btn(1)):
            self.kill_switch()
        if self.edge("RB", btn(5)):
            self.arm()
        if self.edge("LB", btn(4)):
            self.disarm()

        # plane modes
        if self.edge("Y", btn(3)):
            self.set_mode("FBWA")
        if self.edge("X", btn(2)):
            self.set_mode("GUIDED")
        if self.edge("A", btn(0)):
            self.set_mode("AUTO")
        if self.edge("LT", lt_axis > 0.5):
            self.set_mode("MANUAL")

        # otonom güvenlik modları
        if self.edge("START", btn(7)):
            self.set_mode("LOITER")
        if self.edge("BACK", btn(6)):
            self.set_mode("RTL")
        if self.edge("R3", btn(10)):    # GUIDED hedeflerini şu anki duruma eşitle
            self.guided_synced = False

        # VTOL modes on the D-pad
        if self.edge("UP", dy > 0):
            self.set_mode("QHOVER")
        if self.edge("LEFT", dx < 0):
            self.set_mode("QLOITER")
        if self.edge("DOWN", dy < 0):
            self.set_mode("QLAND")
        if self.edge("RIGHT", dx > 0):
            self.set_mode("QRTL")

    # ==========================================
    # GUI
    # ==========================================
    def create_widgets(self):
        self.status_var = tk.StringVar(value="MOD: ?")
        self.status_lbl = tk.Label(
            self.root, textvariable=self.status_var,
            font=("Arial", 13, "bold"), fg="white", bg="gray", pady=6,
        )
        self.status_lbl.pack(fill="x", pady=(0, 10))

        map_frame = tk.LabelFrame(
            self.root,
            text="LOGITECH F310 Fiziksel Tuş Haritası",
            font=("Arial", 10, "bold"),
            padx=10,
            pady=8,
        )
        map_frame.pack(fill="x", pady=(0, 10))

        info_text = (
            "UÇAK  : Y FBWA   X GUIDED   A AUTO   LT MANUAL\n"
            "VTOL  : D-pad ↑ QHOVER  ← QLOITER  ↓ QLAND  → QRTL\n"
            "RB ARM   LB DISARM   B KILL SWITCH\n"
            "START LOITER   BACK RTL   R3 GUIDED hedeflerini eşitle\n"
            "\n"
            "UÇAK modunda : sol stick Y = gaz (kademeli, sabit kalır)\n"
            "VTOL modunda : sol stick Y = tırmanma hızı (ortada = sabit),\n"
            "               sol stick X = yaw\n"
            "GUIDED       : sol stick Y = hız hedefi, sağ stick X = rota,\n"
            "               sağ stick Y = irtifa (geri = tırman). Bırakınca tutar.\n"
            "AUTO/RTL/LOITER : stick devre dışı (nötr RC gönderilir)"
        )
        tk.Label(
            map_frame, text=info_text, justify="left",
            font=("Consolas", 10), fg="darkblue",
        ).pack(anchor="w")

        self.guided_var = tk.StringVar(value="GUIDED: -")
        tk.Label(
            self.root, textvariable=self.guided_var, justify="left",
            font=("Consolas", 10, "bold"), fg="purple",
        ).pack(anchor="w", pady=(0, 6))

        rc_frame = tk.LabelFrame(
            self.root, text="RC Kanalları",
            font=("Arial", 10, "bold"), padx=10, pady=8,
        )
        rc_frame.pack(fill="x", pady=5)

        for ch, name in [(1, "Roll"), (2, "Pitch"), (3, "Throttle"), (4, "Yaw")]:
            row = tk.Frame(rc_frame)
            row.pack(fill="x", pady=6)

            tk.Label(
                row, text=f"CH{ch} {name}:", width=12, anchor="w",
                font=("Arial", 10),
            ).pack(side="left")

            canvas = tk.Canvas(
                row, width=320, height=22, bg="lightgray",
                highlightthickness=1, highlightbackground="gray",
            )
            canvas.pack(side="left", padx=10)

            bar = canvas.create_rectangle(0, 0, 160, 22, fill="limegreen", outline="")
            canvas.create_line(160, 0, 160, 22, fill="black", dash=(2, 2))

            pwm_lbl = tk.Label(
                row, text=str(int(self.current_pwm[ch])), width=6,
                font=("Courier", 11, "bold"),
            )
            pwm_lbl.pack(side="left")

            self.bars[ch] = (canvas, bar)
            self.pwm_labels[ch] = pwm_lbl

    def update_bar(self, ch, pwm):
        bar_width = ((pwm - 1000) / 1000.0) * 320
        canvas, bar = self.bars[ch]
        canvas.coords(bar, 0, 0, bar_width, 22)
        self.pwm_labels[ch].config(text=f"{int(pwm)}")

    def update_status(self):
        name = MODE_NAMES.get(self.mode_id, f"MODE {self.mode_id}")
        reg = self.regime()
        label = {"VTOL": "VTOL", "PLANE": "UÇAK", "GUIDED": "GUIDED", "NAV": "OTONOM"}[reg]
        color = {"VTOL": "darkgreen", "PLANE": "navy",
                 "GUIDED": "#5b2a86", "NAV": "#8a4b00"}[reg]
        arm = "ARMED" if self.armed else "DISARMED"
        self.status_var.set(f"{name}   |   {label}   |   {arm}")
        self.status_lbl.config(bg=color)

    # ==========================================
    # CONTROL LOOP
    # ==========================================
    def axis(self, idx, num_axes):
        if idx >= num_axes:
            return 0.0
        v = self.joystick.get_axis(idx)
        if abs(v) < DEADZONE:
            return 0.0
        # ölü bölgeden sonra 0'dan başlat (zıplama olmasın), uçta yine 1.0
        return math.copysign((abs(v) - DEADZONE) / (1.0 - DEADZONE), v)

    def stick_channel(self, ch, cfg, num_axes):
        raw = expo(self.axis(cfg["axis"], num_axes), cfg["expo"])
        target = 1500 + raw * cfg["scale"]
        a = cfg["alpha"]
        self.current_pwm[ch] = a * target + (1.0 - a) * self.current_pwm[ch]
        self.update_bar(ch, int(self.current_pwm[ch]))

    def update_and_send_loop(self):
        if not self.running:
            self.cleanup()
            return

        now = time.monotonic()
        dt = now - self.last_loop_time
        self.last_loop_time = now

        pygame.event.pump()
        self.poll_mavlink(now)

        if self.joystick:
            self.read_physical_buttons()
            num_axes = self.joystick.get_numaxes()
            regime = self.regime()

            if regime != self.prev_regime:
                if self.prev_regime is not None:
                    self.on_regime_change(self.prev_regime, regime)
                self.prev_regime = regime

            yaw_pwm = 65535                         # varsayılan: yaw'a dokunma
            if regime == "VTOL":
                for ch, cfg in VTOL_CTRL.items():
                    self.stick_channel(ch, cfg, num_axes)
                # climb-rate stick: up = more, spring centered at 1500
                raw = self.axis(THROTTLE_AXIS, num_axes)
                target = 1500 - raw * VTOL_CLIMB_SCALE
                self.current_pwm[3] = 0.3 * target + 0.7 * self.current_pwm[3]
                yaw_pwm = int(self.current_pwm[4])
            elif regime == "PLANE":
                for ch, cfg in PLANE_CTRL.items():
                    self.stick_channel(ch, cfg, num_axes)
                # rate throttle: stick changes it, released = holds
                raw = self.axis(THROTTLE_AXIS, num_axes)
                thr = self.current_pwm[3] - raw * THROTTLE_RATE * dt
                self.current_pwm[3] = max(THROTTLE_MIN, min(THROTTLE_MAX, thr))
                self.pwm_labels[4].config(text="-")     # no rudder in plane mode
            else:
                # GUIDED / AUTO / RTL / LOITER: stick mixing'e karışmasın diye
                # roll/pitch nötr, gaz nötr (keep-alive, RC failsafe olmasın)
                self.current_pwm[1] = 1500
                self.current_pwm[2] = 1500
                self.current_pwm[3] = NAV_KEEPALIVE_THROTTLE
                self.update_bar(1, 1500)
                self.update_bar(2, 1500)
                self.pwm_labels[4].config(text="-")
                if regime == "GUIDED":
                    self.update_guided(now, dt, num_axes)
                else:
                    self.guided_var.set("GUIDED: - (otonom mod)")
            if regime != "GUIDED" and regime != "NAV":
                self.guided_var.set("GUIDED: -")
            if now < self.arm_low_until:
                self.current_pwm[3] = THROTTLE_MIN
            self.update_bar(3, int(self.current_pwm[3]))
            if regime == "VTOL":
                self.update_bar(4, yaw_pwm)
            self.update_status()

            rc_values = [65535] * 18                # 65535 = do not touch
            rc_values[0] = int(self.current_pwm[1])
            rc_values[1] = int(self.current_pwm[2])
            rc_values[2] = int(self.current_pwm[3])
            rc_values[3] = yaw_pwm

            try:
                self.master.mav.rc_channels_override_send(
                    TARGET_ID, 0, *rc_values
                )
            except Exception:
                pass

        self.root.after(50, self.update_and_send_loop)


if __name__ == "__main__":
    app = None
    root = None
    try:
        root = tk.Tk()
        app = X8QuadGCS(root)
        root.mainloop()
    except KeyboardInterrupt:
        print("\nCtrl+C alındı.")
    finally:
        if app is not None:
            app.cleanup()
        elif root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass