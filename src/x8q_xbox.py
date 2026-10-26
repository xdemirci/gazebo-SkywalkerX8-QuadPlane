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
    "GUIDED": 15,
    "QHOVER": 18,
    "QLOITER": 19,
    "QLAND": 20,
    "QRTL": 21,
}
MODE_NAMES = {v: k for k, v in MODES.items()}
# modes where RC3 is a climb-rate stick (1500 = hold). Everything else = plane style.
VTOL_MODES = {18, 19, 20, 21}

# TARGET_ID = 0          # commands and RC override go only to this plane
THROTTLE_MIN = 1000
THROTTLE_MAX = 2000
THROTTLE_RATE = 1200     # plane style: PWM per second at full stick
TRANSITION_THROTTLE = 1600   # throttle when switching VTOL -> plane mode
VTOL_CLIMB_SCALE = 500   # was 400, stick bottom now reaches 1000
DEADZONE = 0.08

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


def expo(x, e):
    """e=0 linear, e=1 fully cubic (very soft around center)."""
    return (1 - e) * x + e * x ** 3


class X8QuadGCS:

    def __init__(self, root):
        self.root = root
        self.root.title("YER KONTROL İSTASYONU - X8 QUADPLANE")
        self.root.geometry("560x640")
        self.root.configure(padx=15, pady=15)

        self.running = True
        self.cleaned = False

        # --- MAVLink (SITL / Gazebo multicast) ---
        print("SITL (Gazebo) multicast bağlantısı dinleniyor...")
        # self.master = mavutil.mavlink_connection("mcast:239.255.145.50:14550")
        self.master = mavutil.mavlink_connection("udp:localhost:14550")

        self.master.wait_heartbeat()
        print(f"İlk Heartbeat alındı! Komut hedefi SYS_ID: {TARGET_ID}")

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
        self.prev_vtol = None          # regime of the previous loop

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
    def is_vtol(self):
        return self.mode_id in VTOL_MODES

    def poll_heartbeat(self, now):
        """Read the real mode and arm state of the target plane."""
        while True:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=False)
            if not msg:
                break
            if msg.get_srcSystem() != TARGET_ID or msg.get_srcComponent() != 1:
                continue
            self.armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            if now > self.mode_hold_until:
                self.mode_id = msg.custom_mode

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
            "\n"
            "UÇAK modunda : sol stick Y = gaz (kademeli, sabit kalır)\n"
            "VTOL modunda : sol stick Y = tırmanma hızı (ortada = sabit),\n"
            "               sol stick X = yaw"
        )
        tk.Label(
            map_frame, text=info_text, justify="left",
            font=("Consolas", 10), fg="darkblue",
        ).pack(anchor="w")

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
        regime = "VTOL" if self.is_vtol() else "UÇAK"
        arm = "ARMED" if self.armed else "DISARMED"
        self.status_var.set(f"{name}   |   {regime}   |   {arm}")
        self.status_lbl.config(bg="darkgreen" if self.is_vtol() else "navy")

    # ==========================================
    # CONTROL LOOP
    # ==========================================
    def axis(self, idx, num_axes):
        if idx >= num_axes:
            return 0.0
        v = self.joystick.get_axis(idx)
        return 0.0 if abs(v) < DEADZONE else v

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
        self.poll_heartbeat(now)

        if self.joystick:
            self.read_physical_buttons()
            num_axes = self.joystick.get_numaxes()
            vtol = self.is_vtol()

            # regime change: reset the throttle to something sensible
            if vtol != self.prev_vtol:
                if self.prev_vtol is not None:
                    if vtol:
                        self.current_pwm[3] = 1500      # climb-rate stick, hold
                        self.current_pwm[4] = 1500
                        print(" Kontrol: VTOL (gaz = tırmanma hızı)")
                    else:
                        self.current_pwm[3] = TRANSITION_THROTTLE
                        print(" Kontrol: UÇAK (gaz = kademeli)")
                self.prev_vtol = vtol

            if vtol:
                for ch, cfg in VTOL_CTRL.items():
                    self.stick_channel(ch, cfg, num_axes)
                # climb-rate stick: up = more, spring centered at 1500
                raw = self.axis(THROTTLE_AXIS, num_axes)
                target = 1500 - raw * VTOL_CLIMB_SCALE
                self.current_pwm[3] = 0.3 * target + 0.7 * self.current_pwm[3]
                yaw_pwm = int(self.current_pwm[4])
            else:
                for ch, cfg in PLANE_CTRL.items():
                    self.stick_channel(ch, cfg, num_axes)
                # rate throttle: stick changes it, released = holds
                raw = self.axis(THROTTLE_AXIS, num_axes)
                thr = self.current_pwm[3] - raw * THROTTLE_RATE * dt
                self.current_pwm[3] = max(THROTTLE_MIN, min(THROTTLE_MAX, thr))
                yaw_pwm = 65535                     # no rudder in plane mode
                self.pwm_labels[4].config(text="-")
            if now < self.arm_low_until:
                self.current_pwm[3] = THROTTLE_MIN
            self.update_bar(3, int(self.current_pwm[3]))
            if vtol:
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