import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math

import serial
import serial.tools.list_ports


HOME_DX = 512
HOME_LX = 95

DX_MIN, DX_MAX = 0, 1023
LX_MIN, LX_MAX = 0, 240

LX_PER_DX = 0.22  # ajuste si requiere


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def clamp_int(x: float, lo: int, hi: int) -> int:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return int(round(x))


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


class MotionThread(threading.Thread):
    """
    - Retroceso: invierte avance/retroceso (fase global).
    - Girar: si está marcado, usa amplitudes de giro; si no, usa amplitudes de avance.
    - Horario: solo aplica si gira; controla el sentido del giro cambiando el signo
               del avance de fase SOLO para horizontales.
    - Fase horizontales fija en 90°.
    """
    def __init__(self, app, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.app = app
        self.stop_event = stop_event
        self.t0 = time.perf_counter()

    def run(self):
        while not self.stop_event.is_set():
            try:
                ids = self.app.selected_ids()
                if not ids:
                    time.sleep(0.2)
                    continue

                horizontals = set(self.app.horizontal_ids())

                freq = float(self.app.freq_var.get())
                rate = float(self.app.rate_var.get())
                if freq <= 0 or rate <= 0:
                    time.sleep(0.1)
                    continue

                sign_v = -1.0 if self.app.chk_reverse.get() else +1.0
                turning = bool(self.app.chk_turn.get())
                cw = bool(self.app.chk_cw.get())

                # En giro, el signo de horizontales cambia para definir horario/antihorario
                if turning:
                    sign_h = sign_v if cw else -sign_v
                else:
                    sign_h = sign_v

                # Modo: amplitudes y fases
                if turning:
                    Av_dx = float(self.app.turn_v_ticks.get())
                    Ah_dx = float(self.app.turn_h_ticks.get())
                    phi_v_deg = float(self.app.turn_phase_v_deg.get())
                    phi_h_deg = 90.0  # FIJO
                else:
                    Av_dx = float(self.app.move_v_ticks.get())
                    Ah_dx = float(self.app.move_h_ticks.get())
                    phi_v_deg = float(self.app.move_phase_v_deg.get())
                    phi_h_deg = 90.0  # FIJO

                Av_lx = Av_dx * LX_PER_DX
                Ah_lx = Ah_dx * LX_PER_DX

                t = time.perf_counter() - self.t0

                n = len(ids)
                for idx, rid in enumerate(ids):
                    phi_idx_deg = (360.0 * idx / n)

                    if rid in horizontals:
                        phase = 2 * math.pi * freq * t + deg2rad(phi_h_deg + sign_h * phi_idx_deg)
                        s = math.sin(phase)

                        if rid in (1, 2, 3):
                            raw = HOME_DX + Ah_dx * s
                            pos = clamp_int(raw, DX_MIN, DX_MAX)
                            self.app.send_line(f"N {rid} POS {pos}")
                        else:
                            raw = HOME_LX + Ah_lx * s
                            ang = clamp_int(raw, LX_MIN, LX_MAX)
                            self.app.send_line(f"N {rid} LX {ang}")

                    else:
                        phase = 2 * math.pi * freq * t + deg2rad(phi_v_deg + sign_v * phi_idx_deg)
                        s = math.sin(phase)

                        if rid in (1, 2, 3):
                            raw = HOME_DX + Av_dx * s
                            pos = clamp_int(raw, DX_MIN, DX_MAX)
                            self.app.send_line(f"N {rid} POS {pos}")
                        else:
                            raw = HOME_LX + Av_lx * s
                            ang = clamp_int(raw, LX_MIN, LX_MAX)
                            self.app.send_line(f"N {rid} LX {ang}")

                mode = "GIRO" if turning else ("RETROCESO" if self.app.chk_reverse.get() else "AVANCE")
                giro_txt = f" | {'Horario' if cw else 'Antihorario'}" if turning else ""

                self.app.set_status(
                    f"Activo | {mode}{giro_txt} | f={freq:.3f}Hz | rate={rate:.1f}Hz | "
                    f"V(A={Av_dx:.0f}, φ={phi_v_deg:.0f}°) | H(A={Ah_dx:.0f}, φ=90° fijo) | "
                    f"Horiz={sorted(horizontals)}"
                )

                time.sleep(1.0 / rate)

            except Exception as e:
                self.app.set_status(f"Detenido por error: {e}")
                self.app.stop_event.set()
                break


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Girar opcional | Amplitudes por modo | Horizontales φ=90° fijo")
        self.geometry("1080x700")

        self.ser = None
        self.wlock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None

        # Conexión
        self.port_var = tk.StringVar(value="")
        self.baud_var = tk.StringVar(value="115200")
        self.eol_var = tk.StringVar(value="LF")
        self.status_var = tk.StringVar(value="Desconectado")

        # Selección de módulos
        self.sel_all = tk.BooleanVar(value=True)
        self.sel_vars = {rid: tk.BooleanVar(value=True) for rid in range(1, 7)}
        self.horiz_vars = {rid: tk.BooleanVar(value=False) for rid in range(1, 7)}

        # Global
        self.freq_var = tk.StringVar(value="0.30")
        self.rate_var = tk.StringVar(value="30")

        # Checkboxes
        self.chk_reverse = tk.BooleanVar(value=False)  # avance/retroceso
        self.chk_turn = tk.BooleanVar(value=False)     # girar sí/no
        self.chk_cw = tk.BooleanVar(value=True)        # horario sí/no (solo si gira)

        # MODO AVANCE/RETROCESO
        self.move_v_ticks = tk.StringVar(value="200")
        self.move_h_ticks = tk.StringVar(value="20")
        self.move_phase_v_deg = tk.StringVar(value="0")   # editable
        # fase horizontal fija a 90°, no se expone

        # MODO GIRO
        self.turn_v_ticks = tk.StringVar(value="20")
        self.turn_h_ticks = tk.StringVar(value="200")
        self.turn_phase_v_deg = tk.StringVar(value="0")   # editable
        # fase horizontal fija a 90°, no se expone

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        top = ttk.LabelFrame(self, text="Conexión", padding=10)
        top.pack(fill="x", padx=10, pady=10)

        ttk.Label(top, text="COM:").grid(row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=18, values=list_serial_ports())
        self.port_cb.grid(row=0, column=1, padx=6, sticky="w")
        ttk.Button(top, text="Refrescar", command=self._refresh_ports).grid(row=0, column=2, padx=6)

        ttk.Label(top, text="Baudios:").grid(row=0, column=3, sticky="w", padx=(16, 0))
        ttk.Entry(top, textvariable=self.baud_var, width=10).grid(row=0, column=4, padx=6, sticky="w")

        ttk.Label(top, text="EOL:").grid(row=0, column=5, sticky="w", padx=(16, 0))
        ttk.Combobox(top, textvariable=self.eol_var, values=["LF", "CRLF"], width=6, state="readonly")\
            .grid(row=0, column=6, padx=6, sticky="w")

        ttk.Button(top, text="Conectar", command=self.connect).grid(row=0, column=7, padx=6)
        ttk.Button(top, text="Desconectar", command=self.disconnect).grid(row=0, column=8, padx=6)

        ttk.Label(top, textvariable=self.status_var).grid(row=1, column=0, columnspan=9, sticky="w", pady=(6, 0))

        sel = ttk.LabelFrame(self, text="Módulos activos y horizontales", padding=10)
        sel.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Checkbutton(sel, text="Todos", variable=self.sel_all, command=self._toggle_all)\
            .grid(row=0, column=0, sticky="w")

        for rid in range(1, 7):
            ttk.Checkbutton(sel, text=f"R{rid}", variable=self.sel_vars[rid])\
                .grid(row=0, column=rid, padx=(8, 0), sticky="w")

        ttk.Separator(sel, orient="horizontal").grid(row=1, column=0, columnspan=8, sticky="ew", pady=8)

        for rid in range(1, 7):
            ttk.Checkbutton(sel, text=f"Horizontal R{rid}", variable=self.horiz_vars[rid])\
                .grid(row=2, column=rid-1, padx=(0, 12), sticky="w")

        ttk.Button(sel, text="Intercambiar H<->V", command=self.swap_roles)\
            .grid(row=2, column=6, padx=(20, 0), sticky="e")

        wave = ttk.LabelFrame(self, text="Parámetros globales (frecuencia común)", padding=10)
        wave.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(wave, text="Frecuencia (Hz):").grid(row=0, column=0, sticky="w")
        ttk.Entry(wave, textvariable=self.freq_var, width=10).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(wave, text="Tasa envío (Hz):").grid(row=0, column=2, sticky="w", padx=(16, 0))
        ttk.Entry(wave, textvariable=self.rate_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ctrl = ttk.LabelFrame(self, text="Control", padding=10)
        ctrl.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Checkbutton(ctrl, text="Retroceso (desmarcado = avance)", variable=self.chk_reverse)\
            .grid(row=0, column=0, sticky="w", padx=(0, 20))

        ttk.Checkbutton(ctrl, text="Girar (si se marca, cambia amplitudes)", variable=self.chk_turn, command=self._update_turn_ui)\
            .grid(row=0, column=1, sticky="w", padx=(0, 20))

        self.chk_cw_btn = ttk.Checkbutton(ctrl, text="Horario (desmarcado = antihorario)", variable=self.chk_cw)
        self.chk_cw_btn.grid(row=0, column=2, sticky="w")

        move = ttk.LabelFrame(self, text="Modo AVANCE/RETROCESO", padding=10)
        move.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(move, text="V(A ticks):").grid(row=0, column=0, sticky="w")
        ttk.Entry(move, textvariable=self.move_v_ticks, width=10).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(move, text="H(A ticks):").grid(row=0, column=2, sticky="w", padx=(16, 0))
        ttk.Entry(move, textvariable=self.move_h_ticks, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(move, text="V(φ°):").grid(row=0, column=4, sticky="w", padx=(16, 0))
        ttk.Entry(move, textvariable=self.move_phase_v_deg, width=10).grid(row=0, column=5, padx=6, sticky="w")

        ttk.Label(move, text="H(φ°): 90° fijo").grid(row=0, column=6, sticky="w", padx=(16, 0))

        turn = ttk.LabelFrame(self, text="Modo GIRO (se usa solo si 'Girar' está marcado)", padding=10)
        turn.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(turn, text="V(A ticks):").grid(row=0, column=0, sticky="w")
        ttk.Entry(turn, textvariable=self.turn_v_ticks, width=10).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(turn, text="H(A ticks):").grid(row=0, column=2, sticky="w", padx=(16, 0))
        ttk.Entry(turn, textvariable=self.turn_h_ticks, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(turn, text="V(φ°):").grid(row=0, column=4, sticky="w", padx=(16, 0))
        ttk.Entry(turn, textvariable=self.turn_phase_v_deg, width=10).grid(row=0, column=5, padx=6, sticky="w")

        ttk.Label(turn, text="H(φ°): 90° fijo").grid(row=0, column=6, sticky="w", padx=(16, 0))

        ttk.Label(self, text=f"Nota: LX16A usa amplitud ≈ {LX_PER_DX} * ticks_DX.").pack(fill="x", padx=10, pady=(0, 10))

        act = ttk.LabelFrame(self, text="Acciones", padding=10)
        act.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Button(act, text="HOME", command=self.home).pack(side="left", padx=6)
        ttk.Button(act, text="Iniciar", command=self.start).pack(side="left", padx=6)
        ttk.Button(act, text="Detener", command=self.stop).pack(side="left", padx=6)

        self._update_turn_ui()

    def _update_turn_ui(self):
        if self.chk_turn.get():
            self.chk_cw_btn.state(["!disabled"])
        else:
            self.chk_cw_btn.state(["disabled"])

    # ---------------- Serial ----------------
    def _refresh_ports(self):
        self.port_cb["values"] = list_serial_ports()

    def _toggle_all(self):
        v = self.sel_all.get()
        for rid in range(1, 7):
            self.sel_vars[rid].set(v)

    def _eol_bytes(self) -> bytes:
        return b"\r\n" if self.eol_var.get() == "CRLF" else b"\n"

    def send_line(self, line: str):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Maestro no conectado")
        payload = line.strip().encode("utf-8") + self._eol_bytes()
        with self.wlock:
            self.ser.write(payload)

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Serial", "Seleccione el COM del Maestro.")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showwarning("Serial", "Baudios inválidos.")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            time.sleep(1.2)
        except Exception as e:
            messagebox.showerror("Serial", f"No fue posible abrir el puerto: {e}")
            self.ser = None
            return

        self.status_var.set(f"Conectado a {port} @ {baud} | EOL={self.eol_var.get()}")

    def disconnect(self):
        self.stop()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.status_var.set("Desconectado")

    def set_status(self, s: str):
        self.status_var.set(s)

    # ---------------- Roles ----------------
    def selected_ids(self):
        return [rid for rid in range(1, 7) if self.sel_vars[rid].get()]

    def horizontal_ids(self):
        ids = self.selected_ids()
        return [rid for rid in ids if self.horiz_vars[rid].get()]

    def swap_roles(self):
        ids = self.selected_ids()
        if not ids:
            messagebox.showwarning("Roles", "Seleccione al menos un módulo.")
            return
        for rid in ids:
            self.horiz_vars[rid].set(not self.horiz_vars[rid].get())

    # ---------------- HOME / start / stop ----------------
    def home(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("HOME", "Conecte primero el Maestro.")
            return
        self.stop()

        ids = self.selected_ids()
        if not ids:
            messagebox.showwarning("Módulos", "Seleccione al menos un módulo.")
            return

        for rid in ids:
            if rid in (1, 2, 3):
                self.send_line(f"N {rid} POS {HOME_DX}")
            else:
                self.send_line(f"N {rid} LX {HOME_LX}")
            time.sleep(0.03)

        self.status_var.set(f"HOME enviado a {ids}")

    def start(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Inicio", "Conecte primero el Maestro.")
            return
        if self.thread and self.thread.is_alive():
            return

        ids = self.selected_ids()
        if not ids:
            messagebox.showwarning("Módulos", "Seleccione al menos un módulo.")
            return

        self.stop_event.clear()
        self.thread = MotionThread(self, self.stop_event)
        self.thread.start()
        self.status_var.set("Movimiento activo")

    def stop(self):
        self.stop_event.set()

    def on_close(self):
        try:
            self.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
