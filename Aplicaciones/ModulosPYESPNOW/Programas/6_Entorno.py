import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math

import serial
import serial.tools.list_ports

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.ticker import MultipleLocator


# =========================
# Parámetros base (servos)
# =========================
HOME_DX = 512
HOME_LX = 95

DX_MIN, DX_MAX = 0, 1023
LX_MIN, LX_MAX = 0, 240

LX_PER_DX = 0.22
PHI_H_DEG_FIXED = 90.0


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def clamp_int(x: float, lo: int, hi: int) -> int:
    return max(lo, min(int(round(x)), hi))


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def theta_from_alignment(aln: str) -> float:
    if aln == "+X":
        return 0.0
    if aln == "+Y":
        return math.pi / 2.0
    if aln == "-X":
        return math.pi
    if aln == "-Y":
        return -math.pi / 2.0
    return 0.0


def build_manhattan_waypoints(ax, ay, bx, by, step_m, x_first=True):
    if step_m <= 0:
        step_m = 0.01

    wps = []
    x, y = ax, ay

    def walk_axis(cur, target):
        if abs(target - cur) < 1e-12:
            return []
        sgn = 1.0 if target > cur else -1.0
        pts = []
        v = cur
        while True:
            nxt = v + sgn * step_m
            if (sgn > 0 and nxt >= target) or (sgn < 0 and nxt <= target):
                pts.append(target)
                break
            pts.append(nxt)
            v = nxt
        return pts

    if x_first:
        for nx in walk_axis(x, bx):
            x = nx
            wps.append((x, y))
        for ny in walk_axis(y, by):
            y = ny
            wps.append((x, y))
    else:
        for ny in walk_axis(y, by):
            y = ny
            wps.append((x, y))
        for nx in walk_axis(x, bx):
            x = nx
            wps.append((x, y))

    if not wps or (wps[-1][0] != bx or wps[-1][1] != by):
        wps.append((bx, by))
    return wps


# ======================================================
# Hilo de movimiento
# ======================================================
class MotionThread(threading.Thread):
    def __init__(self, app, stop_event):
        super().__init__(daemon=True)
        self.app = app
        self.stop_event = stop_event
        self.t0 = time.perf_counter()
        self.last_t = time.perf_counter()

    def run(self):
        while not self.stop_event.is_set():
            try:
                if not self.app.ser or not self.app.ser.is_open:
                    time.sleep(0.2)
                    continue

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

                s = float(self.app.scale_m_per_model.get())
                if s <= 0:
                    s = 1.0

                wp = self.app.current_waypoint()
                if wp is None:
                    self.app.set_status("No hay waypoints. Detenido.")
                    self.stop_event.set()
                    break
                tx_r, ty_r = wp

                x_m, y_m, th = self.app.pose_model
                x_r = x_m * s
                y_r = y_m * s

                dist = math.hypot(tx_r - x_r, ty_r - y_r)
                self.app.nav_dist_var.set(f"{dist:.4f} m | WP {self.app.wp_index+1}/{len(self.app.waypoints)}")

                tol = float(self.app.tol_var.get())
                if dist <= tol:
                    if self.app.advance_waypoint():
                        self.app.set_status("Waypoint alcanzado. Continuando…")
                        time.sleep(0.05)
                        continue
                    else:
                        self.app.set_status("Destino final alcanzado. Enviando HOME…")
                        self.stop_event.set()
                        self.app.after(0, self.app.home)
                        break

                desired = math.atan2(ty_r - y_r, tx_r - x_r)
                err = wrap_pi(desired - th)

                dead = math.radians(float(self.app.deadband_deg_var.get()))
                turning = abs(err) > dead
                cw = err < 0

                if turning:
                    Av = float(self.app.turn_v_ticks.get())
                    Ah = float(self.app.turn_h_ticks.get())
                    phi_v = float(self.app.turn_phase_v_deg.get())
                else:
                    Av = float(self.app.move_v_ticks.get())
                    Ah = float(self.app.move_h_ticks.get())
                    phi_v = float(self.app.move_phase_v_deg.get())

                self._send_commands(ids, horizontals, freq, Av, Ah, phi_v, turning, cw)

                self._update_pose_model(Av, Ah, turning, cw)
                x_m, y_m, _ = self.app.pose_model
                self.app.traj_x.append(x_m * s)
                self.app.traj_y.append(y_m * s)

                mode = "GIRO" if turning else "RECTO"
                giro_txt = " | Horario" if turning and cw else (" | Antihorario" if turning else "")
                self.app.set_status(f"Seguimiento WP | {mode}{giro_txt} | dist={dist:.4f} m | f={freq:.3f} Hz")

                time.sleep(1.0 / rate)

            except Exception as e:
                self.app.set_status(f"Detenido por error: {e}")
                self.stop_event.set()

    def _send_commands(self, ids, horizontals, freq, Av, Ah, phi_v_deg, turning, cw):
        t = time.perf_counter() - self.t0

        # Sentido global (ajustado para “avance” según lo observado)
        sign_v = -1.0

        if turning:
            sign_h = sign_v if cw else -sign_v
        else:
            sign_h = sign_v

        Av_lx = Av * LX_PER_DX
        Ah_lx = Ah * LX_PER_DX

        n = len(ids)
        for i, rid in enumerate(ids):
            phi_idx_deg = (360.0 * i / n)

            if rid in horizontals:
                phase = 2 * math.pi * freq * t + deg2rad(PHI_H_DEG_FIXED + sign_h * phi_idx_deg)
                ss = math.sin(phase)
                if rid <= 3:
                    pos = clamp_int(HOME_DX + Ah * ss, DX_MIN, DX_MAX)
                    self.app.send_line(f"N {rid} POS {pos}")
                else:
                    ang = clamp_int(HOME_LX + Ah_lx * ss, LX_MIN, LX_MAX)
                    self.app.send_line(f"N {rid} LX {ang}")
            else:
                phase = 2 * math.pi * freq * t + deg2rad(phi_v_deg + sign_v * phi_idx_deg)
                ss = math.sin(phase)
                if rid <= 3:
                    pos = clamp_int(HOME_DX + Av * ss, DX_MIN, DX_MAX)
                    self.app.send_line(f"N {rid} POS {pos}")
                else:
                    ang = clamp_int(HOME_LX + Av_lx * ss, LX_MIN, LX_MAX)
                    self.app.send_line(f"N {rid} LX {ang}")

    def _update_pose_model(self, Av, Ah, turning, cw):
        now = time.perf_counter()
        dt = now - self.last_t
        self.last_t = now

        kv = float(self.app.kv_var.get())
        kw = float(self.app.kw_var.get())

        v = kv * max(0.0, Av - Ah)
        w = 0.0
        if turning:
            w = kw * max(0.0, Ah - Av) * (-1.0 if cw else +1.0)

        x_m, y_m, th = self.app.pose_model
        th = th + w * dt
        x_m = x_m + v * math.cos(th) * dt
        y_m = y_m + v * math.sin(th) * dt
        self.app.pose_model = (x_m, y_m, th)


# ======================================================
# Ventana de gráfica (Toplevel)
# ======================================================
class PlotWindow(tk.Toplevel):
    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("Recorrido X–Y (REAL, m)")
        self.geometry("980x720")

        self.fig = Figure(figsize=(9.2, 6.6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)

        self.line, = self.ax.plot([], [])
        self.ptA, = self.ax.plot([], [], marker="o")
        self.ptB, = self.ax.plot([], [], marker="x")
        self.plan_line, = self.ax.plot([], [], linestyle="--")

        # Flecha de orientación: quiver (actualizable sin borrar)
        self.heading = self.ax.quiver([0.0], [0.0], [0.0], [0.0],
                                      angles="xy", scale_units="xy", scale=1)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self._alive = True
        self.after(120, self._refresh)

    def _on_close(self):
        self._alive = False
        self.destroy()

    def _apply_fixed_axes(self):
        # Ejes solicitados: X=[0,1.90], Y=[0,1.20]
        xmin = 0.0
        xmax = 1.90
        ymin = 0.0
        ymax = 1.20

        try:
            tick = float(self.app.tick_var.get())
            if tick <= 0:
                tick = 0.10
        except Exception:
            tick = 0.10

        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)
        self.ax.xaxis.set_major_locator(MultipleLocator(tick))
        self.ax.yaxis.set_major_locator(MultipleLocator(tick))

    def _refresh(self):
        if not self._alive:
            return

        try:
            self.line.set_data(self.app.traj_x, self.app.traj_y)

            axr = float(self.app.ax_var.get())
            ayr = float(self.app.ay_var.get())
            bxr = float(self.app.bx_var.get())
            byr = float(self.app.by_var.get())
            self.ptA.set_data([axr], [ayr])
            self.ptB.set_data([bxr], [byr])

            if self.app.waypoints:
                xs = [axr] + [p[0] for p in self.app.waypoints]
                ys = [ayr] + [p[1] for p in self.app.waypoints]
                self.plan_line.set_data(xs, ys)
            else:
                self.plan_line.set_data([], [])

            # Flecha de orientación (pose actual)
            s = float(self.app.scale_m_per_model.get())
            if s <= 0:
                s = 1.0
            x_m, y_m, th = self.app.pose_model
            x_r = x_m * s
            y_r = y_m * s

            # Longitud de flecha (m): configurable si desea; por defecto 0.10 m
            L = 0.10
            u = L * math.cos(th)
            v = L * math.sin(th)

            self.heading.set_offsets([[x_r, y_r]])
            self.heading.set_UVC([u], [v])

            self._apply_fixed_axes()
            self.canvas.draw_idle()
        except Exception:
            pass
        finally:
            self.after(120, self._refresh)


# ======================================================
# Aplicación principal
# ======================================================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Cadena modular (6) A→B por rejilla | Gráfica en otra ventana")
        self.geometry("1250x520")

        self.ser = None
        self.wlock = threading.Lock()

        self.stop_event = threading.Event()
        self.thread = None

        self.pose_model = (0.0, 0.0, 0.0)

        self.traj_x = [0.0]
        self.traj_y = [0.0]

        self.waypoints = []
        self.wp_index = 0

        self.plot_win = None

        # UI vars
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.eol_var = tk.StringVar(value="LF")

        self.freq_var = tk.StringVar(value="0.30")
        self.rate_var = tk.StringVar(value="30")

        self.move_v_ticks = tk.StringVar(value="200")
        self.move_h_ticks = tk.StringVar(value="20")
        self.move_phase_v_deg = tk.StringVar(value="0")

        self.turn_v_ticks = tk.StringVar(value="20")
        self.turn_h_ticks = tk.StringVar(value="200")
        self.turn_phase_v_deg = tk.StringVar(value="0")

        self.ax_var = tk.StringVar(value="0.00")
        self.ay_var = tk.StringVar(value="0.00")
        self.bx_var = tk.StringVar(value="0.20")
        self.by_var = tk.StringVar(value="0.10")

        self.align_var = tk.StringVar(value="+X")

        self.step_m_var = tk.StringVar(value="0.10")
        self.x_first_var = tk.BooleanVar(value=True)

        self.tol_var = tk.StringVar(value="0.02")
        self.deadband_deg_var = tk.StringVar(value="10")
        self.nav_dist_var = tk.StringVar(value="—")

        self.kv_var = tk.StringVar(value="0.002")
        self.kw_var = tk.StringVar(value="0.0015")
        self.scale_m_per_model = tk.StringVar(value="0.01")

        # Tick para grilla (m); ejes se fijan por requisito a [0..1.90] y [0..1.20]
        self.tick_var = tk.StringVar(value="0.10")

        self.status_var = tk.StringVar(value="Desconectado")

        self.sel_all = tk.BooleanVar(value=True)
        self.sel_vars = {i: tk.BooleanVar(value=True) for i in range(1, 7)}
        self.horiz_vars = {i: tk.BooleanVar(value=False) for i in range(1, 7)}

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ============ Waypoints ============
    def current_waypoint(self):
        if 0 <= self.wp_index < len(self.waypoints):
            return self.waypoints[self.wp_index]
        return None

    def advance_waypoint(self):
        if self.wp_index + 1 < len(self.waypoints):
            self.wp_index += 1
            return True
        return False

    # ============ UI ============
    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)

        for i in range(3):
            root.columnconfigure(i, weight=1)

        col1 = ttk.Frame(root)
        col2 = ttk.Frame(root)
        col3 = ttk.Frame(root)

        col1.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        col2.grid(row=0, column=1, sticky="nsew", padx=(8, 8))
        col3.grid(row=0, column=2, sticky="nsew", padx=(8, 0))

        self._ui_serial(col1)
        self._ui_modules(col1)

        self._ui_global(col2)
        self._ui_gait(col2)
        self._ui_model_axes(col2)

        self._ui_points_grid(col3)
        self._ui_exec(col3)
        self._ui_status(col3)

    def _ui_serial(self, parent):
        f = ttk.LabelFrame(parent, text="Puerto Serial", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="COM").grid(row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(f, textvariable=self.port_var, values=list_serial_ports(), width=16)
        self.port_cb.grid(row=0, column=1, padx=6, sticky="w")
        ttk.Button(f, text="Refrescar", command=self._refresh_ports).grid(row=0, column=2, padx=6)

        ttk.Label(f, text="Baudios").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.baud_var, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))

        ttk.Label(f, text="EOL").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Combobox(f, textvariable=self.eol_var, values=["LF", "CRLF"], width=6, state="readonly")\
            .grid(row=1, column=3, padx=6, sticky="w", pady=(6, 0))

        ttk.Button(f, text="Conectar", command=self.connect).grid(row=2, column=1, padx=6, pady=(8, 0), sticky="w")
        ttk.Button(f, text="Desconectar", command=self.disconnect).grid(row=2, column=2, padx=6, pady=(8, 0), sticky="w")

    def _ui_modules(self, parent):
        f = ttk.LabelFrame(parent, text="Módulos (1..6)", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Checkbutton(f, text="Todos activos", variable=self.sel_all, command=self._toggle_all)\
            .grid(row=0, column=0, columnspan=6, sticky="w")

        ttk.Label(f, text="Activos:").grid(row=1, column=0, columnspan=6, sticky="w", pady=(6, 0))
        for rid in range(1, 7):
            r = 2 if rid <= 3 else 3
            c = (rid - 1) % 3
            ttk.Checkbutton(f, text=f"R{rid}", variable=self.sel_vars[rid])\
                .grid(row=r, column=c, padx=(0, 12), sticky="w")

        ttk.Separator(f, orient="horizontal").grid(row=4, column=0, columnspan=6, sticky="ew", pady=8)

        ttk.Label(f, text="Horizontales:").grid(row=5, column=0, columnspan=6, sticky="w")
        for rid in range(1, 7):
            r = 6 if rid <= 3 else 7
            c = (rid - 1) % 3
            ttk.Checkbutton(f, text=f"R{rid}", variable=self.horiz_vars[rid])\
                .grid(row=r, column=c, padx=(0, 12), sticky="w")

        ttk.Button(f, text="Intercambiar H<->V", command=self.swap_roles)\
            .grid(row=8, column=0, columnspan=6, sticky="e", pady=(8, 0))

    def _ui_global(self, parent):
        f = ttk.LabelFrame(parent, text="Parámetros globales", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="Frecuencia (Hz)").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.freq_var, width=10).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(f, text="Tasa envío (Hz)").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.rate_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

    def _ui_gait(self, parent):
        f1 = ttk.LabelFrame(parent, text="Avance (recto)", padding=10)
        f1.pack(fill="x", pady=(0, 10))
        ttk.Label(f1, text="V(A ticks)").grid(row=0, column=0, sticky="w")
        ttk.Entry(f1, textvariable=self.move_v_ticks, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f1, text="H(A ticks)").grid(row=0, column=2, sticky="w")
        ttk.Entry(f1, textvariable=self.move_h_ticks, width=10).grid(row=0, column=3, padx=6, sticky="w")
        ttk.Label(f1, text="V(φ°)").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f1, textvariable=self.move_phase_v_deg, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Label(f1, text="H(φ°)=90° fijo").grid(row=1, column=2, columnspan=2, sticky="w", pady=(6, 0))

        f2 = ttk.LabelFrame(parent, text="Giro", padding=10)
        f2.pack(fill="x", pady=(0, 10))
        ttk.Label(f2, text="V(A ticks)").grid(row=0, column=0, sticky="w")
        ttk.Entry(f2, textvariable=self.turn_v_ticks, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f2, text="H(A ticks)").grid(row=0, column=2, sticky="w")
        ttk.Entry(f2, textvariable=self.turn_h_ticks, width=10).grid(row=0, column=3, padx=6, sticky="w")
        ttk.Label(f2, text="V(φ°)").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f2, textvariable=self.turn_phase_v_deg, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Label(f2, text="H(φ°)=90° fijo").grid(row=1, column=2, columnspan=2, sticky="w", pady=(6, 0))

    def _ui_model_axes(self, parent):
        f = ttk.LabelFrame(parent, text="Modelo + Escala + Grilla", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="k_v").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.kv_var, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f, text="k_w").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.kw_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f, text="Escala (m / unidad-modelo)").grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.scale_m_per_model, width=10).grid(row=1, column=2, padx=6, sticky="w", pady=(8, 0))

        ttk.Separator(f, orient="horizontal").grid(row=2, column=0, columnspan=4, sticky="ew", pady=8)

        ttk.Label(f, text="Tick ejes (m)").grid(row=3, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.tick_var, width=10).grid(row=3, column=1, padx=6, sticky="w")
        ttk.Label(f, text="Ejes fijos: X=0..1.90, Y=0..1.20").grid(row=3, column=2, columnspan=2, sticky="w")

    def _ui_points_grid(self, parent):
        f = ttk.LabelFrame(parent, text="Navegación A→B por rejilla", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="A.x").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.ax_var, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f, text="A.y").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.ay_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f, text="B.x").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.bx_var, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Label(f, text="B.y").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.by_var, width=10).grid(row=1, column=3, padx=6, sticky="w", pady=(6, 0))

        ttk.Label(f, text="Paso rejilla (m)").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.step_m_var, width=10).grid(row=2, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Checkbutton(f, text="Primero X luego Y", variable=self.x_first_var)\
            .grid(row=2, column=2, columnspan=2, sticky="w", pady=(6, 0))

        ttk.Label(f, text="Tol (m)").grid(row=3, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.tol_var, width=10).grid(row=3, column=1, padx=6, sticky="w", pady=(6, 0))

        ttk.Label(f, text="Deadband (°)").grid(row=3, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.deadband_deg_var, width=10).grid(row=3, column=3, padx=6, sticky="w", pady=(6, 0))

        ttk.Label(f, text="Alineación inicial").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(f, textvariable=self.align_var, values=["+X", "+Y", "-X", "-Y"], width=8, state="readonly")\
            .grid(row=4, column=1, padx=6, sticky="w", pady=(8, 0))

        ttk.Label(f, text="Dist / WP").grid(row=4, column=2, sticky="e", pady=(8, 0))
        ttk.Label(f, textvariable=self.nav_dist_var).grid(row=4, column=3, sticky="w", pady=(8, 0))

    def _ui_exec(self, parent):
        f = ttk.LabelFrame(parent, text="Ejecución", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Button(f, text="Abrir gráfica", command=self.open_plot_window).pack(side="left", padx=6)
        ttk.Button(f, text="HOME", command=self.home).pack(side="left", padx=6)
        ttk.Button(f, text="Iniciar A→B", command=self.start).pack(side="left", padx=6)
        ttk.Button(f, text="Detener", command=self.stop).pack(side="left", padx=6)

    def _ui_status(self, parent):
        f = ttk.LabelFrame(parent, text="Estado", padding=10)
        f.pack(fill="x")
        ttk.Label(f, textvariable=self.status_var, wraplength=420, justify="left").pack(anchor="w")

    # ============ Plot window ============
    def open_plot_window(self):
        if self.plot_win and self.plot_win.winfo_exists():
            self.plot_win.lift()
            return
        self.plot_win = PlotWindow(self)

    # ============ Serial ============
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
            messagebox.showwarning("Serial", "Seleccione el COM.")
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

    # ============ Módulos ============
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

    # ============ HOME / ejecución ============
    def home(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("HOME", "Conecte primero el Maestro.")
            return

        ids = self.selected_ids()
        if not ids:
            messagebox.showwarning("Módulos", "Seleccione al menos un módulo.")
            return

        self.stop_event.set()

        for rid in ids:
            if rid <= 3:
                self.send_line(f"N {rid} POS {HOME_DX}")
            else:
                self.send_line(f"N {rid} LX {HOME_LX}")
            time.sleep(0.03)

        self.set_status(f"HOME enviado a {ids}")

    def start(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Inicio", "Conecte primero el Maestro.")
            return

        ids = self.selected_ids()
        if not ids:
            messagebox.showwarning("Módulos", "Seleccione al menos un módulo.")
            return

        try:
            axr = float(self.ax_var.get())
            ayr = float(self.ay_var.get())
            bxr = float(self.bx_var.get())
            byr = float(self.by_var.get())
            step_m = float(self.step_m_var.get())
            s = float(self.scale_m_per_model.get())
            if s <= 0:
                s = 1.0
        except ValueError:
            messagebox.showwarning("Puntos", "A/B o paso inválidos.")
            return

        theta0 = theta_from_alignment(self.align_var.get())
        self.pose_model = (axr / s, ayr / s, theta0)

        self.waypoints = build_manhattan_waypoints(axr, ayr, bxr, byr, step_m, x_first=self.x_first_var.get())
        self.wp_index = 0

        self.traj_x = [axr]
        self.traj_y = [ayr]

        if self.thread and self.thread.is_alive():
            return

        self.stop_event.clear()
        self.thread = MotionThread(self, self.stop_event)
        self.thread.start()
        self.set_status("A→B por rejilla activo")

        if not (self.plot_win and self.plot_win.winfo_exists()):
            self.open_plot_window()

    def stop(self):
        self.stop_event.set()
        self.set_status("Detenido")

    def on_close(self):
        try:
            self.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
