# -*- coding: utf-8 -*-
"""
GUI Tkinter para:
- Selección de morfología (Serpiente, L, T, Cuadrúpedo)
- Ventana de gráficas con pestañas:
    1) Morfología: posiciones de módulos + orientación H/V
    2) Recorrido: trayectoria Manhattan + flecha del robot (pose)
- Conexión Serial y envío de comandos (DX: POS, LX: LX)
- Movimiento con conmutación Av/Ah por deadband (giro tipo su script)

CONDICIONES INICIALES (según imagen):
- COM: COM7 | Baudios: 115200 | EOL: LF
- Cantidad módulos: 5 | IDs: 3,6,4,5,1 | (Horizontales: ninguno marcado)
- Morfología: Cuadrúpedo
- Navegación: A=(0.000, 1.000), alineación +X; B=(1.000, 1.000)
- Paso(m): 0.010 | Primero X luego Y: habilitado
- Tol(m): 0.1 | Deadband(°): 5 | Tick ejes(m): 0.10
- Frecuencia(Hz): 0.40 | Tasa envío(Hz): 10
- Avance: V(A)=210, H(A)=10, V(φ)=0, H(φ)=90
- Giro:    V(A)=180, H(A)=210, V(φ)=0, H(φ)=90
- Giro: cambio de fase SOLO en un sentido: deshabilitado (checkbox apagado), dir=CW, Δφ=45
- Modelo: k_v=0.00010 | k_w=0.01
"""

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
from matplotlib.patches import Rectangle


# =========================
# Área de trabajo (m)
# =========================
X_MIN, X_MAX = 0.0, 1.90
Y_MIN, Y_MAX = 0.0, 1.20


# =========================
# Parámetros base (servos)
# =========================
HOME_DX = 512
HOME_LX = 95

DX_MIN, DX_MAX = 0, 1023
LX_MIN, LX_MAX = 0, 240

LX_PER_DX = 0.22
PHI_H_DEG_DEFAULT = 90.0


# =========================
# Utilidades
# =========================
def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def clamp_int(x: float, lo: int, hi: int) -> int:
    return max(lo, min(int(round(x)), hi))


def clamp_float(x: float, lo: float, hi: float) -> float:
    return max(lo, min(float(x), hi))


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


def parse_id_list(s: str):
    s = s.strip()
    if not s:
        return []
    out = []
    for part in s.split(","):
        p = part.strip()
        if not p:
            continue
        out.append(int(p))
    return out


def build_manhattan_waypoints(ax, ay, bx, by, step_m, x_first=True):
    """Waypoints en rejilla Manhattan desde A a B, sin obstáculos."""
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


# =========================
# Morfologías (posiciones en rejilla)
# =========================
def morphology_positions(name: str, n: int):
    """
    Devuelve lista de (cx, cy) de tamaño n.
    Coordenadas en "celdas" (solo dibujo del robot).
    """
    n = max(1, int(n))
    pts = []

    if name == "Serpiente":
        for i in range(n):
            pts.append((i, 0))
        return pts

    if name == "L":
        k = max(2, min(n, n // 2 + 1))
        for i in range(k):
            pts.append((i, 0))
        x0, y0 = pts[-1]
        for j in range(1, n - k + 1):
            pts.append((x0, -j))
        return pts

    if name == "T":
        tv = max(2, min(n, n // 2 + 1))
        th = n - tv
        for j in range(tv):
            pts.append((0, -j))
        if th <= 0:
            return pts
        used = 0
        step = 1
        while used < th:
            pts.append((step, 0))
            used += 1
            if used >= th:
                break
            pts.append((-step, 0))
            used += 1
            step += 1
        return pts[:n]

    if name == "Cuadrúpedo":
        pts.append((0, 0))  # cuerpo
        if n == 1:
            return pts
        legs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        i = 0
        while len(pts) < n and i < len(legs):
            pts.append(legs[i])
            i += 1
        if len(pts) >= n:
            return pts[:n]
        ext_dirs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        base = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        ext_len = 2
        while len(pts) < n:
            for b, d in zip(base, ext_dirs):
                if len(pts) >= n:
                    break
                bx, by = b
                dx, dy = d
                pts.append((bx + dx * (ext_len - 1), by + dy * (ext_len - 1)))
            ext_len += 1
        return pts[:n]

    for i in range(n):
        pts.append((i, 0))
    return pts


# ======================================================
# Ventana de gráficas con pestañas
# ======================================================
class PlotWindow(tk.Toplevel):
    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("Gráficas")
        self.geometry("1150x760")

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True)

        self.tab_morph = ttk.Frame(self.nb)
        self.tab_path = ttk.Frame(self.nb)
        self.nb.add(self.tab_morph, text="Morfología")
        self.nb.add(self.tab_path, text="Recorrido")

        # Figura Morfología
        self.fig_m = Figure(figsize=(8.5, 6.5), dpi=100)
        self.ax_m = self.fig_m.add_subplot(111)
        self.ax_m.set_aspect("equal", adjustable="box")
        self.ax_m.grid(True)
        self.canvas_m = FigureCanvasTkAgg(self.fig_m, master=self.tab_morph)
        self.canvas_m.get_tk_widget().pack(fill="both", expand=True)

        # Figura Recorrido
        self.fig_p = Figure(figsize=(8.5, 6.5), dpi=100)
        self.ax_p = self.fig_p.add_subplot(111)
        self.ax_p.set_xlabel("X (m)")
        self.ax_p.set_ylabel("Y (m)")
        self.ax_p.grid(True)
        self.canvas_p = FigureCanvasTkAgg(self.fig_p, master=self.tab_path)
        self.canvas_p.get_tk_widget().pack(fill="both", expand=True)

        # Artistas Recorrido
        self.plan_line, = self.ax_p.plot([], [], linestyle="--", alpha=0.6)
        self.traj_line, = self.ax_p.plot([], [], linewidth=2.0)
        self.robot_tip, = self.ax_p.plot([], [], linestyle="None", marker=(3, 0, 0), markersize=16)
        self.ptA, = self.ax_p.plot([], [], marker="o")
        self.ptB, = self.ax_p.plot([], [], marker="x")

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._alive = True
        self.after(150, self._refresh)

    def _on_close(self):
        self._alive = False
        self.destroy()

    def _apply_fixed_axes_path(self):
        try:
            tick = float(self.app.tick_var.get())
            if tick <= 0:
                tick = 0.10
        except Exception:
            tick = 0.10

        self.ax_p.set_xlim(X_MIN, X_MAX)
        self.ax_p.set_ylim(Y_MIN, Y_MAX)
        self.ax_p.xaxis.set_major_locator(MultipleLocator(tick))
        self.ax_p.yaxis.set_major_locator(MultipleLocator(tick))

    def _draw_morphology(self):
        self.ax_m.clear()
        self.ax_m.set_title("Morfología (H/V por módulo)")
        self.ax_m.grid(True)
        self.ax_m.set_aspect("equal", adjustable="box")

        ids = self.app.ids_runtime[:]
        pts = self.app.morph_positions[:]
        if not ids or not pts or len(ids) != len(pts):
            self.canvas_m.draw_idle()
            return

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        xmin, xmax = min(xs) - 1, max(xs) + 1
        ymin, ymax = min(ys) - 1, max(ys) + 1
        self.ax_m.set_xlim(xmin, xmax)
        self.ax_m.set_ylim(ymin, ymax)

        for rid, (cx, cy) in zip(ids, pts):
            is_h = self.app.horiz_var_by_id.get(rid, tk.BooleanVar(value=False)).get()
            face = "#2b7bff" if is_h else "#ff9b2b"  # H azul, V naranja
            r = Rectangle((cx - 0.45, cy - 0.30), 0.90, 0.60, facecolor=face, edgecolor="black", linewidth=1.5)
            self.ax_m.add_patch(r)
            self.ax_m.text(cx, cy, f"ID {rid}\n{'H' if is_h else 'V'}",
                           ha="center", va="center", fontsize=9, color="black")

        self.ax_m.text(0.02, 0.98, f"N módulos: {len(ids)}",
                       transform=self.ax_m.transAxes, ha="left", va="top", fontsize=10)

        self.canvas_m.draw_idle()

    def _draw_path(self):
        axr = float(self.app.ax_var.get())
        ayr = float(self.app.ay_var.get())
        bxr = float(self.app.bx_var.get())
        byr = float(self.app.by_var.get())
        self.ptA.set_data([axr], [ayr])
        self.ptB.set_data([bxr], [byr])

        wps = self.app.waypoints_runtime[:]
        if wps:
            xs_plan = [axr] + [p[0] for p in wps]
            ys_plan = [ayr] + [p[1] for p in wps]
            self.plan_line.set_data(xs_plan, ys_plan)
        else:
            self.plan_line.set_data([], [])

        self.traj_line.set_data(self.app.traj_x[:], self.app.traj_y[:])

        x_r, y_r, th = self.app.pose_runtime
        ang_deg = math.degrees(th) - 90.0
        self.robot_tip.set_data([x_r], [y_r])
        self.robot_tip.set_marker((3, 0, ang_deg))

        self._apply_fixed_axes_path()
        self.canvas_p.draw_idle()

    def _refresh(self):
        if not self._alive:
            return
        try:
            self._draw_morphology()
            self._draw_path()
        except Exception:
            pass
        finally:
            self.after(150, self._refresh)


# ======================================================
# Hilo de movimiento (giro tipo script)
# ======================================================
class MotionThread(threading.Thread):
    def __init__(self, app, stop_event):
        super().__init__(daemon=True)
        self.app = app
        self.stop_event = stop_event
        self.last_t = time.perf_counter()

    def run(self):
        while not self.stop_event.is_set():
            try:
                if not self.app.ser or not self.app.ser.is_open:
                    time.sleep(0.2)
                    continue

                freq = float(self.app.freq_var.get())
                rate = float(self.app.rate_var.get())
                if freq <= 0 or rate <= 0:
                    time.sleep(0.1)
                    continue

                tol = float(self.app.tol_var.get())
                dead = math.radians(float(self.app.deadband_deg_var.get()))

                now = time.perf_counter()
                dt = now - self.last_t
                self.last_t = now

                kv = float(self.app.kv_var.get())
                kw = float(self.app.kw_var.get())

                shift_enable = bool(self.app.turn_phase_shift_enable.get())
                shift_dir = self.app.turn_phase_shift_dir.get().strip()
                shift_deg = float(self.app.turn_phase_shift_deg.get())

                wp = self.app.current_wp()
                if wp is None:
                    self.app.after(0, self.app.home_all)
                    self.app.set_status("Finalizado: HOME enviado.")
                    self.stop_event.set()
                    break

                tx_r, ty_r = wp
                x_r, y_r, th = self.app.pose_runtime

                dist = math.hypot(tx_r - x_r, ty_r - y_r)
                if dist <= tol:
                    if not self.app.advance_wp():
                        continue
                    time.sleep(0.01)
                    continue

                desired = math.atan2(ty_r - y_r, tx_r - x_r)
                err = wrap_pi(desired - th)

                turning = abs(err) > dead
                cw = err < 0  # CW (derecha)

                if turning:
                    Av = float(self.app.turn_v_ticks.get())
                    Ah = float(self.app.turn_h_ticks.get())
                    phi_v = float(self.app.turn_phase_v_deg.get())
                    phi_h = float(self.app.turn_phase_h_deg.get())
                else:
                    Av = float(self.app.move_v_ticks.get())
                    Ah = float(self.app.move_h_ticks.get())
                    phi_v = float(self.app.move_phase_v_deg.get())
                    phi_h = float(self.app.move_phase_h_deg.get())

                extra_phi_h = 0.0
                if turning and shift_enable:
                    if shift_dir == "CW" and cw:
                        extra_phi_h = shift_deg
                    elif shift_dir == "CCW" and (not cw):
                        extra_phi_h = shift_deg

                t_chain = time.perf_counter() - self.app.t0

                lines = self.app.build_servo_lines(
                    t=t_chain, freq=freq,
                    Av=Av, Ah=Ah,
                    phi_v_deg=phi_v,
                    phi_h_deg=phi_h + extra_phi_h,
                    turning=turning, cw=cw
                )

                if lines:
                    self.app.send_lines_batch(lines)

                self.app.update_pose_model(Av=Av, Ah=Ah, turning=turning, cw=cw, kv=kv, kw=kw, dt=dt)
                self.app.set_status("Movimiento activo.")

                time.sleep(1.0 / rate)

            except Exception as e:
                self.app.set_status(f"Detenido por error: {e}")
                self.stop_event.set()


# ======================================================
# Aplicación principal
# ======================================================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Morfología)")
        self.geometry("1480x640")

        # Serial
        self.ser = None
        self.wlock = threading.Lock()

        # Thread
        self.stop_event = threading.Event()
        self.thread = None

        # Plot
        self.plot_win = None

        # Runtime
        self.ids_runtime = []
        self.morph_positions = []
        self.waypoints_runtime = []
        self.wp_index = 0
        self.pose_runtime = (0.0, 0.0, 0.0)  # (x,y,theta)
        self.traj_x = []
        self.traj_y = []
        self.t0 = time.perf_counter()

        # ====== UI vars (con valores iniciales según imagen) ======
        self.port_var = tk.StringVar(value="COM7")
        self.baud_var = tk.StringVar(value="115200")
        self.eol_var = tk.StringVar(value="LF")
        self.status_var = tk.StringVar(value="Desconectado")

        self.nmods_var = tk.IntVar(value=5)
        self.ids_var = tk.StringVar(value="3,6,4,5,1")

        self.morph_var = tk.StringVar(value="Cuadrúpedo")

        self.ax_var = tk.StringVar(value="0.000")
        self.ay_var = tk.StringVar(value="1.000")
        self.align_var = tk.StringVar(value="+X")
        self.bx_var = tk.StringVar(value="1.000")
        self.by_var = tk.StringVar(value="1.000")

        self.step_m_var = tk.StringVar(value="0.010")
        self.x_first_var = tk.BooleanVar(value=True)
        self.tol_var = tk.StringVar(value="0.1")
        self.deadband_deg_var = tk.StringVar(value="5")
        self.tick_var = tk.StringVar(value="0.10")

        self.freq_var = tk.StringVar(value="0.40")
        self.rate_var = tk.StringVar(value="10")

        self.move_v_ticks = tk.StringVar(value="210")
        self.move_h_ticks = tk.StringVar(value="10")
        self.move_phase_v_deg = tk.StringVar(value="0")
        self.move_phase_h_deg = tk.StringVar(value=str(PHI_H_DEG_DEFAULT))

        self.turn_v_ticks = tk.StringVar(value="180")
        self.turn_h_ticks = tk.StringVar(value="210")
        self.turn_phase_v_deg = tk.StringVar(value="0")
        self.turn_phase_h_deg = tk.StringVar(value=str(PHI_H_DEG_DEFAULT))

        self.turn_phase_shift_enable = tk.BooleanVar(value=False)  # en la imagen está desmarcado
        self.turn_phase_shift_dir = tk.StringVar(value="CW")
        self.turn_phase_shift_deg = tk.StringVar(value="45")

        self.kv_var = tk.StringVar(value="0.00010")
        self.kw_var = tk.StringVar(value="0.01")

        # H/V dinámico por ID (inicialmente todos False, como en la imagen)
        self.horiz_var_by_id = {}

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self._rebuild_hv_checks()
        self.update_morphology_runtime()

    # ================= UI =================
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
        self._ui_morphology(col1)

        self._ui_navigation(col2)
        self._ui_gait(col2)
        self._ui_turn_phase_shift(col2)
        self._ui_model(col2)

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
        f = ttk.LabelFrame(parent, text="Módulos", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="Cantidad").grid(row=0, column=0, sticky="w")
        ttk.Combobox(f, textvariable=self.nmods_var, values=list(range(1, 25)), width=6, state="readonly")\
            .grid(row=0, column=1, padx=6, sticky="w")
        ttk.Button(f, text="Aplicar", command=self._apply_nmods).grid(row=0, column=2, padx=6, sticky="w")

        ttk.Label(f, text="IDs (coma)").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.ids_var, width=26).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0), columnspan=2)

        self.hv_frame = ttk.Frame(f)
        self.hv_frame.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(10, 0))

        ttk.Label(f, text="Nota: H/V se usa para decidir qué onda aplica a cada ID.").grid(
            row=3, column=0, columnspan=3, sticky="w", pady=(8, 0)
        )

    def _ui_morphology(self, parent):
        f = ttk.LabelFrame(parent, text="Morfología", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="Tipo").grid(row=0, column=0, sticky="w")
        ttk.Combobox(f, textvariable=self.morph_var,
                     values=["Serpiente", "L", "T", "Cuadrúpedo"], width=12, state="readonly")\
            .grid(row=0, column=1, padx=6, sticky="w")

        ttk.Button(f, text="Sugerir H/V", command=self.suggest_hv_from_morph).grid(row=0, column=2, padx=6, sticky="w")
        ttk.Button(f, text="Actualizar dibujo", command=self.update_morphology_runtime).grid(row=0, column=3, padx=6, sticky="w")

    def _ui_navigation(self, parent):
        f = ttk.LabelFrame(parent, text="Navegación", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="A.x").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.ax_var, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f, text="A.y").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.ay_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f, text="Alineación").grid(row=0, column=4, sticky="w")
        ttk.Combobox(f, textvariable=self.align_var, values=["+X", "+Y", "-X", "-Y"], width=6, state="readonly")\
            .grid(row=0, column=5, padx=6, sticky="w")

        ttk.Label(f, text="B.x").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.bx_var, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(8, 0))
        ttk.Label(f, text="B.y").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.by_var, width=10).grid(row=1, column=3, padx=6, sticky="w", pady=(8, 0))

        ttk.Label(f, text="Paso (m)").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.step_m_var, width=10).grid(row=2, column=1, padx=6, sticky="w", pady=(8, 0))
        ttk.Checkbutton(f, text="Primero X luego Y", variable=self.x_first_var)\
            .grid(row=2, column=2, columnspan=2, sticky="w", pady=(8, 0))

        ttk.Label(f, text="Tol (m)").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.tol_var, width=10).grid(row=3, column=1, padx=6, sticky="w", pady=(8, 0))

        ttk.Label(f, text="Deadband (°)").grid(row=3, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.deadband_deg_var, width=10).grid(row=3, column=3, padx=6, sticky="w", pady=(8, 0))

        ttk.Label(f, text="Tick ejes (m)").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.tick_var, width=10).grid(row=4, column=1, padx=6, sticky="w", pady=(8, 0))
        ttk.Label(f, text="Ejes fijos: X=0..1.90, Y=0..1.20").grid(row=4, column=2, columnspan=4, sticky="w", pady=(8, 0))

    def _ui_gait(self, parent):
        f = ttk.LabelFrame(parent, text="Movimiento (igual estilo del script)", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="Frecuencia (Hz)").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.freq_var, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f, text="Tasa envío (Hz)").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.rate_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        f1 = ttk.LabelFrame(f, text="Avance (recto)", padding=8)
        f1.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        ttk.Label(f1, text="V(A ticks)").grid(row=0, column=0, sticky="w")
        ttk.Entry(f1, textvariable=self.move_v_ticks, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f1, text="H(A ticks)").grid(row=0, column=2, sticky="w")
        ttk.Entry(f1, textvariable=self.move_h_ticks, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f1, text="V(φ°)").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f1, textvariable=self.move_phase_v_deg, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Label(f1, text="H(φ°)").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(f1, textvariable=self.move_phase_h_deg, width=10).grid(row=1, column=3, padx=6, sticky="w", pady=(6, 0))

        f2 = ttk.LabelFrame(f, text="Giro", padding=8)
        f2.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        ttk.Label(f2, text="V(A ticks)").grid(row=0, column=0, sticky="w")
        ttk.Entry(f2, textvariable=self.turn_v_ticks, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f2, text="H(A ticks)").grid(row=0, column=2, sticky="w")
        ttk.Entry(f2, textvariable=self.turn_h_ticks, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f2, text="V(φ°)").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f2, textvariable=self.turn_phase_v_deg, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Label(f2, text="H(φ°)").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(f2, textvariable=self.turn_phase_h_deg, width=10).grid(row=1, column=3, padx=6, sticky="w", pady=(6, 0))

    def _ui_turn_phase_shift(self, parent):
        f = ttk.LabelFrame(parent, text="Giro: cambio de fase SOLO en un sentido", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Checkbutton(f, text="Habilitar", variable=self.turn_phase_shift_enable).grid(row=0, column=0, sticky="w")
        ttk.Label(f, text="Sentido").grid(row=0, column=1, sticky="w")
        ttk.Combobox(f, textvariable=self.turn_phase_shift_dir, values=["CW", "CCW"], width=6, state="readonly")\
            .grid(row=0, column=2, padx=6, sticky="w")
        ttk.Label(f, text="Δφ (°) aplicado a H").grid(row=0, column=3, sticky="w")
        ttk.Entry(f, textvariable=self.turn_phase_shift_deg, width=10).grid(row=0, column=4, padx=6, sticky="w")
        ttk.Label(f, text="(en el otro sentido Δφ=0)").grid(row=1, column=0, columnspan=5, sticky="w", pady=(6, 0))

    def _ui_model(self, parent):
        f = ttk.LabelFrame(parent, text="Modelo (pose) para la gráfica", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="k_v").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.kv_var, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f, text="k_w").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.kw_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f, text="Interpretación: v = k_v·max(0,Av-Ah) ; w = k_w·max(0,Ah-Av) con signo CW/CCW").grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(8, 0)
        )

    def _ui_exec(self, parent):
        f = ttk.LabelFrame(parent, text="Ejecución", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Button(f, text="Abrir gráficas", command=self.open_plot_window).pack(side="left", padx=6)
        ttk.Button(f, text="Actualizar Morfología", command=self.update_morphology_runtime).pack(side="left", padx=6)
        ttk.Button(f, text="HOME", command=self.home_all).pack(side="left", padx=6)
        ttk.Button(f, text="Iniciar", command=self.start).pack(side="left", padx=6)
        ttk.Button(f, text="Detener", command=self.stop).pack(side="left", padx=6)

    def _ui_status(self, parent):
        f = ttk.LabelFrame(parent, text="Estado", padding=10)
        f.pack(fill="x")
        ttk.Label(f, textvariable=self.status_var, wraplength=520, justify="left").pack(anchor="w")

    # ================== H/V dinámico ==================
    def _apply_nmods(self):
        self._rebuild_hv_checks()
        self.update_morphology_runtime()

    def _rebuild_hv_checks(self):
        for w in self.hv_frame.winfo_children():
            w.destroy()

        n = int(self.nmods_var.get())
        try:
            ids = parse_id_list(self.ids_var.get())
        except Exception:
            ids = []

        # Si no coincide con n, se ajusta a los primeros n (o se completa)
        if len(ids) != n:
            if len(ids) > n:
                ids = ids[:n]
            else:
                # completar con consecutivos que no estén
                used = set(ids)
                nxt = 1
                while len(ids) < n:
                    if nxt not in used:
                        ids.append(nxt)
                        used.add(nxt)
                    nxt += 1
            self.ids_var.set(",".join(str(i) for i in ids))

        # inicial: todos V (ninguno horizontal), como en la imagen
        self.horiz_var_by_id = {}
        for rid in ids:
            self.horiz_var_by_id[rid] = tk.BooleanVar(value=False)

        ttk.Label(self.hv_frame, text="Marcar IDs horizontales (H):").grid(row=0, column=0, columnspan=6, sticky="w")
        for i, rid in enumerate(ids):
            r = 1 + (i // 6)
            c = i % 6
            ttk.Checkbutton(self.hv_frame, text=f"ID {rid}", variable=self.horiz_var_by_id[rid])\
                .grid(row=r, column=c, padx=(0, 10), pady=(6, 0), sticky="w")

    def horizontal_ids_set(self):
        s = set()
        for rid, var in self.horiz_var_by_id.items():
            if var.get():
                s.add(rid)
        return s

    def suggest_hv_from_morph(self):
        # (se deja disponible, pero por defecto inicial = ninguno marcado)
        self.set_status("Sugerencia H/V no aplicada automáticamente (estado inicial: todos V).")

    # ================== Morfología runtime ==================
    def update_morphology_runtime(self):
        try:
            ids = parse_id_list(self.ids_var.get())
        except Exception:
            ids = []
        if not ids:
            self.set_status("IDs vacíos.")
            return
        self.ids_runtime = ids[:]
        self.morph_positions = morphology_positions(self.morph_var.get().strip(), len(ids))

        if self.plot_win and self.plot_win.winfo_exists():
            self.plot_win.lift()

    # ================= Plot =================
    def open_plot_window(self):
        if self.plot_win and self.plot_win.winfo_exists():
            self.plot_win.lift()
            return
        self.plot_win = PlotWindow(self)

    # ================= Serial =================
    def _refresh_ports(self):
        self.port_cb["values"] = list_serial_ports()

    def _eol_bytes(self) -> bytes:
        return b"\r\n" if self.eol_var.get() == "CRLF" else b"\n"

    def send_lines_batch(self, lines):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Maestro no conectado")
        eol = self._eol_bytes()
        payload = b"".join([ln.strip().encode("utf-8") + eol for ln in lines])
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

    # ================= HOME =================
    def home_all(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("HOME", "Conecte primero el Maestro.")
            return

        self.stop_event.set()

        ids = parse_id_list(self.ids_var.get())
        if not ids:
            messagebox.showwarning("HOME", "IDs vacíos.")
            return

        lines = []
        for rid in ids:
            if rid <= 3:
                lines.append(f"N {rid} POS {HOME_DX}")
            else:
                lines.append(f"N {rid} LX {HOME_LX}")
        try:
            self.send_lines_batch(lines)
        except Exception as e:
            self.set_status(f"Error HOME: {e}")
            return

        try:
            ax = float(self.ax_var.get())
            ay = float(self.ay_var.get())
        except Exception:
            ax, ay = 0.0, 0.0
        th0 = theta_from_alignment(self.align_var.get().strip())
        self.pose_runtime = (ax, ay, th0)
        self.traj_x = [ax]
        self.traj_y = [ay]
        self.t0 = time.perf_counter()

        self.set_status("HOME enviado y pose reiniciada.")

    # ================= Planificación y ejecución =================
    def build_plan(self):
        try:
            ax = float(self.ax_var.get()); ay = float(self.ay_var.get())
            bx = float(self.bx_var.get()); by = float(self.by_var.get())
            step_m = float(self.step_m_var.get())
            x_first = bool(self.x_first_var.get())
        except Exception as e:
            raise ValueError(f"Parámetros de navegación inválidos: {e}")

        ax = clamp_float(ax, X_MIN, X_MAX)
        ay = clamp_float(ay, Y_MIN, Y_MAX)
        bx = clamp_float(bx, X_MIN, X_MAX)
        by = clamp_float(by, Y_MIN, Y_MAX)
        self.ax_var.set(f"{ax:.3f}"); self.ay_var.set(f"{ay:.3f}")
        self.bx_var.set(f"{bx:.3f}"); self.by_var.set(f"{by:.3f}")

        wps = build_manhattan_waypoints(ax, ay, bx, by, step_m, x_first=x_first)
        return (ax, ay, bx, by, wps)

    def start(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Inicio", "Conecte primero el Maestro.")
            return

        try:
            ids = parse_id_list(self.ids_var.get())
        except Exception:
            ids = []
        if not ids:
            messagebox.showwarning("Inicio", "IDs vacíos.")
            return
        self.ids_runtime = ids[:]
        self.morph_positions = morphology_positions(self.morph_var.get().strip(), len(ids))

        try:
            ax, ay, bx, by, wps = self.build_plan()
        except Exception as e:
            messagebox.showerror("Plan", str(e))
            return

        self.waypoints_runtime = wps[:]
        self.wp_index = 0

        th0 = theta_from_alignment(self.align_var.get().strip())
        self.pose_runtime = (ax, ay, th0)
        self.traj_x = [ax]
        self.traj_y = [ay]
        self.t0 = time.perf_counter()

        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = MotionThread(self, self.stop_event)
        self.thread.start()
        self.set_status("Movimiento iniciado (giro tipo script).")

        if not (self.plot_win and self.plot_win.winfo_exists()):
            self.open_plot_window()
        else:
            self.plot_win.lift()

    def stop(self):
        self.stop_event.set()
        self.set_status("Detenido")

    # ================== Waypoints helpers ==================
    def current_wp(self):
        if 0 <= self.wp_index < len(self.waypoints_runtime):
            return self.waypoints_runtime[self.wp_index]
        return None

    def advance_wp(self):
        if self.wp_index + 1 < len(self.waypoints_runtime):
            self.wp_index += 1
            return True
        self.wp_index = len(self.waypoints_runtime)
        return False

    # ================== Comandos servo ==================
    def build_servo_lines(self, t, freq, Av, Ah, phi_v_deg, phi_h_deg, turning, cw):
        sign_v = -1.0  # si avanza al revés, cambie a +1.0
        if turning:
            sign_h = sign_v if cw else -sign_v
        else:
            sign_h = sign_v

        Av_lx = Av * LX_PER_DX
        Ah_lx = Ah * LX_PER_DX

        ids = self.ids_runtime
        n = len(ids)
        if n <= 0:
            return []

        horizontals = self.horizontal_ids_set()
        phase_origin_deg = 0.0

        out = []
        for i, rid in enumerate(ids):
            phi_idx_deg = phase_origin_deg + (360.0 * i / n)

            if rid in horizontals:
                phase = 2 * math.pi * freq * t + deg2rad(phi_h_deg + sign_h * phi_idx_deg)
                ss = math.sin(phase)
                if rid <= 3:
                    pos = clamp_int(HOME_DX + Ah * ss, DX_MIN, DX_MAX)
                    out.append(f"N {rid} POS {pos}")
                else:
                    ang = clamp_int(HOME_LX + Ah_lx * ss, LX_MIN, LX_MAX)
                    out.append(f"N {rid} LX {ang}")
            else:
                phase = 2 * math.pi * freq * t + deg2rad(phi_v_deg + sign_v * phi_idx_deg)
                ss = math.sin(phase)
                if rid <= 3:
                    pos = clamp_int(HOME_DX + Av * ss, DX_MIN, DX_MAX)
                    out.append(f"N {rid} POS {pos}")
                else:
                    ang = clamp_int(HOME_LX + Av_lx * ss, LX_MIN, LX_MAX)
                    out.append(f"N {rid} LX {ang}")

        return out

    # ================== Pose modelo ==================
    def update_pose_model(self, Av, Ah, turning, cw, kv, kw, dt):
        v = kv * max(0.0, Av - Ah)
        w = 0.0
        if turning:
            w = kw * max(0.0, Ah - Av) * (-1.0 if cw else +1.0)

        x, y, th = self.pose_runtime
        th = th + w * dt
        x = x + v * math.cos(th) * dt
        y = y + v * math.sin(th) * dt

        x = clamp_float(x, X_MIN, X_MAX)
        y = clamp_float(y, Y_MIN, Y_MAX)

        self.pose_runtime = (x, y, th)
        self.traj_x.append(x)
        self.traj_y.append(y)

    def on_close(self):
        try:
            self.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
