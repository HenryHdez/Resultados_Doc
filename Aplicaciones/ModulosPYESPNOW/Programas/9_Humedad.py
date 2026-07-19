import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math
import random

import serial
import serial.tools.list_ports

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.ticker import MultipleLocator
from matplotlib.colors import LinearSegmentedColormap


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

# Fase H por defecto (configurable en UI)
PHI_H_DEG_DEFAULT = 90.0


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
# Hilo de movimiento (multicadena) - ENVÍO SINCRONIZADO
# - Reinicio seno por cadena al finalizar o al HOME
# - Destino por cadena (opcional) pero por defecto común
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

                freq = float(self.app.freq_var.get())
                rate = float(self.app.rate_var.get())
                if freq <= 0 or rate <= 0:
                    time.sleep(0.1)
                    continue

                s = float(self.app.scale_m_per_model.get())
                if s <= 0:
                    s = 1.0

                tol = float(self.app.tol_var.get())
                dead = math.radians(float(self.app.deadband_deg_var.get()))

                now = time.perf_counter()
                dt = now - self.last_t
                self.last_t = now

                kv = float(self.app.kv_var.get())
                kw = float(self.app.kw_var.get())

                any_active = False
                all_done = True
                batch_lines = []

                for c in self.app.chains_runtime:
                    if c["done"]:
                        continue

                    all_done = False
                    any_active = True

                    wp = self._current_wp(c)
                    if wp is None:
                        # Termina cadena -> HOME y reinicio seno
                        c["done"] = True
                        c["t0_chain"] = time.perf_counter()
                        self.app.after(0, lambda ids=c["ids"]: self.app.home_ids(ids))
                        continue

                    tx_r, ty_r = wp

                    x_m, y_m, th = c["pose_model"]
                    x_r = x_m * s
                    y_r = y_m * s

                    dist = math.hypot(tx_r - x_r, ty_r - y_r)

                    if dist <= tol:
                        if self._advance_wp(c):
                            pass
                        else:
                            # Llegó al último waypoint de su destino -> HOME y reinicio seno
                            c["done"] = True
                            c["t0_chain"] = time.perf_counter()
                            self.app.after(0, lambda ids=c["ids"]: self.app.home_ids(ids))
                        continue

                    desired = math.atan2(ty_r - y_r, tx_r - x_r)
                    err = wrap_pi(desired - th)

                    turning = abs(err) > dead
                    cw = err < 0

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

                    # tiempo local de cadena para reinicio de seno por cadena
                    t_chain = time.perf_counter() - c.get("t0_chain", self.t0)

                    batch_lines.extend(self._build_lines_chain(
                        chain=c, t=t_chain, freq=freq,
                        Av=Av, Ah=Ah,
                        phi_v_deg=phi_v,
                        phi_h_deg=phi_h,
                        turning=turning, cw=cw
                    ))

                    self._update_pose_model_chain(c, Av, Ah, turning, cw, kv, kw, dt)

                    x_m2, y_m2, _ = c["pose_model"]
                    c["traj_x"].append(x_m2 * s)
                    c["traj_y"].append(y_m2 * s)

                if all_done and not any_active:
                    self.app.set_status("Sin cadenas activas.")
                    self.stop_event.set()
                    break

                if all_done:
                    self.app.set_status("Todas las cadenas terminaron. HOME completado.")
                    self.stop_event.set()
                    break

                if batch_lines:
                    self.app.send_lines_batch(batch_lines)

                self.app.set_status("Movimiento multicadena sincronizado activo.")
                time.sleep(1.0 / rate)

            except Exception as e:
                self.app.set_status(f"Detenido por error: {e}")
                self.stop_event.set()

    def _current_wp(self, chain):
        i = chain["wp_index"]
        if 0 <= i < len(chain["waypoints"]):
            return chain["waypoints"][i]
        return None

    def _advance_wp(self, chain):
        if chain["wp_index"] + 1 < len(chain["waypoints"]):
            chain["wp_index"] += 1
            return True
        return False

    def _build_lines_chain(self, chain, t, freq, Av, Ah, phi_v_deg, phi_h_deg, turning, cw):
        # Si su robot avanza al revés, cambie a +1.0
        sign_v = -1.0

        if turning:
            sign_h = sign_v if cw else -sign_v
        else:
            sign_h = sign_v

        Av_lx = Av * LX_PER_DX
        Ah_lx = Ah * LX_PER_DX

        ids = chain["ids"]
        n = len(ids)
        if n <= 0:
            return []

        horizontals = set(self.app.horizontal_ids_global().intersection(set(ids)))
        phase_origin_deg = chain["phase_origin_deg"]

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

    def _update_pose_model_chain(self, chain, Av, Ah, turning, cw, kv, kw, dt):
        v = kv * max(0.0, Av - Ah)
        w = 0.0
        if turning:
            w = kw * max(0.0, Ah - Av) * (-1.0 if cw else +1.0)

        x_m, y_m, th = chain["pose_model"]
        th = th + w * dt
        x_m = x_m + v * math.cos(th) * dt
        y_m = y_m + v * math.sin(th) * dt
        chain["pose_model"] = (x_m, y_m, th)


# ======================================================
# Ventana de gráfica (Toplevel) + COLORBAR (escala)
# Opción 1: mapa de humedad centrado en B común (no por cadena)
# ======================================================
class PlotWindow(tk.Toplevel):
    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("Mapa de humedad")
        self.geometry("1050x780")

        self.fig = Figure(figsize=(9.6, 7.0), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)

        # Colormap: bajo=azul, alto=rojo
        self.h_cmap = LinearSegmentedColormap.from_list(
            "humidity5",
            ["#2f3fb8", "#00a6ff", "#a8e61d", "#ff8c1a", "#ff1e1e"],
            N=256
        )

        self.chain_colors = ["#000000", "#9400D3", "#8B4513"]
        self.chain_lines = []
        self.chain_plan_lines = []
        self.chain_tips = []
        self.chain_labels = []

        self.h_scatter = None
        self.h_max_pt, = self.ax.plot([], [], marker="*", markersize=14)  # máximo humedad
        self.ptB, = self.ax.plot([], [], marker="x")  # B común

        self.cbar = None

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self._alive = True
        self._init_artists()
        self.after(150, self._refresh)

    def _on_close(self):
        self._alive = False
        self.destroy()

    def _apply_fixed_axes(self):
        try:
            tick = float(self.app.tick_var.get())
            if tick <= 0:
                tick = 0.10
        except Exception:
            tick = 0.10

        self.ax.set_xlim(X_MIN, X_MAX)
        self.ax.set_ylim(Y_MIN, Y_MAX)
        self.ax.xaxis.set_major_locator(MultipleLocator(tick))
        self.ax.yaxis.set_major_locator(MultipleLocator(tick))

    def _init_artists(self):
        for ln in self.chain_lines:
            ln.remove()
        for ln in self.chain_plan_lines:
            ln.remove()
        for tip in self.chain_tips:
            tip.remove()
        for txt in self.chain_labels:
            txt.remove()

        self.chain_lines = []
        self.chain_plan_lines = []
        self.chain_tips = []
        self.chain_labels = []

        for i, _ in enumerate(self.app.chains_runtime):
            color = self.chain_colors[i % len(self.chain_colors)]
            ln, = self.ax.plot([], [], color=color)
            pln, = self.ax.plot([], [], linestyle="--", color=color, alpha=0.6)

            tip, = self.ax.plot([], [], linestyle="None",
                                marker=(3, 0, 0), markersize=14,
                                color=color)
            txt = self.ax.text(0, 0, "", color=color, fontsize=10,
                               va="center", ha="left")

            self.chain_lines.append(ln)
            self.chain_plan_lines.append(pln)
            self.chain_tips.append(tip)
            self.chain_labels.append(txt)

    def _ensure_colorbar(self):
        if self.h_scatter is None:
            return
        if self.cbar is not None:
            return
        self.cbar = self.fig.colorbar(self.h_scatter, ax=self.ax, fraction=0.046, pad=0.04)
        self.cbar.set_label("Humedad (normalizada)")
        self.cbar.set_ticks([0.0, 0.25, 0.50, 0.75, 1.0])
        self.cbar.set_ticklabels(["0 (baja)", "0.25", "0.50", "0.75", "1 (alta)"])

    def _draw_humidity_points(self):
        pts = self.app.humidity_points
        if not pts:
            if self.h_scatter is not None:
                self.h_scatter.remove()
                self.h_scatter = None
            if self.cbar is not None:
                self.cbar.remove()
                self.cbar = None
            self.h_max_pt.set_data([], [])
            return

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        hs = [p[2] for p in pts]

        if self.h_scatter is None:
            self.h_scatter = self.ax.scatter(
                xs, ys,
                c=hs,
                cmap=self.h_cmap,
                vmin=0.0, vmax=1.0,
                s=24,
                marker="o",
                alpha=0.95,
                linewidths=0.0
            )
        else:
            self.h_scatter.set_offsets(list(zip(xs, ys)))
            self.h_scatter.set_array(hs)
            self.h_scatter.set_clim(0.0, 1.0)

        self._ensure_colorbar()

        mx = self.app.humidity_max_point
        if mx is not None:
            self.h_max_pt.set_data([mx[0]], [mx[1]])
        else:
            self.h_max_pt.set_data([], [])

    def _refresh(self):
        if not self._alive:
            return

        try:
            self._draw_humidity_points()

            # B común (opción 1: mapa centrado en B común)
            bx = float(self.app.bx_var.get())
            by = float(self.app.by_var.get())
            self.ptB.set_data([bx], [by])

            if len(self.chain_lines) != len(self.app.chains_runtime):
                self._init_artists()

            s = float(self.app.scale_m_per_model.get())
            if s <= 0:
                s = 1.0

            for i, c in enumerate(self.app.chains_runtime):
                self.chain_lines[i].set_data(c["traj_x"], c["traj_y"])

                axr, ayr = c["A"]
                if c["waypoints"]:
                    xs_plan = [axr] + [p[0] for p in c["waypoints"]]
                    ys_plan = [ayr] + [p[1] for p in c["waypoints"]]
                    self.chain_plan_lines[i].set_data(xs_plan, ys_plan)
                else:
                    self.chain_plan_lines[i].set_data([], [])

                x_m, y_m, th = c["pose_model"]
                x_r = x_m * s
                y_r = y_m * s

                ang_deg = math.degrees(th) - 90.0
                self.chain_tips[i].set_data([x_r], [y_r])
                self.chain_tips[i].set_marker((3, 0, ang_deg))

                nmods = len(c["ids"])
                self.chain_labels[i].set_position((x_r + 0.03, y_r + 0.02))
                self.chain_labels[i].set_text(f"{nmods} módulos")

            self._apply_fixed_axes()
            self.canvas.draw_idle()

        except Exception:
            pass
        finally:
            self.after(150, self._refresh)


# ======================================================
# Aplicación principal
# ======================================================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Principal")
        self.geometry("1480x700")

        self.ser = None
        self.wlock = threading.Lock()

        self.stop_event = threading.Event()
        self.thread = None
        self.plot_win = None

        # humidity_points: [(x,y,h_norm)]
        self.humidity_points = []
        self.humidity_max_point = None  # (x,y,h_norm)

        # -------- Serial --------
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.eol_var = tk.StringVar(value="LF")
        self.status_var = tk.StringVar(value="Desconectado")

        # -------- Global --------
        self.freq_var = tk.StringVar(value="0.40")
        self.rate_var = tk.StringVar(value="10")

        # Avance
        self.move_v_ticks = tk.StringVar(value="210")
        self.move_h_ticks = tk.StringVar(value="10")
        self.move_phase_v_deg = tk.StringVar(value="45")
        self.move_phase_h_deg = tk.StringVar(value=str(PHI_H_DEG_DEFAULT))  # H configurable

        # Giro
        self.turn_v_ticks = tk.StringVar(value="80")
        self.turn_h_ticks = tk.StringVar(value="210")
        self.turn_phase_v_deg = tk.StringVar(value="120")
        self.turn_phase_h_deg = tk.StringVar(value=str(PHI_H_DEG_DEFAULT))  # H configurable

        # Modelo/escala
        self.kv_var = tk.StringVar(value="0.01")
        self.kw_var = tk.StringVar(value="0.0005")
        self.scale_m_per_model = tk.StringVar(value="0.01")  # 0.01 => 1 unidad-modelo = 1cm

        # Navegación (rejilla Manhattan)
        self.step_m_var = tk.StringVar(value="0.10")
        self.x_first_var = tk.BooleanVar(value=True)
        self.tol_var = tk.StringVar(value="0.015")
        self.deadband_deg_var = tk.StringVar(value="5")

        # Ejes/ticks
        self.tick_var = tk.StringVar(value="0.10")

        # Punto final B común (destino por defecto)
        self.bx_var = tk.StringVar(value="1")
        self.by_var = tk.StringVar(value="0.6")

        # Horizontales globales (IDs 1..6)
        self.horiz_vars = {i: tk.BooleanVar(value=False) for i in range(1, 7)}

        # --- Multicadena (hasta 3 cadenas) ---
        self.num_chains_var = tk.IntVar(value=1)

        self.chain_vars = []
        for k in range(3):
            self.chain_vars.append({
                "ids": tk.StringVar(value="" if k > 0 else "1,2,3,4,5,6"),
                "ax": tk.StringVar(value="0.00"),
                "ay": tk.StringVar(value="0.00"),
                "align": tk.StringVar(value="+X"),
                # destino por cadena (opcional): vacío => usa B común
                "bx": tk.StringVar(value=""),
                "by": tk.StringVar(value=""),
            })

        self.chains_runtime = []

        # --- Parámetros mapa de humedad ---
        self.h_nx_var = tk.StringVar(value="45")
        self.h_ny_var = tk.StringVar(value="30")
        self.h_seed_var = tk.StringVar(value="1")
        self.h_noise_var = tk.StringVar(value="0.08")
        self.h_red_w_rel_var = tk.StringVar(value="0.3")
        self.h_red_h_rel_var = tk.StringVar(value="0.2")

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

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

        # Columna 1: Serial + módulos + mapa humedad
        self._ui_serial(col1)
        self._ui_horizontals(col1)
        self._ui_humidity(col1)

        # Columna 2: parámetros globales + marcha + modelo
        self._ui_global(col2)
        self._ui_gait(col2)
        self._ui_model(col2)

        # Columna 3: cadenas + destino + ejecución + estado
        self._ui_multichain(col3)
        self._ui_destination(col3)
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

    def _ui_horizontals(self, parent):
        f = ttk.LabelFrame(parent, text="Módulos horizontales (global)", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="Marcar qué IDs (1..6) son horizontales:").grid(row=0, column=0, columnspan=3, sticky="w")
        for rid in range(1, 7):
            r = 1 if rid <= 3 else 2
            c = (rid - 1) % 3
            ttk.Checkbutton(f, text=f"R{rid}", variable=self.horiz_vars[rid])\
                .grid(row=r, column=c, padx=(0, 12), pady=(6, 0), sticky="w")

    def _ui_humidity(self, parent):
        f = ttk.LabelFrame(parent, text="Mapa de humedad", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="Nx").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.h_nx_var, width=6).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f, text="Ny").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.h_ny_var, width=6).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f, text="Seed").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.h_seed_var, width=6).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Label(f, text="Ruido (0..1)").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.h_noise_var, width=6).grid(row=1, column=3, padx=6, sticky="w", pady=(6, 0))

        ttk.Label(f, text="Ancho rojo (rel)").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.h_red_w_rel_var, width=6).grid(row=2, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Label(f, text="Alto rojo (rel)").grid(row=2, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=self.h_red_h_rel_var, width=6).grid(row=2, column=3, padx=6, sticky="w", pady=(6, 0))

        btns = ttk.Frame(f)
        btns.grid(row=3, column=0, columnspan=4, sticky="w", pady=(10, 0))
        ttk.Button(btns, text="Generar mapa", command=self.generate_humidity_map).pack(side="left", padx=(0, 8))

    def _ui_global(self, parent):
        f = ttk.LabelFrame(parent, text="Parámetros globales", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="Frecuencia (Hz)").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.freq_var, width=10).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(f, text="Tasa envío (Hz)").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.rate_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f, text="Paso rejilla (m)").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.step_m_var, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(8, 0))
        ttk.Checkbutton(f, text="Primero X luego Y", variable=self.x_first_var)\
            .grid(row=1, column=2, columnspan=2, sticky="w", pady=(8, 0))

        ttk.Label(f, text="Tol (m)").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.tol_var, width=10).grid(row=2, column=1, padx=6, sticky="w", pady=(8, 0))
        ttk.Label(f, text="Deadband (°)").grid(row=2, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.deadband_deg_var, width=10).grid(row=2, column=3, padx=6, sticky="w", pady=(8, 0))

        ttk.Label(f, text="Tick ejes (m)").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.tick_var, width=10).grid(row=3, column=1, padx=6, sticky="w", pady=(8, 0))
        ttk.Label(f, text="Ejes fijos: X=0..1.90, Y=0..1.20").grid(row=3, column=2, columnspan=2, sticky="w", pady=(8, 0))

    def _ui_gait(self, parent):
        f1 = ttk.LabelFrame(parent, text="Avance (recto)", padding=10)
        f1.pack(fill="x", pady=(0, 10))

        ttk.Label(f1, text="V(A ticks)").grid(row=0, column=0, sticky="w")
        ttk.Entry(f1, textvariable=self.move_v_ticks, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f1, text="H(A ticks)").grid(row=0, column=2, sticky="w")
        ttk.Entry(f1, textvariable=self.move_h_ticks, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f1, text="V(φ°)").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f1, textvariable=self.move_phase_v_deg, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Label(f1, text="H(φ°)").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(f1, textvariable=self.move_phase_h_deg, width=10).grid(row=1, column=3, padx=6, sticky="w", pady=(6, 0))

        f2 = ttk.LabelFrame(parent, text="Giro", padding=10)
        f2.pack(fill="x", pady=(0, 10))

        ttk.Label(f2, text="V(A ticks)").grid(row=0, column=0, sticky="w")
        ttk.Entry(f2, textvariable=self.turn_v_ticks, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f2, text="H(A ticks)").grid(row=0, column=2, sticky="w")
        ttk.Entry(f2, textvariable=self.turn_h_ticks, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f2, text="V(φ°)").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f2, textvariable=self.turn_phase_v_deg, width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))
        ttk.Label(f2, text="H(φ°)").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(f2, textvariable=self.turn_phase_h_deg, width=10).grid(row=1, column=3, padx=6, sticky="w", pady=(6, 0))

    def _ui_model(self, parent):
        f = ttk.LabelFrame(parent, text="Escala", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="k_v").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.kv_var, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f, text="k_w").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.kw_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(f, text="Escala (m / unidad-modelo)").grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Entry(f, textvariable=self.scale_m_per_model, width=10).grid(row=1, column=2, padx=6, sticky="w", pady=(8, 0))

    def _ui_multichain(self, parent):
        f = ttk.LabelFrame(parent, text="Cadenas (1..3)", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="Cantidad de cadenas").grid(row=0, column=0, sticky="w")
        ttk.Combobox(f, textvariable=self.num_chains_var, values=[1, 2, 3], width=5, state="readonly")\
            .grid(row=0, column=1, padx=6, sticky="w")
        ttk.Button(f, text="Aplicar", command=self._apply_num_chains).grid(row=0, column=2, padx=6, sticky="w")

        self.chain_frames = []
        for k in range(3):
            cf = ttk.LabelFrame(f, text=f"Cadena {k+1}", padding=10)
            cf.grid(row=1+k, column=0, columnspan=3, sticky="ew", pady=(10, 0))
            self.chain_frames.append(cf)

            ttk.Label(cf, text="IDs módulos (coma)").grid(row=0, column=0, sticky="w")
            ttk.Entry(cf, textvariable=self.chain_vars[k]["ids"], width=24).grid(row=0, column=1, padx=6, sticky="w")

            ttk.Label(cf, text="A.x").grid(row=1, column=0, sticky="w", pady=(6, 0))
            ttk.Entry(cf, textvariable=self.chain_vars[k]["ax"], width=10).grid(row=1, column=1, padx=6, sticky="w", pady=(6, 0))

            ttk.Label(cf, text="A.y").grid(row=1, column=2, sticky="w", pady=(6, 0))
            ttk.Entry(cf, textvariable=self.chain_vars[k]["ay"], width=10).grid(row=1, column=3, padx=6, sticky="w", pady=(6, 0))

            ttk.Label(cf, text="Alineación").grid(row=0, column=2, sticky="w")
            ttk.Combobox(cf, textvariable=self.chain_vars[k]["align"], values=["+X", "+Y", "-X", "-Y"], width=6, state="readonly")\
                .grid(row=0, column=3, padx=6, sticky="w")

            # Destino por cadena (opcional); vacío => usa B común
            ttk.Label(cf, text="B.x (opcional)").grid(row=2, column=0, sticky="w", pady=(6, 0))
            ttk.Entry(cf, textvariable=self.chain_vars[k]["bx"], width=10).grid(row=2, column=1, padx=6, sticky="w", pady=(6, 0))

            ttk.Label(cf, text="B.y (opcional)").grid(row=2, column=2, sticky="w", pady=(6, 0))
            ttk.Entry(cf, textvariable=self.chain_vars[k]["by"], width=10).grid(row=2, column=3, padx=6, sticky="w", pady=(6, 0))

        self._apply_num_chains()

    def _apply_num_chains(self):
        n = int(self.num_chains_var.get())
        for k, cf in enumerate(self.chain_frames):
            if k < n:
                cf.grid()
            else:
                cf.grid_remove()

    def _ui_destination(self, parent):
        f = ttk.LabelFrame(parent, text="Destino común (B)", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Label(f, text="B.x").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.bx_var, width=10).grid(row=0, column=1, padx=6, sticky="w")
        ttk.Label(f, text="B.y").grid(row=0, column=2, sticky="w")
        ttk.Entry(f, textvariable=self.by_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

    def _ui_exec(self, parent):
        f = ttk.LabelFrame(parent, text="Ejecución", padding=10)
        f.pack(fill="x", pady=(0, 10))

        ttk.Button(f, text="Abrir gráfica", command=self.open_plot_window).pack(side="left", padx=6)
        ttk.Button(f, text="HOME (todas)", command=self.home_all).pack(side="left", padx=6)
        ttk.Button(f, text="Iniciar (cadenas → sus B)", command=self.start_all).pack(side="left", padx=6)
        ttk.Button(f, text="Detener", command=self.stop).pack(side="left", padx=6)

    def _ui_status(self, parent):
        f = ttk.LabelFrame(parent, text="Estado", padding=10)
        f.pack(fill="x")
        ttk.Label(f, textvariable=self.status_var, wraplength=520, justify="left").pack(anchor="w")

    # ================= Humedad: transiciones SOLO en 2 líneas de punticos =================
    # Opción 1: el bloque rojo se centra SIEMPRE en el B común
    def generate_humidity_map(self):
        """
        Punticos en grilla con:
          - cuadrantes (valores base)
          - bloque rojo centrado en B común (rojo sólido)
          - transiciones SOLO en dos líneas (~2 columnas/filas) alrededor de xmid y ymid
        """
        try:
            nx = int(self.h_nx_var.get())
            ny = int(self.h_ny_var.get())
            seed = int(self.h_seed_var.get())
            noise = float(self.h_noise_var.get())

            w_rel = float(self.h_red_w_rel_var.get())
            h_rel = float(self.h_red_h_rel_var.get())

            bx = float(self.bx_var.get())
            by = float(self.by_var.get())
        except Exception as e:
            messagebox.showerror("Humedad", f"Parámetros inválidos: {e}")
            return

        nx = max(8, min(nx, 500))
        ny = max(8, min(ny, 500))
        noise = max(0.0, min(noise, 1.0))
        w_rel = max(0.05, min(w_rel, 1.50))
        h_rel = max(0.05, min(h_rel, 1.50))

        bx = max(X_MIN, min(bx, X_MAX))
        by = max(Y_MIN, min(by, Y_MAX))
        self.bx_var.set(f"{bx:.3f}")
        self.by_var.set(f"{by:.3f}")

        random.seed(seed)

        xs = [X_MIN + (X_MAX - X_MIN) * i / (nx - 1) for i in range(nx)]
        ys = [Y_MIN + (Y_MAX - Y_MIN) * j / (ny - 1) for j in range(ny)]

        xmid = 0.5 * (X_MIN + X_MAX)
        ymid = 0.5 * (Y_MIN + Y_MAX)

        W = X_MAX - X_MIN
        H = Y_MAX - Y_MIN

        # bloque rojo móvil centrado en B común
        rect_w = w_rel * W
        rect_h = h_rel * H
        rx0, rx1 = bx - rect_w / 2.0, bx + rect_w / 2.0
        ry0, ry1 = by - rect_h / 2.0, by + rect_h / 2.0

        def in_red_rect(x, y):
            return (rx0 <= x <= rx1) and (ry0 <= y <= ry1)

        # niveles 0..1
        V_BLUE = 0.10
        V_CYAN = 0.30
        V_GREEN = 0.55
        V_ORANGE = 0.75
        V_RED = 0.98

        dx = (X_MAX - X_MIN) / (nx - 1)
        dy = (Y_MAX - Y_MIN) / (ny - 1)

        band_x = 2.0 * dx
        band_y = 2.0 * dy

        def lerp(a, b, t):
            return a * (1.0 - t) + b * t

        def smooth01(t):
            t = max(0.0, min(1.0, t))
            return 0.5 - 0.5 * math.cos(math.pi * t)

        iB = min(range(nx), key=lambda i: (xs[i] - bx) ** 2)
        jB = min(range(ny), key=lambda j: (ys[j] - by) ** 2)

        pts = []
        max_xyh = None
        max_h = -1e9

        for j, y in enumerate(ys):
            for i, x in enumerate(xs):

                # base por cuadrante
                if x < xmid and y >= ymid:
                    hn = V_BLUE
                elif x >= xmid and y >= ymid:
                    hn = V_ORANGE
                elif x < xmid and y < ymid:
                    hn = V_CYAN
                else:
                    hn = V_GREEN

                # transición SOLO cerca de xmid
                if abs(x - xmid) <= band_x:
                    if y >= ymid:
                        left, right = V_BLUE, V_ORANGE
                    else:
                        left, right = V_CYAN, V_GREEN
                    t = (x - (xmid - band_x)) / (2.0 * band_x)
                    t = smooth01(t)
                    hn = lerp(left, right, t)

                # transición SOLO cerca de ymid
                if abs(y - ymid) <= band_y:
                    if x < xmid:
                        bottom, top = V_CYAN, V_BLUE
                    else:
                        bottom, top = V_GREEN, V_ORANGE
                    t = (y - (ymid - band_y)) / (2.0 * band_y)
                    t = smooth01(t)
                    hn = lerp(bottom, top, t)

                # bloque rojo (sólido)
                if in_red_rect(x, y):
                    hn = V_RED

                # ruido leve
                hn += noise * 0.02 * (2.0 * random.random() - 1.0)
                hn = max(0.0, min(hn, 1.0))

                # máximo absoluto en B común
                if i == iB and j == jB:
                    hn = 1.0

                pts.append((x, y, hn))

                if hn > max_h:
                    max_h = hn
                    max_xyh = (x, y, hn)

        self.humidity_points = pts
        self.humidity_max_point = max_xyh

        mx, my, mh = self.humidity_max_point
        self.set_status(
            f"Mapa generado (transición solo en ~2 líneas). Máximo en ({mx:.2f},{my:.2f}) h={mh:.3f}."
        )

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

    # ================= Horizontales globales =================
    def horizontal_ids_global(self):
        s = set()
        for rid in range(1, 7):
            if self.horiz_vars[rid].get():
                s.add(rid)
        return s

    # ================= HOME helpers =================
    def home_ids(self, ids):
        if not self.ser or not self.ser.is_open:
            return
        lines = []
        for rid in ids:
            if rid <= 3:
                lines.append(f"N {rid} POS {HOME_DX}")
            else:
                lines.append(f"N {rid} LX {HOME_LX}")
        self.send_lines_batch(lines)

    def home_all(self):
        ids = self._all_configured_ids()
        if not ids:
            messagebox.showwarning("HOME", "No hay IDs configurados en cadenas.")
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("HOME", "Conecte primero el Maestro.")
            return

        # Detener hilo (si estaba) y reiniciar seno por cadena (si existen runtimes)
        self.stop_event.set()
        for c in self.chains_runtime:
            c["t0_chain"] = time.perf_counter()

        self.home_ids(sorted(ids))
        self.set_status(f"HOME enviado a {sorted(ids)}")

    def _all_configured_ids(self):
        n = int(self.num_chains_var.get())
        ids_all = set()
        for k in range(n):
            try:
                ids = parse_id_list(self.chain_vars[k]["ids"].get())
            except Exception:
                ids = []
            for rid in ids:
                ids_all.add(rid)
        return ids_all

    # ================= Ejecución multicadena =================
    def _build_runtime_from_ui(self):
        n = int(self.num_chains_var.get())

        # B común (por defecto)
        try:
            bx_common = float(self.bx_var.get())
            by_common = float(self.by_var.get())
        except ValueError:
            raise ValueError("B.x/B.y inválidos")

        bx_common = max(X_MIN, min(bx_common, X_MAX))
        by_common = max(Y_MIN, min(by_common, Y_MAX))
        self.bx_var.set(f"{bx_common:.3f}")
        self.by_var.set(f"{by_common:.3f}")

        seen = set()
        chains = []
        for k in range(n):
            ids = parse_id_list(self.chain_vars[k]["ids"].get())
            if not ids:
                raise ValueError(f"Cadena {k+1}: IDs vacíos")

            for rid in ids:
                if rid in seen:
                    raise ValueError(f"El ID {rid} está repetido entre cadenas")
                seen.add(rid)

            try:
                ax = float(self.chain_vars[k]["ax"].get())
                ay = float(self.chain_vars[k]["ay"].get())
            except ValueError:
                raise ValueError(f"Cadena {k+1}: A.x/A.y inválidos")

            ax = max(X_MIN, min(ax, X_MAX))
            ay = max(Y_MIN, min(ay, Y_MAX))
            self.chain_vars[k]["ax"].set(f"{ax:.3f}")
            self.chain_vars[k]["ay"].set(f"{ay:.3f}")

            align = self.chain_vars[k]["align"].get().strip()
            th0 = theta_from_alignment(align)

            # Destino por cadena (si bx/by vacíos => usa B común)
            bx_s = self.chain_vars[k]["bx"].get().strip()
            by_s = self.chain_vars[k]["by"].get().strip()
            if bx_s == "" or by_s == "":
                bxk, byk = bx_common, by_common
            else:
                try:
                    bxk = float(bx_s)
                    byk = float(by_s)
                except ValueError:
                    raise ValueError(f"Cadena {k+1}: B.x/B.y inválidos")
                bxk = max(X_MIN, min(bxk, X_MAX))
                byk = max(Y_MIN, min(byk, Y_MAX))
                self.chain_vars[k]["bx"].set(f"{bxk:.3f}")
                self.chain_vars[k]["by"].set(f"{byk:.3f}")

            s = float(self.scale_m_per_model.get())
            if s <= 0:
                s = 1.0
            pose_model = (ax / s, ay / s, th0)

            step_m = float(self.step_m_var.get())
            x_first = bool(self.x_first_var.get())
            wps = build_manhattan_waypoints(ax, ay, bxk, byk, step_m, x_first=x_first)

            chains.append({
                "ids": ids,
                "A": (ax, ay),
                "B": (bxk, byk),
                "pose_model": pose_model,
                "waypoints": wps,
                "wp_index": 0,
                "traj_x": [ax],
                "traj_y": [ay],
                "done": False,
                "phase_origin_deg": 120.0 * k,
                "t0_chain": time.perf_counter(),  # reinicio seno por cadena
            })

        return chains

    def start_all(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Inicio", "Conecte primero el Maestro.")
            return

        # Opción 1: regenerar mapa usando SIEMPRE B común (centra el bloque rojo en B común)
        self.generate_humidity_map()

        try:
            self.chains_runtime = self._build_runtime_from_ui()
        except Exception as e:
            messagebox.showerror("Configuración", str(e))
            return

        if self.thread and self.thread.is_alive():
            return

        self.stop_event.clear()
        self.thread = MotionThread(self, self.stop_event)
        self.thread.start()
        self.set_status("Movimiento multicadena sincronizado iniciado.")

        if not (self.plot_win and self.plot_win.winfo_exists()):
            self.open_plot_window()
        else:
            self.plot_win.lift()

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
