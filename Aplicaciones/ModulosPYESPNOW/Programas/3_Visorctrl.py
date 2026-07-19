import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math
import queue

import serial
import serial.tools.list_ports

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk


# =========================
# Configuración general
# =========================
CENTER_DX = 512         # Robots 1..3
CENTER_LX = 95          # Robots 4..6
DX_MIN, DX_MAX = 0, 1023
LX_MIN, LX_MAX = 0, 240

CONNECTED_TTL_S = 2.5   # Si no se ve un nodo en este tiempo, se considera desconectado
PLOT_WINDOW_S = 5.0     # Ventana de gráfica en segundos


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def clamp_int(x: float, lo: int, hi: int) -> int:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return int(round(x))


# =========================
# Hilo receptor: detecta módulos conectados
# =========================
class SerialReceiver(threading.Thread):
    """
    Detecta módulos conectados leyendo telemetría del Maestro.
    Espera líneas tipo:
      - CSV sensores: node_id,in1,in2,in3
      - logs: # ...
    Con solo ver node_id, se marca como "conectado".
    """
    def __init__(self, ser: serial.Serial, out_queue: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.ser = ser
        self.out_queue = out_queue
        self.stop_event = stop_event

    def run(self):
        buf = b""
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.strip().decode(errors="ignore")
                    if not s:
                        continue

                    if s.startswith("#"):
                        self.out_queue.put(("__log__", s))
                        continue

                    parts = s.split(",")
                    if len(parts) == 4:
                        try:
                            node_id = int(parts[0])
                            self.out_queue.put(("__seen__", node_id, time.time()))
                        except Exception:
                            pass

            except Exception as e:
                self.out_queue.put(("__error__", str(e)))
                break


# =========================
# Hilo de envío: genera seno y envía; reporta frame de lo enviado
# =========================
class SineSender(threading.Thread):
    """
    Envía comandos senoidales al Maestro únicamente a módulos detectados.
    Y reporta al GUI exactamente lo enviado en cada frame.

      - IDs 1..3: N <id> POS <pos>
      - IDs 4..6: N <id> LX  <ang>
    """
    def __init__(self, ser: serial.Serial, write_lock: threading.Lock, app, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.ser = ser
        self.wlock = write_lock
        self.app = app
        self.stop_event = stop_event
        self.t0 = time.perf_counter()

    def _send_line(self, line: str):
        with self.wlock:
            self.ser.write((line.strip() + "\n").encode("utf-8"))

    def run(self):
        while not self.stop_event.is_set():
            try:
                freq = float(self.app.freq_var.get())
                rate = float(self.app.rate_var.get())
                amp_dx = float(self.app.amp_dx_var.get())
                amp_lx = float(self.app.amp_lx_var.get())

                if freq <= 0 or rate <= 0:
                    time.sleep(0.1)
                    continue
                if amp_dx < 0 or amp_lx < 0:
                    time.sleep(0.1)
                    continue

                invert = (self.app.dir_var.get().strip().lower() == "retroceso")

                ids = self.app.get_connected_ids_snapshot()
                n = len(ids)
                if n == 0:
                    # sin módulos detectados
                    time.sleep(0.2)
                    continue

                t = time.perf_counter() - self.t0

                # Construir en memoria TODO lo que se va a enviar en este frame
                # (rid -> valor enviado)
                frame_values = {}

                for k, rid in enumerate(ids):
                    # fase depende de la cantidad de módulos detectados
                    phase = (360.0 * k / n) if n > 0 else 0.0
                    if invert:
                        phase = -phase

                    if rid in (1, 2, 3):
                        raw = CENTER_DX + amp_dx * math.sin(2 * math.pi * freq * t + math.radians(phase))
                        pos = clamp_int(raw, DX_MIN, DX_MAX)
                        self._send_line(f"N {rid} POS {pos}")
                        frame_values[rid] = pos

                    elif rid in (4, 5, 6):
                        raw = CENTER_LX + amp_lx * math.sin(2 * math.pi * freq * t + math.radians(phase))
                        ang = clamp_int(raw, LX_MIN, LX_MAX)
                        self._send_line(f"N {rid} LX {ang}")
                        frame_values[rid] = ang

                # Reportar al GUI exactamente el frame enviado
                self.app.push_frame(t, frame_values, ids)
                self.app.set_status_runtime(n, ids)

                time.sleep(1.0 / rate)

            except Exception as e:
                self.app.set_status(f"Error en envío: {e}")
                self.stop_event.set()
                break


# =========================
# App GUI
# =========================
class AutoDetectSineApp(tk.Tk):
    """
    App:
      - Conecta al Maestro
      - Detecta módulos conectados automáticamente
      - Envía setpoints senoidales
      - Grafica en tiempo real EXACTAMENTE lo enviado (pos/ang) por módulo disponible
    """
    def __init__(self):
        super().__init__()
        self.title("Seno enviado (tiempo real) | Auto-detección de módulos | Avance/Retroceso")
        self.geometry("1100x650")

        # Serial y threads
        self.ser = None
        self.write_lock = threading.Lock()
        self.rx_stop = threading.Event()
        self.tx_stop = threading.Event()
        self.rx_thread = None
        self.tx_thread = None
        self.q = queue.Queue()

        # Estado de módulos detectados: id -> last_seen_epoch
        self.connected = {}
        self.lock_conn = threading.Lock()

        # Parámetros
        self.port_var = tk.StringVar(value="")
        self.baud_var = tk.StringVar(value="115200")
        self.freq_var = tk.StringVar(value="0.30")
        self.rate_var = tk.StringVar(value="30")
        self.amp_dx_var = tk.StringVar(value="120")
        self.amp_lx_var = tk.StringVar(value="30")
        self.dir_var = tk.StringVar(value="avance")  # avance | retroceso

        self.status_var = tk.StringVar(value="Desconectado")
        self.log_var = tk.StringVar(value="")

        # Buffers de gráfica (alineados por frame)
        self.t_hist = []          # lista de tiempos (uno por frame)
        self.y_hist = {}          # rid -> lista (misma longitud que t_hist), con int o None
        self.lock_hist = threading.Lock()

        self._build_ui()
        self._build_plot()

        self.after(100, self._poll_queue)
        self.after(100, self._refresh_plot)
        self.after(250, self._prune_connected)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # -------------------------
    # UI
    # -------------------------
    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        box = ttk.LabelFrame(top, text="Conexión Maestro", padding=10)
        box.pack(fill="x")

        ttk.Label(box, text="COM:").grid(row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(box, textvariable=self.port_var, width=16, values=list_serial_ports())
        self.port_cb.grid(row=0, column=1, padx=6, sticky="w")

        ttk.Button(box, text="Refrescar", command=self._refresh_ports).grid(row=0, column=2, padx=6)

        ttk.Label(box, text="Baudios:").grid(row=0, column=3, sticky="w", padx=(16, 0))
        ttk.Entry(box, textvariable=self.baud_var, width=10).grid(row=0, column=4, padx=6, sticky="w")

        self.btn_connect = ttk.Button(box, text="Conectar", command=self.connect)
        self.btn_connect.grid(row=0, column=5, padx=6)

        self.btn_start = ttk.Button(box, text="Iniciar envío", command=self.start, state="disabled")
        self.btn_start.grid(row=0, column=6, padx=6)

        self.btn_stop = ttk.Button(box, text="Detener envío", command=self.stop, state="disabled")
        self.btn_stop.grid(row=0, column=7, padx=6)

        ttk.Label(box, textvariable=self.status_var).grid(row=1, column=0, columnspan=8, sticky="w", pady=(6, 0))
        ttk.Label(box, textvariable=self.log_var).grid(row=2, column=0, columnspan=8, sticky="w", pady=(4, 0))

        pbox = ttk.LabelFrame(top, text="Parámetros senoidales", padding=10)
        pbox.pack(fill="x", pady=(10, 0))

        ttk.Label(pbox, text="Frecuencia (Hz):").grid(row=0, column=0, sticky="w")
        ttk.Entry(pbox, textvariable=self.freq_var, width=10).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(pbox, text="Tasa envío (Hz):").grid(row=0, column=2, sticky="w", padx=(16, 0))
        ttk.Entry(pbox, textvariable=self.rate_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(pbox, text="Amp DX (ticks):").grid(row=0, column=4, sticky="w", padx=(16, 0))
        ttk.Entry(pbox, textvariable=self.amp_dx_var, width=10).grid(row=0, column=5, padx=6, sticky="w")

        ttk.Label(pbox, text="Amp LX (unid):").grid(row=0, column=6, sticky="w", padx=(16, 0))
        ttk.Entry(pbox, textvariable=self.amp_lx_var, width=10).grid(row=0, column=7, padx=6, sticky="w")

        ttk.Label(pbox, text="Dirección:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Radiobutton(pbox, text="Avance", variable=self.dir_var, value="avance").grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Radiobutton(pbox, text="Retroceso", variable=self.dir_var, value="retroceso").grid(row=1, column=2, sticky="w", pady=(8, 0))

        note = ttk.Label(top, text="La gráfica muestra el setpoint enviado por frame (posición/ángulo), una curva por cada módulo detectado.")
        note.pack(fill="x", pady=(10, 0))

    def _build_plot(self):
        plot_frame = ttk.Frame(self, padding=(10, 0, 10, 10))
        plot_frame.pack(fill="both", expand=True)

        self.fig = Figure(figsize=(9, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("t (s)")
        self.ax.set_ylabel("Setpoint enviado")
        self.ax.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)
        toolbar.update()

    # -------------------------
    # Serial
    # -------------------------
    def _refresh_ports(self):
        self.port_cb["values"] = list_serial_ports()

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

        self.rx_stop.clear()
        self.rx_thread = SerialReceiver(self.ser, self.q, self.rx_stop)
        self.rx_thread.start()

        self.status_var.set(f"Conectado a {port} @ {baud} | Detectando módulos...")
        self.btn_connect.configure(state="disabled")
        self.btn_start.configure(state="normal")
        self.btn_stop.configure(state="disabled")

    def start(self):
        if not self.ser:
            messagebox.showwarning("Inicio", "Conecte primero el Maestro.")
            return

        try:
            freq = float(self.freq_var.get())
            rate = float(self.rate_var.get())
            amp_dx = float(self.amp_dx_var.get())
            amp_lx = float(self.amp_lx_var.get())
            if freq <= 0 or rate <= 0:
                raise ValueError("Frecuencia y tasa deben ser > 0.")
            if amp_dx < 0 or amp_lx < 0:
                raise ValueError("Amplitudes deben ser >= 0.")
        except Exception as e:
            messagebox.showwarning("Parámetros", str(e))
            return

        self.tx_stop.clear()
        self.tx_thread = SineSender(self.ser, self.write_lock, self, self.tx_stop)
        self.tx_thread.start()

        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="normal")

    def stop(self):
        self.tx_stop.set()
        self.btn_start.configure(state="normal")
        self.btn_stop.configure(state="disabled")

    # -------------------------
    # Auto-detección (telemetría)
    # -------------------------
    def _poll_queue(self):
        try:
            while True:
                msg = self.q.get_nowait()

                if msg[0] == "__error__":
                    self.set_status(f"Error RX: {msg[1]}")
                    self._safe_disconnect()
                    break

                if msg[0] == "__log__":
                    self.log_var.set(msg[1])
                    continue

                if msg[0] == "__seen__":
                    _, node_id, ts = msg
                    if 1 <= node_id <= 6:
                        with self.lock_conn:
                            self.connected[node_id] = ts

        except queue.Empty:
            pass

        self.after(100, self._poll_queue)

    def _prune_connected(self):
        now = time.time()
        with self.lock_conn:
            stale = [rid for rid, ts in self.connected.items() if (now - ts) > CONNECTED_TTL_S]
            for rid in stale:
                del self.connected[rid]
        self.after(250, self._prune_connected)

    def get_connected_ids_snapshot(self):
        with self.lock_conn:
            return sorted(self.connected.keys())

    def set_status_runtime(self, n: int, ids):
        self.status_var.set(f"Conectado | Envío activo | Módulos: {n} -> {ids}")

    def set_status(self, text: str):
        self.status_var.set(text)

    # -------------------------
    # Historial por frame (clave)
    # -------------------------
    def push_frame(self, t: float, frame_values: dict, active_ids: list):
        """
        Inserta un frame completo:
          - agrega t una sola vez
          - para cada rid en y_hist mantiene longitud == len(t_hist)
          - si en este frame no hubo valor para rid, agrega None
        """
        with self.lock_hist:
            self.t_hist.append(t)

            # asegurar claves para ids activos
            for rid in active_ids:
                if rid not in self.y_hist:
                    # crear y rellenar con None para frames anteriores
                    self.y_hist[rid] = [None] * (len(self.t_hist) - 1)

            # para todos los rids ya conocidos, agregar valor o None
            for rid in list(self.y_hist.keys()):
                v = frame_values.get(rid, None)
                self.y_hist[rid].append(v)

            # recortar por ventana temporal
            while self.t_hist and (self.t_hist[-1] - self.t_hist[0] > PLOT_WINDOW_S):
                self.t_hist.pop(0)
                for rid in list(self.y_hist.keys()):
                    if self.y_hist[rid]:
                        self.y_hist[rid].pop(0)

    def _refresh_plot(self):
        self.ax.clear()
        self.ax.set_xlabel("t (s)")
        self.ax.set_ylabel("Setpoint enviado")
        self.ax.grid(True)

        ids = self.get_connected_ids_snapshot()

        has_curves = False
        with self.lock_hist:
            if self.t_hist:
                t0 = self.t_hist[0]
                tt = [x - t0 for x in self.t_hist]

                for rid in ids:
                    yy = self.y_hist.get(rid, [])
                    if len(yy) != len(tt) or len(tt) < 3:
                        continue

                    # construir series sin None
                    xs = []
                    ys = []
                    for x, y in zip(tt, yy):
                        if y is None:
                            continue
                        xs.append(x)
                        ys.append(y)

                    if len(xs) >= 3:
                        self.ax.plot(xs, ys, label=f"R{rid}")
                        has_curves = True

                if has_curves:
                    self.ax.legend(loc="upper right", fontsize=8)
                elif not ids:
                    self.ax.set_title("Sin módulos detectados (esperando telemetría del Maestro).")
                else:
                    self.ax.set_title("Módulos detectados; esperando frames de envío…")
            else:
                self.ax.set_title("Sin datos de seno aún.")

        self.canvas.draw()
        self.after(100, self._refresh_plot)

    # -------------------------
    # Cierre
    # -------------------------
    def _safe_disconnect(self):
        try:
            self.tx_stop.set()
        except Exception:
            pass
        try:
            self.rx_stop.set()
        except Exception:
            pass

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        self.ser = None
        self.btn_connect.configure(state="normal")
        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="disabled")

    def on_close(self):
        self._safe_disconnect()
        self.destroy()


if __name__ == "__main__":
    app = AutoDetectSineApp()
    app.mainloop()
