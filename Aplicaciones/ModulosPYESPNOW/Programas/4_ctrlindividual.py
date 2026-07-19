import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math
import queue

import serial
import serial.tools.list_ports


# =========================
# HOME (según su referencia)
# =========================
HOME_DX = 512   # Dynamixel 1..3
HOME_LX = 95    # LX16A 4..6


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def clamp_int(x: float, lo: int, hi: int) -> int:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return int(round(x))


class SerialRX(threading.Thread):
    """Lee del Maestro para mostrar logs/respuestas (opcional)."""
    def __init__(self, ser: serial.Serial, out_q: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.ser = ser
        self.out_q = out_q
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
                    if s:
                        self.out_q.put(("RX", s))
            except Exception as e:
                self.out_q.put(("ERR", str(e)))
                break


class App(tk.Tk):
    """
    Programa:
      - Conexión Serial al Maestro
      - Selección de módulos "horizontales" (no se mueven durante seno)
      - HOME: lleva a origen TODOS los módulos, incluyendo los horizontales
      - Movimiento senoidal: SOLO módulos NO horizontales
    """
    def __init__(self):
        super().__init__()
        self.title("Control módulos | Horizontales inmóviles + HOME (horizontales sí vuelven a origen)")
        self.geometry("980x620")

        # Serial
        self.ser = None
        self.wlock = threading.Lock()

        # RX
        self.rx_q = queue.Queue()
        self.rx_stop = threading.Event()
        self.rx_thread = None

        # Seno
        self.sine_stop = threading.Event()
        self.sine_thread = None

        # UI vars
        self.port_var = tk.StringVar(value="")
        self.baud_var = tk.StringVar(value="115200")
        self.status_var = tk.StringVar(value="Desconectado")

        # EOL (por compatibilidad)
        self.eol_var = tk.StringVar(value="LF")  # "LF" o "CRLF"

        # Horizontales
        self.horiz_vars = {rid: tk.BooleanVar(value=False) for rid in range(1, 7)}

        # Parámetros seno
        self.freq_var = tk.StringVar(value="0.30")
        self.rate_var = tk.StringVar(value="30")
        self.amp_dx_var = tk.StringVar(value="120")
        self.amp_lx_var = tk.StringVar(value="30")
        self.dir_var = tk.StringVar(value="avance")  # avance/retroceso

        self._build_ui()
        self.after(100, self._poll_rx)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # -------------------------
    # UI
    # -------------------------
    def _build_ui(self):
        top = ttk.LabelFrame(self, text="Conexión Maestro", padding=10)
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

        self.btn_connect = ttk.Button(top, text="Conectar", command=self.connect)
        self.btn_connect.grid(row=0, column=7, padx=6)

        self.btn_disc = ttk.Button(top, text="Desconectar", command=self.disconnect, state="disabled")
        self.btn_disc.grid(row=0, column=8, padx=6)

        ttk.Label(top, textvariable=self.status_var).grid(row=1, column=0, columnspan=9, sticky="w", pady=(6, 0))

        horiz = ttk.LabelFrame(self, text="Módulos horizontales", padding=10)
        horiz.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(
            horiz,
            text="Horizontales: NO se mueven durante el seno. HOME sí los lleva a origen."
        ).grid(row=0, column=0, columnspan=12, sticky="w")

        for rid in range(1, 7):
            ttk.Checkbutton(horiz, text=f"R{rid}", variable=self.horiz_vars[rid])\
                .grid(row=1, column=rid-1, padx=(0, 14), sticky="w")

        ctrl = ttk.LabelFrame(self, text="Acciones", padding=10)
        ctrl.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Button(ctrl, text="HOME (todos a origen)", command=self.send_home_all).grid(row=0, column=0, padx=6, pady=4)

        move = ttk.LabelFrame(self, text="Movimiento senoidal (solo NO horizontales)", padding=10)
        move.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(move, text="Frecuencia (Hz):").grid(row=0, column=0, sticky="w")
        ttk.Entry(move, textvariable=self.freq_var, width=10).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(move, text="Tasa envío (Hz):").grid(row=0, column=2, sticky="w", padx=(16, 0))
        ttk.Entry(move, textvariable=self.rate_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        ttk.Label(move, text="Amp DX (ticks):").grid(row=0, column=4, sticky="w", padx=(16, 0))
        ttk.Entry(move, textvariable=self.amp_dx_var, width=10).grid(row=0, column=5, padx=6, sticky="w")

        ttk.Label(move, text="Amp LX (unid):").grid(row=0, column=6, sticky="w", padx=(16, 0))
        ttk.Entry(move, textvariable=self.amp_lx_var, width=10).grid(row=0, column=7, padx=6, sticky="w")

        ttk.Label(move, text="Dirección:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Radiobutton(move, text="Avance", variable=self.dir_var, value="avance").grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Radiobutton(move, text="Retroceso", variable=self.dir_var, value="retroceso").grid(row=1, column=2, sticky="w", pady=(8, 0))

        self.btn_start = ttk.Button(move, text="Iniciar seno", command=self.start_sine, state="disabled")
        self.btn_start.grid(row=1, column=6, padx=6, pady=(8, 0), sticky="e")

        self.btn_stop = ttk.Button(move, text="Detener seno", command=self.stop_sine, state="disabled")
        self.btn_stop.grid(row=1, column=7, padx=6, pady=(8, 0), sticky="w")

        io = ttk.LabelFrame(self, text="Consola RX (opcional)", padding=10)
        io.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.txt = tk.Text(io, height=14)
        self.txt.pack(fill="both", expand=True)

    # -------------------------
    # Serial helpers
    # -------------------------
    def _refresh_ports(self):
        self.port_cb["values"] = list_serial_ports()

    def _eol_bytes(self) -> bytes:
        return b"\r\n" if self.eol_var.get() == "CRLF" else b"\n"

    def _send_line(self, line: str):
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

        self.rx_stop.clear()
        self.rx_thread = SerialRX(self.ser, self.rx_q, self.rx_stop)
        self.rx_thread.start()

        self.status_var.set(f"Conectado | {port} @ {baud} | EOL={self.eol_var.get()}")
        self.btn_connect.configure(state="disabled")
        self.btn_disc.configure(state="normal")
        self.btn_start.configure(state="normal")

    def disconnect(self):
        self.stop_sine()
        self.rx_stop.set()

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        self.ser = None
        self.status_var.set("Desconectado")
        self.btn_connect.configure(state="normal")
        self.btn_disc.configure(state="disabled")
        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="disabled")

    # -------------------------
    # HOME (incluye horizontales)
    # -------------------------
    def send_home_all(self):
        """
        HOME: lleva a origen TODOS los módulos, incluyendo los horizontales.
        """
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("HOME", "Conecte primero el Maestro.")
            return

        try:
            # Detener seno para que no "pelee" con HOME
            self.stop_sine()

            for rid in (1, 2, 3):
                self._send_line(f"N {rid} POS {HOME_DX}")
                time.sleep(0.03)

            for rid in (4, 5, 6):
                self._send_line(f"N {rid} LX {HOME_LX}")
                time.sleep(0.03)

            self.status_var.set("HOME enviado: horizontales incluidos -> origen")
        except Exception as e:
            messagebox.showerror("HOME", str(e))

    # -------------------------
    # Seno (solo NO horizontales)
    # -------------------------
    def start_sine(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Seno", "Conecte primero el Maestro.")
            return
        if self.sine_thread and self.sine_thread.is_alive():
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

        # Si todos están horizontales, no hay nada que mover
        if all(self.horiz_vars[r].get() for r in range(1, 7)):
            messagebox.showwarning("Horizontales", "Todos los módulos están marcados como horizontales. Ninguno se moverá.")
            return

        self.sine_stop.clear()
        self.sine_thread = threading.Thread(target=self._sine_loop, daemon=True)
        self.sine_thread.start()

        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="normal")
        self.status_var.set("Seno activo (solo NO horizontales)")

    def stop_sine(self):
        self.sine_stop.set()
        if self.ser and self.ser.is_open:
            self.btn_start.configure(state="normal")
        else:
            self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="disabled")

    def _sine_loop(self):
        t0 = time.perf_counter()
        base_phases = {1: 0, 2: 60, 3: 120, 4: 180, 5: 240, 6: 300}

        while not self.sine_stop.is_set():
            try:
                freq = float(self.freq_var.get())
                rate = float(self.rate_var.get())
                amp_dx = float(self.amp_dx_var.get())
                amp_lx = float(self.amp_lx_var.get())
                invert = (self.dir_var.get().strip().lower() == "retroceso")

                t = time.perf_counter() - t0

                for rid in range(1, 7):
                    if self.horiz_vars[rid].get():
                        continue  # horizontales inmóviles

                    ph = base_phases[rid]
                    if invert:
                        ph = -ph

                    if rid in (1, 2, 3):
                        raw = HOME_DX + amp_dx * math.sin(2 * math.pi * freq * t + math.radians(ph))
                        pos = clamp_int(raw, 0, 1023)
                        self._send_line(f"N {rid} POS {pos}")
                    else:
                        raw = HOME_LX + amp_lx * math.sin(2 * math.pi * freq * t + math.radians(ph))
                        ang = clamp_int(raw, 0, 240)
                        self._send_line(f"N {rid} LX {ang}")

                time.sleep(1.0 / rate)

            except Exception as e:
                self.status_var.set(f"Seno detenido por error: {e}")
                self.sine_stop.set()
                break

    # -------------------------
    # RX polling
    # -------------------------
    def _poll_rx(self):
        try:
            while True:
                typ, payload = self.rx_q.get_nowait()
                if typ == "ERR":
                    self.status_var.set(f"RX error: {payload}")
                    self.disconnect()
                    break
                if typ == "RX":
                    self.txt.insert("end", payload + "\n")
                    self.txt.see("end")
        except queue.Empty:
            pass
        self.after(100, self._poll_rx)

    # -------------------------
    # Close
    # -------------------------
    def on_close(self):
        try:
            self.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()
