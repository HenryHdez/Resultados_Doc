# -*- coding: utf-8 -*-
"""
Sistema integrado:
- Simulación o Serial
- Tabla de humedad por sensor
- Mapa desde Excel (matriz discreta)
- Búsqueda sobre grilla discreta
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading, time
import serial
import serial.tools.list_ports
import numpy as np
from datetime import datetime

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from openpyxl import load_workbook


# =========================================================
# Utilidades
# =========================================================
def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# =========================================================
# Mapa discreto desde Excel
# =========================================================
class DiscreteHumidityMap:
    def __init__(self):
        self.grid = None
        self.nx = 0
        self.ny = 0

    def load_from_excel(self, path):
        wb = load_workbook(path, data_only=True)
        ws = wb.active

        data = []
        for row in ws.iter_rows(values_only=True):
            r = []
            for v in row:
                if v is None:
                    continue
                r.append(float(v))
            if r:
                data.append(r)

        self.grid = np.array(data, dtype=float)
        self.ny, self.nx = self.grid.shape

        # normalizar 0..1
        mn = np.min(self.grid)
        mx = np.max(self.grid)
        if mx - mn > 1e-9:
            self.grid = (self.grid - mn) / (mx - mn)

    def value_at_index(self, ix, iy):
        ix = clamp(ix, 0, self.nx - 1)
        iy = clamp(iy, 0, self.ny - 1)
        return self.grid[iy, ix]


# =========================================================
# Aplicación principal
# =========================================================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Mapa de Humedad Discreto + Sensores")
        self.geometry("1350x800")

        # ---------------- Estado ----------------
        self.mode_var = tk.StringVar(value="SIMULACIÓN")
        self.ser = None
        self.stop_event = threading.Event()

        self.sensor_values = {}   # id -> humedad
        self.map = DiscreteHumidityMap()

        # posición discreta del robot
        self.rx = 0
        self.ry = 0

        # ---------------- UI ----------------
        self._build_ui()

    # =====================================================
    # UI
    # =====================================================
    def _build_ui(self):
        main = ttk.Frame(self, padding=10)
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main)
        left.pack(side="left", fill="y", padx=10)

        right = ttk.Frame(main)
        right.pack(side="right", fill="both", expand=True)

        # -------- Modo --------
        fmode = ttk.LabelFrame(left, text="Modo")
        fmode.pack(fill="x", pady=5)

        ttk.Radiobutton(fmode, text="Simulación",
                        variable=self.mode_var, value="SIMULACIÓN").pack(anchor="w")
        ttk.Radiobutton(fmode, text="Serial",
                        variable=self.mode_var, value="SERIAL").pack(anchor="w")

        # -------- Serial --------
        fser = ttk.LabelFrame(left, text="Serial")
        fser.pack(fill="x", pady=5)

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")

        ttk.Label(fser, text="COM").grid(row=0, column=0)
        ttk.Combobox(fser, textvariable=self.port_var,
                     values=list_serial_ports(), width=10).grid(row=0, column=1)
        ttk.Label(fser, text="Baud").grid(row=1, column=0)
        ttk.Entry(fser, textvariable=self.baud_var, width=10).grid(row=1, column=1)

        ttk.Button(fser, text="Conectar", command=self.connect).grid(row=2, column=0, columnspan=2)

        # -------- Excel --------
        fexcel = ttk.LabelFrame(left, text="Mapa Excel")
        fexcel.pack(fill="x", pady=5)

        ttk.Button(fexcel, text="Cargar matriz Excel",
                   command=self.load_excel).pack(fill="x")

        # -------- Sensores --------
        fsens = ttk.LabelFrame(left, text="Humedad por sensor")
        fsens.pack(fill="both", expand=True, pady=5)

        self.sensor_table = ttk.Treeview(
            fsens, columns=("id", "h", "t"), show="headings", height=10)
        self.sensor_table.heading("id", text="Sensor")
        self.sensor_table.heading("h", text="Humedad")
        self.sensor_table.heading("t", text="Tiempo")
        self.sensor_table.pack(fill="both", expand=True)

        # -------- Gráfica --------
        fig = Figure(figsize=(6.5, 6), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_title("Mapa de Humedad (discreto)")
        self.im = None

        self.canvas = FigureCanvasTkAgg(fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        ttk.Button(left, text="Iniciar", command=self.start).pack(fill="x", pady=10)
        ttk.Button(left, text="Detener", command=self.stop).pack(fill="x")

    # =====================================================
    # Serial
    # =====================================================
    def connect(self):
        try:
            self.ser = serial.Serial(
                self.port_var.get(),
                int(self.baud_var.get()),
                timeout=0.2
            )
            messagebox.showinfo("Serial", "Conectado")
        except Exception as e:
            messagebox.showerror("Serial", str(e))

    # =====================================================
    # Excel
    # =====================================================
    def load_excel(self):
        path = filedialog.askopenfilename(
            filetypes=[("Excel", "*.xlsx")]
        )
        if not path:
            return
        self.map.load_from_excel(path)
        self.rx = self.map.nx // 2
        self.ry = self.map.ny // 2
        self.update_map()

    # =====================================================
    # Loop principal
    # =====================================================
    def start(self):
        self.stop_event.clear()
        threading.Thread(target=self.loop, daemon=True).start()

    def stop(self):
        self.stop_event.set()

    def loop(self):
        while not self.stop_event.is_set():
            if self.mode_var.get() == "SERIAL":
                self.read_serial()
            else:
                self.simulate_sensors()

            self.update_table()
            self.update_map()

            time.sleep(0.5)

    # =====================================================
    # Sensores
    # =====================================================
    def read_serial(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            line = self.ser.readline().decode().strip()
            if not line:
                return
            sid, val = line.split(",")
            h = float(val) / 1023.0
            self.sensor_values[int(sid)] = clamp(h, 0, 1)
        except Exception:
            pass

    def simulate_sensors(self):
        for sid in range(1, 5):
            self.sensor_values[sid] = clamp(
                np.random.normal(0.6, 0.05), 0, 1)

    # =====================================================
    # UI Updates
    # =====================================================
    def update_table(self):
        for i in self.sensor_table.get_children():
            self.sensor_table.delete(i)

        now = datetime.now().strftime("%H:%M:%S")
        for sid, h in sorted(self.sensor_values.items()):
            self.sensor_table.insert(
                "", "end", values=(sid, f"{h:.3f}", now))

    def update_map(self):
        if self.map.grid is None:
            return

        # promedio sensores
        if self.sensor_values:
            avg_h = sum(self.sensor_values.values()) / len(self.sensor_values)
        else:
            avg_h = 0.0

        display = self.map.grid.copy()
        display[self.ry, self.rx] = avg_h  # overlay promedio

        self.ax.clear()
        self.im = self.ax.imshow(display, origin="lower",
                                 cmap="jet", vmin=0, vmax=1)
        self.ax.plot(self.rx, self.ry, "wx", markersize=12)
        self.ax.set_title(
            f"Humedad promedio sensores = {avg_h:.3f}")

        self.canvas.draw_idle()


# =========================================================
if __name__ == "__main__":
    App().mainloop()
