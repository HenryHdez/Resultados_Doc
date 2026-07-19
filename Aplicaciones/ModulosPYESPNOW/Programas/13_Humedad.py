# -*- coding: utf-8 -*-

import threading
import time
import queue
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import serial
import serial.tools.list_ports

import tkinter as tk
from tkinter import ttk, messagebox


# ---------------------------
# Modelo de datos
# ---------------------------
@dataclass
class ProxData:
    in1: int = 0
    in2: int = 0
    in4: int = 0
    last_ts: float = 0.0


@dataclass
class HumData:
    hum: int = 0         # 0..1000 (según su nodo)
    last_ts: float = 0.0


class SerialController:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.stop_evt = threading.Event()

        self.lock = threading.Lock()
        self.prox: Dict[int, ProxData] = {}  # id -> ProxData
        self.hum: Dict[int, HumData] = {}    # id -> HumData

        self.log_q: "queue.Queue[str]" = queue.Queue()

    # -------- Serial open/close --------
    def open(self, port: str, baud: int = 115200) -> None:
        if self.ser and self.ser.is_open:
            self.close()

        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.2,
            write_timeout=0.5
        )
        self.stop_evt.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.log_q.put(f"[INFO] Conectado a {port} @ {baud}")

    def close(self) -> None:
        self.stop_evt.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        self.rx_thread = None

        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.log_q.put("[INFO] Desconectado")

    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    # -------- Commands --------
    def send_line(self, line: str) -> None:
        if not self.is_open():
            raise RuntimeError("Serial no está conectado.")
        if not line.endswith("\n"):
            line += "\n"
        self.ser.write(line.encode("utf-8", errors="ignore"))
        self.ser.flush()

    def cmd_pos(self, node_id: int, pos: int) -> None:
        pos = int(max(0, min(1023, pos)))
        self.send_line(f"N {node_id} POS {pos}")
        self.log_q.put(f"[TX] N {node_id} POS {pos}")

    def cmd_spd(self, node_id: int, spd: int) -> None:
        spd = int(max(0, min(1023, spd)))
        self.send_line(f"N {node_id} SPD {spd}")
        self.log_q.put(f"[TX] N {node_id} SPD {spd}")

    def cmd_home(self, node_id: int) -> None:
        self.send_line(f"N {node_id} HOME")
        self.log_q.put(f"[TX] N {node_id} HOME")

    def cmd_ang(self, node_id: int, deg: float) -> None:
        # Se envía como viene; el nodo decide.
        self.send_line(f"N {node_id} ANG {deg}")
        self.log_q.put(f"[TX] N {node_id} ANG {deg}")

    # -------- RX parsing --------
    def _rx_loop(self) -> None:
        assert self.ser is not None
        buf = ""
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk.decode("utf-8", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    if line.startswith("#"):
                        # Mensajes de log del HUB
                        self.log_q.put(f"[HUB] {line}")
                        continue
                    self._parse_csv_line(line)
            except Exception as e:
                self.log_q.put(f"[ERR] RX: {e}")
                time.sleep(0.2)

    def _parse_csv_line(self, line: str) -> None:
        # Esperado:
        #  - 4 campos: id,in1,in2,in4
        #  - 2 campos: id,humedad
        parts = [p.strip() for p in line.split(",")]
        # Filtrar cosas raras
        if len(parts) not in (2, 4):
            self.log_q.put(f"[WARN] Línea no reconocida: {line}")
            return

        try:
            node_id = int(parts[0])
        except ValueError:
            self.log_q.put(f"[WARN] ID inválido: {line}")
            return

        ts = time.time()

        if len(parts) == 4:
            try:
                in1 = int(parts[1]); in2 = int(parts[2]); in4 = int(parts[3])
            except ValueError:
                self.log_q.put(f"[WARN] ADC inválido: {line}")
                return
            with self.lock:
                self.prox[node_id] = ProxData(in1=in1, in2=in2, in4=in4, last_ts=ts)

        elif len(parts) == 2:
            try:
                hum = int(parts[1])
            except ValueError:
                self.log_q.put(f"[WARN] Humedad inválida: {line}")
                return
            with self.lock:
                self.hum[node_id] = HumData(hum=hum, last_ts=ts)

    # -------- Snapshot for UI --------
    def snapshot(self) -> Tuple[Dict[int, ProxData], Dict[int, HumData]]:
        with self.lock:
            prox_copy = dict(self.prox)
            hum_copy = dict(self.hum)
        return prox_copy, hum_copy

    def pop_logs(self, max_n: int = 50) -> str:
        lines = []
        for _ in range(max_n):
            try:
                lines.append(self.log_q.get_nowait())
            except queue.Empty:
                break
        return "\n".join(lines)


# ---------------------------
# UI Tkinter
# ---------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("HUB ESP-NOW: Control Motores + Monitor Proximidad/Humedad")
        self.geometry("980x620")

        self.ctrl = SerialController()

        self._build_ui()
        self._refresh_ports()
        self._tick_ui()

    def _build_ui(self):
        # --- Top: Serial connection ---
        top = ttk.LabelFrame(self, text="Conexión Serial (HUB)")
        top.pack(fill="x", padx=10, pady=8)

        self.port_var = tk.StringVar(value="")
        self.baud_var = tk.IntVar(value=115200)

        ttk.Label(top, text="Puerto:").grid(row=0, column=0, padx=6, pady=6, sticky="w")
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_cb.grid(row=0, column=1, padx=6, pady=6, sticky="w")

        ttk.Button(top, text="Refrescar", command=self._refresh_ports).grid(row=0, column=2, padx=6, pady=6)
        ttk.Label(top, text="Baud:").grid(row=0, column=3, padx=6, pady=6, sticky="e")
        ttk.Entry(top, textvariable=self.baud_var, width=10).grid(row=0, column=4, padx=6, pady=6, sticky="w")

        self.btn_conn = ttk.Button(top, text="Conectar", command=self._toggle_connection)
        self.btn_conn.grid(row=0, column=5, padx=6, pady=6, sticky="w")

        self.status_var = tk.StringVar(value="Desconectado")
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=6, padx=10, pady=6, sticky="w")

        for i in range(7):
            top.grid_columnconfigure(i, weight=0)
        top.grid_columnconfigure(6, weight=1)

        # --- Control motors ---
        ctl = ttk.LabelFrame(self, text="Control de motores (envío de comandos al HUB)")
        ctl.pack(fill="x", padx=10, pady=8)

        self.id_var = tk.IntVar(value=1)
        self.pos_var = tk.IntVar(value=512)
        self.spd_var = tk.IntVar(value=200)
        self.ang_var = tk.DoubleVar(value=0.0)

        ttk.Label(ctl, text="ID motor (1..6):").grid(row=0, column=0, padx=6, pady=6, sticky="w")
        self.id_spin = ttk.Spinbox(ctl, from_=1, to=6, textvariable=self.id_var, width=6)
        self.id_spin.grid(row=0, column=1, padx=6, pady=6, sticky="w")

        ttk.Label(ctl, text="POS (0..1023):").grid(row=0, column=2, padx=6, pady=6, sticky="e")
        ttk.Entry(ctl, textvariable=self.pos_var, width=10).grid(row=0, column=3, padx=6, pady=6, sticky="w")
        ttk.Button(ctl, text="Enviar POS", command=self._send_pos).grid(row=0, column=4, padx=6, pady=6)

        ttk.Label(ctl, text="SPD (0..1023):").grid(row=1, column=2, padx=6, pady=6, sticky="e")
        ttk.Entry(ctl, textvariable=self.spd_var, width=10).grid(row=1, column=3, padx=6, pady=6, sticky="w")
        ttk.Button(ctl, text="Enviar SPD", command=self._send_spd).grid(row=1, column=4, padx=6, pady=6)

        ttk.Label(ctl, text="ANG (deg):").grid(row=1, column=0, padx=6, pady=6, sticky="w")
        ttk.Entry(ctl, textvariable=self.ang_var, width=10).grid(row=1, column=1, padx=6, pady=6, sticky="w")
        ttk.Button(ctl, text="Enviar ANG", command=self._send_ang).grid(row=1, column=4, padx=6, pady=6, sticky="e")

        ttk.Button(ctl, text="HOME", command=self._send_home).grid(row=0, column=5, padx=6, pady=6)
        ttk.Button(ctl, text="HOME (todos 1..6)", command=self._send_home_all).grid(row=1, column=5, padx=6, pady=6)

        # --- Monitor area ---
        mid = ttk.Frame(self)
        mid.pack(fill="both", expand=True, padx=10, pady=8)

        # Tables
        left = ttk.LabelFrame(mid, text="Proximidad / ADC (robots 1..6)")
        left.pack(side="left", fill="both", expand=True, padx=(0, 6))

        right = ttk.LabelFrame(mid, text="Humedad (nodos 7..9 o 2 columnas)")
        right.pack(side="left", fill="both", expand=True, padx=(6, 0))

        self.prox_tree = ttk.Treeview(left, columns=("id", "in1", "in2", "in4", "age"), show="headings", height=12)
        for c, w in [("id", 60), ("in1", 90), ("in2", 90), ("in4", 90), ("age", 120)]:
            self.prox_tree.heading(c, text=c)
            self.prox_tree.column(c, width=w, anchor="center")
        self.prox_tree.pack(fill="both", expand=True, padx=6, pady=6)

        self.hum_tree = ttk.Treeview(right, columns=("id", "hum", "age"), show="headings", height=12)
        for c, w in [("id", 60), ("hum", 120), ("age", 120)]:
            self.hum_tree.heading(c, text=c)
            self.hum_tree.column(c, width=w, anchor="center")
        self.hum_tree.pack(fill="both", expand=True, padx=6, pady=6)

        # --- Log box ---
        logf = ttk.LabelFrame(self, text="Log (TX/RX/HUB)")
        logf.pack(fill="both", expand=True, padx=10, pady=8)

        self.log_txt = tk.Text(logf, height=10, wrap="word")
        self.log_txt.pack(fill="both", expand=True, padx=6, pady=6)
        self.log_txt.configure(state="disabled")

        # Close hook
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _refresh_ports(self):
        ports = []
        for p in serial.tools.list_ports.comports():
            # p.device típico: "COM3"
            ports.append(p.device)

        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.ctrl.is_open():
            self.ctrl.close()
            self.status_var.set("Desconectado")
            self.btn_conn.config(text="Conectar")
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Seleccione un puerto COM.")
            return

        try:
            self.ctrl.open(port, int(self.baud_var.get()))
            self.status_var.set(f"Conectado: {port}")
            self.btn_conn.config(text="Desconectar")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo abrir {port}: {e}")

    def _require_conn(self) -> bool:
        if not self.ctrl.is_open():
            messagebox.showwarning("Serial", "No hay conexión Serial con el HUB.")
            return False
        return True

    def _send_pos(self):
        if not self._require_conn():
            return
        try:
            node_id = int(self.id_var.get())
            pos = int(self.pos_var.get())
            self.ctrl.cmd_pos(node_id, pos)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _send_spd(self):
        if not self._require_conn():
            return
        try:
            node_id = int(self.id_var.get())
            spd = int(self.spd_var.get())
            self.ctrl.cmd_spd(node_id, spd)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _send_ang(self):
        if not self._require_conn():
            return
        try:
            node_id = int(self.id_var.get())
            deg = float(self.ang_var.get())
            self.ctrl.cmd_ang(node_id, deg)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _send_home(self):
        if not self._require_conn():
            return
        try:
            node_id = int(self.id_var.get())
            self.ctrl.cmd_home(node_id)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _send_home_all(self):
        if not self._require_conn():
            return
        try:
            for node_id in range(1, 7):
                self.ctrl.cmd_home(node_id)
                time.sleep(0.03)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _tick_ui(self):
        # Update tables
        prox, hum = self.ctrl.snapshot()
        now = time.time()

        # Prox table
        for row in self.prox_tree.get_children():
            self.prox_tree.delete(row)

        for node_id in sorted(prox.keys()):
            p = prox[node_id]
            age = now - p.last_ts if p.last_ts else 0.0
            self.prox_tree.insert("", "end", values=(node_id, p.in1, p.in2, p.in4, f"{age:0.1f}s"))

        # Hum table
        for row in self.hum_tree.get_children():
            self.hum_tree.delete(row)

        for node_id in sorted(hum.keys()):
            h = hum[node_id]
            age = now - h.last_ts if h.last_ts else 0.0
            self.hum_tree.insert("", "end", values=(node_id, h.hum, f"{age:0.1f}s"))

        # Logs
        logs = self.ctrl.pop_logs(80)
        if logs:
            self.log_txt.configure(state="normal")
            self.log_txt.insert("end", logs + "\n")
            self.log_txt.see("end")
            self.log_txt.configure(state="disabled")

        # Next tick
        self.after(200, self._tick_ui)

    def _on_close(self):
        try:
            self.ctrl.close()
        except Exception:
            pass
        self.destroy()


def main():
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
