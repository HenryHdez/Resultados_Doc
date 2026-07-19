import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import time
import serial
import serial.tools.list_ports

#Umbral para fila roja
THRESHOLD = 200  


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


class SerialWorker(threading.Thread):
    """
    Lee del puerto serial:
      - CSV: id,in1,in2,in4
      - Logs: líneas que empiezan por '#'
    Permite enviar comandos enteros al ESP32 maestro:
      N <id> POS <0..1023>
      N <id> SPD <0..1023>
      N <id> HOME
    """
    def __init__(self, port, baud, out_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.stop_event = stop_event
        self.ser = None
        self.wlock = threading.Lock()

    def send_line(self, text: str):
        with self.wlock:
            if self.ser and self.ser.is_open:
                self.ser.write((text.strip() + "\n").encode("utf-8"))

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(1.2)  # reset típico ESP32 al abrir puerto
        except Exception as e:
            self.out_queue.put(("__error__", str(e)))
            return

        buf = b""
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip().decode(errors="ignore")
                    if not line:
                        continue

                    if line.startswith("#"):
                        # Logs del maestro (opcional)
                        self.out_queue.put(("__log__", line))
                        continue

                    parts = line.split(",")
                    if len(parts) != 4:
                        continue

                    try:
                        node_id = int(parts[0])
                        in1 = int(parts[1])
                        in2 = int(parts[2])
                        in4 = int(parts[3])
                        self.out_queue.put((node_id, in1, in2, in4))
                    except ValueError:
                        continue

            except Exception as e:
                self.out_queue.put(("__error__", str(e)))
                break

        try:
            with self.wlock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
        except Exception:
            pass


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ADC + Control Motor (ESP32 Maestro)")
        self.geometry("780x480")

        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.worker = None

        # Estado por nodo: {node_id: (in1,in2,in4,last_ts)}
        self.state = {}

        self._build_ui()
        self._poll_queue()

    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Puerto:").pack(side="left")
        self.port_var = tk.StringVar(value="")
        self.port_combo = ttk.Combobox(
            top, textvariable=self.port_var, width=18,
            values=list_serial_ports()
        )
        self.port_combo.pack(side="left", padx=6)

        ttk.Button(top, text="Refrescar", command=self._refresh_ports).pack(side="left", padx=6)

        ttk.Label(top, text="Baudios:").pack(side="left", padx=(18, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side="left", padx=6)

        self.btn_connect = ttk.Button(top, text="Conectar", command=self._connect)
        self.btn_connect.pack(side="left", padx=(18, 6))

        self.btn_disconnect = ttk.Button(top, text="Desconectar", command=self._disconnect, state="disabled")
        self.btn_disconnect.pack(side="left")

        # ===== Tabla ADC =====
        mid = ttk.Frame(self, padding=10)
        mid.pack(fill="both", expand=True)

        columns = ("node", "in1", "in2", "in4", "last_seen")
        self.tree = ttk.Treeview(mid, columns=columns, show="headings", height=10)
        self.tree.pack(side="left", fill="both", expand=True)

        # Tags para colorear filas
        self.tree.tag_configure("alert", background="#ffcccc")
        self.tree.tag_configure("normal", background="white")

        headings = {
            "node": "Nodo",
            "in1": "IN1 (ADC)",
            "in2": "IN2 (ADC)",
            "in4": "IN4 (ADC)",
            "last_seen": "Última muestra (s)"
        }
        widths = {"node": 70, "in1": 150, "in2": 150, "in4": 150, "last_seen": 170}
        for c in columns:
            self.tree.heading(c, text=headings[c])
            self.tree.column(c, width=widths[c], anchor="center")

        scroll = ttk.Scrollbar(mid, orient="vertical", command=self.tree.yview)
        scroll.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=scroll.set)

        # ===== Panel de control motor (ENTEROS) =====
        ctrl = ttk.LabelFrame(self, text="Control Motor (solo enteros)", padding=10)
        ctrl.pack(fill="x", padx=10, pady=(0, 10))

        ttk.Label(ctrl, text="Nodo (1-6):").grid(row=0, column=0, sticky="w")
        self.node_sel = tk.StringVar(value="1")
        ttk.Combobox(
            ctrl, textvariable=self.node_sel, width=6,
            values=[str(i) for i in range(1, 7)]
        ).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(ctrl, text="Posición (0-1023):").grid(row=0, column=2, sticky="w", padx=(18, 0))
        self.pos_var = tk.StringVar(value="512")
        ttk.Entry(ctrl, textvariable=self.pos_var, width=10).grid(row=0, column=3, padx=6, sticky="w")
        ttk.Button(ctrl, text="Enviar POS", command=self._send_pos).grid(row=0, column=4, padx=6)

        ttk.Label(ctrl, text="Velocidad (0-1023):").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.spd_var = tk.StringVar(value="100")
        ttk.Entry(ctrl, textvariable=self.spd_var, width=10).grid(row=1, column=1, padx=6, pady=(8, 0), sticky="w")
        ttk.Button(ctrl, text="Enviar SPD", command=self._send_spd).grid(row=1, column=2, padx=6, pady=(8, 0), sticky="w")

        ttk.Button(ctrl, text="HOME (512)", command=self._send_home).grid(row=1, column=3, padx=6, pady=(8, 0), sticky="w")

        # ===== Estado / logs =====
        bottom = ttk.Frame(self, padding=(10, 0, 10, 10))
        bottom.pack(fill="x")

        self.status_var = tk.StringVar(value="Desconectado.")
        ttk.Label(bottom, textvariable=self.status_var).pack(side="left")

        self.log_var = tk.StringVar(value="")
        ttk.Label(bottom, textvariable=self.log_var).pack(side="left", padx=12)

        ttk.Button(bottom, text="Limpiar tabla", command=self._clear).pack(side="right")

    def _refresh_ports(self):
        self.port_combo["values"] = list_serial_ports()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Puerto", "Seleccione un puerto COM.")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showwarning("Baudios", "Baudios inválidos.")
            return

        self.stop_event.clear()
        self.worker = SerialWorker(port, baud, self.q, self.stop_event)
        self.worker.start()

        self.btn_connect.config(state="disabled")
        self.btn_disconnect.config(state="normal")
        self.status_var.set(f"Conectado a {port} @ {baud}")

    def _disconnect(self):
        if self.worker:
            self.stop_event.set()
            self.worker = None

        self.btn_connect.config(state="normal")
        self.btn_disconnect.config(state="disabled")
        self.status_var.set("Desconectado.")

    def _clear(self):
        self.state.clear()
        for item in self.tree.get_children():
            self.tree.delete(item)

    def _node_id(self):
        nid = int(self.node_sel.get().strip())
        if nid < 1 or nid > 6:
            raise ValueError("Nodo fuera de rango (1..6)")
        return nid

    def _send_raw(self, cmd: str):
        if not self.worker:
            messagebox.showwarning("Serial", "No está conectado.")
            return
        self.worker.send_line(cmd)

    def _send_pos(self):
        try:
            nid = self._node_id()
            pos = int(self.pos_var.get().strip())
            if pos < 0 or pos > 1023:
                raise ValueError("POS fuera de rango (0..1023)")
        except Exception as e:
            messagebox.showwarning("Entrada", str(e))
            return
        self._send_raw(f"N {nid} POS {pos}")

    def _send_spd(self):
        try:
            nid = self._node_id()
            spd = int(self.spd_var.get().strip())
            if spd < 0 or spd > 1023:
                raise ValueError("SPD fuera de rango (0..1023)")
        except Exception as e:
            messagebox.showwarning("Entrada", str(e))
            return
        self._send_raw(f"N {nid} SPD {spd}")

    def _send_home(self):
        try:
            nid = self._node_id()
        except Exception as e:
            messagebox.showwarning("Entrada", str(e))
            return
        self._send_raw(f"N {nid} HOME")

    def _poll_queue(self):
        try:
            while True:
                msg = self.q.get_nowait()

                if msg[0] == "__error__":
                    self.status_var.set(f"Error: {msg[1]}")
                    self.btn_connect.config(state="normal")
                    self.btn_disconnect.config(state="disabled")
                    break

                if msg[0] == "__log__":
                    # Mostrar último log en la barra de estado
                    self.log_var.set(msg[1])
                    continue

                node_id, in1, in2, in4 = msg
                ts = time.time()
                self.state[node_id] = (in1, in2, in4, ts)
                self._upsert_row(node_id)

        except queue.Empty:
            pass

        self._refresh_last_seen()
        self.after(100, self._poll_queue)

    def _upsert_row(self, node_id):
        in1, in2, in4, ts = self.state[node_id]
        iid = f"node_{node_id}"

        # Regla: si cualquiera < THRESHOLD -> rojo
        is_alert = (in1 < THRESHOLD) or (in2 < THRESHOLD) or (in4 < THRESHOLD)
        tag = "alert" if is_alert else "normal"

        values = (node_id, in1, in2, in4, "0.0")
        if self.tree.exists(iid):
            self.tree.item(iid, values=values, tags=(tag,))
        else:
            self.tree.insert("", "end", iid=iid, values=values, tags=(tag,))

        self.status_var.set(f"Nodos vistos: {len(self.state)} | Umbral: {THRESHOLD}")

    def _refresh_last_seen(self):
        now = time.time()
        for node_id, (in1, in2, in4, ts) in list(self.state.items()):
            age = now - ts
            iid = f"node_{node_id}"
            if self.tree.exists(iid):
                vals = list(self.tree.item(iid, "values"))
                vals[-1] = f"{age:.1f}"
                # Mantener tag (alert/normal) ya asignado
                current_tags = self.tree.item(iid, "tags")
                self.tree.item(iid, values=tuple(vals), tags=current_tags)

    def on_close(self):
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
