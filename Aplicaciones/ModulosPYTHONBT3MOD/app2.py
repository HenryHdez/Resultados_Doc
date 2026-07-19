import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import time

import serial
import serial.tools.list_ports


THRESHOLD = 200  # IN1/IN2/IN3 < THRESHOLD -> fila roja


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


class MasterSerialWorker(threading.Thread):
    """
    Lee del ESP32 Maestro:
      - CSV: node_id,in1,in2,in3
      - Logs: líneas que empiezan con '#'
    Permite enviar comandos al Maestro (que reenviará por BLE).
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
            time.sleep(1.2)  # reset típico ESP32
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
                        self.out_queue.put(("__log__", line))
                        continue

                    parts = line.split(",")
                    if len(parts) != 4:
                        continue

                    try:
                        node_id = int(parts[0])
                        in1 = int(parts[1])
                        in2 = int(parts[2])
                        in3 = int(parts[3])
                        self.out_queue.put(("__adc__", node_id, in1, in2, in3))
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
        self.title("Robots 1–6 | Maestro único | Sensores + Motores (AX/LX)")
        self.geometry("920x620")

        self.q = queue.Queue()

        self.stop_event = threading.Event()
        self.worker = None

        # node_id -> (in1,in2,in3,last_ts)
        self.state = {}

        self._build_ui()
        self._poll_queue()

    def _build_ui(self):
        # =======================
        # Conexión Maestro
        # =======================
        frm_top = ttk.LabelFrame(self, text="Conexión Maestro (ESP32 Central)", padding=10)
        frm_top.pack(fill="x", padx=10, pady=10)

        ttk.Label(frm_top, text="COM Maestro:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value="")
        self.port_cb = ttk.Combobox(frm_top, textvariable=self.port_var, width=14, values=list_serial_ports())
        self.port_cb.grid(row=0, column=1, padx=6, sticky="w")

        ttk.Button(frm_top, text="Refrescar", command=self._refresh_ports).grid(row=0, column=2, padx=6)

        ttk.Label(frm_top, text="Baudios:").grid(row=0, column=3, sticky="w", padx=(16, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(frm_top, textvariable=self.baud_var, width=10).grid(row=0, column=4, padx=6, sticky="w")

        self.btn_connect = ttk.Button(frm_top, text="Conectar", command=self._connect)
        self.btn_connect.grid(row=0, column=5, padx=6)

        self.btn_disc = ttk.Button(frm_top, text="Desconectar", command=self._disconnect, state="disabled")
        self.btn_disc.grid(row=0, column=6, padx=6)

        self.status_var = tk.StringVar(value="Maestro: desconectado")
        ttk.Label(frm_top, textvariable=self.status_var).grid(row=1, column=0, columnspan=7, sticky="w", pady=(6, 0))

        self.log_var = tk.StringVar(value="")
        ttk.Label(frm_top, textvariable=self.log_var).grid(row=2, column=0, columnspan=7, sticky="w")

        # =======================
        # Tabla sensores
        # =======================
        frm_mid = ttk.LabelFrame(self, text="Sensores (fila roja si IN < 200)", padding=10)
        frm_mid.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        cols = ("node", "in1", "in2", "in3", "last_seen")
        self.tree = ttk.Treeview(frm_mid, columns=cols, show="headings", height=10)
        self.tree.pack(side="left", fill="both", expand=True)

        self.tree.tag_configure("alert", background="#ffcccc")
        self.tree.tag_configure("normal", background="white")

        heads = {
            "node": "Robot",
            "in1": "IN1 (ADC)",
            "in2": "IN2 (ADC)",
            "in3": "IN3 (ADC)",
            "last_seen": "Última muestra (s)"
        }
        widths = {"node": 90, "in1": 150, "in2": 150, "in3": 150, "last_seen": 170}
        for c in cols:
            self.tree.heading(c, text=heads[c])
            self.tree.column(c, width=widths[c], anchor="center")

        scr = ttk.Scrollbar(frm_mid, orient="vertical", command=self.tree.yview)
        scr.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=scr.set)

        # =======================
        # Control (mismo combobox)
        # Sección A arriba, B abajo
        # =======================
        frm_ctrl = ttk.LabelFrame(self, text="Control por robot (motores diferentes)", padding=10)
        frm_ctrl.pack(fill="x", padx=10, pady=(0, 10))

        # --- Selector común ---
        ttk.Label(frm_ctrl, text="Robot (1–6):").grid(row=0, column=0, sticky="w")
        self.robot_sel = tk.StringVar(value="1")
        self.robot_cb = ttk.Combobox(frm_ctrl, textvariable=self.robot_sel, width=6,
                                     values=[str(i) for i in range(1, 7)])
        self.robot_cb.grid(row=0, column=1, padx=6, sticky="w")
        self.robot_cb.bind("<<ComboboxSelected>>", lambda e: self._update_motor_panels())

        # estado general
        self.ctrl_status = tk.StringVar(value="Seleccione un robot.")
        ttk.Label(frm_ctrl, textvariable=self.ctrl_status).grid(row=0, column=2, columnspan=6, sticky="w", padx=(12, 0))

        # -------- Sección A (AX/Dynamixel) --------
        self.frm_a = ttk.LabelFrame(frm_ctrl, text="Sección A - AX/Dynamixel (robots 1–3)", padding=10)
        self.frm_a.grid(row=1, column=0, columnspan=8, sticky="ew", pady=(10, 6))

        ttk.Label(self.frm_a, text="POS (0-1023):").grid(row=0, column=0, sticky="w")
        self.pos_var = tk.StringVar(value="512")
        ttk.Entry(self.frm_a, textvariable=self.pos_var, width=10).grid(row=0, column=1, padx=6, sticky="w")
        self.btn_pos = ttk.Button(self.frm_a, text="Enviar POS", command=self._send_pos)
        self.btn_pos.grid(row=0, column=2, padx=6)

        ttk.Label(self.frm_a, text="SPD (0-1023):").grid(row=0, column=3, sticky="w", padx=(16, 0))
        self.spd_var = tk.StringVar(value="25")
        ttk.Entry(self.frm_a, textvariable=self.spd_var, width=10).grid(row=0, column=4, padx=6, sticky="w")
        self.btn_spd = ttk.Button(self.frm_a, text="Enviar SPD", command=self._send_spd)
        self.btn_spd.grid(row=0, column=5, padx=6)

        self.btn_home = ttk.Button(self.frm_a, text="HOME (512)", command=self._send_home)
        self.btn_home.grid(row=0, column=6, padx=(16, 6))

        self.status_a = tk.StringVar(value="A: listo")
        ttk.Label(self.frm_a, textvariable=self.status_a).grid(row=1, column=0, columnspan=7, sticky="w", pady=(6, 0))

        # -------- Sección B (LX16A) debajo --------
        self.frm_b = ttk.LabelFrame(frm_ctrl, text="Sección B - LX16A (robots 4–6)", padding=10)
        self.frm_b.grid(row=2, column=0, columnspan=8, sticky="ew", pady=(6, 0))

        ttk.Label(self.frm_b, text="Ángulo (0–240):").grid(row=0, column=0, sticky="w")
        self.lx_ang_var = tk.StringVar(value="120")
        ttk.Entry(self.frm_b, textvariable=self.lx_ang_var, width=10).grid(row=0, column=1, padx=6, sticky="w")
        self.btn_lx = ttk.Button(self.frm_b, text="Mover LX", command=self._send_lx)
        self.btn_lx.grid(row=0, column=2, padx=6)

        ttk.Label(self.frm_b, text="(envía: LX <angulo>)").grid(row=0, column=3, sticky="w", padx=(16, 0))

        self.status_b = tk.StringVar(value="B: listo")
        ttk.Label(self.frm_b, textvariable=self.status_b).grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))

        # -----------------------
        # inferior
        # -----------------------
        frm_bottom = ttk.Frame(self, padding=(10, 0, 10, 10))
        frm_bottom.pack(fill="x")
        ttk.Button(frm_bottom, text="Limpiar tabla", command=self._clear).pack(side="right")

        # Paneles iniciales según robot 1
        self._update_motor_panels()

    # =========================
    # Maestro connect/disconnect
    # =========================
    def _refresh_ports(self):
        self.port_cb["values"] = list_serial_ports()
        self.robot_cb["values"] = [str(i) for i in range(1, 7)]

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Maestro", "Seleccione COM Maestro.")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showwarning("Maestro", "Baudios inválidos.")
            return

        self.stop_event.clear()
        self.worker = MasterSerialWorker(port, baud, self.q, self.stop_event)
        self.worker.start()

        self.btn_connect.config(state="disabled")
        self.btn_disc.config(state="normal")
        self.status_var.set(f"Maestro: conectado a {port} @ {baud}")

    def _disconnect(self):
        if self.worker:
            self.stop_event.set()
            self.worker = None
        self.btn_connect.config(state="normal")
        self.btn_disc.config(state="disabled")
        self.status_var.set("Maestro: desconectado")

    # =========================
    # Utilidad / envío maestro
    # =========================
    def _clear(self):
        self.state.clear()
        for item in self.tree.get_children():
            self.tree.delete(item)

    def _robot_id(self) -> int:
        rid = int(self.robot_sel.get().strip())
        if rid < 1 or rid > 6:
            raise ValueError("Robot debe ser 1..6")
        return rid

    def _send_to_master(self, line: str):
        if not self.worker:
            raise RuntimeError("Maestro no conectado")
        self.worker.send_line(line)

    # =========================
    # Habilitar/Deshabilitar paneles según robot
    # =========================
    def _update_motor_panels(self):
        try:
            rid = self._robot_id()
        except Exception:
            rid = 1

        if rid in (1, 2, 3):
            # AX habilitado, LX deshabilitado
            self._set_section_state(self.frm_a, "normal")
            self._set_section_state(self.frm_b, "disabled")
            self.ctrl_status.set("Motor activo: AX/Dynamixel (Sección A). LX16A deshabilitado.")
        else:
            # LX habilitado, AX deshabilitado
            self._set_section_state(self.frm_a, "disabled")
            self._set_section_state(self.frm_b, "normal")
            self.ctrl_status.set("Motor activo: LX16A (Sección B). AX/Dynamixel deshabilitado.")

    def _set_section_state(self, frame: ttk.LabelFrame, state: str):
        # Aplica state a widgets hijos (botones/entradas)
        for child in frame.winfo_children():
            cls = child.winfo_class()
            if cls in ("TEntry", "TCombobox", "TButton"):
                try:
                    child.configure(state=state)
                except Exception:
                    pass

    # =========================
    # Sección A: AX/Dynamixel
    # =========================
    def _send_pos(self):
        try:
            rid = self._robot_id()
            if rid not in (1, 2, 3):
                raise ValueError("POS aplica solo a robots 1–3 (AX/Dynamixel).")

            pos = int(self.pos_var.get().strip())
            if pos < 0 or pos > 1023:
                raise ValueError("POS fuera de rango (0..1023)")

            self._send_to_master(f"N {rid} POS {pos}")
            self.status_a.set(f"A: enviado -> N {rid} POS {pos}")
        except Exception as e:
            messagebox.showwarning("Sección A", str(e))

    def _send_spd(self):
        try:
            rid = self._robot_id()
            if rid not in (1, 2, 3):
                raise ValueError("SPD aplica solo a robots 1–3 (AX/Dynamixel).")

            spd = int(self.spd_var.get().strip())
            if spd < 0 or spd > 1023:
                raise ValueError("SPD fuera de rango (0..1023)")

            self._send_to_master(f"N {rid} SPD {spd}")
            self.status_a.set(f"A: enviado -> N {rid} SPD {spd}")
        except Exception as e:
            messagebox.showwarning("Sección A", str(e))

    def _send_home(self):
        try:
            rid = self._robot_id()
            if rid not in (1, 2, 3):
                raise ValueError("HOME aplica solo a robots 1–3 (AX/Dynamixel).")
            self._send_to_master(f"N {rid} HOME")
            self.status_a.set(f"A: enviado -> N {rid} HOME")
        except Exception as e:
            messagebox.showwarning("Sección A", str(e))

    # =========================
    # Sección B: LX16A
    # =========================
    def _send_lx(self):
        """
        Envía al robot seleccionado:
          N <id> LX <angulo_int>
        Rango solicitado: 0..240
        """
        try:
            rid = self._robot_id()
            if rid not in (4, 5, 6):
                raise ValueError("LX aplica solo a robots 4–6 (LX16A).")

            ang = int(self.lx_ang_var.get().strip())
            if ang < 0 or ang > 240:
                raise ValueError("Ángulo LX fuera de rango (0..240)")

            self._send_to_master(f"N {rid} LX {ang}")
            self.status_b.set(f"B: enviado -> N {rid} LX {ang}")
        except Exception as e:
            messagebox.showwarning("Sección B", str(e))

    # =========================
    # Actualización GUI (cola)
    # =========================
    def _poll_queue(self):
        try:
            while True:
                msg = self.q.get_nowait()

                if msg[0] == "__error__":
                    self.status_var.set(f"Maestro error: {msg[1]}")
                    self.btn_connect.config(state="normal")
                    self.btn_disc.config(state="disabled")
                    self.worker = None
                    break

                if msg[0] == "__log__":
                    self.log_var.set(msg[1])
                    continue

                if msg[0] == "__adc__":
                    _, node_id, in1, in2, in3 = msg
                    ts = time.time()
                    self.state[node_id] = (in1, in2, in3, ts)
                    self._upsert_row(node_id)

        except queue.Empty:
            pass

        self._refresh_last_seen()
        self.after(100, self._poll_queue)

    def _upsert_row(self, node_id: int):
        in1, in2, in3, ts = self.state[node_id]
        iid = f"robot_{node_id}"

        is_alert = (in1 < THRESHOLD) or (in2 < THRESHOLD) or (in3 < THRESHOLD)
        tag = "alert" if is_alert else "normal"

        values = (node_id, in1, in2, in3, "0.0")
        if self.tree.exists(iid):
            self.tree.item(iid, values=values, tags=(tag,))
        else:
            self.tree.insert("", "end", iid=iid, values=values, tags=(tag,))

    def _refresh_last_seen(self):
        now = time.time()
        for node_id, (in1, in2, in3, ts) in list(self.state.items()):
            age = now - ts
            iid = f"robot_{node_id}"
            if self.tree.exists(iid):
                vals = list(self.tree.item(iid, "values"))
                vals[-1] = f"{age:.1f}"
                tags = self.tree.item(iid, "tags")
                self.tree.item(iid, values=tuple(vals), tags=tags)

    def on_close(self):
        try:
            self._disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
