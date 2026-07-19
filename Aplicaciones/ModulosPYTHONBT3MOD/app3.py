import asyncio
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Deque
from collections import deque

from bleak import BleakScanner, BleakClient

# ===== UUIDs =====
SVC_UUID = "12345678-1234-1234-1234-1234567890ab"
CHR_NOTIFY_UUID = "12345678-1234-1234-1234-1234567890ac"
CHR_WRITE_UUID  = "12345678-1234-1234-1234-1234567890ad"

# ===== Nodos =====
MAX_NODES = 6
NODE_NAMES = [f"ROBOT_{i}" for i in range(1, MAX_NODES + 1)]

# ===== Timing =====
SCAN_TIMEOUT_S = 1.2
RESCAN_EVERY_S = 6.0
CONNECT_TIMEOUT_S = 10.0

DWELL_S = 1.5          # tiempo conectado a cada nodo (ajuste)
IDLE_BETWEEN_S = 0.15  # pausa entre desconectar y siguiente

# ===== Payload (debe coincidir con el struct del nodo) =====
# node_id(uint8), in1(uint16), in2(uint16), in4(uint16)
PAYLOAD_FMT = "<BHHH"
PAYLOAD_LEN = struct.calcsize(PAYLOAD_FMT)

def log(msg: str):
    print(f"# {msg}", flush=True)

def parse_payload(data: bytearray):
    if len(data) != PAYLOAD_LEN:
        return None
    return struct.unpack(PAYLOAD_FMT, data)  # (node_id,in1,in2,in4)

def notify_handler(sender: int, data: bytearray):
    p = parse_payload(data)
    if not p:
        return
    node_id, in1, in2, in4 = p
    print(f"{node_id},{in1},{in2},{in4}", flush=True)

@dataclass
class NodeInfo:
    node_id: int
    name: str
    address: Optional[str] = None
    last_seen: float = 0.0

nodes: Dict[int, NodeInfo] = {i: NodeInfo(i, NODE_NAMES[i-1]) for i in range(1, MAX_NODES + 1)}

# Cola de comandos por nodo: si no está conectado, se envían cuando toque su turno
pending_cmds: Dict[int, Deque[str]] = {i: deque() for i in range(1, MAX_NODES + 1)}

last_scan = 0.0

async def scan_update():
    """Escanea y actualiza direcciones por nombre."""
    global last_scan
    now = time.time()
    if now - last_scan < 0.5:
        return
    last_scan = now

    devs = await BleakScanner.discover(timeout=SCAN_TIMEOUT_S)
    for d in devs:
        if not d.name:
            continue
        for node_id, n in nodes.items():
            if d.name == n.name:
                if n.address != d.address:
                    n.address = d.address
                    log(f"FOUND {n.name} @ {n.address}")
                n.last_seen = now

def all_have_addr():
    return all(n.address for n in nodes.values())

async def console_task():
    """
    Lee stdin para comandos:
      N <id> <cmd...>
    En este modo rotativo, el comando se encola y se envía cuando el nodo
    esté conectado en su turno.
    """
    loop = asyncio.get_running_loop()
    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if not line:
            await asyncio.sleep(0.05)
            continue
        line = line.strip()
        if not line or line.startswith("#"):
            continue

        parts = line.split()
        if len(parts) < 3 or parts[0].upper() != "N":
            log("ERR formato: N <id> <cmd...>")
            continue

        try:
            node_id = int(parts[1])
        except ValueError:
            log("ERR id inválido")
            continue
        if node_id < 1 or node_id > MAX_NODES:
            log("ERR id fuera de rango 1..6")
            continue

        cmd = " ".join(parts[2:])
        pending_cmds[node_id].append(cmd)
        log(f"QUEUED N{node_id} {cmd}")

async def visit_node(node_id: int):
    """Conecta a un nodo, recibe notificaciones por DWELL_S, envía comandos pendientes, y desconecta."""
    n = nodes[node_id]
    if not n.address:
        return

    log(f"CONNECT {n.name}")
    client = BleakClient(n.address)

    try:
        await client.connect(timeout=CONNECT_TIMEOUT_S)

        # Subscribe notify
        await client.start_notify(CHR_NOTIFY_UUID, notify_handler)

        log(f"CONNECT_OK {n.name}")

        # Enviar comandos pendientes (si los hay)
        while pending_cmds[node_id]:
            cmd = pending_cmds[node_id].popleft()
            try:
                await client.write_gatt_char(CHR_WRITE_UUID, cmd.encode("utf-8"), response=False)
                log(f"SENT N{node_id} {cmd}")
            except Exception as e:
                log(f"ERR write N{node_id} ({e})")
                # si falla, re-encolar al frente para reintentar en el próximo ciclo
                pending_cmds[node_id].appendleft(cmd)
                break

        # Permanecer conectado un rato para recibir NOTIFY
        await asyncio.sleep(DWELL_S)

    except Exception as e:
        log(f"CONNECT_FAIL {n.name} ({e})")

    finally:
        # Desuscribirse/Desconectar y "eliminar objeto" liberando recursos
        try:
            if client.is_connected:
                try:
                    await client.stop_notify(CHR_NOTIFY_UUID)
                except Exception:
                    pass
                await client.disconnect()
        except Exception:
            pass

        # Eliminación explícita (liberar referencia)
        del client
        log(f"DISCONNECT {n.name}")

async def main():
    log("CENTRAL_ROTATE_START (Python/Bleak)")
    asyncio.create_task(console_task())

    # Escaneo inicial agresivo hasta tener direcciones (o 10s)
    t0 = time.time()
    while (not all_have_addr()) and (time.time() - t0 < 10.0):
        await scan_update()

    idx = 1
    last_rescan = 0.0

    while True:
        now = time.time()

        # Re-escaneo periódico (por si cambian direcciones o aparece un nodo nuevo)
        if now - last_rescan > RESCAN_EVERY_S or not all_have_addr():
            last_rescan = now
            await scan_update()

        # Visitar nodo actual
        await visit_node(idx)

        # Siguiente
        idx += 1
        if idx > MAX_NODES:
            idx = 1

        await asyncio.sleep(IDLE_BETWEEN_S)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
