# app.py — tolerant parser + fast plotting in two figures
# pip install pyserial matplotlib

import serial, time, threading, collections, math
from serial.tools import list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------- Serial ----------
BAUD = 115200
PREFERRED = "COM5"  # optional hint

def find_serial_port():
    ports = list(list_ports.comports())
    for p in ports:
        if p.device.upper() == PREFERRED.upper():
            return p.device
    # pick a likely USB serial
    keywords = ("USB", "CP210", "CH340", "SILICON", "FTDI", "UART")
    for p in ports:
        desc = f"{p.description} {p.hwid}".upper()
        if any(k in desc for k in keywords):
            return p.device
    return ports[0].device if ports else None

# ---------- Buffers ----------
N = 1200  # ~4 minutes at 5 Hz
t_ms  = collections.deque(maxlen=N)
roll  = collections.deque(maxlen=N)
pitch = collections.deque(maxlen=N)
ax_d  = collections.deque(maxlen=N)
ay_d  = collections.deque(maxlen=N)
az_d  = collections.deque(maxlen=N)
amag  = collections.deque(maxlen=N)
lat_d = collections.deque(maxlen=N)
lon_d = collections.deque(maxlen=N)

# speed buffers
spd_raw   = collections.deque(maxlen=N)   # windowed distance / time (mph)
spd_med   = collections.deque(maxlen=N)   # median filtered
spd_ma    = collections.deque(maxlen=N)   # moving average
spd_ema   = collections.deque(maxlen=N)   # exponential moving average (final plotted)

# ---------- Helpers ----------
def try_float(s):
    try:
        return float(s)
    except:
        return float('nan')

def parse_triplet(s):  # "x, y, z"
    parts = [p.strip() for p in s.split(',')]
    while len(parts) < 3: parts.append('nan')
    return try_float(parts[0]), try_float(parts[1]), try_float(parts[2])

def parse_pair(s):     # "x, y"
    parts = [p.strip() for p in s.split(',')]
    while len(parts) < 2: parts.append('nan')
    return try_float(parts[0]), try_float(parts[1])

def parse_line(line):
    """
    Accepts:
      t[ms]=1234 | A[g]: ax, ay, az | R/P[deg]: r, p | Hdg[deg]: NA | Lat/Lon: NO_FIX | ...
      t[ms]=1234 | A[g]: ax, ay, az | R/P[deg]: r, p | Hdg[deg]: 123.4 | Lat/Lon: lat, lon | ...
    Returns (t, ax, ay, az, r, p, lat, lon)   (lat/lon can be NaN)
    """
    fields = [f.strip() for f in line.split('|')]
    t = ax = ay = az = r = p = None
    lat = lon = float('nan')

    for f in fields:
        if f.startswith('t[ms]='):
            try:
                t = int(f.split('=',1)[1].strip().split()[0])
            except:
                pass
        elif f.startswith('A[g]:'):
            ax, ay, az = parse_triplet(f.split(':',1)[1])
        elif f.startswith('R/P[deg]:'):
            r, p = parse_pair(f.split(':',1)[1])
        elif f.startswith('Lat/Lon:'):
            rest = f.split(':',1)[1].strip()
            if rest.upper().startswith('NO_FIX'):
                lat = lon = float('nan')
            else:
                lat, lon = parse_pair(rest)

    if None in (t, ax, ay, az, r, p):
        return None
    return t, ax, ay, az, r, p, lat, lon

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi   = math.radians(lat2 - lat1)
    dlambda= math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlambda/2)**2
    return 2*R*math.asin(math.sqrt(a))

# ---------- Speed estimation (new) ----------
# robust speed from GPS positions over a small time window + filtering pipeline

WINDOW_SEC = 1.5       # look 1.5 s back for distance/time (reduces jitter)
MIN_MOVE_M = 0.5       # ignore tiny moves (GPS noise) below 0.5 m
MA_LEN     = 8         # moving average length
EMA_ALPHA  = 0.7       # 0..1 (higher = smoother)

_med_buf = collections.deque(maxlen=5)  # small median filter

def _median(x):
    xs = sorted(x)
    n = len(xs)
    return xs[n//2] if n else float('nan')

def compute_speed_windowed_mph():
    """Speed from haversine over ~WINDOW_SEC."""
    if len(t_ms) < 2:
        return None
    t2 = t_ms[-1]
    # find an index ~WINDOW_SEC back with valid lat/lon
    target = t2 - int(WINDOW_SEC * 1000)
    j = len(t_ms) - 2
    while j >= 0 and t_ms[j] > target:
        j -= 1
    # ensure we have any valid pair
    if j < 0:
        j = 0
    lat1, lon1 = lat_d[j],  lon_d[j]
    lat2, lon2 = lat_d[-1], lon_d[-1]
    if any(math.isnan(v) for v in (lat1, lon1, lat2, lon2)):
        return None
    dt = (t_ms[-1] - t_ms[j]) / 1000.0
    if dt <= 0:
        return None
    d_m = haversine_m(lat1, lon1, lat2, lon2)
    if d_m < MIN_MOVE_M:
        return 0.0  # treat tiny moves as stop
    return (d_m / dt) * 2.23693629  # m/s -> mph

def push_speed_pipeline(v):
    """median -> moving average -> EMA; returns final smoothed speed"""
    # median
    _med_buf.append(v)
    med = _median(_med_buf)

    # moving average
    if not hasattr(push_speed_pipeline, "_ma"):
        push_speed_pipeline._ma = collections.deque(maxlen=MA_LEN)
    push_speed_pipeline._ma.append(med)
    ma = sum(push_speed_pipeline._ma) / len(push_speed_pipeline._ma)

    # ema
    if not hasattr(push_speed_pipeline, "_ema"):
        push_speed_pipeline._ema = ma
    else:
        push_speed_pipeline._ema = EMA_ALPHA * push_speed_pipeline._ema + (1.0 - EMA_ALPHA) * ma
    return push_speed_pipeline._ema

# ---------- Reader ----------
def reader():
    printed = 12  # show a dozen lines to verify format
    while True:
        port = find_serial_port()
        if not port:
            print("No serial ports found. Retrying in 2s…"); time.sleep(2); continue
        try:
            with serial.Serial(port, BAUD, timeout=1) as ser:
                print(f"Connected on {port} @ {BAUD} baud.")
                while True:
                    raw = ser.readline().decode(errors='ignore').strip()
                    if not raw:
                        continue

                    # Accept both human-readable and pure CSV
                    if raw[0].isdigit() or raw.startswith('-') or raw.startswith(' '):
                        # CSV from TX: ax,ay,az,roll,pitch[,lat,lon]
                        parts = [p.strip() for p in raw.split(',')]
                        if len(parts) >= 5:
                            try:
                                ax = float(parts[0]); ay = float(parts[1]); az = float(parts[2])
                                r  = float(parts[3]);  p  = float(parts[4])
                                # lat/lon if present (>=7 fields)
                                if len(parts) >= 7:
                                    lat = float(parts[5]); lon = float(parts[6])
                                else:
                                    lat = lon = float('nan')
                                t   = int(time.monotonic()*1000)
                                if printed>0: print(f"[csv] {raw}"); printed-=1
                            except:
                                continue
                        else:
                            continue
                    else:
                        parsed = parse_line(raw)
                        if not parsed:
                            if printed>0: print(f"[unmatched] {raw}"); printed-=1
                            continue
                        t, ax, ay, az, r, p, lat, lon = parsed
                        if printed>0: print(f"[ok] {raw}"); printed-=1

                    # store
                    t_ms.append(t)
                    ax_d.append(ax); ay_d.append(ay); az_d.append(az)
                    amag.append((ax*ax + ay*ay + az*az)**0.5)
                    roll.append(r); pitch.append(p)
                    lat_d.append(lat); lon_d.append(lon)

                    # speed pipeline
                    s = compute_speed_windowed_mph()
                    if s is None:
                        # carry last value if no GPS yet
                        s = spd_ema[-1] if len(spd_ema) else 0.0
                    spd_raw.append(s)
                    spd_med.append(_median(_med_buf) if _med_buf else s)  # optional to inspect
                    spd_ma.append(s)  # (intermediate MA kept inside push_speed_pipeline)
                    spd_ema.append(push_speed_pipeline(s))

        except serial.SerialException as e:
            print(f"Serial error on {port}: {e}. Retrying in 2s…"); time.sleep(2)

threading.Thread(target=reader, daemon=True).start()

# ---------- Figures & lines (fast updates) ----------
# Figure 1: Roll/Pitch + Accel
fig1 = plt.figure("IMU Attitude & Acceleration", figsize=(11,6))
ax_rp = fig1.add_subplot(2,1,1); ax_rp.grid(True); ax_rp.set_title("Roll / Pitch (deg)"); ax_rp.set_ylabel("deg")
ax_ag = fig1.add_subplot(2,1,2); ax_ag.grid(True); ax_ag.set_title("Acceleration (g)"); ax_ag.set_xlabel("Time (s)"); ax_ag.set_ylabel("g")
ln_roll,  = ax_rp.plot([], [], label="Roll")
ln_pitch, = ax_rp.plot([], [], label="Pitch")
ax_rp.legend(loc="upper right")
ln_mag, = ax_ag.plot([], [], label="|a|")
ln_ax,  = ax_ag.plot([], [], label="Ax", alpha=0.7)
ln_ay,  = ax_ag.plot([], [], label="Ay", alpha=0.7)
ln_az,  = ax_ag.plot([], [], label="Az", alpha=0.7)
ax_ag.legend(loc="upper right")

# Figure 2: Speed + GPS trail
fig2 = plt.figure("GPS Speed & Trail", figsize=(9,6))
ax_sp  = fig2.add_subplot(2,1,1); ax_sp.grid(True); ax_sp.set_title("Speed (mph) — windowed + median + MA + EMA"); ax_sp.set_ylabel("mph")
ax_map = fig2.add_subplot(2,1,2); ax_map.grid(True); ax_map.set_title("GPS Trail"); ax_map.set_xlabel("Lon"); ax_map.set_ylabel("Lat")
ln_spd, = ax_sp.plot([], [], label="Speed (mph)")
ax_sp.legend(loc="upper right")
ln_path, = ax_map.plot([], [], marker='.', linestyle='-')

def update_figs(_):
    if len(t_ms) < 2:
        return
    t0 = t_ms[0]
    tx = [(ti - t0)/1000.0 for ti in t_ms]

    # Figure 1
    ln_roll.set_data(tx, list(roll))
    ln_pitch.set_data(tx, list(pitch))
    ax_rp.relim(); ax_rp.autoscale_view()

    ln_mag.set_data(tx, list(amag))
    ln_ax.set_data(tx, list(ax_d))
    ln_ay.set_data(tx, list(ay_d))
    ln_az.set_data(tx, list(az_d))
    ax_ag.relim(); ax_ag.autoscale_view()

    # Figure 2 — use EMA (final smoothed)
    k = min(len(spd_ema), len(tx))
    ln_spd.set_data(tx[-k:], list(spd_ema)[-k:])
    ax_sp.relim(); ax_sp.autoscale_view()

    # GPS trail (filter NaNs)
    xs = [lo for lo in lon_d if not math.isnan(lo)]
    ys = [la for la in lat_d if not math.isnan(la)]
    ln_path.set_data(xs, ys)
    ax_map.relim(); ax_map.autoscale_view()

# two separate animations so both figures refresh
ani1 = FuncAnimation(fig1, update_figs, interval=200, cache_frame_data=False)
ani2 = FuncAnimation(fig2, update_figs, interval=200, cache_frame_data=False)

plt.show()
