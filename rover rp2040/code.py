import board, busio, pwmio, time, gc

uart = busio.UART(tx=board.GP0, rx=board.GP1, baudrate=115200, timeout=0, receiver_buffer_size=256)

PWM_FREQ = 20000
MAX_DUTY = 65535
FAILSAFE_S = 0.4
TICK_S = 0.01

# order: FL, FR, RL, RR  -> (forward pin, backward pin)
motors = [
    (pwmio.PWMOut(board.GP9, frequency=PWM_FREQ), pwmio.PWMOut(board.GP8, frequency=PWM_FREQ)),
    (pwmio.PWMOut(board.GP12, frequency=PWM_FREQ), pwmio.PWMOut(board.GP13, frequency=PWM_FREQ)),
    (pwmio.PWMOut(board.GP11, frequency=PWM_FREQ), pwmio.PWMOut(board.GP10, frequency=PWM_FREQ)),
    (pwmio.PWMOut(board.GP14, frequency=PWM_FREQ), pwmio.PWMOut(board.GP15, frequency=PWM_FREQ)),
]

invert = [1, 1, 1, 1]
strafe_sign = 1
rot_sign = 1
min_duty = 28500
accel = 25.0
spin_ms = 1400

target = [0.0] * 4
current = [0.0] * 4
was_stopped = [True] * 4
stiction_kick = [0] * 4
last_cmd = time.monotonic()
macro = []
macro_end = 0.0
rx = b""


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def mix(x, y, z, spd):
    x *= strafe_sign
    z *= rot_sign
    w = [y + x + z, y - x - z, y - x + z, y + x - z]
    m = max(abs(v) for v in w)
    if m > 100.0:
        w = [v * 100.0 / m for v in w]
    k = spd / 100.0
    return [v * k for v in w]


def drive(i, speed):
    fwd, bck = motors[i]
    speed *= invert[i]
    a = abs(speed)
    if a < 2.0:
        fwd.duty_cycle = 0
        bck.duty_cycle = 0
        was_stopped[i] = True
        stiction_kick[i] = 0
        return
    if was_stopped[i]:
        was_stopped[i] = False
        stiction_kick[i] = 6  # 6 ticks * 10ms = 60ms breakaway kick to break mecanum stiction
    duty = min_duty + int((a - 2.0) / 98.0 * (MAX_DUTY - min_duty))
    if stiction_kick[i] > 0:
        stiction_kick[i] -= 1
        duty = max(duty, 38000)
    elif a < 16.0:
        dither = 1200 if (int(time.monotonic() * 100) % 4 < 2) else -1200
        duty = clamp(duty + dither, min_duty, MAX_DUTY)
    duty = clamp(duty, 0, MAX_DUTY)
    if speed > 0:
        bck.duty_cycle = 0
        fwd.duty_cycle = duty
    else:
        fwd.duty_cycle = 0
        bck.duty_cycle = duty


def stop_all():
    global macro
    macro = []
    for i in range(4):
        target[i] = 0.0
        current[i] = 0.0
        was_stopped[i] = True
        stiction_kick[i] = 0
        drive(i, 0)


def start_macro(mid, direction):
    global macro, macro_end
    d = 1 if direction >= 0 else -1
    s = spin_ms / 1000.0
    if mid == 1:    # spin 360 in place
        macro = [(0, 0, 100 * d, 100, s), (0, 0, 0, 0, 0.15)]
    elif mid == 2:  # spin 180
        macro = [(0, 0, 100 * d, 100, s / 2), (0, 0, 0, 0, 0.15)]
    elif mid == 3:  # J-turn
        macro = [(0, -100, 0, 100, 0.45), (0, 0, 100 * d, 100, s / 2), (0, 100, 0, 100, 0.45), (0, 0, 0, 0, 0.15)]
    elif mid == 4:  # strafing circle (translate while rotating)
        macro = [(100 * d, 0, -60 * d, 100, s * 1.5), (0, 0, 0, 0, 0.15)]
    else:
        return
    macro_end = 0.0


def run_macro(now):
    global macro, macro_end
    if not macro:
        return
    if macro_end == 0.0 or now >= macro_end:
        if macro_end != 0.0:
            macro.pop(0)
            if not macro:
                macro_end = 0.0
                return
        x, y, z, s, dur = macro[0]
        t = mix(x, y, z, s)
        for i in range(4):
            target[i] = t[i]
        macro_end = now + dur


def handle(line):
    global last_cmd, strafe_sign, rot_sign, min_duty, accel, spin_ms, macro
    p = line.split(",")
    c = p[0]
    last_cmd = time.monotonic()
    if c == "D" and not macro and len(p) >= 5:
        t = mix(float(p[1]), float(p[2]), float(p[3]), float(p[4]))
        for i in range(4):
            target[i] = t[i]
    elif c == "S":
        stop_all()
    elif c == "W" and not macro and len(p) >= 3:
        for i in range(4):
            target[i] = 0.0
        target[int(p[1])] = float(p[2])
    elif c == "M" and len(p) >= 3:
        start_macro(int(p[1]), int(p[2]))
    elif c == "K" and len(p) >= 10:
        for i in range(4):
            invert[i] = -1 if p[1 + i] == "1" else 1
        strafe_sign = -1 if p[5] == "1" else 1
        rot_sign = -1 if p[6] == "1" else 1
        min_duty = clamp(int(p[7]), 0, 60000)
        accel = clamp(float(p[8]), 1.0, 100.0)
        spin_ms = clamp(int(p[9]), 200, 10000)


def poll_uart():
    global rx
    data = uart.read(64)
    if data:
        rx += data
        if len(rx) > 200:
            rx = b""
    while b"\n" in rx:
        line, rx = rx.split(b"\n", 1)
        try:
            handle(line.decode("utf-8", "ignore").strip())
        except Exception:
            pass


stop_all()
last_tel = 0.0
gc_n = 0
while True:
    now = time.monotonic()
    poll_uart()
    run_macro(now)
    if not macro and now - last_cmd > FAILSAFE_S:
        for i in range(4):
            target[i] = 0.0
    for i in range(4):
        d = target[i] - current[i]
        if abs(d) <= accel:
            current[i] = target[i]
        else:
            current[i] += accel if d > 0 else -accel
        drive(i, current[i])
    if now - last_tel > 0.2:
        last_tel = now
        uart.write("T,%d,%d,%d,%d,%d\n" % (current[0], current[1], current[2], current[3], 1 if macro else 0))
    gc_n += 1
    if gc_n > 200:
        gc.collect()
        gc_n = 0
    time.sleep(TICK_S)
