# Pico + TMC2209 UART + FastAccelStepper

Configure the TMC2209 over UART (current, microsteps), move it with STEP/DIR via FastAccelStepper.

## Hardware wiring

| TMC2209 | Pico / supply | Notes |
|---------|---------------|-------|
| STEP | **GP3** | |
| DIR | **GP2** | |
| EN | **GND** | Active low. Or wire to a GPIO and set `ENABLE_PIN` in `src/main.cpp` |
| PDN_UART | see UART below | Required for this project |
| VIO / VCC | Pico **3.3V** | Logic |
| GND | Pico GND + PSU GND | Common ground |
| VM / VS | Motor PSU (e.g. 12–24 V) | Not from Pico USB |
| A1/A2/B1/B2 | Motor coils | |

### UART half-duplex (required)

TMC2209 uses one UART wire (`PDN_UART`):

```
Pico GP4 (TX) ----[ 1 kΩ ]----+---- TMC PDN_UART
                              |
Pico GP5 (RX) ----------------+
```

- **1 kΩ** between TX and the shared node
- RX tied **directly** to that node
- UART address jumpers MS1/MS2 both low/open → address `0b00` (default in code)

Leave any onboard “UART” jumper in the position your board docs call for UART mode.

## Build / upload (PlatformIO)

Pick your board env in `platformio.ini`:

```bash
pio run -e rpipico2w -t upload
pio device monitor -e rpipico2w
```

Other envs: `rpipico`, `rpipicow`, `rpipico2w`.

Monitor baud: **115200**.

## First test

1. Power **VM** and **VIO**, common GND, EN→GND, UART resistor network in place.
2. Open serial monitor, you should see UART test `OK`.
3. If UART `FAIL`, fix wiring/address before worrying about motion.
4. Type:
   - `t` — UART test again
   - `c 300` — set ~300 mA RMS (raise slowly for your NEMA 8)
   - `+` / `-` — nudge
   - `i` — status
   - `h` — help

## Serial commands

| Cmd | Action |
|-----|--------|
| `+` / `-` | nudge |
| `n 16` | step size |
| `s 200` | speed Hz |
| `a 400` | acceleration |
| `g 100` | goto position |
| `c 350` | RMS current mA (UART) |
| `m 16` | microsteps |
| `t` | UART test |
| `i` | status |
| `x` | stop |

## Notes

- Current is set by **UART** (`rms_current`), not the pot.
- Start low current on a NEMA 8; watch heat.
- StallGuard / DIAG can be added later once UART + motion work.
- FastAccelStepper on Pico needs FreeRTOS (`-D__FREERTOS=1`) and Max Gerhardt’s platform (already in `platformio.ini`).
