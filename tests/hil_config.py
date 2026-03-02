# Hardware-in-the-loop test configuration
# Adjust to match your physical wiring.

# --- Saleae Logic channel assignments ---
# Connect Logic channels to Pico GPIO pins as listed below.
STEP_CHANNEL = 0    # Saleae ch 0  →  Pico GPIO 13 (STEP_PIN)
DIR_CHANNEL = 1     # Saleae ch 1  →  Pico GPIO 14 (DIR_PIN)
STEP_PIN = 13       # Pico GPIO for step signal

# --- Saleae capture settings ---
DIGITAL_SAMPLE_RATE = 12_500_000   # 12.5 MHz; handles step pulses up to ~1 MHz
LOGIC2_PORT = 10430                 # Logic 2 → Settings → Automation

# --- Pico connection ---
# 'u0' selects the first USB device; use explicit path if multiple devices present.
PICO_PORT = '/dev/cu.usbmodem314201'                    # or e.g. '/dev/cu.usbmodem314201'
