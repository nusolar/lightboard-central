Lightboard — Central (Lightboard Unit)
======================================

Firmware for the lightboard half of Northwestern's solar-car **Lightboard**
system. This board sits near the car battery and drives all high-power
outputs: headlights, brake/turn lights, hazards, horn, and the
battery-fault strobe. It has two distinct input sources:

1. **BLE commands from the steering wheel** (``../peripheral/``), parsed
   in ``ble_data_received``.
2. **Direct GPIO inputs from elsewhere on the car** — currently the
   strobe trigger, eventually the brake pedal, and likely more as the
   car grows.

This is the integration point of the lights system: assume new
"something happens on the car → a light turns on" features land here,
not on the steering-wheel board.

Hardware
--------

- **MCU:** Nordic nRF54L15 (custom PCB; also builds for ``nrf54l15dk``).
- **Outputs:** GPIO lines that gate the high-side switches feeding the
  LED strips, horn, and strobe lamp.
- **Power / programming:** SWD header on the custom PCB, programmed
  from an nRF5340 DK acting as a J-Link.

GPIO map
~~~~~~~~

==================  ==========  =============================  =================================================================
Signal              nRF54L15    Direction                       Notes
==================  ==========  =============================  =================================================================
``LEFT_TURN`` out   P0.01       Output                          Steady or 500 ms blink, depending on ``current_blink``.
``RIGHT_TURN`` out  P1.08       Output                          Steady or 500 ms blink.
``HORN`` out        P0.03       Output                          Steady on/off.
``HEADLIGHT`` out   P0.04       Output                          Steady on/off.
``STROBE_OUT``      P0.00       Output                          Central-generated blink while ``STROBE_IN`` is asserted. **Stub.**
``STROBE_IN``       P0.02       Input, pull-down, edge IRQ      Steady fault assert from external battery-monitor circuit. **Stub.**
==================  ==========  =============================  =================================================================

Pin assignments are confirmed against the as-built central PCB; the
firmware is the source of truth.

Wire protocol (received)
------------------------

One ASCII byte per BLE NUS write from the steering wheel.
**Uppercase = ON / pressed, lowercase = OFF / released.** Unknown bytes
log a warning. ``\r``/``\n`` are ignored.

==========  ============================
Char        Meaning
==========  ============================
``L`` / ``l``   Left turn signal
``R`` / ``r``   Right turn signal
``Z`` / ``z``   Hazard (both blink)
``F`` / ``f``   Headlights
``H`` / ``h``   Horn
==========  ============================

The protocol is steering-wheel inputs only. Brake-pedal and
strobe-trigger inputs are wired directly into this board's GPIO and
never travel over BLE.

Build & flash
-------------

Primary workflow is the **nRF Connect for VS Code** extension — the
Build / Flash buttons handle this app against the
``nrf54l15dk/nrf54l15/cpuapp`` board target.

CLI equivalent::

    west build -b nrf54l15dk/nrf54l15/cpuapp --sysbuild
    west flash

Sysbuild is required (the IPC-radio image lives in ``sysbuild/ipc_radio``).
When multiple J-Links are connected, pick the target in the VS Code
extension's flash dialog or pass ``--dev-id <segger-serial>`` on the CLI.

A stale ``build_1/`` directory is checked into the working tree — ignore
it; only ``build/`` is the live output.

Logs
----

UART/console is intentionally torn out in ``app.overlay``; logging goes
over **SEGGER RTT** only (``CONFIG_LOG_BACKEND_RTT=y``,
``CONFIG_LOG_BACKEND_UART=n``). The custom PCB doesn't expose a serial
path. Read with the VS Code "RTT" terminal, ``JLinkRTTViewer``, or
``nrfutil device rtt``.

BLE / pairing
-------------

- BLE Central. Scans for the steering wheel's NUS service UUID plus any
  bonded-address filters, and reconnects automatically on disconnect.
- Pairing is **Just Works** — neither side has a passkey display or
  input. First connection bonds automatically; bond persists in flash
  via the Settings subsystem and is reloaded on boot.
- Vestigial passkey scaffolding (``KEY_PASSKEY_ACCEPT/REJECT``,
  ``conn_auth_callbacks``, etc.) is still present in ``src/main.c`` from
  the Nordic sample but is unreachable in practice — Just Works is the
  actual mode. Safe to delete when next touching that area.

DK-as-central bench history
---------------------------

Before the central PCB arrived, an **nRF5340 DK flashed with this
image** acted as a stand-in for the lightboard. The DK's four on-board
LEDs mirrored what the GPIO outputs would do (see ``update_leds()`` —
all call sites are now commented out). The custom central PCB has no
LEDs, so ``update_leds()`` would be a no-op there even if reactivated;
keep it for future DK-based debugging only.

Provenance
----------

Started from the Nordic ``central_uart`` sample (NCS 3.1.0). The
vestigial ``sample.yaml`` and Nordic copyright header in ``src/main.c``
are leftovers from that origin and don't reflect current behavior.
