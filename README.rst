Lightboard — Central (Lightboard Unit)
======================================

Firmware for the lightboard half of Northwestern's solar-car **Lightboard**
system. This board sits near the car battery and drives all high-power
outputs: headlights, brake/turn lights, hazards, horn, and the
battery-fault strobe. It has two distinct input sources:

1. **BLE commands from the steering wheel** (``../peripheral/``), parsed
   in ``ble_data_received``.
2. **Direct GPIO inputs from elsewhere on the car** — currently the
   strobe trigger, with likely additions as the car grows.

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
``STROBE_OUT``      P0.00       Output                          Central-generated blink while ``STROBE_IN`` is asserted.
``STROBE_IN``       P0.02       Input, pull-down, edge IRQ      Steady fault assert from external battery-monitor circuit.
==================  ==========  =============================  =================================================================

Pin assignments are confirmed against the as-built central PCB; the
firmware is the source of truth.

Wire protocol (received)
------------------------

One ASCII byte per BLE NUS write from the steering wheel.
**Uppercase = ON / pressed, lowercase = OFF / released.** Unknown bytes
are ignored. ``\r``/``\n`` are ignored.

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

Logging
-------

Runtime logging is disabled in the current configuration. UART/console is
still removed in ``app.overlay`` and there is no serial log path on the
custom PCB.

BLE / pairing
-------------

- BLE Central. Scans for the steering wheel's NUS service UUID plus any
  bonded-address filters, and reconnects automatically on disconnect.
- Pairing is **Just Works** — neither side has a passkey display or
  input. First connection bonds automatically; bond persists in flash
  via the Settings subsystem and is reloaded on boot.

Provenance
----------

Started from the Nordic ``central_uart`` sample (NCS 3.1.0). The Nordic
copyright header in ``src/main.c`` is a leftover from that origin and
doesn't reflect current behavior.
