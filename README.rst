Lightboard — Central (Lightboard Unit)
======================================

Firmware for the lightboard half of Northwestern's solar-car **Lightboard**
system. This board sits near the car battery and drives all high-power
outputs: headlights, turn lights, hazards, horn, and the battery-fault
strobe. It has two distinct input sources:

1. **BLE NUS notifications from the steering wheel** (``../peripheral/``),
   parsed in ``ble_data_received``.
2. **Direct GPIO inputs from elsewhere on the car** -- currently the
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

One ASCII byte per BLE NUS notification from the steering wheel.
**Uppercase = ON / pressed, lowercase = OFF / released.** Unknown bytes
are ignored.

==========  ============================
Char        Meaning
==========  ============================
``L`` / ``l``   Left turn signal
``R`` / ``r``   Right turn signal
``Z`` / ``z``   Hazard (both blink)
``F`` / ``f``   Headlights
``H`` / ``h``   Horn
==========  ============================

The protocol is steering-wheel inputs only. Strobe-trigger input is wired
directly into this board's GPIO and never travels over BLE.

Runtime behavior
----------------

- ``ble_data_received`` updates the latched horn/headlight/turn/hazard
  state from each one-byte NUS command.
- ``blink_handler`` runs from a delayable work item and toggles turn or
  hazard outputs every 500 ms while a blink mode is active.
- ``strobe_handler`` is independent of BLE. It debounces ``STROBE_IN`` and
  toggles ``STROBE_OUT`` every 250 ms while the fault line is asserted.
- Hazard mode overrides the requested left/right turn direction, but the
  requested turn is remembered and resumes after hazards are released.

Configuration notes
-------------------

``prj.conf`` keeps the image focused on the custom lightboard hardware:

- UART, console, printk, RTT, and logging are disabled because the custom
  PCB has no runtime log transport.
- The firmware is a BLE Central/GATT Client with the NUS client, scan
  helper, UUID filter support, and GATT discovery manager enabled.
- Bonding is stored through the Settings subsystem so the board can
  reconnect after power cycles.
- nRF54L15 GPIOTE instances are enabled for the ``STROBE_IN`` edge
  interrupt, and the internal 32 kHz RC source is selected because the
  custom PCB has no external low-frequency crystal.

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

- BLE Central. Scans for the steering wheel's NUS service UUID and
  reconnects automatically on disconnect.
- Pairing is **Just Works** — neither side has a passkey display or
  input. First connection bonds automatically; bond persists in flash
  via the Settings subsystem and is reloaded on boot.

Provenance
----------

Started from the Nordic ``central_uart`` sample (NCS 3.1.0). The Nordic
copyright header in ``src/main.c`` is a leftover from that origin and
doesn't reflect current behavior.
