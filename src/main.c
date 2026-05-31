/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Lightboard central. Connects to the steering-wheel peripheral over BLE
 *        NUS, parses 1-byte commands, and drives the car's lighting GPIOs.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/settings/settings.h>

static struct k_work scan_work;
static struct bt_conn *default_conn;
static struct bt_nus_client nus_client;

static const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

/* Pin numbers are port-relative: PIN_RIGHT_TURN = 8 is P1.08 because it's used
 * against gpio1; everything else is on gpio0. */
#define PIN_STROBE_OUT 0  // P0.00
#define PIN_LEFT_TURN  1  // P0.01
#define PIN_STROBE_IN  2  // P0.02
#define PIN_HORN       3  // P0.03
#define PIN_HEADLIGHT  4  // P0.04
#define PIN_RIGHT_TURN 8  // P1.08

static struct gpio_callback gpio0_cb;

#define STROBE_PERIOD_MS 250
#define BLINK_PERIOD_MS 500
static struct k_work_delayable strobe_work;

/* Blink output state. Hazards are modeled explicitly because they override the
 * remembered directional request rather than being treated as LEFT | RIGHT. */
enum blink_state_t {
    BLINK_NONE,
    BLINK_LEFT,
    BLINK_RIGHT,
    BLINK_HAZARD
};

/* volatile: blink_handler runs on the system workqueue, ble_data_received runs
 * on the Bluetooth RX thread — both touch current_blink concurrently. */
static volatile enum blink_state_t current_blink = BLINK_NONE;
static struct k_work_delayable blink_work;

static bool headlight_on = false;
static bool horn_on = false;
static volatile enum blink_state_t requested_turn = BLINK_NONE;
static volatile bool hazard_on = false;

/* Hazards override directional requests, but the requested turn direction is
 * kept latched so it can resume automatically once hazards are released. */
static void refresh_blink_state(void)
{
    if (hazard_on) {
        current_blink = BLINK_HAZARD;
    } else {
        current_blink = requested_turn;
    }
}

/* Single point of truth for every output pin driven by steering-wheel commands.
 * Strobe is independent (own input source, own cadence, own writer in
 * strobe_handler) and is intentionally not touched here. Called on each blink-
 * handler tick (with the current toggle phase) and whenever a BLE command shifts
 * state. Turn signals only assert when current_blink != NONE *and* the toggle is
 * high, so the same routine handles steady state and the blink phase in one shot. */
static void update_car_outputs(bool blink_toggle_state) {
    bool horn_state = horn_on;
    bool headlight_state = headlight_on;
    bool left_state = false;
    bool right_state = false;

    if (current_blink != BLINK_NONE && blink_toggle_state) {
        if (current_blink == BLINK_LEFT) {
            left_state = true;
        } else if (current_blink == BLINK_RIGHT) {
            right_state = true;
        } else {
            left_state = true;
            right_state = true;
        }
    }

    gpio_pin_set(gpio0, PIN_LEFT_TURN, left_state ? 1 : 0);
    gpio_pin_set(gpio1, PIN_RIGHT_TURN, right_state ? 1 : 0);
    gpio_pin_set(gpio0, PIN_HORN, horn_state ? 1 : 0);
    gpio_pin_set(gpio0, PIN_HEADLIGHT, headlight_state ? 1 : 0);
}

/* 500 ms-toggle work item. Re-arms itself while a turn signal is active;
 * the final tick clears outputs and the work doesn't re-arm. */
static void blink_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    static bool toggle_state = false;
    toggle_state = !toggle_state;

    update_car_outputs(toggle_state);

    if (current_blink != BLINK_NONE) {
        k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
    } else {
        update_car_outputs(false);
    }
}

/* NUS notify callback — fires once per byte the peripheral sends. Decodes the
 * command, updates state flags, then re-arms or cancels blink_work depending
 * on whether any turn signal is now active. Returns BT_GATT_ITER_CONTINUE to
 * keep the subscription alive. */
static uint8_t ble_data_received(struct bt_nus_client *nus,
                        const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(nus);

    if (len == 0) {
        return BT_GATT_ITER_CONTINUE;
    }
    char cmd = data[0];

    /* 1-byte ASCII protocol from the steering wheel. Uppercase = ON / pressed,
     * lowercase = OFF / released. Left/right update the remembered directional
     * request; lowercase only clears the matching direction so a stale release
     * can't cancel a newer opposite turn. Hazards live on a separate latch
     * that overrides the directional request while active. */
    switch (cmd) {
        case 'L': requested_turn = BLINK_LEFT; break;
        case 'l': if (requested_turn == BLINK_LEFT) requested_turn = BLINK_NONE; break;

        case 'R': requested_turn = BLINK_RIGHT; break;
        case 'r': if (requested_turn == BLINK_RIGHT) requested_turn = BLINK_NONE; break;

        case 'Z': hazard_on = true; break;
        case 'z': hazard_on = false; break;

        case 'F': headlight_on = true; break;
        case 'f': headlight_on = false; break;

        case 'H': horn_on = true; break;
        case 'h': horn_on = false; break;

        default: break;
    }

    refresh_blink_state();

    if (current_blink != BLINK_NONE) {
        k_work_reschedule(&blink_work, K_NO_WAIT);
    } else {
        k_work_cancel_delayable(&blink_work);
        update_car_outputs(false);
    }

    return BT_GATT_ITER_CONTINUE;
}

/* GATT discovery succeeded. Bind the NUS client to the discovered handles and
 * subscribe for receive notifications — from here, ble_data_received fires on
 * every byte the peripheral sends. */
static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
    struct bt_nus_client *nus = context;

    bt_nus_handles_assign(dm, nus);
    bt_nus_subscribe_receive(nus);

    bt_gatt_dm_data_release(dm);
}

/* GATT discovery failed. The peer will likely time out and disconnect, after
 * which scan_work re-arms scanning. */
static void discovery_error(struct bt_conn *conn, int err, void *context)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(err);
    ARG_UNUSED(context);
}

struct bt_gatt_dm_cb discovery_cb = {
    .completed   = discovery_complete,
    .error_found = discovery_error,
};

/* Kick off NUS GATT discovery. Called from security_changed (success path)
 * and from connected() if security setup failed. */
static void gatt_discover(struct bt_conn *conn)
{
    /* Defends against stale callbacks from a previous connection attempt. */
    if (conn != default_conn) {
        return;
    }

    (void)bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb, &nus_client);
}

/* Connection callback. On failure, drop default_conn and re-arm scanning. On
 * success, raise the link to encrypted+bonded — security_changed will then
 * trigger GATT discovery — and stop active scanning while we have a peer. */
static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    int err;

    if (conn_err) {
        (void)k_work_submit(&scan_work);
        return;
    }

    default_conn = bt_conn_ref(conn);

    /* Request encryption + (re)bond. On success, security_changed kicks off
     * GATT discovery; on failure we fall through and discover unencrypted. */
    err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        gatt_discover(conn);
    }

    (void)bt_scan_stop();
}

/* Drop the conn ref, clear default_conn, and re-arm scanning. The bond persists
 * across resets, so the rescan auto-reconnects the same peer once it advertises. */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(reason);

    if (default_conn != conn) {
        return;
    }

    bt_conn_unref(default_conn);
    default_conn = NULL;

    (void)k_work_submit(&scan_work);
}

/* SMP outcome (Just Works pairing for us). On success this is what triggers
 * GATT discovery; on failure we still call gatt_discover so the link can keep
 * progressing without security. */
static void security_changed(struct bt_conn *conn, bt_security_t level,
                 enum bt_security_err err)
{
    ARG_UNUSED(level);
    ARG_UNUSED(err);

    gatt_discover(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed
};

/* Initialize the NUS client. Only the receive callback is wired up; the
 * central never sends to the peripheral, so .sent is omitted. */
static int nus_client_init(void)
{
    int err;
    struct bt_nus_client_init_param init = {
        .cb = {
            .received = ble_data_received,
        }
    };

    err = bt_nus_client_init(&nus_client, &init);
    if (err) {
        return err;
    }

    return 0;
}

/* (Re)arm scanning with a NUS UUID filter so the peripheral's scan-response
 * (which carries the NUS UUID) gets matched. Called at boot and from
 * scan_work_handler after a disconnect. */
static int scan_start(void)
{
    int err;

    err = bt_scan_stop();
    if (err && err != -EALREADY) {
        return err;
    }

    bt_scan_filter_remove_all();

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
    if (err) {
        return err;
    }

    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) {
        return err;
    }

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        return err;
    }

    return 0;
}

/* Workqueue trampoline so disconnected() and connection-failure paths can
 * re-arm the scanner outside the BT thread. */
static void scan_work_handler(struct k_work *item)
{
    ARG_UNUSED(item);

    (void)scan_start();
}

/* One-time scan-module setup. connect_if_match = true makes the stack auto-
 * initiate a connection as soon as any scan filter matches. */
static void scan_init(void)
{
    struct bt_scan_init_param scan_init = {
        .connect_if_match = true,
    };

    bt_scan_init(&scan_init);

    k_work_init(&scan_work, scan_work_handler);
}

/* While STROBE_IN is asserted (steady fault from the battery monitor), self-rearm
 * to toggle STROBE_OUT at STROBE_PERIOD_MS. When STROBE_IN goes low, drop the
 * output and stop rescheduling. This handler also serves as the post-IRQ debounce:
 * port0_changed schedules us 30 ms after the edge, and k_work_reschedule's
 * cancel-and-replace semantics let a falling edge cleanly preempt an in-flight
 * blink tick. */
static void strobe_handler(struct k_work *work) {
    ARG_UNUSED(work);
    static bool strobe_on = false;
    bool fault_active = gpio_pin_get(gpio0, PIN_STROBE_IN) == 1;

    if (fault_active) {
        strobe_on = !strobe_on;
        gpio_pin_set(gpio0, PIN_STROBE_OUT, strobe_on ? 1 : 0);
        k_work_reschedule(&strobe_work, K_MSEC(STROBE_PERIOD_MS));
    } else {
        strobe_on = false;
        gpio_pin_set(gpio0, PIN_STROBE_OUT, 0);
    }
}

/* GPIO ISR for gpio0 — defer the read by 30 ms to debounce. */
void port0_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    if (pins & BIT(PIN_STROBE_IN)) {
        k_work_reschedule(&strobe_work, K_MSEC(30));
    }
}

/* GPIO setup at boot: STROBE_IN as edge-IRQ input, every other lighting pin as
 * inactive output. Returns -ENODEV if either GPIO controller isn't ready. */
static int configure_car_outputs(void) {
    if (!device_is_ready(gpio0) || !device_is_ready(gpio1)) {
        return -ENODEV;
    }

    gpio_pin_configure(gpio0, PIN_STROBE_IN, GPIO_INPUT | GPIO_PULL_DOWN);
    gpio_pin_interrupt_configure(gpio0, PIN_STROBE_IN, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&gpio0_cb, port0_changed, BIT(PIN_STROBE_IN));
    gpio_add_callback(gpio0, &gpio0_cb);

    gpio_pin_configure(gpio0, PIN_STROBE_OUT, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_LEFT_TURN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_HORN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_HEADLIGHT, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio1, PIN_RIGHT_TURN, GPIO_OUTPUT_INACTIVE);

    k_work_init_delayable(&strobe_work, strobe_handler);

    /* If the fault line is already asserted at boot, we missed the rising edge —
     * kick off the strobe loop manually so it doesn't sit idle until the next
     * low→high cycle. */
    if (gpio_pin_get(gpio0, PIN_STROBE_IN) == 1) {
        k_work_reschedule(&strobe_work, K_NO_WAIT);
    }

    return 0;
}

/* Boot order: BLE → settings (loads bonds) → blink work → NUS client → scanner
 * → GPIO. The main loop is idle; everything happens in callbacks/work items. */
int main(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        return err;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    k_work_init_delayable(&blink_work, blink_handler);

    err = nus_client_init();
    if (err != 0) {
        return err;
    }

    scan_init();
    err = scan_start();
    if (err) {
        return err;
    }

    err = configure_car_outputs();
    if (err) {
        return err;
    }

    for (;;) {
        k_sleep(K_FOREVER);
    }
}
