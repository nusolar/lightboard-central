/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Lightboard central. Connects to the steering-wheel peripheral over BLE
 *        NUS, parses 1-byte commands, and drives the car's lighting GPIOs.
 *        See ../CLAUDE.md for the cross-app architecture notes.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/settings/settings.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME central
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

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

static struct k_work_delayable strobe_work;

/* Single-state turn-signal machine. Hazard is its own state, not LEFT | RIGHT,
 * so a hazard command cleanly overrides whatever direction was previously set. */
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

/* Single point of truth for every output pin. Called on each blink-handler tick
 * (with the current toggle phase) and whenever a BLE command shifts state.
 * Turn signals only assert when current_blink != NONE *and* the toggle is high,
 * so the same routine handles steady state and the blink phase in one shot. */
static void update_car_outputs(bool blink_toggle_state) {
    bool horn_state = horn_on;
    bool headlight_state = headlight_on;
    bool left_state = false;
    bool right_state = false;
    bool strobe_state = false;

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
    gpio_pin_set(gpio0, PIN_STROBE_OUT, strobe_state ? 1 : 0);
}

/* 500 ms-toggle work item. Re-arms itself while a turn signal is active;
 * the final tick clears outputs and the work doesn't re-arm. */
static void blink_handler(struct k_work *work)
{
    static bool toggle_state = false;
    toggle_state = !toggle_state;

    update_car_outputs(toggle_state);

    if (current_blink != BLINK_NONE) {
        k_work_reschedule(&blink_work, K_MSEC(500));
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

    LOG_INF("Car Command Received: %c", cmd);

    /* 1-byte ASCII protocol from the steering wheel. Uppercase = ON / pressed,
     * lowercase = OFF / released. Lowercase only clears its own direction so a
     * stale 'l' release doesn't cancel a newer right turn (and vice versa). */
    switch (cmd) {
        case 'L': current_blink = BLINK_LEFT; break;
        case 'l': if (current_blink == BLINK_LEFT) current_blink = BLINK_NONE; break;

        case 'R': current_blink = BLINK_RIGHT; break;
        case 'r': if (current_blink == BLINK_RIGHT) current_blink = BLINK_NONE; break;

        case 'Z': current_blink = BLINK_HAZARD; break;
        case 'z': if (current_blink == BLINK_HAZARD) current_blink = BLINK_NONE; break;

        case 'F': headlight_on = true; break;
        case 'f': headlight_on = false; break;

        case 'H': horn_on = true; break;
        case 'h': horn_on = false; break;

        default: LOG_INF("Unknown command received"); break;
    }

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
    LOG_INF("Service discovery completed");

    bt_gatt_dm_data_print(dm);

    bt_nus_handles_assign(dm, nus);
    bt_nus_subscribe_receive(nus);

    bt_gatt_dm_data_release(dm);
}

/* GATT discovery failed. The peer will likely time out and disconnect, after
 * which scan_work re-arms scanning. */
static void discovery_error(struct bt_conn *conn, int err, void *context)
{
    LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
    .completed   = discovery_complete,
    .error_found = discovery_error,
};

/* Kick off NUS GATT discovery. Called from security_changed (success path)
 * and from connected() if security setup failed. */
static void gatt_discover(struct bt_conn *conn)
{
    int err;

    /* Defends against stale callbacks from a previous connection attempt. */
    if (conn != default_conn) {
        return;
    }

    err = bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb, &nus_client);
    if (err) {
        LOG_ERR("could not start the discovery procedure, error code: %d", err);
    }
}

/* Connection callback. On failure, drop default_conn and re-arm scanning. On
 * success, raise the link to encrypted+bonded — security_changed will then
 * trigger GATT discovery — and stop active scanning while we have a peer. */
static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        LOG_INF("Failed to connect to %s, 0x%02x %s", addr, conn_err,
            bt_hci_err_to_str(conn_err));

        if (default_conn == conn) {
            bt_conn_unref(default_conn);
            default_conn = NULL;

            (void)k_work_submit(&scan_work);
        }

        return;
    }

    LOG_INF("Connected: %s", addr);

    /* Request encryption + (re)bond. On success, security_changed kicks off
     * GATT discovery; on failure we fall through and discover unencrypted. */
    err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        LOG_WRN("Failed to set security: %d", err);
        gatt_discover(conn);
    }

    err = bt_scan_stop();
    if ((!err) && (err != -EALREADY)) {
        LOG_ERR("Stop LE scan failed (err %d)", err);
    }
}

/* Drop the conn ref, clear default_conn, and re-arm scanning. Bonded peers
 * reconnect quickly because scan_start re-adds their address filter at boot
 * (and the bond persists across resets). */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

    if (default_conn != conn) {
        return;
    }

    bt_conn_unref(default_conn);
    default_conn = NULL;

    (void)k_work_submit(&scan_work);
}

/* SMP outcome (Just Works pairing for us). On success this is what triggers
 * GATT discovery; on failure we still call gatt_discover so the next failure
 * surfaces in the log instead of going silent. */
static void security_changed(struct bt_conn *conn, bt_security_t level,
                 enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_INF("Security changed: %s level %u", addr, level);
    } else {
        LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
            bt_security_err_to_str(err));
    }

    gatt_discover(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed
};

/* Diagnostic: log every scan response that matched a filter. Useful during
 * bring-up to confirm the central is actually seeing the peripheral. */
static void scan_filter_match(struct bt_scan_device_info *device_info,
                  struct bt_scan_filter_match *filter_match,
                  bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable);
}

/* Auto-connect started — capture the conn ref now so connected() and
 * disconnected() can reconcile against default_conn. */
static void scan_connecting(struct bt_scan_device_info *device_info,
                struct bt_conn *conn)
{
    default_conn = bt_conn_ref(conn);
}

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
        LOG_ERR("NUS Client initialization failed (err %d)", err);
        return err;
    }

    LOG_INF("NUS Client module initialized");
    return err;
}

/* Macro slots: filter_match, filter_no_match, connecting_error, connecting.
 * We only use filter_match (diagnostic) and connecting (capture conn ref). */
BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, NULL, scan_connecting);

/* For each bonded peer, add its address to the scan filter alongside the NUS
 * UUID filter — lets us reconnect specifically to a paired peripheral instead
 * of any nearby NUS device. Skip bonds we're already connected to. */
static void try_add_address_filter(const struct bt_bond_info *info, void *user_data)
{
    int err;
    char addr[BT_ADDR_LE_STR_LEN];
    uint8_t *filter_mode = user_data;

    bt_addr_le_to_str(&info->addr, addr, sizeof(addr));

    struct bt_conn *conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &info->addr);

    if (conn) {
        bt_conn_unref(conn);
        return;
    }

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, &info->addr);
    if (err) {
        LOG_ERR("Address filter cannot be added (err %d): %s", err, addr);
        return;
    }

    LOG_INF("Address filter added: %s", addr);
    *filter_mode |= BT_SCAN_ADDR_FILTER;
}

/* (Re)arm scanning. Sets up the NUS UUID filter plus address filters for any
 * bonded peers, then enables active scanning so the peripheral's scan-response
 * (which carries the NUS UUID) is visible. Called at boot and from
 * scan_work_handler after a disconnect. */
static int scan_start(void)
{
    int err;
    uint8_t filter_mode = 0;

    err = bt_scan_stop();
    if (err) {
        LOG_ERR("Failed to stop scanning (err %d)", err);
        return err;
    }

    /* Wipe filters from any prior scan_start so we can rebuild from scratch. */
    bt_scan_filter_remove_all();

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
    if (err) {
        LOG_ERR("UUID filter cannot be added (err %d)", err);
        return err;
    }
    filter_mode |= BT_SCAN_UUID_FILTER;

    bt_foreach_bond(BT_ID_DEFAULT, try_add_address_filter, &filter_mode);

    /* false = match-any: connect on UUID match OR bonded-address match (OR, not AND). */
    err = bt_scan_filter_enable(filter_mode, false);
    if (err) {
        LOG_ERR("Filters cannot be turned on (err %d)", err);
        return err;
    }

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return err;
    }

    LOG_INF("Scan started");
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
    bt_scan_cb_register(&scan_cb);

    k_work_init(&scan_work, scan_work_handler);
    LOG_INF("Scan module initialized");
}

/* TODO: while STROBE_IN is asserted (steady fault from the battery monitor),
 * drive STROBE_OUT in a self-generated blink cadence. Currently only logs.
 * BUG: reads PIN_HORN below — should be PIN_STROBE_IN. */
static void strobe_handler(struct k_work *work) {
    bool on = gpio_pin_get(gpio0, PIN_HORN) == 1;
    LOG_INF("Strobe %s", on ? "ON" : "OFF");
}

/* GPIO ISR for gpio0 — defer the read by 30 ms to debounce. */
void port0_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (pins & BIT(PIN_STROBE_IN)) {
        k_work_reschedule(&strobe_work, K_MSEC(30));
    }
}

/* GPIO setup at boot: STROBE_IN as edge-IRQ input, every other lighting pin as
 * inactive output. Logs (but does not fatally fail) if a gpio device isn't
 * ready — the device-readiness check uses `!device_is_ready` so a non-ready
 * device produces err == 1, matching the existing log-only handling. */
static void configure_car_outputs(void) {
    int err;

    err = !device_is_ready(gpio0);
    if (err) {
        LOG_ERR("GPIO port 0 not ready!");
    }

    err = !device_is_ready(gpio1);
    if (err) {
        LOG_ERR("GPIO port 1 not ready!");
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

    LOG_INF("GPIO configured");
}

/* Boot order: BLE → settings (loads bonds) → blink work → NUS client → scanner
 * → GPIO. The main loop is idle; everything happens in callbacks/work items. */
int main(void)
{
    LOG_INF("Starting Bluetooth Central");
    int err;

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }
    LOG_INF("Bluetooth initialized");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    k_work_init_delayable(&blink_work, blink_handler);

    err = nus_client_init();
    if (err != 0) {
        LOG_ERR("nus_client_init failed (err %d)", err);
        return 0;
    }

    scan_init();
    err = scan_start();
    if (err) {
        return 0;
    }

    configure_car_outputs();

    for (;;) {
        k_sleep(K_FOREVER);
    }
}
