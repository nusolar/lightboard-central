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

/**
 * @brief Recompute the active blink mode from the latched command state.
 *
 * Hazards have priority over directional turn requests. The directional
 * request remains latched while hazards are active so that the selected turn
 * signal can resume automatically when hazards are released.
 *
 * @param None.
 *
 * @return None.
 */
static void refresh_blink_state(void)
{
    if (hazard_on) {
        current_blink = BLINK_HAZARD;
    } else {
        current_blink = requested_turn;
    }
}

/**
 * @brief Write the current steering-wheel command state to the vehicle outputs.
 *
 * This routine is the single point of truth for the turn signal, horn, and
 * headlight output pins. The strobe output is intentionally excluded because it
 * is driven by the battery-monitor fault input and by @ref strobe_handler.
 *
 * Turn outputs assert only when an active blink mode is selected and the caller
 * supplies a high blink phase. Horn and headlight outputs are steady-state
 * latches and are written on every call so command changes take effect
 * immediately.
 *
 * @param blink_toggle_state Current blink phase. Pass true to allow the active
 *                           turn signal or hazard outputs to turn on; pass
 *                           false to force them off for this phase.
 *
 * @return None.
 */
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

/**
 * @brief Delayable work handler that generates the turn-signal blink cadence.
 *
 * The handler toggles an internal phase bit every @ref BLINK_PERIOD_MS and
 * writes the outputs through @ref update_car_outputs. It reschedules itself
 * while any turn signal mode is active. When blinking is no longer active, it
 * performs one final output update with the blink phase forced low.
 *
 * @param work Pointer to the Zephyr work item that invoked the handler. The
 *             pointer is unused because all required state is held globally.
 *
 * @return None.
 */
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

/**
 * @brief Handle a received Nordic UART Service notification from the peripheral.
 *
 * The steering-wheel peripheral sends a one-byte ASCII command. Uppercase
 * values represent pressed/on states and lowercase values represent
 * released/off states. This callback decodes the first byte, updates the local
 * output latches, refreshes the derived blink state, and starts or stops the
 * blink work item as needed.
 *
 * @param nus  NUS client instance that received the notification. The value is
 *             unused because this module has only one client instance.
 * @param data Pointer to the received notification payload.
 * @param len  Number of bytes available at @p data.
 *
 * @return BT_GATT_ITER_CONTINUE so the notification subscription remains
 *         active.
 */
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

/**
 * @brief Complete Nordic UART Service discovery on a connected peer.
 *
 * Assigns the discovered handles to the global NUS client, subscribes to the
 * receive characteristic, and releases the discovery manager data buffer. After
 * this succeeds, @ref ble_data_received is called for each NUS notification.
 *
 * @param dm      Discovery manager object containing the discovered handles.
 * @param context Callback context supplied to bt_gatt_dm_start(); expected to
 *                point to the NUS client instance.
 *
 * @return None.
 */
static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
    struct bt_nus_client *nus = context;

    bt_nus_handles_assign(dm, nus);
    bt_nus_subscribe_receive(nus);

    bt_gatt_dm_data_release(dm);
}

/**
 * @brief Handle a GATT discovery error.
 *
 * The implementation intentionally performs no recovery in this callback. If
 * discovery fails, the connection is expected to time out or disconnect, and the
 * disconnect path will re-arm scanning.
 *
 * @param conn    Connection on which discovery failed.
 * @param err     Zephyr error code reported by the discovery manager.
 * @param context User context supplied to bt_gatt_dm_start().
 *
 * @return None.
 */
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

/**
 * @brief Start GATT discovery for the Nordic UART Service on the active peer.
 *
 * Discovery is only started if @p conn matches @ref default_conn. This protects
 * against stale callbacks from an older connection attempt after a reconnect.
 *
 * @param conn Bluetooth connection to inspect for the NUS service.
 *
 * @return None.
 */
static void gatt_discover(struct bt_conn *conn)
{
    /* Defends against stale callbacks from a previous connection attempt. */
    if (conn != default_conn) {
        return;
    }

    (void)bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb, &nus_client);
}

/**
 * @brief Handle completion of a Bluetooth connection attempt.
 *
 * On connection failure, scanning is scheduled again through @ref scan_work.
 * On success, the connection is retained as @ref default_conn, link security is
 * requested, and scanning is stopped. If the security request fails immediately,
 * GATT discovery is started without waiting for the security callback.
 *
 * @param conn     Connection object supplied by the Bluetooth stack.
 * @param conn_err Zero on success, otherwise the Bluetooth connection error.
 *
 * @return None.
 */
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

/**
 * @brief Handle disconnection from the steering-wheel peripheral.
 *
 * If the disconnected peer is the active default connection, this function
 * releases the retained connection reference, clears @ref default_conn, and
 * schedules scanning so the central can reconnect when the peripheral
 * advertises again. Stored bonding data is left intact.
 *
 * @param conn   Connection object that disconnected.
 * @param reason Bluetooth HCI disconnect reason. Unused by this handler.
 *
 * @return None.
 */
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

/**
 * @brief Handle the result of a Bluetooth security level change.
 *
 * This project uses the callback as the gate to begin GATT discovery after the
 * stack has completed or rejected the security request. Discovery is attempted
 * regardless of @p err so the link can still operate when security setup fails.
 *
 * @param conn  Connection whose security state changed.
 * @param level Current security level reported by the stack.
 * @param err   Security error value; zero means the level change succeeded.
 *
 * @return None.
 */
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

/**
 * @brief Initialize the Nordic UART Service client instance.
 *
 * Registers the NUS client callbacks used by the central. Only the receive
 * callback is configured because this central consumes steering-wheel commands
 * and does not send application payloads back to the peripheral.
 *
 * @param None.
 *
 * @return 0 on success, or a negative Zephyr error code from bt_nus_client_init().
 */
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

/**
 * @brief Start active scanning for peripherals advertising the NUS UUID.
 *
 * Any existing scan is stopped, scan filters are reset, the Nordic UART Service
 * UUID filter is installed, and active scanning is started. Active scanning is
 * required because the peripheral exposes the NUS UUID in its scan-response
 * data.
 *
 * @param None.
 *
 * @return 0 on success, or a negative Zephyr error code from the scan API.
 */
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

/**
 * @brief Workqueue trampoline used to restart scanning outside Bluetooth callbacks.
 *
 * Bluetooth connection callbacks submit this work item when scanning needs to
 * be restarted. Running @ref scan_start from the system workqueue keeps scan
 * setup out of timing-sensitive Bluetooth callback context.
 *
 * @param item Pointer to the submitted work item. Unused by this handler.
 *
 * @return None.
 */
static void scan_work_handler(struct k_work *item)
{
    ARG_UNUSED(item);

    (void)scan_start();
}

/**
 * @brief Initialize the Bluetooth scan helper module.
 *
 * Configures scanning so the Bluetooth stack automatically initiates a
 * connection when a configured scan filter matches. Also initializes the work
 * item used to restart scanning after disconnects or failed connection attempts.
 *
 * @param None.
 *
 * @return None.
 */
static void scan_init(void)
{
    struct bt_scan_init_param scan_init = {
        .connect_if_match = true,
    };

    bt_scan_init(&scan_init);

    k_work_init(&scan_work, scan_work_handler);
}

/**
 * @brief Drive the strobe output while the battery-monitor fault input is active.
 *
 * The handler reads @ref PIN_STROBE_IN after the debounce delay scheduled by
 * @ref port0_changed. When the input is high, it toggles @ref PIN_STROBE_OUT
 * and reschedules itself at @ref STROBE_PERIOD_MS. When the input is low, it
 * clears the output and stops rescheduling.
 *
 * @param work Pointer to the delayable work item that invoked the handler. The
 *             pointer is unused because strobe state is held locally and in
 *             GPIO.
 *
 * @return None.
 */
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

/**
 * @brief GPIO port 0 interrupt callback for the strobe fault input.
 *
 * The ISR-level callback does not read the input directly. Instead, it schedules
 * @ref strobe_handler 30 ms later so mechanical or electrical edge noise can
 * settle before the strobe state is evaluated.
 *
 * @param dev  GPIO device that raised the callback.
 * @param cb   Callback structure registered for the port.
 * @param pins Bit mask of pins that triggered the callback.
 *
 * @return None.
 */
void port0_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    if (pins & BIT(PIN_STROBE_IN)) {
        k_work_reschedule(&strobe_work, K_MSEC(30));
    }
}

/**
 * @brief Configure all vehicle-side GPIO inputs, outputs, callbacks, and work.
 *
 * Sets the battery-monitor strobe input as a pulled-down edge interrupt and
 * configures the lighting, horn, and strobe outputs as inactive outputs. The
 * function also registers the GPIO callback, initializes the strobe work item,
 * and manually starts strobing if the fault input is already asserted at boot.
 *
 * @param None.
 *
 * @return 0 on success, or -ENODEV if either GPIO controller is unavailable.
 */
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

/**
 * @brief Application entry point for the lightboard central.
 *
 * Initializes Bluetooth, loads persisted settings and bonds when enabled,
 * initializes the blink work item, configures the NUS client, starts filtered
 * scanning for the steering-wheel peripheral, and configures the vehicle-side
 * GPIO outputs. After initialization, the thread sleeps forever because all
 * runtime behavior is handled by Bluetooth callbacks, GPIO callbacks, and
 * workqueue handlers.
 *
 * @param None.
 *
 * @return 0 is not expected during normal operation. Returns a negative Zephyr
 *         error code if initialization fails before the main sleep loop starts.
 */
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
