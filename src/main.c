/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Nordic UART Service Client sample
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>

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
#include <dk_buttons_and_leds.h>

#define LOG_MODULE_NAME central
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

static struct k_work scan_work;
static struct bt_conn *default_conn;
static struct bt_nus_client nus_client;

static const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

#define PIN_STROBE     0  // P0.00
#define PIN_LEFT_TURN  1  // P0.01
#define PIN_STROBE_IN  2  // P0.02
#define PIN_HORN       3  // P0.03
#define PIN_HEADLIGHT  4  // P0.04
#define PIN_RIGHT_TURN 8  // P1.08

static struct gpio_callback gpio0_cb;

enum blink_state_t {
    BLINK_NONE,
    BLINK_LEFT,
    BLINK_RIGHT,
    BLINK_HAZARD
};

static volatile enum blink_state_t current_blink = BLINK_NONE;
static struct k_work_delayable blink_work;

static bool headlight_on = false;
static bool horn_on = false;

static void update_leds(bool blink_toggle_state) 
{
    uint32_t led_states = 0;

    if (headlight_on) {
        led_states |= (DK_LED1_MSK | DK_LED2_MSK); 
    }
    if (horn_on) {
        led_states |= (DK_LED3_MSK | DK_LED4_MSK);
    }

    uint32_t blink_pins = 0;
    if (current_blink == BLINK_LEFT) {
        blink_pins = (DK_LED1_MSK | DK_LED3_MSK);
    } else if (current_blink == BLINK_RIGHT) {
        blink_pins = (DK_LED2_MSK | DK_LED4_MSK);
    } else if (current_blink == BLINK_HAZARD) {
        blink_pins = DK_ALL_LEDS_MSK;
    }

    if (current_blink != BLINK_NONE) {
        if (blink_toggle_state) {
            led_states |= blink_pins;
        } else {
            led_states &= ~blink_pins;
        }
    }

    dk_set_leds(led_states);

}

static void blink_handler(struct k_work *work)
{
    static bool toggle_state = false;
    toggle_state = !toggle_state;

    // update_car_outputs(toggle_state)
    update_leds(toggle_state);

    if (current_blink != BLINK_NONE) {
        k_work_reschedule(&blink_work, K_MSEC(500));
    } else {
        update_leds(false);
        // update_car_outputs(false);
    }
}

static void ble_data_sent(struct bt_nus_client *nus, uint8_t err,
                    const uint8_t *const data, uint16_t len)
{
    ARG_UNUSED(nus);
    ARG_UNUSED(data);
    ARG_UNUSED(len);

    if (err) {
        LOG_WRN("ATT error code: 0x%02X", err);
    }
}

static uint8_t ble_data_received(struct bt_nus_client *nus,
                        const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(nus);

    if (len == 0) {
        return BT_GATT_ITER_CONTINUE;
    }
    char cmd = data[0];

    if (cmd == '\r' || cmd == '\n') {
        return BT_GATT_ITER_CONTINUE;
    }

    LOG_INF("Car Command Received: %c\n", cmd);

    switch (cmd) {
        case 'L':
            /* Blink left LEDS on dk */
            current_blink = (current_blink == BLINK_LEFT) ? BLINK_NONE : BLINK_LEFT;
            break;
        case 'R':
            /* Blink right LEDS on dk */
            current_blink = (current_blink == BLINK_RIGHT) ? BLINK_NONE : BLINK_RIGHT;
            break;
        case 'Z':
            /* Blink all LEDS on dk */
            current_blink = (current_blink == BLINK_HAZARD) ? BLINK_NONE : BLINK_HAZARD;
            break;
        case 'F':
            /* Turns top LEDS on/off */
            headlight_on = !headlight_on;
            break;
        case 'H':
            /* Turns bottom LEDS on */
            horn_on = true;
            break;
        case 'h':
            horn_on = false;
            break;
        default:
            LOG_INF("Unknown command received");
            break;
    }
    
    if (current_blink != BLINK_NONE) {
        k_work_reschedule(&blink_work, K_NO_WAIT);
    } else {
        k_work_cancel_delayable(&blink_work);
        update_leds(false);
        // update_car_outputs(false);
    }

    return BT_GATT_ITER_CONTINUE;
}

static void discovery_complete(struct bt_gatt_dm *dm,
                   void *context)
{
    struct bt_nus_client *nus = context;
    LOG_INF("Service discovery completed");

    bt_gatt_dm_data_print(dm);

    bt_nus_handles_assign(dm, nus);
    bt_nus_subscribe_receive(nus);

    bt_gatt_dm_data_release(dm);
}

static void discovery_service_not_found(struct bt_conn *conn,
                    void *context)
{
    LOG_INF("Service not found");
}

static void discovery_error(struct bt_conn *conn,
                int err,
                void *context)
{
    LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
    .completed         = discovery_complete,
    .service_not_found = discovery_service_not_found,
    .error_found       = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
    int err;

    if (conn != default_conn) {
        return;
    }

    err = bt_gatt_dm_start(conn,
                   BT_UUID_NUS_SERVICE,
                   &discovery_cb,
                   &nus_client);
    if (err) {
        LOG_ERR("could not start the discovery procedure, error "
            "code: %d", err);
    }
}

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
    if (!err) {
        LOG_INF("MTU exchange done");
    } else {
        LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
    }
}

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

    static struct bt_gatt_exchange_params exchange_params;

    exchange_params.func = exchange_func;
    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_WRN("MTU exchange failed (err %d)", err);
    }

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

static void scan_filter_match(struct bt_scan_device_info *device_info,
                  struct bt_scan_filter_match *filter_match,
                  bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filters matched. Address: %s connectable: %d",
        addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    LOG_WRN("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
                struct bt_conn *conn)
{
    default_conn = bt_conn_ref(conn);
}

static int nus_client_init(void)
{
    int err;
    struct bt_nus_client_init_param init = {
        .cb = {
            .received = ble_data_received,
            .sent = ble_data_sent,
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

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
        scan_connecting_error, scan_connecting);

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

static int scan_start(void)
{
    int err;
    uint8_t filter_mode = 0;

    err = bt_scan_stop();
    if (err) {
        LOG_ERR("Failed to stop scanning (err %d)", err);
        return err;
    }

    bt_scan_filter_remove_all();

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
    if (err) {
        LOG_ERR("UUID filter cannot be added (err %d", err);
        return err;
    }
    filter_mode |= BT_SCAN_UUID_FILTER;

    bt_foreach_bond(BT_ID_DEFAULT, try_add_address_filter, &filter_mode);

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

static void scan_work_handler(struct k_work *item)
{
    ARG_UNUSED(item);

    (void)scan_start();
}

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

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_WRN("Pairing failed conn: %s, reason %d %s", addr, reason,
        bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed
};

void port0_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  if (pins & BIT(PIN_STROBE_IN)) {
    LOG_INF("STROBE HIGH");
    gpio_pin_toggle(gpio0, PIN_STROBE);
  }
}

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
    gpio_pin_interrupt_configure(gpio0, PIN_STROBE_IN, GPIO_INT_EDGE_RISING);

    gpio_init_callback(&gpio0_cb, port0_changed, BIT(PIN_STROBE_IN));
    gpio_add_callback(gpio0, &gpio0_cb);

    gpio_pin_configure(gpio0, PIN_STROBE, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_LEFT_TURN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_HORN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio0, PIN_HEADLIGHT, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio1, PIN_RIGHT_TURN, GPIO_OUTPUT_INACTIVE);
}

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
    gpio_pin_set(gpio0, PIN_STROBE, strobe_state ? 1 : 0);
}

int main(void)
{   
    int err;

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err) {
        LOG_ERR("Failed to register authorization callbacks.");
        return 0;
    }

    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
    if (err) {
        LOG_INF("Failed to register authorization info callbacks.\n");
        return 0;
    }

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }
    LOG_INF("Bluetooth initialized");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = dk_leds_init();
    if (err) {
        LOG_ERR("Could not initialize LEDs (err %d)", err);
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

    // configure_car_outputs();

    LOG_INF("Starting Bluetooth Central");

    for (;;) {
        k_sleep(K_FOREVER);
    }
}