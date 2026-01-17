#include "gnarl.h"
#include "led.h"
#include "module.h"

#include <unistd.h>

#include <esp_timer.h>
#include <esp_system.h>
#include <host/ble_gap.h>
#include <os/os_mbuf.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <nvs_flash.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

#include "adc.h"
#include "commands.h"
//#include "led.h"


#define MAX_DATA	260

void ble_store_ram_init(void);
int ble_errors = 0;

#define B0(x)	((x) & 0xFF)
#define B1(x)	(((x) >> 8) & 0xFF)
#define B2(x)	(((x) >> 16) & 0xFF)
#define B3(x)	(((x) >> 24) & 0xFF)
#define B4(x)	(((x) >> 32) & 0xFF)
#define B5(x)	(((x) >> 40) & 0xFF)

#define UUID128_CONST(a32, b16, c16, d16, e48)	\
	BLE_UUID128_INIT(						\
		B0(e48), B1(e48), B2(e48), B3(e48), B4(e48), B5(e48),	\
		B0(d16), B1(d16),					\
		B0(c16), B1(c16),					\
		B0(b16), B1(b16),					\
		B0(a32), B1(a32), B2(a32), B3(a32),			\
	)

static ble_uuid128_t service_uuid          = UUID128_CONST(0x0235733b, 0x99c5, 0x4197, 0xb856, 0x69219c2a3845);
static ble_uuid128_t data_uuid             = UUID128_CONST(0xc842e849, 0x5028, 0x42e2, 0x867c, 0x016adada9155);
static ble_uuid128_t response_count_uuid   = UUID128_CONST(0x6e6c7910, 0xb89e, 0x43a5, 0xa0fe, 0x50c5e2b81f4a);
static ble_uuid128_t timer_tick_uuid       = UUID128_CONST(0x6e6c7910, 0xb89e, 0x43a5, 0x78af, 0x50c5e2b86f7e);
static ble_uuid128_t custom_name_uuid      = UUID128_CONST(0xd93b2af0, 0x1e28, 0x11e4, 0x8c21, 0x0800200c9a66);
static ble_uuid128_t firmware_version_uuid = UUID128_CONST(0x30d99dc9, 0x7c91, 0x4295, 0xa051, 0x0a104d238cf2);
static ble_uuid128_t led_mode_uuid         = UUID128_CONST(0xc6d84241, 0xf1a7, 0x4f9c, 0xa25f, 0xfce16732f14e);

static ble_uuid16_t battery_service_uuid = BLE_UUID16_INIT(0x180F);
static ble_uuid16_t battery_level_uuid = BLE_UUID16_INIT(0x2A19);

static ble_gap_event_fn handle_gap_event;
static uint8_t addr_type;
static ble_gatt_access_fn data_access;
static ble_gatt_access_fn custom_name_access;
static ble_gatt_access_fn led_mode_access;
static ble_gatt_access_fn firmware_version_access;
static ble_gatt_access_fn battery_level_access;
static ble_gatt_access_fn no_access;

static bool connected;
static uint16_t connection_handle;
static bool ble_ready;  // Set to true after NimBLE sync callback

bool ble_is_connected(void) {
	return connected;
}

bool ble_is_ready(void) {
	return ble_ready;
}

static uint16_t response_count_notify_handle;
static int response_count_notify_state;
static uint8_t response_count;

static uint16_t timer_tick_notify_handle;
static int timer_tick_notify_state;
static uint8_t timer_tick;
static void timer_tick_callback(void *);

static uint16_t battery_level_notify_handle;
static int battery_level_notify_state;
static uint8_t battery_level = 100;  // Will be updated from ADC

static const struct ble_gatt_svc_def service_list[] = {
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &service_uuid.u,
		.characteristics = (struct ble_gatt_chr_def[]){
			{
				.uuid = &data_uuid.u,
				.access_cb = data_access,
				.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
			},
			{
				.uuid = &response_count_uuid.u,
				.access_cb = no_access,
				.val_handle = &response_count_notify_handle,
				.flags = BLE_GATT_CHR_F_NOTIFY,
			},
			{
				.uuid = &timer_tick_uuid.u,
				.access_cb = no_access,
				.val_handle = &timer_tick_notify_handle,
				.flags = BLE_GATT_CHR_F_NOTIFY,
			},
			{
				.uuid = &custom_name_uuid.u,
				.access_cb = custom_name_access,
				.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
			},
			{
				.uuid = &firmware_version_uuid.u,
				.access_cb = firmware_version_access,
				.flags = BLE_GATT_CHR_F_READ,
			},
			{
				.uuid = &led_mode_uuid.u,
				.access_cb = led_mode_access,
				.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
			},
			{}, // End of characteristic list.
		},
        },
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &battery_service_uuid.u,
		.characteristics = (struct ble_gatt_chr_def[]){
			{
				.uuid = &battery_level_uuid.u,
				.access_cb = battery_level_access,
				.val_handle = &battery_level_notify_handle,
				.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
			},
			{}, // End of characteristic list.
		},
	},
	{}, // End of service list.
};

static void server_init(void) {
	int err;
	char u[60];

	ble_svc_gap_init();
	ble_svc_gatt_init();

	err = ble_gatts_count_cfg(service_list);
	if (err) {
		ESP_LOGE(TAG, "CRITICAL: ble_gatts_count_cfg failed: %d", err);
		ESP_LOGE(TAG, "BLE initialization failed - device cannot function without BLE");
		ESP_LOGE(TAG, "Restarting in 3 seconds...");
		vTaskDelay(pdMS_TO_TICKS(3000));
		esp_restart();
	}

	err = ble_gatts_add_svcs(service_list);
	if (err) {
		ESP_LOGE(TAG, "CRITICAL: ble_gatts_add_svcs failed: %d", err);
		ESP_LOGE(TAG, "BLE services registration failed - device cannot function");
		ESP_LOGE(TAG, "Restarting in 3 seconds...");
		vTaskDelay(pdMS_TO_TICKS(3000));
		esp_restart();
	}

	ble_uuid_to_str(&service_uuid.u, u);
	ESP_LOGD(TAG, "service UUID %s", u);

	esp_timer_handle_t t;
	esp_timer_create_args_t timer_args = {
		.callback = timer_tick_callback,
	};
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &t));
	ESP_ERROR_CHECK(esp_timer_start_periodic(t, 60*SECONDS));
}

static void advertise(void) {
	ESP_LOGI(TAG, "advertise() called");

	// Stop any existing advertising first
	int stop_err = ble_gap_adv_stop();
	if (stop_err != 0 && stop_err != BLE_HS_EALREADY) {
		ESP_LOGW(TAG, "ble_gap_adv_stop returned: %d", stop_err);
	}

	// Small delay to let BLE stack settle after disconnect
	vTaskDelay(pdMS_TO_TICKS(50));

	struct ble_hs_adv_fields fields;
	memset(&fields, 0, sizeof(fields));

	fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

	fields.tx_pwr_lvl_is_present = 1;
	fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

	const char *name = ble_svc_gap_device_name();
	fields.name = (uint8_t *)name;
	fields.name_len = strlen(name);
	fields.name_is_complete = 1;

	fields.uuids128 = &service_uuid;
	fields.num_uuids128 = 1;
	fields.uuids128_is_complete = 1;

	int err = ble_gap_adv_set_fields(&fields);
	if (err) {
		ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d - trying fallback", err);
		struct ble_hs_adv_fields fallback;
		memset(&fallback, 0, sizeof(fallback));
		fallback.flags = fields.flags;
		fallback.name = (uint8_t *)name;
		fallback.name_len = strlen(name);
		fallback.name_is_complete = 0;
		int ferr = ble_gap_adv_set_fields(&fallback);
		if (ferr) {
			ESP_LOGE(TAG, "fallback ble_gap_adv_set_fields failed: %d", ferr);
			return;
		}
		ESP_LOGI(TAG, "fallback advertising fields applied");
	}

	struct ble_gap_adv_params adv;
	memset(&adv, 0, sizeof(adv));
	adv.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

	// Retry advertising start up to 3 times
	for (int retry = 0; retry < 3; retry++) {
		err = ble_gap_adv_start(addr_type, 0, BLE_HS_FOREVER, &adv, handle_gap_event, 0);
		if (err == 0) {
			ESP_LOGI(TAG, "advertising started");
			break;
		}
		if (err == BLE_HS_EALREADY) {
			ESP_LOGI(TAG, "already advertising");
			break;
		}
		ESP_LOGW(TAG, "ble_gap_adv_start failed: %d (retry %d/3)", err, retry + 1);
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	if (err != 0 && err != BLE_HS_EALREADY) {
		ESP_LOGE(TAG, "advertising failed after retries, err=%d", err);
		return;
	}

	// Set scan response
	struct ble_hs_adv_fields rsp;
	memset(&rsp, 0, sizeof(rsp));
	rsp.name = (uint8_t *)name;
	rsp.name_len = strlen(name);
	rsp.name_is_complete = 1;
	rsp.uuids128 = &service_uuid;
	rsp.num_uuids128 = 1;
	int rerr = ble_gap_adv_rsp_set_fields(&rsp);
	if (rerr) {
		ESP_LOGW(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rerr);
	}
}


static int handle_gap_event(struct ble_gap_event *e, void *arg) {
	switch (e->type) {
	case BLE_GAP_EVENT_CONNECT:
		if (e->connect.status != 0) {
			ESP_LOGE(TAG, "connection failed");
			advertise();
			return 0;
		}
		connected = true;
		//led_on();
		connection_handle = e->connect.conn_handle;
		ble_errors = 0;  // Reset error counter on successful connection
		ESP_LOGI(TAG, "connected");
		ESP_LOGD(TAG, "connection handle %04X", connection_handle);
		ESP_LOGD(TAG, "response count notify handle %04X", response_count_notify_handle);
		ESP_LOGD(TAG, "timer tick notify handle %04X", timer_tick_notify_handle);
		
		// Update connection parameters for reliability with iAPS/Loop
		// BLE spec requires: supervision_timeout > (1 + latency) * interval_max * 2
		// With latency=0, interval=40: timeout > 80 (800ms minimum)
		struct ble_gap_upd_params conn_params = {
			.itvl_min = 12,   // 15ms (12 * 1.25ms) - faster response
			.itvl_max = 24,   // 30ms (24 * 1.25ms)
			.latency = 0,     // no skipping - always respond
			.supervision_timeout = 600,  // 6000ms = 6 seconds
			.min_ce_len = 0,
			.max_ce_len = 0,
		};
		int err = ble_gap_update_params(connection_handle, &conn_params);
		if (err != 0) {
			ESP_LOGW(TAG, "ble_gap_update_params failed: %d", err);
		} else {
			ESP_LOGI(TAG, "BLE params: itvl=15-30ms latency=0 timeout=6s");
		}
		break;
	case BLE_GAP_EVENT_DISCONNECT:
		connected = false;
		//led_off();
		ESP_LOGI(TAG, "disconnected: reason=%d conn_handle=%04X", e->disconnect.reason, connection_handle);
		advertise();
		ble_errors = ble_errors + 1;
		ESP_LOGD(TAG, "disconnect count: %d", ble_errors);
		// Restart only after many consecutive disconnects (not after normal reconnections)
		if (ble_errors > 10) { 
			ESP_LOGW(TAG, "Too many disconnects (%d), restarting...", ble_errors);
			esp_restart();		
		}
		break;
	case BLE_GAP_EVENT_ADV_COMPLETE:
		ESP_LOGD(TAG, "advertising complete");
		advertise();
		break;
	case BLE_GAP_EVENT_SUBSCRIBE:
		if (e->subscribe.attr_handle == response_count_notify_handle) {
			ESP_LOGD(TAG, "notify %d for response count", e->subscribe.cur_notify);
			response_count_notify_state = e->subscribe.cur_notify;
			break;
		}
		if (e->subscribe.attr_handle == timer_tick_notify_handle) {
			ESP_LOGD(TAG, "notify %d for timer tick", e->subscribe.cur_notify);
			timer_tick_notify_state = e->subscribe.cur_notify;
			break;
		}
		if (e->subscribe.attr_handle == battery_level_notify_handle) {
			ESP_LOGI(TAG, "Battery Service: client %s notifications", e->subscribe.cur_notify ? "ENABLED" : "DISABLED");
			battery_level_notify_state = e->subscribe.cur_notify;
			break;
		}
		ESP_LOGD(TAG, "notify %d for unknown handle %04X", e->subscribe.cur_notify, e->subscribe.attr_handle);
		break;
	default:
		ESP_LOGD(TAG, "GAP event %d", e->type);
		break;
	}
	return 0;
}

static void sync_callback(void) {
	int err;

	ESP_LOGI(TAG, "NimBLE sync callback invoked");
	err = ble_hs_util_ensure_addr(0);
	if (err) {
		ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", err);
		return;
	}

	err = ble_hs_id_infer_auto(0, &addr_type);
	if (err) {
		ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", err);
		return;
	}

	uint8_t addr[6];
	ble_hs_id_copy_addr(addr_type, addr, 0);
	if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
		printf("device address: ");
		for (int i = 0; i < sizeof(addr); i++) {
			printf("%s%02x", i == 0 ? "" : ":", addr[i]);
		}
		printf("\n");
	}
	advertise();

	// Mark BLE as ready - safe to use light sleep now
	ble_ready = true;
}

static uint8_t data_in[MAX_DATA];
static uint16_t data_in_len;

static uint8_t data_out[MAX_DATA];
static uint16_t data_out_len;

static void response_notify(void) {
	response_count++;
	if (!response_count_notify_state) {
		ESP_LOGD(TAG, "not notifying for response count %d", response_count);
		return;
	}
	struct os_mbuf *om = ble_hs_mbuf_from_flat(&response_count, sizeof(response_count));
	if (om == NULL) {
		ESP_LOGE(TAG, "response_notify: failed to allocate mbuf");
		return;
	}
	int err = ble_gattc_notify_custom(connection_handle, response_count_notify_handle, om);
	if (err) {
		ESP_LOGW(TAG, "ble_gattc_notify_custom failed: %d", err);
		os_mbuf_free_chain(om);  // Free mbuf on error
	}
	ESP_LOGD(TAG, "notify for response count %d", response_count);
}

void send_code(const uint8_t code) {
	ESP_LOGD(TAG, "send_code %02X", code);
	data_out[0] = code;
	data_out_len = 1;
	led_indicate_ble_activity();
	response_notify();
}

void send_bytes(const uint8_t *buf, int count) {
	print_bytes("BLE >", buf, count);
	// Prevent buffer overflow: data_out[MAX_DATA], need 1 byte for status code
	if (count < 0 || count > MAX_DATA - 1) {
		ESP_LOGE(TAG, "send_bytes: count %d exceeds buffer size %d", count, MAX_DATA - 1);
		send_code(RESPONSE_CODE_PARAM_ERROR);
		return;
	}
	data_out[0] = RESPONSE_CODE_SUCCESS;
	memcpy(data_out + 1, buf, count);
	data_out_len = count + 1;
	led_indicate_ble_activity();
	response_notify();
}

void print_bytes(const char *msg, const uint8_t *buf, int count) {
#ifdef BLE_DEBUG
	printf("%s", msg);
	for (int i = 0; i < count; i++) {
		printf(" %02X", buf[i]);
	}
	printf("\n");
#else
	(void)msg; (void)buf; (void)count;
#endif
}

static void timer_tick_callback(void *arg) {
	timer_tick++;
	ESP_LOGD(TAG, "timer tick %d", timer_tick);
	if (!timer_tick_notify_state) {
		if (connected) {
			ESP_LOGD(TAG, "not notifying for timer tick");
		}
		return;
	}
	struct os_mbuf *om = ble_hs_mbuf_from_flat(&timer_tick, sizeof(timer_tick));
	if (om == NULL) {
		ESP_LOGE(TAG, "timer_tick: failed to allocate mbuf");
		return;
	}
	int err = ble_gattc_notify_custom(connection_handle, timer_tick_notify_handle, om);
	if (err) {
		ESP_LOGW(TAG, "timer tick notify failed: %d", err);
		os_mbuf_free_chain(om);  // Free mbuf on error
	}
	ESP_LOGD(TAG, "notify for timer tick");

	// Notify battery level every 60 seconds
	if (battery_level_notify_state) {
		battery_level = battery_percent(get_battery_voltage());
		ESP_LOGI(TAG, "battery notify: voltage=%d mV, level=%d%%", get_battery_voltage(), battery_level);
		struct os_mbuf *batt_om = ble_hs_mbuf_from_flat(&battery_level, sizeof(battery_level));
		if (batt_om == NULL) {
			ESP_LOGE(TAG, "battery_notify: failed to allocate mbuf");
		} else {
			err = ble_gattc_notify_custom(connection_handle, battery_level_notify_handle, batt_om);
			if (err) {
				ESP_LOGW(TAG, "battery notify failed: %d", err);
				os_mbuf_free_chain(batt_om);  // Free mbuf on error
			}
		}
		ESP_LOGD(TAG, "notify for battery level %d%%", battery_level);
	}
}

static int data_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	int err;
	int8_t rssi;
	if (ble_uuid_cmp(ctxt->chr->uuid, &data_uuid.u) != 0) {
		ESP_LOGE(TAG, "data_access: unexpected UUID");
		return BLE_ATT_ERR_UNLIKELY;
	}
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR:
		print_bytes("BLE >", data_out, data_out_len);
		if (os_mbuf_append(ctxt->om, data_out, data_out_len) != 0) {
			return BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		return 0;
	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		err = ble_hs_mbuf_to_flat(ctxt->om, data_in, sizeof(data_in), &data_in_len);
		if (err) {
			ESP_LOGE(TAG, "data_access: mbuf_to_flat failed: %d", err);
			return BLE_ATT_ERR_UNLIKELY;
		}
		print_bytes("BLE <", data_in, data_in_len);
		ble_gap_conn_rssi(conn_handle, &rssi);
		rfspy_command(data_in, data_in_len, (int)rssi);
		return 0;
	default:
		ESP_LOGE(TAG, "data_access: unknown op %d", ctxt->op);
		return BLE_ATT_ERR_UNLIKELY;
	}
	return 0;
}

static uint8_t custom_name[30];
static uint16_t custom_name_len;

static int custom_name_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	int err;
	if (ble_uuid_cmp(ctxt->chr->uuid, &custom_name_uuid.u) != 0) {
		ESP_LOGE(TAG, "custom_name_access: unexpected UUID");
		return BLE_ATT_ERR_UNLIKELY;
	}
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR:
		print_bytes("BLE >", custom_name, custom_name_len);
		if (os_mbuf_append(ctxt->om, custom_name, custom_name_len) != 0) {
			return BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		return 0;
	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		err = ble_hs_mbuf_to_flat(ctxt->om, custom_name, sizeof(custom_name), &custom_name_len);
		if (err) {
			ESP_LOGE(TAG, "custom_name_access: mbuf_to_flat failed: %d", err);
			return BLE_ATT_ERR_UNLIKELY;
		}
		print_bytes("BLE <", custom_name, custom_name_len);
		return 0;
	default:
		ESP_LOGE(TAG, "custom_name_access: unknown op %d", ctxt->op);
		return BLE_ATT_ERR_UNLIKELY;
	}
}

static int firmware_version_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (ble_uuid_cmp(ctxt->chr->uuid, &firmware_version_uuid.u) != 0) {
		ESP_LOGE(TAG, "firmware_version_access: unexpected UUID");
		return BLE_ATT_ERR_UNLIKELY;
	}
	if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
		ESP_LOGE(TAG, "firmware_version_access: unexpected op %d", ctxt->op);
		return BLE_ATT_ERR_UNLIKELY;
	}
	ESP_LOGD(TAG, "BLE firmware version = %s", BLE_RFSPY_VERSION);
	if (os_mbuf_append(ctxt->om, (const uint8_t *)BLE_RFSPY_VERSION, strlen(BLE_RFSPY_VERSION)) != 0) {
		return BLE_ATT_ERR_INSUFFICIENT_RES;
	}
	return 0;
}

// Use external current_led_mode from gnarl.c instead of local variable
extern uint8_t current_led_mode;
extern void led_set_mode(uint8_t mode);

static int led_mode_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	int err;
	uint16_t n;
	uint8_t mode;
	if (ble_uuid_cmp(ctxt->chr->uuid, &led_mode_uuid.u) != 0) {
		ESP_LOGE(TAG, "led_mode_access: unexpected UUID");
		return BLE_ATT_ERR_UNLIKELY;
	}
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR:
		ESP_LOGI(TAG, "led_mode_access: READ mode = 0x%02X", current_led_mode);
		if (os_mbuf_append(ctxt->om, &current_led_mode, sizeof(current_led_mode))) {
			return BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		return 0;
	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		err = ble_hs_mbuf_to_flat(ctxt->om, &mode, sizeof(mode), &n);
		if (err) {
			ESP_LOGE(TAG, "led_mode_access: mbuf_to_flat failed: %d", err);
			return BLE_ATT_ERR_UNLIKELY;
		}
		ESP_LOGI(TAG, "led_mode_access: WRITE received mode = 0x%02X (ignored, use CmdLED)", mode);
		// BLE writes ignored - LED control happens via CmdLED command
		return 0;
	default:
		ESP_LOGE(TAG, "led_mode_access: unknown op %d", ctxt->op);
		return BLE_ATT_ERR_UNLIKELY;
	}
	return 0;
}

static int battery_level_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (ble_uuid_cmp(ctxt->chr->uuid, &battery_level_uuid.u) != 0) {
		ESP_LOGE(TAG, "battery_level_access: unexpected UUID");
		return BLE_ATT_ERR_UNLIKELY;
	}
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR:
		battery_level = battery_percent(get_battery_voltage());
		ESP_LOGI(TAG, "battery_level_access: READ voltage=%d mV, level=%d%%", get_battery_voltage(), battery_level);
		if (os_mbuf_append(ctxt->om, &battery_level, sizeof(battery_level))) {
			return BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		return 0;
	default:
		ESP_LOGE(TAG, "battery_level_access: unknown op %d", ctxt->op);
		return BLE_ATT_ERR_UNLIKELY;
	}
}

static int no_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	char u[60];
	ble_uuid_to_str(ctxt->chr->uuid, u);
	ESP_LOGE(TAG, "should not happen: op %d, attr handle %04X, uuid %s", ctxt->op, attr_handle, u);
	return 0;
}

static void host_task(void *arg) {
	ESP_LOGI(TAG, "NimBLE host_task started");
	nimble_port_run();
	ESP_LOGI(TAG, "NimBLE host_task exiting");
}

void gnarl_init(void) {
	start_gnarl_task();

	// NVS already initialized in app_main()
	
	ESP_LOGI(TAG, "Starting NimBLE init...");
	esp_err_t ret = nimble_port_init();
	ESP_LOGI(TAG, "nimble_port_init returned %d", ret);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to init nimble %d", ret);
		return;
	}

	ble_hs_cfg.sync_cb = sync_callback;

	server_init();
	ESP_LOGI(TAG, "NimBLE server_init done");

	int err = ble_svc_gap_device_name_set(BLE_NAME);
	if (err) {
		ESP_LOGE(TAG, "ble_init: device_name_set failed: %d", err);
	}

	ble_store_ram_init();
	nimble_port_freertos_init(host_task);
}
