/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <ctype.h>
#include <logging/log.h>
#include <drivers/uart.h>
#include <string.h>
#include <init.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>

LOG_MODULE_REGISTER(at_host, CONFIG_AT_CMD_LOG_LEVEL);

#include "slm_util.h"
#include "slm_at_host.h"
#include "slm_at_gps.h"


#define SLM_UART_0_NAME	"UART_0"

#define OK_STR		"OK\r\n"
#define ERROR_STR	"ERROR\r\n"
#define FATAL_STR	"FATAL ERROR\r\n"
#define SLM_SYNC_STR	"Ready\r\n"

#define SLM_VERSION	"#XSLMVER: 1.2\r\n"
#define AT_CMD_SLMVER	"AT#XSLMVER"
#define AT_CMD_LTEMODE	"AT#XLTEMODE"
#define AT_CMD_EXIT	"AT#XEXIT"
#define AT_CMD_CLAC	"AT#XCLAC"

#define AT_MAX_CMD_LEN	CONFIG_AT_CMD_RESPONSE_MAX_LEN
#define UART_RX_BUF_NUM	2
#define UART_RX_LEN	256
#define UART_RX_TIMEOUT 1

/** @brief Termination Modes. */
enum term_modes {
	MODE_NULL_TERM, /**< Null Termination */
	MODE_CR,        /**< CR Termination */
	MODE_LF,        /**< LF Termination */
	MODE_CR_LF,     /**< CR+LF Termination */
	MODE_COUNT      /* Counter of term_modes */
};

static enum term_modes term_mode = MODE_CR_LF;
static const struct device *uart_dev;
static uint8_t at_buf[AT_MAX_CMD_LEN];
static size_t at_buf_len;
static struct k_work cmd_send_work;
static const char termination[3] = { '\0', '\r', '\n' };

static uint8_t uart_rx_buf[UART_RX_BUF_NUM][UART_RX_LEN];
static uint8_t *next_buf = uart_rx_buf[1];
static uint8_t *uart_tx_buf;

static K_SEM_DEFINE(tx_done, 0, 1);

static mode_retrieve_handler_t mode_retrieve_fn;
static mode_store_handler_t mode_store_fn;


/* global functions defined in different files */
bool do_exit_slm_at_host(void);

/* global variable defined in different files */
extern struct at_param_list at_param_list;
extern char rsp_buf[CONFIG_AT_CMD_RESPONSE_MAX_LEN];


/* forward declaration */
void slm_at_host_uninit(void);

void rsp_send(const uint8_t *str, size_t len)
{
	int ret;

	k_sem_take(&tx_done, K_FOREVER);

	uart_tx_buf = k_malloc(len);
	if (uart_tx_buf == NULL) {
		LOG_WRN("No ram buffer");
		k_sem_give(&tx_done);
		return;
	}

	LOG_HEXDUMP_DBG(str, len, "TX");

	memcpy(uart_tx_buf, str, len);
	ret = uart_tx(uart_dev, uart_tx_buf, len, SYS_FOREVER_MS);
	if (ret) {
		LOG_WRN("uart_tx failed: %d", ret);
		k_free(uart_tx_buf);
		k_sem_give(&tx_done);
	}
}

static void response_handler(void *context, const char *response)
{
	int len = strlen(response);

	ARG_UNUSED(context);

	/* Forward the data over UART */
	if (len > 0) {
		rsp_send(response, len);
	}
}

static void handle_at_clac(void)
{
	rsp_send(AT_CMD_SLMVER, sizeof(AT_CMD_SLMVER) - 1);
	rsp_send("\r\n", 2);
	rsp_send(AT_CMD_LTEMODE, sizeof(AT_CMD_LTEMODE) - 1);
	rsp_send("\r\n", 2);
	rsp_send(AT_CMD_EXIT, sizeof(AT_CMD_EXIT) - 1);
	rsp_send("\r\n", 2);
	rsp_send(AT_CMD_CLAC, sizeof(AT_CMD_CLAC) - 1);
	rsp_send("\r\n", 2);
	slm_at_gps_clac();
}

/**@brief handle AT#XLTEMODE commands
 *  AT#XLTEMODE=<mode>
 *  AT#XLTEMODE?
 *  AT#XLTEMODE=? TEST command not supported
 */
static int handle_at_ltemode(enum at_cmd_type cmd_type)
{
	int err = -EINVAL;
	uint16_t op;
	uint8_t mode;

	switch (cmd_type) {
	case AT_CMD_TYPE_SET_COMMAND:
		if (at_params_valid_count_get(&at_param_list) != 2) {
			return -EINVAL;
		}
		err = at_params_short_get(&at_param_list, 1, &op);
		if (err < 0) {
			return err;
		}
		if (op > 1) {
			return err;
		}
		mode = op;
		err = mode_store_fn(mode);
		break;

	case AT_CMD_TYPE_READ_COMMAND:
		mode_retrieve_fn(&mode);
		sprintf(rsp_buf, "#XLTEMODE: %d\r\n", mode);
		rsp_send(rsp_buf, strlen(rsp_buf));
		err = 0;
		break;

	case AT_CMD_TYPE_TEST_COMMAND:
		sprintf(rsp_buf, "#XLTEMODE: (0, 1)\r\n");
		rsp_send(rsp_buf, strlen(rsp_buf));
		err = 0;
		break;

	default:
		break;
	}

	return err;
}

static void cmd_send(struct k_work *work)
{
	size_t chars;
	char str[24];
	static char buf[AT_MAX_CMD_LEN];
	enum at_cmd_state state;
	int err;

	ARG_UNUSED(work);

	/* Make sure the string is 0-terminated */
	at_buf[MIN(at_buf_len, AT_MAX_CMD_LEN - 1)] = 0;

	LOG_HEXDUMP_DBG(at_buf, at_buf_len, "RX");

	if (slm_util_cmd_casecmp(at_buf, AT_CMD_SLMVER)) {
		rsp_send(SLM_VERSION, sizeof(SLM_VERSION) - 1);
		rsp_send(OK_STR, sizeof(OK_STR) - 1);
		goto done;
	}

	if (slm_util_cmd_casecmp(at_buf, AT_CMD_CLAC)) {
		handle_at_clac();
		rsp_send(OK_STR, sizeof(OK_STR) - 1);
		goto done;
	}

	if (slm_util_cmd_casecmp(at_buf, AT_CMD_LTEMODE)) {
		err = at_parser_params_from_str(at_buf, NULL, &at_param_list);
		if (err < 0) {
			LOG_ERR("Failed to parse AT command %d", err);
		} else {
			err = handle_at_ltemode(at_parser_cmd_type_get(at_buf));
			if (err == 0) {
				rsp_send(OK_STR, sizeof(OK_STR) - 1);
				goto done;
			}
		}
		rsp_send(ERROR_STR, sizeof(ERROR_STR) - 1);
		goto done;
	}

	if (slm_util_cmd_casecmp(at_buf, AT_CMD_EXIT)) {
		if (do_exit_slm_at_host() == false) {
			rsp_send(ERROR_STR, sizeof(ERROR_STR) - 1);
			goto done;
		}
		goto done;
	}

	err = slm_at_gps_parse(at_buf);
	if (err == 0) {
		rsp_send(OK_STR, sizeof(OK_STR) - 1);
		goto done;
	} else if (err != -ENOTSUP) {
		rsp_send(ERROR_STR, sizeof(ERROR_STR) - 1);
		goto done;
	}

	err = at_cmd_write(at_buf, buf, AT_MAX_CMD_LEN, &state);
	if (err < 0) {
		LOG_ERR("AT command error: %d", err);
		state = AT_CMD_ERROR;
	}

	switch (state) {
	case AT_CMD_OK:
		rsp_send(buf, strlen(buf));
		rsp_send(OK_STR, sizeof(OK_STR) - 1);
		break;
	case AT_CMD_ERROR:
		rsp_send(ERROR_STR, sizeof(ERROR_STR) - 1);
		break;
	case AT_CMD_ERROR_CMS:
		chars = sprintf(str, "+CMS: %d\r\n", err);
		rsp_send(str, ++chars);
		break;
	case AT_CMD_ERROR_CME:
		chars = sprintf(str, "+CME: %d\r\n", err);
		rsp_send(str, ++chars);
		break;
	default:
		break;
	}

done:
	err = uart_rx_enable(uart_dev, uart_rx_buf[0],
				sizeof(uart_rx_buf[0]), UART_RX_TIMEOUT);
	if (err) {
		LOG_ERR("UART RX failed: %d", err);
		rsp_send(FATAL_STR, sizeof(FATAL_STR) - 1);
	}
}

static void uart_rx_handler(uint8_t character)
{
	static bool inside_quotes;
	static size_t cmd_len;
	size_t pos;

	cmd_len += 1;
	pos = cmd_len - 1;

	/* Handle special characters. */
	switch (character) {
	case 0x08: /* Backspace. */
		/* Fall through. */
	case 0x7F: /* DEL character */
		pos = pos ? pos - 1 : 0;
		at_buf[pos] = 0;
		cmd_len = cmd_len <= 1 ? 0 : cmd_len - 2;
		break;
	case '"':
		inside_quotes = !inside_quotes;
		 /* Fall through. */
	default:
		/* Detect AT command buffer overflow or zero length */
		if (cmd_len > AT_MAX_CMD_LEN) {
			LOG_ERR("Buffer overflow, dropping '%c'\n", character);
			cmd_len = AT_MAX_CMD_LEN;
			return;
		} else if (cmd_len < 1) {
			LOG_ERR("Invalid AT command length: %d", cmd_len);
			cmd_len = 0;
			return;
		}

		at_buf[pos] = character;
		break;
	}

	if (inside_quotes) {
		return;
	}

	/* Check if the character marks line termination. */
	switch (term_mode) {
	case MODE_NULL_TERM:
		goto send;
	case MODE_CR:
		if (character == termination[term_mode]) {
			cmd_len--;
			goto send;
		}
		break;
	case MODE_LF:
		if ((at_buf[pos - 1]) &&
			character == termination[term_mode]) {
			cmd_len--;
			goto send;
		}
		break;
	case MODE_CR_LF:
		if ((at_buf[pos - 1] == '\r') && (character == '\n')) {
			cmd_len -= 2;
			goto send;
		}
		break;
	default:
		LOG_ERR("Invalid termination mode: %d", term_mode);
		break;
	}

	return;
send:
	uart_rx_disable(uart_dev);
	k_work_submit(&cmd_send_work);
	at_buf_len = cmd_len;
	cmd_len = 0;
}

static void uart_callback(const struct device *dev, struct uart_event *evt,
			  void *user_data)
{
	ARG_UNUSED(dev);

	int err;
	static uint16_t pos;

	ARG_UNUSED(user_data);

	switch (evt->type) {
	case UART_TX_DONE:
		k_free(uart_tx_buf);
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		k_free(uart_tx_buf);
		k_sem_give(&tx_done);
		LOG_INF("TX_ABORTED");
		break;
	case UART_RX_RDY:
		for (int i = pos; i < (pos + evt->data.rx.len); i++) {
			uart_rx_handler(evt->data.rx.buf[i]);
		}
		pos += evt->data.rx.len;
		break;
	case UART_RX_BUF_REQUEST:
		pos = 0;
		err = uart_rx_buf_rsp(uart_dev, next_buf,
					sizeof(uart_rx_buf[0]));
		if (err) {
			LOG_WRN("UART RX buf rsp: %d", err);
		}
		break;
	case UART_RX_BUF_RELEASED:
		next_buf = evt->data.rx_buf.buf;
		break;
	case UART_RX_STOPPED:
		LOG_WRN("RX_STOPPED (%d)", evt->data.rx_stop.reason);
		break;
	case UART_RX_DISABLED:
		LOG_DBG("RX_DISABLED");
		break;
	default:
		break;
	}
}

int slm_at_host_init(mode_retrieve_handler_t mode_retrieve, mode_store_handler_t mode_store)
{
	int err;
	uint32_t start_time;

	if ((mode_retrieve == NULL) || (mode_store == NULL)) {
		return -EINVAL;
	}
	mode_retrieve_fn = mode_retrieve;
	mode_store_fn = mode_store;

	/* Initialize the UART module */
	uart_dev = device_get_binding(SLM_UART_0_NAME);
	if (uart_dev == NULL) {
		LOG_ERR("Cannot bind %s\n", SLM_UART_0_NAME);
		return -EINVAL;
	}
	/* Wait for the UART line to become valid */
	start_time = k_uptime_get_32();
	do {
		err = uart_err_check(uart_dev);
		if (err) {
			if (k_uptime_get_32() - start_time > 500) {
				LOG_ERR("UART check failed: %d. "
					"UART initialization timed out.", err);
				return -EIO;
			}
		}
	} while (err);
	/* Register async handling callback */
	err = uart_callback_set(uart_dev, uart_callback, NULL);
	if (err) {
		LOG_ERR("Cannot set callback: %d", err);
		return -EFAULT;
	}
	/* Power on UART module */
	device_set_power_state(uart_dev, DEVICE_PM_ACTIVE_STATE,
				NULL, NULL);
	err = uart_rx_enable(uart_dev, uart_rx_buf[0],
				sizeof(uart_rx_buf[0]), UART_RX_TIMEOUT);
	if (err) {
		LOG_ERR("Cannot enable rx: %d", err);
		return -EFAULT;
	}

#if defined(CONFIG_BSD_LIBRARY_TRACE_ENABLED)
	err = at_cmd_write("AT%XMODEMTRACE=1,2", NULL, 0, NULL);
	if (err) {
		LOG_ERR("Failed to activate modem trace: %d", err);
		return -EIO;
	}
#endif

	err = at_notif_register_handler(NULL, response_handler);
	if (err) {
		LOG_ERR("Can't register handler err=%d", err);
		return err;
	}


	err = slm_at_gps_init();
	if (err) {
		LOG_ERR("GPS could not be initialized: %d", err);
		return -EFAULT;
	}

	k_work_init(&cmd_send_work, cmd_send);
	k_sem_give(&tx_done);
	rsp_send(SLM_SYNC_STR, sizeof(SLM_SYNC_STR)-1);

	LOG_DBG("at_host init done");
	return err;
}

void slm_at_host_uninit(void)
{
	int err;

	err = slm_at_gps_uninit();
	if (err) {
		LOG_WRN("GPS could not be uninitialized: %d", err);
	}
	err = at_notif_deregister_handler(NULL, response_handler);
	if (err) {
		LOG_WRN("Can't deregister handler: %d", err);
	}

	// turn off modem
	err = at_cmd_write("AT+CFUN=0", NULL, 0, NULL);
	if (err < 0) {
		LOG_ERR("AT+CFUN=0 error: %d", err);
	}

	/* Power off UART module */
	uart_rx_disable(uart_dev);
	k_sleep(K_MSEC(100));
#if 0 // demo app will still be using uart_dev
	err = device_set_power_state(uart_dev, DEVICE_PM_OFF_STATE,
				NULL, NULL);
	if (err) {
		LOG_WRN("Can't power off uart: %d", err);
	}
#endif

	LOG_DBG("at_host uninit done");
}
