/***************************************************************************//**
 * @file
 * @brief HCI host example project
 *
 * This is a Linux host application intended to be used as a test tool
 * to use RF test modes on an EFR32 device running RCP (HCI) firmware.
 *******************************************************************************
 * # License
 * <b>Copyright 2026 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <time.h>
#include <getopt.h>

#define OPTSTRING "hv"

#define USAGE "%s " OPTIONS

// Options info.
#define OPTIONS    \
  "\nOPTIONS\n"    \
"  -h                      		Print help message\n"\
"  --help\n"\
"  --version               	   Print version number defined in application.\n"\
"  --time   <duration ms>      Set time for test in milliseconds, 0 for infinite mode (exit with control-c)\n"\
"  --packet_type <payload/modulation type, 0:PBRS9 packet payload, 1:11110000 packet payload, 2:10101010 packet payload, 3: PRBS15\n"\
"								4:11111111 packet payload, 5:00000000 packet payload, 6:00001111 packet payload, 7:01010101 packet payload>\n"\
"  --power  <power level>      Set power level for test in 1 dBm steps\n"\
"  --channel <channel index>   Set channel index for test, frequency=2402 MHz + 2*channel>\n"\
"  --len <test packet length>  Set test packet length>\n"\
"  --rx                        DTM receive test. Prints number of received DTM packets.\n"\
"  --phy  <PHY selection for test packets/waveforms/RX mode, 1:1Mbps, 2:2Mbps, 3:125k LR coded (S=8), 4:500k LR coded (S=2).>\n"\
"  --hci_port <hci port num>    Number of the DUT's HCI port (0=hci0, 1=hci1, 2=hci2, etc.)\n"\
"  --adv <name>                Advertise with Complete Local Name set to <name>\n"\
"  --advscan                   Scan for advertisements and print MAC, RSSI, and AD types\n"\

#define LONG_OPT_VERSION 0
#define LONG_OPT_TIME 1
#define LONG_OPT_PACKET_TYPE 2
#define LONG_OPT_POWER 3
#define LONG_OPT_CHANNEL 4
#define LONG_OPT_LEN 5
#define LONG_OPT_RX 6
#define LONG_OPT_PHY 7
#define LONG_OPT_PORT 8
#define LONG_OPT_ADV 9
#define LONG_OPT_ADVSCAN 10
#define LONG_OPT_HELP 'h'

static struct option long_options[] = {
		{"version",    no_argument,       0,  LONG_OPT_VERSION },
		{"time",       required_argument, 0,  LONG_OPT_TIME },
		{"packet_type",required_argument, 0,  LONG_OPT_PACKET_TYPE },
		{"power",      required_argument, 0,  LONG_OPT_POWER },
		{"channel",    required_argument, 0,  LONG_OPT_CHANNEL },
		{"len",        required_argument, 0,  LONG_OPT_LEN },
		{"rx",         no_argument,       0,  LONG_OPT_RX },
		{"phy",        required_argument, 0,  LONG_OPT_PHY },
		{"hci_port",   required_argument, 0,  LONG_OPT_PORT },
		{"adv",        required_argument, 0,  LONG_OPT_ADV },
		{"advscan",    no_argument,       0,  LONG_OPT_ADVSCAN },
		{"help",   	   no_argument, 0,  LONG_OPT_HELP },
		{0,           0,                 0,  0  }
		};

#define VERSION_MAJ	0u
#define VERSION_MIN	4u

#define TRUE   1u
#define FALSE  0u

#define DEFAULT_PHY 0x01	//default to use 1Mbit PHY

/* Program defaults - will use these if not specified on command line */
#ifndef DEFAULT_DURATION
  #define DEFAULT_DURATION		0	//default=0 (can override with global #define)
#endif
#ifndef DEFAULT_PACKET_TYPE
  #define DEFAULT_PACKET_TYPE		0x00 	
#endif
#define DEFAULT_CHANNEL			0	//2.402 GHz
#define DEFAULT_POWER_LEVEL		5	//5 dBm
#define DEFAULT_PACKET_LENGTH	25
#define CMP_LENGTH	2

#define MAX_LE_ADV_DATA_LEN	31
#define ADV_TYPE_COMPLETE_LOCAL_NAME 0x09
#define DEFAULT_ADV_INTERVAL	0x00A0
#define DEFAULT_ADV_CHANNEL_MAP 0x07
#define DEFAULT_SCAN_INTERVAL	0x0010
#define DEFAULT_SCAN_WINDOW	0x0010
#define HCI_EVENT_BUFFER_SIZE	260

#ifndef OCF_LE_SET_ADVERTISING_PARAMETERS
#define OCF_LE_SET_ADVERTISING_PARAMETERS 0x0006
#endif
#ifndef OCF_LE_SET_ADVERTISING_DATA
#define OCF_LE_SET_ADVERTISING_DATA 0x0008
#endif
#ifndef OCF_LE_SET_ADVERTISE_ENABLE
#define OCF_LE_SET_ADVERTISE_ENABLE 0x000A
#endif
#ifndef OCF_LE_SET_SCAN_PARAMETERS
#define OCF_LE_SET_SCAN_PARAMETERS 0x000B
#endif
#ifndef OCF_LE_SET_SCAN_ENABLE
#define OCF_LE_SET_SCAN_ENABLE 0x000C
#endif
#ifndef EVT_LE_META_EVENT
#define EVT_LE_META_EVENT 0x3E
#endif
#ifndef EVT_LE_ADVERTISING_REPORT
#define EVT_LE_ADVERTISING_REPORT 0x02
#endif
#ifndef HCI_EVENT_HDR_SIZE
#define HCI_EVENT_HDR_SIZE 2
#endif

// Custom Silabs vendor specific commands (AN1328)
#define HCI_VS_SiliconLabs_Set_Min_Max_TX_Power_OCF 0x14
#define HCI_VS_SiliconLabs_Read_Current_TX_Power_Configuration_OCF 0x17
#define HCI_VS_SiliconLabs_Get_Counters_OCF 0x12

// Define for return parameters for HCI_VS_SiliconLabs_Get_Counters
typedef struct {
	uint8_t status;
	uint16_t tx_packets;
	uint16_t rx_packets;
	uint16_t crc_errors;
	uint16_t failures;
} __attribute__ ((packed)) vs_SiliconLabs_Get_Counters_cp;


// Defines for extended HCI test commands in the Bluetooth Core spec V5.2
// Vol 4, part E, section 7.8.28-29
#define OCF_LE_RECEIVER_TEST_V2			0x0033
typedef struct {
	uint8_t		frequency;
	uint8_t 	phy;
	uint8_t 	modulation_index;
} __attribute__ ((packed)) le_receiver_test_v2_cp;
#define LE_RECEIVER_TEST_V2_CP_SIZE 3

#define OCF_LE_TRANSMITTER_TEST_V4			0x007B
typedef struct {
	uint8_t		frequency;
	uint8_t		length;
	uint8_t		payload;
	uint8_t 	phy;
	uint8_t 	cte_len;
	uint8_t		cte_type;
	uint8_t 	switching_pattern_len;
//	uint8_t 	antenna_IDs[]; // we are always going to use len=0, omitting this element
	int8_t 		tx_power_level; // in dBm
} __attribute__ ((packed)) le_transmitter_test_v4_cp;
#define LE_TRANSMITTER_TEST_V4_CP_SIZE 8

typedef struct {
	uint16_t	min_interval;
	uint16_t	max_interval;
	uint8_t		adv_type;
	uint8_t		own_bdaddr_type;
	uint8_t		direct_bdaddr_type;
	bdaddr_t	direct_bdaddr;
	uint8_t		channel_map;
	uint8_t		filter_policy;
} __attribute__ ((packed)) app_le_set_advertising_parameters_cp;
#define APP_LE_SET_ADVERTISING_PARAMETERS_CP_SIZE 15

typedef struct {
	uint8_t		length;
	uint8_t		data[MAX_LE_ADV_DATA_LEN];
} __attribute__ ((packed)) app_le_set_advertising_data_cp;
#define APP_LE_SET_ADVERTISING_DATA_CP_SIZE 32

typedef struct {
	uint8_t		enable;
} __attribute__ ((packed)) app_le_set_advertise_enable_cp;
#define APP_LE_SET_ADVERTISE_ENABLE_CP_SIZE 1

typedef struct {
	uint8_t		scan_type;
	uint16_t	interval;
	uint16_t	window;
	uint8_t		own_bdaddr_type;
	uint8_t		filter_policy;
} __attribute__ ((packed)) app_le_set_scan_parameters_cp;
#define APP_LE_SET_SCAN_PARAMETERS_CP_SIZE 7

typedef struct {
	uint8_t		enable;
	uint8_t		filter_dup;
} __attribute__ ((packed)) app_le_set_scan_enable_cp;
#define APP_LE_SET_SCAN_ENABLE_CP_SIZE 2

/* The modulation type */
static uint8_t packet_type=DEFAULT_PACKET_TYPE;

/* The power level */
static int16_t power_level=DEFAULT_POWER_LEVEL;

/* The duration in us */
static uint32_t duration_usec=DEFAULT_DURATION;

/* channel */
static uint8_t channel=DEFAULT_CHANNEL;

/* packet length */
static uint8_t packet_length=DEFAULT_PACKET_LENGTH;

/* phy */
static uint8_t selected_phy=DEFAULT_PHY;

static int hci_device = -1;
static char *advertising_name = NULL;
static volatile sig_atomic_t stop_requested = FALSE;

/* application state machine */
static enum app_states {
  dtm_rx_begin,
  dtm_tx_begin,
  adv_begin,
  advscan_begin
} app_state =  dtm_tx_begin; //default to TX

/* Prototypes */
void close_hci(void);
void wait_for_duration_or_signal(uint32_t duration_us);
void send_le_status_command(uint16_t ocf, void *cparam, int clen, const char *description);
void set_power(int16_t power_ddbm);
void start_tx(uint8_t channel, uint8_t len, uint8_t packet_type, uint8_t phy, int8_t power);
void start_rx(uint8_t channel, uint8_t phy);
void start_advertising(const char *name);
void stop_advertising(void);
void scan_advertisements(uint32_t duration_us);
void get_power_config(void);
void exit_with_results(void);
const char *ad_type_name(uint8_t type);
void format_ad_type_list(const uint8_t *data, uint8_t data_len, char *out, size_t out_len);
void print_advertising_reports(const uint8_t *buf, ssize_t len);


struct hci_request ble_hci_ctl_request(uint16_t ocf, void * cparam, int clen, void * rparam, int rlen)
{
	// Create an hci request
	struct hci_request rq;
	memset(&rq, 0, sizeof(rq));
	rq.ogf = OGF_LE_CTL;
	rq.ocf = ocf;
	rq.cparam = cparam;
	rq.clen = clen;
	rq.rparam = rparam;
	rq.rlen = rlen;
	return rq;
}

struct hci_request ble_hci_vs_request(uint16_t ocf, void * cparam, int clen, void * rparam, int rlen)
{
	// Create an hci request
	struct hci_request rq;
	memset(&rq, 0, sizeof(rq));
	rq.ogf = OGF_VENDOR_CMD;
	rq.ocf = ocf;
	rq.cparam = cparam;
	rq.clen = clen;
	rq.rparam = rparam;
	rq.rlen = rlen;
	return rq;
}

void close_hci(void)
{
	if (hci_device >= 0) {
		hci_close_dev(hci_device);
		hci_device = -1;
	}
}

void send_le_status_command(uint16_t ocf, void *cparam, int clen, const char *description)
{
	uint8_t status = 0;
	int ret;
	struct hci_request rq = ble_hci_ctl_request(ocf, cparam, clen, &status, sizeof(status));

	ret = hci_send_req(hci_device, &rq, 1000);
	if (ret < 0) {
		perror(description);
		close_hci();
		exit(-1);
	}
	if (status != 0) {
		printf("%s hci req status = 0x%x\r\n", description, status);
		close_hci();
		exit(-1);
	}
}

void wait_for_duration_or_signal(uint32_t duration_us)
{
	if (duration_us == 0) {
		printf("Infinite mode. Press control-c to exit...\r\n");
		while (!stop_requested) {
			usleep(100000);
		}
		return;
	}

	while (!stop_requested && duration_us > 0) {
		uint32_t sleep_us = duration_us > 100000 ? 100000 : duration_us;
		if (usleep(sleep_us) == 0) {
			duration_us -= sleep_us;
		} else if (errno != EINTR) {
			perror("Sleep interrupted");
			break;
		}
	}
}

// cleanup and exit the program with exit code 0
void exit_with_results()
{
	// end the test and report packet count
	int ret;
	uint8_t reset;

	le_test_end_rp test_end_rp;
	vs_SiliconLabs_Get_Counters_cp get_counters_cp;
	struct hci_request le_test_end_rq = ble_hci_ctl_request(OCF_LE_TEST_END, NULL, 0, &test_end_rp, sizeof(test_end_rp));

	ret = hci_send_req(hci_device, &le_test_end_rq, 1000);
	if ( ret < 0 ) {
		perror("Failed to end the test.");
		close_hci();
		exit(-1);
	}

	if (test_end_rp.status == 0) {
		// print Results.
		if (app_state == dtm_tx_begin) {
			// Here we will use the vendor specific command to read the tx counters
			memset(&get_counters_cp, 0, sizeof(get_counters_cp));
			reset = 0;
			struct hci_request read_counters_rq = ble_hci_vs_request(HCI_VS_SiliconLabs_Get_Counters_OCF, &reset, 
												sizeof(reset), &get_counters_cp, sizeof(get_counters_cp));

			ret = hci_send_req(hci_device, &read_counters_rq, 1000);
			if ( ret < 0 ) {
				perror("Failed to read counters");
				close_hci();
				exit(-1);
			}
			if (get_counters_cp.status != 0) {
				printf("HCI_VS_SiliconLabs_Get_Counters hci req status = 0x%x\r\n", get_counters_cp.status);
				close_hci();
				exit(-1);
			}
			printf("Test completed successfully. Number of packets transmitted = %d\r\n", 
					get_counters_cp.tx_packets);
		} else {
			printf("Test completed successfully. Number of packets received = %d\r\n", 
					test_end_rp.num_pkts);
		}
	} else {
		printf("OCF_LE_TEST_END error status=0x%x\r\n", test_end_rp.status);
	}

	close_hci();
	exit( 0 );
}

// handle interrupt (control-c)
void signal_handler( int s )
{
	(void) s;
	stop_requested = TRUE;
}

int main(int argc, char *argv[])
{
	int ret;
	uint8_t status;
	int opt;
	int option_index = 0;
	char *temp;
	int hci_port = -1;

	// Process command line options.
	while ((opt = getopt_long(argc, argv, OPTSTRING, long_options, &option_index)) != -1) {
		switch (opt) {
		// Print help.
		case 'h':
			printf(USAGE, argv[0]);
			exit(EXIT_SUCCESS);

		case 'v':
		case LONG_OPT_VERSION:
			printf("%s version %u.%u\n",argv[0],VERSION_MAJ,VERSION_MIN);
			break;

		case LONG_OPT_TIME:
			duration_usec = atoi(optarg)*1000;
			break;

		case LONG_OPT_PACKET_TYPE:
			/* packet / modulation type */
			packet_type = (uint8_t) strtoul(optarg, &temp, 0);
			break;

		case LONG_OPT_POWER:
			power_level = atoi(optarg);
			if (power_level>20)
			{
			  printf("Error in power level: max value 20 dBm\n");
			  exit(EXIT_FAILURE);
			}
			break;

		case LONG_OPT_CHANNEL:
			channel = atoi(optarg);
			if (channel>39)
			{
			  printf("Error in channel: max value 39\n");
			  exit(EXIT_FAILURE);
			}
			break;

		case LONG_OPT_LEN:
			packet_length = atoi(optarg);
			break;

		case LONG_OPT_RX:
			/* DTM receive */
			app_state = dtm_rx_begin;
			break;

		case LONG_OPT_PHY:
			/* Select PHY for test packets/waveforms */
			selected_phy = atoi(optarg);
			if (selected_phy != 1 && selected_phy != 2 &&
			selected_phy != 3 && selected_phy != 4 ) {
				printf("Error! Invalid phy argument, 0x%02x\n",selected_phy);
				exit(EXIT_FAILURE);
			}
			break;

		case LONG_OPT_PORT:
			/* Select HCI port */
			hci_port = atoi(optarg);
			break;

		case LONG_OPT_ADV:
			advertising_name = optarg;
			app_state = adv_begin;
			break;

		case LONG_OPT_ADVSCAN:
			app_state = advscan_begin;
			break;

		default:
			break;
		}
	}

	// Install a signal handler so we can exit gracefully on control-c and print results 
	if ( signal( SIGINT, signal_handler ) == SIG_ERR )
	{
		close_hci();
		perror( "Could not install signal handler\n" );
		return 0;
	}

	// Install a signal handler so that we can exit gracefully if terminated
	if ( signal( SIGTERM, signal_handler ) == SIG_ERR )
	{
		close_hci();
		perror( "Could not install signal handler\n" );
		exit(-1);
	}

	// Open HCI device at specified HCI port
	if (hci_port == -1) {
		printf("HCI port not specified: use --hci_port to specify the number of the hci port\r\n");
		exit(-1);
	}
	printf("Opening hci port %d\r\n", hci_port);
	hci_device = hci_open_dev(hci_port);
	if ( hci_device < 0 ) {
		perror("Failed to open HCI device");
		exit(-1);
		
	}

	// Reset device
	struct hci_request reset_rq;
	memset(&reset_rq,0,sizeof(reset_rq));
	reset_rq.ogf = OGF_HOST_CTL;
	reset_rq.ocf = OCF_RESET;
	reset_rq.rparam = &status;
	reset_rq.rlen=1;
	ret = hci_send_req(hci_device,&reset_rq,1000);
	if (ret < 0 ) {
		perror(" ERROR: Failed to reset.");
		close_hci();
		return 0;
	}
	if (status != 0) {
		printf("OCF_RESET error status=0x%x\r\n", status);
		close_hci();
		exit(-1);
	}

	switch (app_state) {
		case dtm_tx_begin:
			printf("Outputting modulation type 0x%02X for %u ms at %d MHz at %d dBm, phy=0x%02X\n",
				packet_type, duration_usec/1000, 2402+(2*channel), power_level,
				selected_phy);
			start_tx(channel, packet_length, packet_type, selected_phy, power_level);
			wait_for_duration_or_signal(duration_usec);
			exit_with_results();
			break;

		case dtm_rx_begin:
		 	printf("DTM receive enabled, freq=%d MHz, phy=0x%02X\n",2402+(2*channel), selected_phy);
			start_rx(channel, selected_phy);
			wait_for_duration_or_signal(duration_usec);
			exit_with_results();
			break;

		case adv_begin:
			printf("Advertising Complete Local Name \"%s\" for %u ms\r\n",
					advertising_name, duration_usec/1000);
			start_advertising(advertising_name);
			wait_for_duration_or_signal(duration_usec);
			stop_advertising();
			printf("Advertising stopped\r\n");
			close_hci();
			return 0;

		case advscan_begin:
			printf("Scanning for advertisements for %u ms\r\n", duration_usec/1000);
			scan_advertisements(duration_usec);
			close_hci();
			return 0;

		default:
			break;
	}	

	return 0;
}

void start_tx(uint8_t channel, uint8_t len, uint8_t packet_type, uint8_t phy, int8_t power) {
	// Set LE transmitter test parameters using the v4 tx test command
	uint8_t status;
	int ret;
	uint8_t reset;

	vs_SiliconLabs_Get_Counters_cp get_counters_cp;
	le_transmitter_test_v4_cp tx_test_params_cp;

	// first use vendor specific command to reset counters
	memset(&get_counters_cp, 0, sizeof(get_counters_cp));
	reset = 1;
	struct hci_request reset_counters_rq = ble_hci_vs_request(HCI_VS_SiliconLabs_Get_Counters_OCF, &reset, 
										sizeof(reset), &get_counters_cp, sizeof(get_counters_cp));

	ret = hci_send_req(hci_device, &reset_counters_rq, 1000);
	if ( ret < 0 ) {
		perror("Failed to get counters");
		close_hci();
		exit(-1);
	}
	if (get_counters_cp.status != 0) {
		printf("HCI_VS_SiliconLabs_Get_Counters hci req status = 0x%x\r\n", get_counters_cp.status);
		close_hci();
		exit(-1);
	}
	
	memset(&tx_test_params_cp, 0, sizeof(tx_test_params_cp));
	tx_test_params_cp.frequency 			= channel;
	tx_test_params_cp.phy 			= phy;
	tx_test_params_cp.length 		= len;
	tx_test_params_cp.payload 			= packet_type;
	tx_test_params_cp.tx_power_level 			= power;

	struct hci_request tx_test_rq = ble_hci_ctl_request(OCF_LE_TRANSMITTER_TEST_V4, &tx_test_params_cp, 
							LE_TRANSMITTER_TEST_V4_CP_SIZE, &status, sizeof(status));

	ret = hci_send_req(hci_device, &tx_test_rq, 1000);
	if ( ret < 0 ) {
		perror("Failed to set transmit test data.");
		close_hci();
		exit(-1);
	}
	if (status != 0) {
		printf("start_tx hci req status = 0x%x\r\n",status);
		close_hci();
		exit(-1);
	}
}

void start_rx(uint8_t channel, uint8_t phy) {
	// set LE receiver test parameters using the v2 command

	uint8_t status;
	int ret;

	le_receiver_test_v2_cp rx_test_params_cp;
	memset(&rx_test_params_cp, 0, sizeof(rx_test_params_cp));
	rx_test_params_cp.frequency 	= channel;
	rx_test_params_cp.phy 			= phy;
	struct hci_request rx_test_rq = ble_hci_ctl_request(OCF_LE_RECEIVER_TEST_V2, &rx_test_params_cp, LE_RECEIVER_TEST_V2_CP_SIZE, 
										&status, sizeof(status));

	ret = hci_send_req(hci_device, &rx_test_rq, 1000);
	if ( ret < 0 ) {
		perror("Failed to set transmit test data.");
		close_hci();
		exit(-1);
	}
	if (status != 0) {
		printf("start_rx hci req status = 0x%x\r\n", status);
		close_hci();
		exit(-1);
	}
}

void start_advertising(const char *name)
{
	app_le_set_advertising_parameters_cp adv_params_cp;
	app_le_set_advertising_data_cp adv_data_cp;
	app_le_set_advertise_enable_cp adv_enable_cp;
	size_t name_len;

	if (name == NULL) {
		printf("Missing advertising name\r\n");
		close_hci();
		exit(-1);
	}

	name_len = strlen(name);
	if (name_len > (MAX_LE_ADV_DATA_LEN - 2)) {
		printf("Advertising name too long: max %d bytes for Complete Local Name\r\n",
				MAX_LE_ADV_DATA_LEN - 2);
		close_hci();
		exit(-1);
	}

	memset(&adv_enable_cp, 0, sizeof(adv_enable_cp));
	send_le_status_command(OCF_LE_SET_ADVERTISE_ENABLE,
			&adv_enable_cp,
			APP_LE_SET_ADVERTISE_ENABLE_CP_SIZE,
			"Failed to disable advertising");

	memset(&adv_params_cp, 0, sizeof(adv_params_cp));
	adv_params_cp.min_interval = htobs(DEFAULT_ADV_INTERVAL);
	adv_params_cp.max_interval = htobs(DEFAULT_ADV_INTERVAL);
	adv_params_cp.adv_type = 0x00; // ADV_IND, connectable undirected advertising.
	adv_params_cp.own_bdaddr_type = 0x00;
	adv_params_cp.direct_bdaddr_type = 0x00;
	adv_params_cp.channel_map = DEFAULT_ADV_CHANNEL_MAP;
	adv_params_cp.filter_policy = 0x00;
	send_le_status_command(OCF_LE_SET_ADVERTISING_PARAMETERS,
			&adv_params_cp,
			APP_LE_SET_ADVERTISING_PARAMETERS_CP_SIZE,
			"Failed to set advertising parameters");

	memset(&adv_data_cp, 0, sizeof(adv_data_cp));
	adv_data_cp.length = (uint8_t)(name_len + 2);
	adv_data_cp.data[0] = (uint8_t)(name_len + 1);
	adv_data_cp.data[1] = ADV_TYPE_COMPLETE_LOCAL_NAME;
	memcpy(&adv_data_cp.data[2], name, name_len);
	send_le_status_command(OCF_LE_SET_ADVERTISING_DATA,
			&adv_data_cp,
			APP_LE_SET_ADVERTISING_DATA_CP_SIZE,
			"Failed to set advertising data");

	adv_enable_cp.enable = TRUE;
	send_le_status_command(OCF_LE_SET_ADVERTISE_ENABLE,
			&adv_enable_cp,
			APP_LE_SET_ADVERTISE_ENABLE_CP_SIZE,
			"Failed to enable advertising");
}

void stop_advertising(void)
{
	app_le_set_advertise_enable_cp adv_enable_cp;

	if (hci_device < 0) {
		return;
	}

	memset(&adv_enable_cp, 0, sizeof(adv_enable_cp));
	send_le_status_command(OCF_LE_SET_ADVERTISE_ENABLE,
			&adv_enable_cp,
			APP_LE_SET_ADVERTISE_ENABLE_CP_SIZE,
			"Failed to disable advertising");
}

void set_scan_enable(uint8_t enable)
{
	app_le_set_scan_enable_cp scan_enable_cp;

	memset(&scan_enable_cp, 0, sizeof(scan_enable_cp));
	scan_enable_cp.enable = enable;
	scan_enable_cp.filter_dup = FALSE;
	send_le_status_command(OCF_LE_SET_SCAN_ENABLE,
			&scan_enable_cp,
			APP_LE_SET_SCAN_ENABLE_CP_SIZE,
			enable ? "Failed to enable scanning" : "Failed to disable scanning");
}

uint64_t monotonic_ms(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0) {
		perror("clock_gettime");
		close_hci();
		exit(-1);
	}

	return ((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u);
}

void scan_advertisements(uint32_t duration_us)
{
	app_le_set_scan_parameters_cp scan_params_cp;
	struct hci_filter original_filter;
	struct hci_filter scan_filter;
	socklen_t original_filter_len = sizeof(original_filter);
	uint64_t end_ms = 0;

	memset(&scan_params_cp, 0, sizeof(scan_params_cp));
	scan_params_cp.scan_type = 0x00; // Passive scanning.
	scan_params_cp.interval = htobs(DEFAULT_SCAN_INTERVAL);
	scan_params_cp.window = htobs(DEFAULT_SCAN_WINDOW);
	scan_params_cp.own_bdaddr_type = 0x00;
	scan_params_cp.filter_policy = 0x00;
	send_le_status_command(OCF_LE_SET_SCAN_PARAMETERS,
			&scan_params_cp,
			APP_LE_SET_SCAN_PARAMETERS_CP_SIZE,
			"Failed to set scan parameters");

	if (getsockopt(hci_device, SOL_HCI, HCI_FILTER,
			&original_filter, &original_filter_len) < 0) {
		perror("Failed to get HCI filter");
		close_hci();
		exit(-1);
	}

	hci_filter_clear(&scan_filter);
	hci_filter_set_ptype(HCI_EVENT_PKT, &scan_filter);
	hci_filter_set_event(EVT_LE_META_EVENT, &scan_filter);
	if (setsockopt(hci_device, SOL_HCI, HCI_FILTER,
			&scan_filter, sizeof(scan_filter)) < 0) {
		perror("Failed to set HCI filter");
		close_hci();
		exit(-1);
	}

	set_scan_enable(TRUE);
	if (duration_us == 0) {
		printf("Infinite mode. Press control-c to exit...\r\n");
	} else {
		end_ms = monotonic_ms() + (duration_us / 1000u);
	}

	while (!stop_requested) {
		fd_set read_fds;
		struct timeval timeout;
		struct timeval *timeout_ptr = NULL;
		int select_ret;
		uint8_t event_buf[HCI_EVENT_BUFFER_SIZE];
		ssize_t event_len;

		if (duration_us != 0) {
			uint64_t now_ms = monotonic_ms();
			uint64_t remaining_ms;
			if (now_ms >= end_ms) {
				break;
			}
			remaining_ms = end_ms - now_ms;
			if (remaining_ms > 1000u) {
				remaining_ms = 1000u;
			}
			timeout.tv_sec = (time_t)(remaining_ms / 1000u);
			timeout.tv_usec = (suseconds_t)((remaining_ms % 1000u) * 1000u);
			timeout_ptr = &timeout;
		}

		FD_ZERO(&read_fds);
		FD_SET(hci_device, &read_fds);
		select_ret = select(hci_device + 1, &read_fds, NULL, NULL, timeout_ptr);
		if (select_ret < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("Failed while waiting for advertisements");
			break;
		}
		if (select_ret == 0) {
			continue;
		}

		event_len = read(hci_device, event_buf, sizeof(event_buf));
		if (event_len < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("Failed to read advertising report");
			break;
		}

		print_advertising_reports(event_buf, event_len);
	}

	set_scan_enable(FALSE);
	if (setsockopt(hci_device, SOL_HCI, HCI_FILTER,
			&original_filter, original_filter_len) < 0) {
		perror("Failed to restore HCI filter");
	}
}

const char *ad_type_name(uint8_t type)
{
	switch (type) {
		case 0x01: return "Flags";
		case 0x02: return "Incomplete List of 16-bit Service UUIDs";
		case 0x03: return "Complete List of 16-bit Service UUIDs";
		case 0x04: return "Incomplete List of 32-bit Service UUIDs";
		case 0x05: return "Complete List of 32-bit Service UUIDs";
		case 0x06: return "Incomplete List of 128-bit Service UUIDs";
		case 0x07: return "Complete List of 128-bit Service UUIDs";
		case 0x08: return "Shortened Local Name";
		case 0x09: return "Complete Local Name";
		case 0x0A: return "TX Power Level";
		case 0x0D: return "Class of Device";
		case 0x0E: return "Simple Pairing Hash C-192";
		case 0x0F: return "Simple Pairing Randomizer R-192";
		case 0x10: return "Security Manager TK Value";
		case 0x11: return "Security Manager Out of Band Flags";
		case 0x12: return "Peripheral Connection Interval Range";
		case 0x14: return "List of 16-bit Service Solicitation UUIDs";
		case 0x15: return "List of 128-bit Service Solicitation UUIDs";
		case 0x16: return "Service Data - 16-bit UUID";
		case 0x17: return "Public Target Address";
		case 0x18: return "Random Target Address";
		case 0x19: return "Appearance";
		case 0x1A: return "Advertising Interval";
		case 0x1B: return "LE Bluetooth Device Address";
		case 0x1C: return "LE Role";
		case 0x1D: return "Simple Pairing Hash C-256";
		case 0x1E: return "Simple Pairing Randomizer R-256";
		case 0x1F: return "List of 32-bit Service Solicitation UUIDs";
		case 0x20: return "Service Data - 32-bit UUID";
		case 0x21: return "Service Data - 128-bit UUID";
		case 0x22: return "LE Secure Connections Confirmation Value";
		case 0x23: return "LE Secure Connections Random Value";
		case 0x24: return "URI";
		case 0x25: return "Indoor Positioning";
		case 0x26: return "Transport Discovery Data";
		case 0x27: return "LE Supported Features";
		case 0x28: return "Channel Map Update Indication";
		case 0x29: return "PB-ADV";
		case 0x2A: return "Mesh Message";
		case 0x2B: return "Mesh Beacon";
		case 0x2C: return "BIGInfo";
		case 0x2D: return "Broadcast Code";
		case 0x2E: return "Resolvable Set Identifier";
		case 0x2F: return "Advertising Interval - long";
		case 0x30: return "Broadcast Name";
		case 0x31: return "Encrypted Advertising Data";
		case 0x32: return "Periodic Advertising Response Timing Information";
		case 0x34: return "Electronic Shelf Label";
		case 0xFF: return "Manufacturer Specific Data";
		default: return "Unknown";
	}
}

void append_to_ad_type_list(char *out, size_t out_len, size_t *used,
		const char *separator, const char *name, uint8_t type)
{
	int written;

	if (*used >= out_len) {
		return;
	}

	written = snprintf(out + *used, out_len - *used,
			"%s%s (0x%02X)", separator, name, type);
	if (written < 0) {
		return;
	}
	if ((size_t)written >= (out_len - *used)) {
		*used = out_len - 1;
	} else {
		*used += (size_t)written;
	}
}

void format_ad_type_list(const uint8_t *data, uint8_t data_len, char *out, size_t out_len)
{
	size_t used = 0;
	uint8_t index = 0;
	uint8_t first = TRUE;

	if (out_len == 0) {
		return;
	}
	out[0] = '\0';

	while (index < data_len) {
		uint8_t field_len = data[index++];
		uint8_t type;

		if (field_len == 0) {
			break;
		}
		if ((index + field_len) > data_len) {
			append_to_ad_type_list(out, out_len, &used,
					first ? "" : ", ", "Malformed AD structure", 0x00);
			first = FALSE;
			break;
		}

		type = data[index];
		append_to_ad_type_list(out, out_len, &used,
				first ? "" : ", ", ad_type_name(type), type);
		first = FALSE;
		index = (uint8_t)(index + field_len);
	}

	if (first) {
		snprintf(out, out_len, "none");
	}
}

void print_advertising_reports(const uint8_t *buf, ssize_t len)
{
	const hci_event_hdr *event_hdr;
	const uint8_t *ptr;
	size_t remaining;
	uint8_t reports;
	uint8_t report_index;

	if (len < (ssize_t)(1 + HCI_EVENT_HDR_SIZE + 2)) {
		return;
	}
	if (buf[0] != HCI_EVENT_PKT) {
		return;
	}

	event_hdr = (const hci_event_hdr *)(buf + 1);
	if (event_hdr->evt != EVT_LE_META_EVENT) {
		return;
	}

	remaining = (size_t)len - 1u - HCI_EVENT_HDR_SIZE;
	if (event_hdr->plen < remaining) {
		remaining = event_hdr->plen;
	}

	ptr = buf + 1 + HCI_EVENT_HDR_SIZE;
	if (remaining < 2 || ptr[0] != EVT_LE_ADVERTISING_REPORT) {
		return;
	}
	ptr++;
	remaining--;

	reports = *ptr++;
	remaining--;

	for (report_index = 0; report_index < reports; report_index++) {
		bdaddr_t bdaddr;
		char addr[18];
		char ad_types[512];
		uint8_t data_len;
		const uint8_t *ad_data;
		int8_t rssi;

		if (remaining < 10) {
			break;
		}

		ptr += 2; // Event_Type and Address_Type.
		remaining -= 2;

		memcpy(&bdaddr, ptr, sizeof(bdaddr));
		ptr += sizeof(bdaddr);
		remaining -= sizeof(bdaddr);

		data_len = *ptr++;
		remaining--;
		if ((size_t)data_len + 1u > remaining) {
			break;
		}

		ad_data = ptr;
		ptr += data_len;
		remaining -= data_len;
		rssi = (int8_t)*ptr++;
		remaining--;

		ba2str(&bdaddr, addr);
		format_ad_type_list(ad_data, data_len, ad_types, sizeof(ad_types));
		printf("%s RSSI %d dBm AD types: %s\r\n", addr, rssi, ad_types);
		fflush(stdout);
	}
}

void set_power(int16_t power_ddbm) {
	// set power level using EFR32 vendor specific command
	// Refer to AN1328 HCI_VS_SiliconLabs_Set_Min_Max_TX_Power

	uint8_t status;
	int ret;
	struct tx_power_struct {
		int16_t min_tx_power;
		int16_t max_tx_power;
	} tx_power_cp;
	tx_power_cp.min_tx_power = power_ddbm;
	tx_power_cp.max_tx_power = power_ddbm;

	struct hci_request set_tx_power_rq = ble_hci_vs_request(HCI_VS_SiliconLabs_Set_Min_Max_TX_Power_OCF, &tx_power_cp, 
											sizeof(tx_power_cp), &status, sizeof(status));

	ret = hci_send_req(hci_device, &set_tx_power_rq, 1000);
	if ( ret < 0 ) {
		perror("Failed to set tx power");
		close_hci();
		exit(-1);
	}
	if (status != 0) {
		printf("set_power hci req status = 0x%x\r\n", status);
		close_hci();
		exit(-1);
	}

}

void get_power_config(void) {
	// get current tx power configuration and print to console

	int ret;
	struct tx_power_config_rp_struct {
		uint8_t status;
		int16_t min_supported_tx_power;
		int16_t max_supported_tx_power;
		int16_t min_configured_tx_power;
		int16_t max_configured_tx_power;
		int16_t tx_rf_path_compensation;
	} __attribute__ ((packed)) txp_config_rp;

	struct hci_request set_tx_power_rq = ble_hci_vs_request(HCI_VS_SiliconLabs_Read_Current_TX_Power_Configuration_OCF, NULL, 0, 
										&txp_config_rp, sizeof(txp_config_rp));

	ret = hci_send_req(hci_device, &set_tx_power_rq, 1000);
	if ( ret < 0 ) {
		perror("Failed to set tx power");
		close_hci();
		exit(-1);
	}
	if (txp_config_rp.status != 0) {
		printf("get tx power config hci req status = %d\r\n", txp_config_rp.status);
		close_hci();
		exit(-1);
	} else {
		printf("min supported tx power = %d\r\n", txp_config_rp.min_supported_tx_power);
		printf("max supported tx power = %d\r\n", txp_config_rp.max_supported_tx_power);
		printf("min configured tx power = %d\r\n", txp_config_rp.min_configured_tx_power);
		printf("max configured tx power = %d\r\n", txp_config_rp.max_configured_tx_power);
		printf("tx RF path compensation = %d\r\n", txp_config_rp.tx_rf_path_compensation);
	}
}
