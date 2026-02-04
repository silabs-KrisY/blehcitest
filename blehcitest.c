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
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <sys/ioctl.h>
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

#define LONG_OPT_VERSION 0
#define LONG_OPT_TIME 1
#define LONG_OPT_PACKET_TYPE 2
#define LONG_OPT_POWER 3
#define LONG_OPT_CHANNEL 4
#define LONG_OPT_LEN 5
#define LONG_OPT_RX 6
#define LONG_OPT_PHY 7
#define LONG_OPT_PORT 8
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
		{"help",   	   no_argument, 0,  LONG_OPT_HELP },
		{0,           0,                 0,  0  }
		};

#define VERSION_MAJ	0u
#define VERSION_MIN	3u

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

static int hci_device;

/* application state machine */
static enum app_states {
  dtm_rx_begin,
  dtm_tx_begin
} app_state =  dtm_tx_begin; //default to TX

/* Prototypes */
void set_power(int16_t power_ddbm);
void start_tx(uint8_t channel, uint8_t len, uint8_t packet_type, uint8_t phy, int8_t power);
void start_rx(uint8_t channel, uint8_t phy);
void get_power_config(void);
void exit_with_results(void);


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
		hci_close_dev(hci_device);
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
				hci_close_dev(hci_device);
				exit(-1);
			}
			if (get_counters_cp.status != 0) {
				printf("HCI_VS_SiliconLabs_Get_Counters hci req status = 0x%x\r\n", get_counters_cp.status);
				hci_close_dev(hci_device);
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

	hci_close_dev(hci_device);
	exit( 0 );
}

// handle interrupt (control-c)
void signal_handler( int s )
{
	(void) s;
	exit_with_results();
}

int main(int argc, char *argv[])
{
	int ret, status;
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
			printf("%s version %d.%d\n",argv[0],VERSION_MAJ,VERSION_MIN);
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

		default:
			break;
		}
	}

	// Install a signal handler so we can exit gracefully on control-c and print results 
	if ( signal( SIGINT, signal_handler ) == SIG_ERR )
	{
		hci_close_dev(hci_device);
		perror( "Could not install signal handler\n" );
		return 0;
	}

	// Install a signal handler so that we can exit gracefully if terminated
	if ( signal( SIGTERM, signal_handler ) == SIG_ERR )
	{
		hci_close_dev(hci_device);
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
		return 0;
		}

	switch (app_state) {
		case dtm_tx_begin:
			printf("Outputting modulation type 0x%02X for %d ms at %d MHz at %d dBm, phy=0x%02X\n",
				packet_type, duration_usec/1000, 2402+(2*channel), power_level,
				selected_phy);
			start_tx(channel, packet_length, packet_type, selected_phy, power_level);
			break;

		case dtm_rx_begin:
		 	printf("DTM receive enabled, freq=%d MHz, phy=0x%02X\n",2402+(2*channel), selected_phy);
			start_rx(channel, selected_phy);
			break;
		
		default:
			break;
	}	

	// pause
	if (duration_usec != 0) {
	usleep(duration_usec);	/* sleep during test */
	} else {
		// Infinite mode
		printf("Infinite mode. Press control-c to exit...\r\n");
		while(1) {
		// wait here for control-c, sleeping periodically to save host CPU cycles
		usleep(1000);
		}
	}

	exit_with_results();
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
		hci_close_dev(hci_device);
		exit(-1);
	}
	if (get_counters_cp.status != 0) {
		printf("HCI_VS_SiliconLabs_Get_Counters hci req status = 0x%x\r\n", get_counters_cp.status);
		hci_close_dev(hci_device);
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
		hci_close_dev(hci_device);
		exit(-1);
	}
	if (status != 0) {
		printf("start_tx hci req status = 0x%x\r\n",status);
		hci_close_dev(hci_device);
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
		hci_close_dev(hci_device);
		exit(-1);
	}
	if (status != 0) {
		printf("start_rx hci req status = 0x%x\r\n", status);
		hci_close_dev(hci_device);
		exit(-1);
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
		hci_close_dev(hci_device);
		exit(-1);
	}
	if (status != 0) {
		printf("set_power hci req status = 0x%x\r\n", status);
		hci_close_dev(hci_device);
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
		hci_close_dev(hci_device);
		exit(-1);
	}
	if (txp_config_rp.status != 0) {
		printf("get tx power config hci req status = %d\r\n", txp_config_rp.status);
		hci_close_dev(hci_device);
		exit(-1);
	} else {
		printf("min supported tx power = %d\r\n", txp_config_rp.min_supported_tx_power);
		printf("max supported tx power = %d\r\n", txp_config_rp.max_supported_tx_power);
		printf("min configured tx power = %d\r\n", txp_config_rp.min_configured_tx_power);
		printf("max configured tx power = %d\r\n", txp_config_rp.max_configured_tx_power);
		printf("tx RF path compensation = %d\r\n", txp_config_rp.tx_rf_path_compensation);
	}
}
