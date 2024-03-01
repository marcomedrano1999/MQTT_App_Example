/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_RAW && LWIP_NETCONN && LWIP_DHCP && LWIP_DNS

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_phy.h"

#include "lwip/api.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dhcp.h"
#include "lwip/netdb.h"
#include "lwip/netifapi.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "lwip_mqtt_id.h"

#include "ctype.h"
#include "stdio.h"

#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"
#include "fsl_device_registers.h"
#include "fsl_rnga.h"
#include "fsl_pit.h"
#include "math.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @TEST_ANCHOR */

/* MAC address configuration. */
#ifndef configMAC_ADDR
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x1B \
    }
#endif

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

/* GPIO pin configuration. */
#define BOARD_LED_GPIO       BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN   BOARD_LED_RED_GPIO_PIN
#define BOARD_SW_GPIO        BOARD_SW3_GPIO
#define BOARD_SW_GPIO_PIN    BOARD_SW3_GPIO_PIN
#define BOARD_SW_PORT        BOARD_SW3_PORT
#define BOARD_SW_IRQ         BOARD_SW3_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW3_IRQ_HANDLER


#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/*! @brief MQTT server host name or IP address. */
#define EXAMPLE_MQTT_SERVER_HOST "broker.hivemq.com"

/*! @brief MQTT server port number. */
#define EXAMPLE_MQTT_SERVER_PORT 1883

/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*! @brief Stack size of the temporary initialization thread. */
#define APP_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary initialization thread. */
#define APP_THREAD_PRIO DEFAULT_THREAD_PRIO


/*
 * Robot app definitions
 */

#define ROBOT_RETURN_TO_BASE_EVENT		(1 << 0)

#define ROBOT_REACH_POSITION			(1 << 1)

#define ROBOT_REACH_BASE				(1 << 2)


#define BATTERY_PIT_BASEADDR PIT
#define BATTERY_PIT_CHANNEL  kPIT_Chnl_0
#define PIT_LED_HANDLER   PIT0_IRQHandler
#define PIT_IRQ_ID        PIT0_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define LED_INIT()       LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE()     LED_RED_TOGGLE()

#define MAX_STEP_TIME		20
#define X_BASE				0
#define Y_BASE				0

#define ROBOT_TRASH_STORAGE_MAXIMUM			50
#define ROBOT_BATTERY_LEVEL_MAXIMUM			10000

typedef enum {
	ROBOT_STATE_BASE,
	ROBOT_STATE_COLLECTING,
	ROBOT_STATE_RETURNING_TO_BASE
}Robot_State_t;

typedef enum {
	ROBOT_TOPIC_USER_SET_ROBOT_STATE,
	ROBOT_TOPIC_USER_SET_COORDS
}Robot_Topics_t;


typedef struct
{
	uint32_t x;
	uint32_t y;
}Coords;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void connect_to_mqtt(void *ctx);

inline uint32_t getDistance(Coords base_coords,Coords new_coords);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle   = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

/*! @brief MQTT client data. */
static mqtt_client_t *mqtt_client;

/*! @brief MQTT client ID string. */
static char client_id[40];

/*! @brief MQTT client information. */
static const struct mqtt_connect_client_info_t mqtt_client_info = {
    .client_id   = (const char *)&client_id[0],
    .client_user = NULL,
    .client_pass = NULL,
    .keep_alive  = 100,
    .will_topic  = NULL,
    .will_msg    = NULL,
    .will_qos    = 0,
    .will_retain = 0,
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    .tls_config = NULL,
#endif
};

/*! @brief MQTT broker IP address. */
static ip_addr_t mqtt_addr;

/*! @brief Indicates connection to MQTT broker. */
static volatile bool connected = false;

/*
 * APP Variables
 */

uint8_t topic_data[10][40];
uint8_t topic_name[10];

volatile uint8_t subscriber_idx=0;
uint8_t saveData = 0;
uint8_t isMessageAvailable=0;

/*******************************************************************************
 * Code
 ******************************************************************************/


/*
 * Computes the distance between two points
 */
uint32_t getDistance(Coords base_coords,Coords new_coords)
{
	uint32_t distance = sqrt(pow((double)base_coords.x - (double)new_coords.x, 2) + pow((double)base_coords.y - (double)new_coords.y, 2));
	return distance;
}






/*!
 * @brief Called when subscription request finishes.
 */
static void mqtt_topic_subscribed_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Subscribed to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to subscribe to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Called when there is a message on a subscribed topic.
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    LWIP_UNUSED_ARG(arg);

    PRINTF("Received %u bytes from the topic \"%s\": \"", tot_len, topic);

    // We'll save the topic name
    if(strcmp(topic,"med_p2024/lwip_topic/user_set_robot_state")==0)
    {
    	saveData = 1;
    	topic_name[subscriber_idx] = ROBOT_TOPIC_USER_SET_ROBOT_STATE;
    }
    else if(strcmp(topic,"med_p2024/lwip_topic/user_coords")==0)
    {
    	saveData = 1;
    	topic_name[subscriber_idx] = ROBOT_TOPIC_USER_SET_COORDS;
    }


}

/*!
 * @brief Called when recieved incoming published message fragment.
 */
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    int i;

    LWIP_UNUSED_ARG(arg);

    if(saveData)
    {
    	saveData = 0;

    	// Clean buffer if there is any data
    	memset(topic_data[subscriber_idx],0, 40);

    	// Copy data to buffer
    	memcpy(topic_data[subscriber_idx],data, len);

    	// Increment idx
    	subscriber_idx = (subscriber_idx+1) % 10;

    }

    for (i = 0; i < len; i++)
    {
        if (isprint(data[i]))
        {
            PRINTF("%c", (char)data[i]);
        }
        else
        {
            PRINTF("\\x%02x", data[i]);
        }
    }

    if (flags & MQTT_DATA_FLAG_LAST)
    {
        PRINTF("\"\r\n");
    }
}

/*!
 * @brief Subscribe to MQTT topics.
 */
static void mqtt_subscribe_topics(mqtt_client_t *client)
{
    static const char *topics[] = {"med_p2024/lwip_topic/#"};
    int qos[]                   = {1};
    err_t err;
    int i;

    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb,
                            LWIP_CONST_CAST(void *, &mqtt_client_info));

    for (i = 0; i < ARRAY_SIZE(topics); i++)
    {
        err = mqtt_subscribe(client, topics[i], qos[i], mqtt_topic_subscribed_cb, LWIP_CONST_CAST(void *, topics[i]));

        if (err == ERR_OK)
        {
            PRINTF("Subscribing to the topic \"%s\" with QoS %d...\r\n", topics[i], qos[i]);
        }
        else
        {
            PRINTF("Failed to subscribe to the topic \"%s\" with QoS %d: %d.\r\n", topics[i], qos[i], err);
        }
    }
}

/*!
 * @brief Called when connection state changes.
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    const struct mqtt_connect_client_info_t *client_info = (const struct mqtt_connect_client_info_t *)arg;

    connected = (status == MQTT_CONNECT_ACCEPTED);

    switch (status)
    {
        case MQTT_CONNECT_ACCEPTED:
            PRINTF("MQTT client \"%s\" connected.\r\n", client_info->client_id);
            mqtt_subscribe_topics(client);
            break;

        case MQTT_CONNECT_DISCONNECTED:
            PRINTF("MQTT client \"%s\" not connected.\r\n", client_info->client_id);
            /* Try to reconnect 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_TIMEOUT:
            PRINTF("MQTT client \"%s\" connection timeout.\r\n", client_info->client_id);
            /* Try again 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_REFUSED_PROTOCOL_VERSION:
        case MQTT_CONNECT_REFUSED_IDENTIFIER:
        case MQTT_CONNECT_REFUSED_SERVER:
        case MQTT_CONNECT_REFUSED_USERNAME_PASS:
        case MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_:
            PRINTF("MQTT client \"%s\" connection refused: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;

        default:
            PRINTF("MQTT client \"%s\" connection status: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;
    }
}

/*!
 * @brief Starts connecting to MQTT broker. To be called on tcpip_thread.
 */
static void connect_to_mqtt(void *ctx)
{
    LWIP_UNUSED_ARG(ctx);

    PRINTF("Connecting to MQTT broker at %s...\r\n", ipaddr_ntoa(&mqtt_addr));

    mqtt_client_connect(mqtt_client, &mqtt_addr, EXAMPLE_MQTT_SERVER_PORT, mqtt_connection_cb,
                        LWIP_CONST_CAST(void *, &mqtt_client_info), &mqtt_client_info);
}

/*!
 * @brief Called when publish request finishes.
 */
static void mqtt_message_published_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Published to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to publish to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Publishes a message. To be called on tcpip_thread.
 */
static void publish_message(void *ctx)
{
    static const char *topic   = "med_p2024/lwip_topic/100";
    static const char *message = "message from board";

    LWIP_UNUSED_ARG(ctx);

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);

    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);
}


/*!
 * @brief Publishes a message on topic 'med_p2024/lwip_topic/battery_level'. To be called on tcpip_thread.
 */
void publish_battery_level(void *ctx)
{
	uint32_t *pBattery_level = ctx;
    static const char *topic   = "med_p2024/lwip_topic/battery_level";
    char message[20];

    sprintf(message, "%d",(*pBattery_level));

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);

    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0,
    		mqtt_message_published_cb, (void *)topic);
}

/*!
 * @brief Publishes a message on topic 'med_p2024/lwip_topic/trash_storage_level'. To be called on tcpip_thread.
 */
void publish_trash_storage_level(void *ctx)
{
	uint32_t *pTrash_storage_level = ctx;
    static const char *topic   = "med_p2024/lwip_topic/trash_storage_level";
    char message[20];

    sprintf(message, "%d", (*pTrash_storage_level));

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);

    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);

}

/*!
 * @brief Publishes a message on topic 'med_p2024/lwip_topic/coords'. To be called on tcpip_thread.
 */
void publish_coords(void *ctx)
{
	Coords *pcoords = ctx;
    static const char *topic   = "med_p2024/lwip_topic/coords";
    char message[20];

    sprintf(message, "%d %d", pcoords->x, pcoords->y);

    PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);

    mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);

}

/*!
 * @brief Application thread.
 */
static void app_thread(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    struct dhcp *dhcp;
    err_t err;
    int i;

    /* Wait for address from DHCP */

    PRINTF("Getting IP address from DHCP...\r\n");

    do
    {
        if (netif_is_up(netif))
        {
            dhcp = netif_dhcp_data(netif);
        }
        else
        {
            dhcp = NULL;
        }

        sys_msleep(20U);

    } while ((dhcp == NULL) || (dhcp->state != DHCP_STATE_BOUND));

    PRINTF("\r\nIPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
    PRINTF("IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
    PRINTF("IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));

    /*
     * Check if we have an IP address or host name string configured.
     * Could just call netconn_gethostbyname() on both IP address or host name,
     * but we want to print some info if goint to resolve it.
     */
    if (ipaddr_aton(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr) && IP_IS_V4(&mqtt_addr))
    {
        /* Already an IP address */
        err = ERR_OK;
    }
    else
    {
        /* Resolve MQTT broker's host name to an IP address */
        PRINTF("Resolving \"%s\"...\r\n", EXAMPLE_MQTT_SERVER_HOST);
        err = netconn_gethostbyname(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr);
    }

    if (err == ERR_OK)
    {
        /* Start connecting to MQTT broker from tcpip_thread */
        err = tcpip_callback(connect_to_mqtt, NULL);
        if (err != ERR_OK)
        {
            PRINTF("Failed to invoke broker connection on the tcpip_thread: %d.\r\n", err);
        }
    }
    else
    {
        PRINTF("Failed to obtain IP address: %d.\r\n", err);
    }


    /*
     * Robot App code
     */

	uint32_t battery_level = ROBOT_BATTERY_LEVEL_MAXIMUM;
	uint32_t distance_to_base = 0;
	uint32_t trash_storage_level=0;
	Coords robot_coords;
	Coords base_coords;
	Coords new_coords;
	uint8_t user_set_new_coords=0;
	robot_coords.x = X_BASE;
	robot_coords.y = Y_BASE;
	base_coords.x = X_BASE;
	base_coords.y = Y_BASE;
	Robot_State_t robot_state = ROBOT_STATE_BASE;
	status_t status;
	uint32_t data[2];
	uint8_t app_idx = 0;
	const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;

	// Initialize random number generator for the simulation
	//  of the trash detection
	RNGA_Init(RNG);

	// Init the leds
	LED_RED_INIT(LOGIC_LED_OFF);
	LED_BLUE_INIT(LOGIC_LED_OFF);

	while(1)
	{
		// Check if there are any new messages from MQTT
		if(app_idx != subscriber_idx)
		{
			if(topic_name[app_idx] == ROBOT_TOPIC_USER_SET_ROBOT_STATE)
			{
				if(strcmp(topic_data[app_idx],"0") == 0)
				{
					robot_state = ROBOT_STATE_RETURNING_TO_BASE;
				}
				else if(strcmp(topic_data[app_idx],"1") == 0)
				{
					robot_state = ROBOT_STATE_COLLECTING;
				}

			}
			else if(topic_name[app_idx] == ROBOT_TOPIC_USER_SET_COORDS)
			{
				// Get the idx of the delimiter between the two values
				uint8_t pos = strcspn(topic_data[app_idx], " ");
				char numBuff[40]={0};

				if(pos != strlen(topic_data[app_idx]))
				{
					// Extract the first number
					strncat(numBuff, topic_data[app_idx], pos);

					// Convert it to num
					new_coords.x = atoi(numBuff);
					new_coords.y = atoi(&topic_data[app_idx][pos+1]);

					// Set new user coords
					user_set_new_coords = 1;
				}
			}

			app_idx = (app_idx+1)%10;
		}



		switch(robot_state)
		{
		case ROBOT_STATE_BASE:
			// Turn red led on to indicate that we're on base
			LED_RED_ON();
			LED_BLUE_OFF();
			break;

		case ROBOT_STATE_RETURNING_TO_BASE:
			// Compute the distance between the base and the robot location
			distance_to_base = getDistance(base_coords, robot_coords);

			// Simulate the time it takes for the robot to get back
			for(uint32_t i=0; i<distance_to_base; i++)
			{
				for(uint32_t j=0; j<MAX_STEP_TIME; j++)
				{ }
				// Discounts 5 to the battery level for each step it takes
				battery_level -= 5;
			}
			// Change state to Base
			robot_state = ROBOT_STATE_BASE;
			// Reset the robot coords to the base coords
			robot_coords = base_coords;

			// Reset battery level
			battery_level = ROBOT_BATTERY_LEVEL_MAXIMUM;
			// Reset trash storage level
			trash_storage_level = 0;
			break;

		case ROBOT_STATE_COLLECTING:

			// Turn off red led to indicate that we're no longer on base
			LED_RED_OFF();

			// Check if user send new coords
			if(user_set_new_coords == 1)
			{
				user_set_new_coords = 0;
			}
			else
			{
				// Simulate the localization of a trash piece
				status = RNGA_GetRandomData(RNG, data, sizeof(data));

				if (status != kStatus_Success)
				{
					break;
				}

				// Here we use the remainder of 50 to limit the coordinates
				new_coords.x = data[0] % 50;
				new_coords.y = data[1] % 50;
			}

			// Compute the distance between our location and the new location
			uint32_t distance_to_trash = getDistance(robot_coords, new_coords);

			// Compute the distance between the new location and the base
			distance_to_base = getDistance(base_coords, new_coords);

			// Check if we have enough battery to at least get there and go
			//  back to the base
			if(battery_level < ((distance_to_base + distance_to_trash) * 5))
			{
				// If not, reset the robot state to returning to base
				robot_state = ROBOT_STATE_RETURNING_TO_BASE;
				break;
			}

			// Simulate the time it takes for the robot to get back
			for(uint32_t i=0; i<distance_to_trash; i++)
			{
				for(uint32_t j=0; j<MAX_STEP_TIME; j++)
				{ }

				// Discounts 5 to the battery level for each step it takes
				battery_level -= 5;
			}

			// Simulate the collection of trash by toggling the blue led
			LED_BLUE_TOGGLE();

			// Update the robot coords to the new coords
			robot_coords = new_coords;

			// Increment the trash storage level
			trash_storage_level++;

			// Check if the trash storage level has not reach its maximum
			if(trash_storage_level >= ROBOT_TRASH_STORAGE_MAXIMUM)
			{
				// Switch state to return to base
				robot_state = ROBOT_STATE_RETURNING_TO_BASE;
			}

			break;
		}


		// Publish the battery level
		err = tcpip_callback(publish_battery_level, (void*)&battery_level);

		// Publish the trash storage level
		err = tcpip_callback(publish_trash_storage_level, (void*)&trash_storage_level);

		// Publish the position
		err = tcpip_callback(publish_coords, (void*)&robot_coords);


		// Add some delay to avoid
		vTaskDelay(xDelay);

	}

    vTaskDelete(NULL);
}

static void generate_client_id(void)
{
    uint32_t mqtt_id[MQTT_ID_SIZE];
    int res;

    get_mqtt_id(&mqtt_id[0]);

    res = snprintf(client_id, sizeof(client_id), "nxp_%08lx%08lx%08lx%08lx", mqtt_id[3], mqtt_id[2], mqtt_id[1],
                   mqtt_id[0]);
    if ((res < 0) || (res >= sizeof(client_id)))
    {
        PRINTF("snprintf failed: %d\r\n", res);
        while (1)
        {
        }
    }
}

/*!
 * @brief Initializes lwIP stack.
 *
 * @param arg unused
 */
static void stack_init(void *arg)
{
    static struct netif netif;
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
    ethernetif_config_t enet_config = {
        .phyHandle  = &phyHandle,
        .macAddress = configMAC_ADDR,
    };

    LWIP_UNUSED_ARG(arg);
    generate_client_id();

    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    IP4_ADDR(&netif_ipaddr, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_netmask, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_gw, 0U, 0U, 0U, 0U);

    tcpip_init(NULL, NULL);

    LOCK_TCPIP_CORE();
    mqtt_client = mqtt_client_new();
    UNLOCK_TCPIP_CORE();
    if (mqtt_client == NULL)
    {
        PRINTF("mqtt_client_new() failed.\r\n");
        while (1)
        {
        }
    }

    netifapi_netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN,
                       tcpip_input);
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

    netifapi_dhcp_start(&netif);

    PRINTF("\r\n************************************************\r\n");
    PRINTF(" MQTT client example\r\n");
    PRINTF("************************************************\r\n");

    if (sys_thread_new("app_task", app_thread, &netif, APP_THREAD_STACKSIZE, APP_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("stack_init(): Task creation failed.", 0);
    }

    vTaskDelete(NULL);
}

/*!
 * @brief Main function
 */
int main(void)
{
    SYSMPU_Type *base = SYSMPU;
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;

    /* Initialize lwIP from thread */
    if (sys_thread_new("main", stack_init, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("main(): Task creation failed.", 0);
    }

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}






#endif
