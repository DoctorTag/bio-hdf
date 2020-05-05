/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "health_hw.h"
#include "health_comm.h"
#include "health_comm_protocol.h"

#include "health_tcp.h"


/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define FEELKIT_WIFI_SSID "mywifissid"
*/
#define FEELKIT_WIFI_SSID CONFIG_WIFI_SSID
#define FEELKIT_WIFI_PASS CONFIG_WIFI_PASSWORD

#define PORT CONFIG_HEALTH_PORT

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

static hsensor_comm_protocol_t tcp_rx_pc;

static int h_connected_sock;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "FeelKit_TCP";

static QueueHandle_t tcp_cb_queue;
uint8_t ipaddr[5];
static esp_err_t  health_tcp_framecb(uint8_t *dframe)
{
    hdata_type_t creq;

    creq.type = REQ_FROM_TCP;
    creq.pdata = (void*)dframe;

    xQueueSend (tcp_cb_queue, (void * )&creq, 0);
    return ESP_OK;

}
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            /* enable ipv6 */
            tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
        {
            hdata_type_t ind_ip;
            
            ipaddr[0] = H_INFO_IPADDR ;
            ipaddr[1] = ip4_addr1(&event->event_info.got_ip.ip_info.ip) ;
            ipaddr[2] = ip4_addr2(&event->event_info.got_ip.ip_info.ip) ;
            ipaddr[3] = ip4_addr3(&event->event_info.got_ip.ip_info.ip) ;
            ipaddr[4] = ip4_addr4(&event->event_info.got_ip.ip_info.ip) ;
            ind_ip.type = IND_STATUS;
            ind_ip.pdata = (void*)ipaddr;

            xQueueSend (tcp_cb_queue, (void * )&ind_ip, 0);
            xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
            // ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");

        }
        break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
            xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
            break;
        case SYSTEM_EVENT_AP_STA_GOT_IP6:
            xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

            char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
            ESP_LOGI(TAG, "IPv6: %s", ip6);
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config,tmp_cfg;
    /*  wifi_config_t wifi_config=
          {
              .sta = {
                  .ssid = FEELKIT_WIFI_SSID,
                  .password = FEELKIT_WIFI_PASS,
              },
          };
          */
    memset(&wifi_config, 0x00, sizeof(wifi_config_t));
    memset(&tmp_cfg, 0x00, sizeof(wifi_config_t));
    esp_wifi_get_config(ESP_IF_WIFI_STA, &tmp_cfg);

    strcpy((char*)wifi_config.sta.ssid,(char*)tmp_cfg.sta.ssid);
    strcpy((char*)wifi_config.sta.password,(char*)tmp_cfg.sta.password);
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s,PWD:%s", wifi_config.sta.ssid,wifi_config.sta.password);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );

    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");
}

static void htcp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    wait_for_ip();
    while (1)
    {


#ifdef CONFIG_HEALTH_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket binded");

        err = listen(listen_sock, 1);
        if (err != 0)
        {
            ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
        uint addrLen = sizeof(sourceAddr);
        while(1)
        {
            h_connected_sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
            if ( h_connected_sock < 0)
            {
                ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Socket accepted");

            while (1)
            {
                int len = recv( h_connected_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
                // Error occured during receiving
                if (len < 0)
                {
                    ESP_LOGE(TAG, "recv failed: errno %d", errno);
                    break;
                }
                // Connection closed
                else if (len == 0)
                {
                    ESP_LOGI(TAG, "Connection closed");
                    break;
                }
                // Data received
                else
                {
                    for(int i=0; i<len; i++)
                        hcomm_RecvFrame( &tcp_rx_pc,rx_buffer[i]);
                }
            }

            if ( h_connected_sock != -1)
            {
                ESP_LOGE(TAG, "Shutting down socket and restarting...");
                shutdown( h_connected_sock, 0);
                close( h_connected_sock);
                h_connected_sock = -1;

            }
        }

    }
    vTaskDelete(NULL);
}

static void htcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    wait_for_ip();
    while (1)
    {

#ifdef CONFIG_HEALTH_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr((char*)pvParameters);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton((char*)pvParameters, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        h_connected_sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (h_connected_sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = connect(h_connected_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        while (1)
        {


            int len = recv(h_connected_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occured during receiving
            if (len < 0)
            {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else
            {
                for(int i=0; i<len; i++)
                    hcomm_RecvFrame( &tcp_rx_pc,rx_buffer[i]);
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (h_connected_sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(h_connected_sock, 0);
            close(h_connected_sock);
            h_connected_sock = -1;
        }
    }
    vTaskDelete(NULL);
}

esp_err_t  health_tcp_send(uint8_t *pdata,uint16_t plen)
{
    if(h_connected_sock < 0)
        return ESP_FAIL;
    return send( h_connected_sock, pdata, plen, 0);
}

void  health_tcp_connect(char * host_ip)
{
    if(h_connected_sock < 0)
        xTaskCreate(htcp_client_task, "htcp_client_task", 4096, (void*)(host_ip), 5, NULL);

}

esp_err_t  health_tcp_init(QueueHandle_t xQueue_Req)
{
    h_connected_sock = -1;
    hcomm_RecvInit(&tcp_rx_pc);
    tcp_rx_pc.frame_cb = health_tcp_framecb;
    tcp_cb_queue = xQueue_Req;
    initialise_wifi();

//health_tcp_connect("192.168.1.105");
    xTaskCreate(htcp_server_task, "htcp_server_task", 4096, (void*)(0), 5, NULL);
    return ESP_OK;
}


esp_err_t  health_tcp_deinit()
{
    return ESP_OK;
}

uint8_t*  health_tcp_getIp(void)
{
    return (ipaddr+1);
}