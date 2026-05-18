/*
    logs
*/
#define TAG "ZF_TX"
#define FILE_ID 0x24
#include "zoecare/logs/logs.h"
#include "zoecare/err/err.h"
#include "zoecare/tx/err.h"

/* Console example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "zoecare/tx/config.h"
#include "zc_debug.h"
#include "zoecare/tx/zoecare_functions.h"
#include "zoecare/tx/ota_update.h"
#include "zoecare/tx/tx_cred_switch.h"

#include <pthread.h>                    // POSIX thread management
#include <esp_console.h>                // CLI (Command Line Interface) framework
#include <linenoise/linenoise.h>        // Line editing and history management for the console
#include <esp_vfs_fat.h>                // FAT Virtual File System support
#include <nvs_flash.h>                  // Non-volatile storage (NVS) management
#include <esp_eap_client.h>             // Extensible Authentication Protocol (EAP) client
#include <dhcpserver/dhcpserver.h>      // Dynamic Host Configuration Protocol (DHCP) server
#include "zoecare/tx/cmd_decl.h"           // Command declarations for the CLI
#include <esp_http_server.h>            // HTTP server implementation
#include <lwip/lwip_napt.h>             // Network Address and Port Translation (NAPT) with LWIP
#include "zoecare/tx/router_globals.h"             // Global declarations for router functionality
#include <driver/uart.h>                // Driver for UART (Universal Asynchronous Receiver-Transmitter)
#include <esp_mac.h>                    // Management of ESP MAC addresses
#include <driver/uart_vfs.h>           // UART VFS driver for serial communication
#include "zoecare/tx/version_info.h"
#include "zoecare/wifi/wifi.h"
#include "zoecare/report/report.h"
#include "zoecare/common/config.h"


#define DEFAULT_AP_IP "192.168.4.1" // Default IP for the AP
#define DEFAULT_DNS "8.8.8.8"       // Default DNS server

#define BLINK_GPIO 2    // GPIO pin for the LED
#define BOOT_BUTTON_GPIO GPIO_NUM_0 // GPIO pin for the boot button

bool has_started = false;   // Flag to start sending error messages


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;
uint8_t* mac = NULL;            // define at startup. MAC address of the device
char* ssid = NULL;     // upstream WiFi SSID (loaded from zc_cfg, overridable via NVS)
char* passwd = NULL;   // upstream WiFi password (loaded from zc_cfg, overridable via NVS)
char* ent_username = "";        // for WPA2 Enterprise
char* ent_identity = "";        // for WPA2 Enterprise
char* static_ip = NULL;         // if needed to set a static IP
char* subnet_mask = NULL;       // if needed to set a static IP
char* gateway_addr = NULL;      // if needed to set a static IP
uint8_t* ap_mac = NULL;         // if needed to set the MAC address of the AP
char* ap_ssid = NULL;           // define at startup. Name of the AP to create
char* ap_passwd = "a9qgwZYn5ApgrgXxZD%q";   // define here. Password of the AP to create
char* ap_ip = NULL;             // define at startup. IP of the AP to create

/* Global vars */
uint16_t connect_count = 0;
bool ap_connect = false;
bool has_static_ip = false;

uint32_t my_ip;
uint32_t my_ap_ip;

struct portmap_table_entry {
  u32_t daddr;
  u16_t mport;
  u16_t dport;
  u8_t proto;
  u8_t valid;
};
struct portmap_table_entry portmap_tab[IP_PORTMAP_MAX];

esp_netif_t* wifiAP;
esp_netif_t* wifiSTA;

httpd_handle_t start_webserver(void);


// Global WDT configuration (initialized once)
esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = 0, // Monitor all cores
};


void esp_task_wdt_isr_user_handler(void){
    esp_restart();
}


/* Console command history can be stored to and loaded from a file.
 * The easiest way to do this is to use FATFS filesystem on top of
 * wear_levelling library.
 */
#if CONFIG_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        LOGE("Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_STORE_HISTORY

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t apply_portmap_tab() {
    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid) {
            ip_portmap_add(portmap_tab[i].proto, my_ip, portmap_tab[i].mport, portmap_tab[i].daddr, portmap_tab[i].dport);
        }
    }
    return ESP_OK;
}

esp_err_t delete_portmap_tab() {
    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid) {
            ip_portmap_remove(portmap_tab[i].proto, portmap_tab[i].mport);
        }
    }
    return ESP_OK;
}

void print_portmap_tab() {
    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid) {
            printf ("%s", portmap_tab[i].proto == PROTO_TCP?"TCP ":"UDP ");
            ip4_addr_t addr;
            addr.addr = my_ip;
            printf (IPSTR":%d -> ", IP2STR(&addr), portmap_tab[i].mport);
            addr.addr = portmap_tab[i].daddr;
            printf (IPSTR":%d\n", IP2STR(&addr), portmap_tab[i].dport);
        }
    }
}

esp_err_t get_portmap_tab() {
    esp_err_t err;
    nvs_handle_t nvs;
    size_t len;

    err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_get_blob(nvs, "portmap_tab", NULL, &len);
    if (err == ESP_OK) {
        if (len != sizeof(portmap_tab)) {
            err = ESP_ERR_NVS_INVALID_LENGTH;
        } else {
            err = nvs_get_blob(nvs, "portmap_tab", portmap_tab, &len);
            if (err != ESP_OK) {
                memset(portmap_tab, 0, sizeof(portmap_tab));
            }
        }
    }
    nvs_close(nvs);

    return err;
}

esp_err_t add_portmap(u8_t proto, u16_t mport, u32_t daddr, u16_t dport) {
    esp_err_t err;
    nvs_handle_t nvs;

    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (!portmap_tab[i].valid) {
            portmap_tab[i].proto = proto;
            portmap_tab[i].mport = mport;
            portmap_tab[i].daddr = daddr;
            portmap_tab[i].dport = dport;
            portmap_tab[i].valid = 1;

            err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
            if (err != ESP_OK) {
                return err;
            }
            err = nvs_set_blob(nvs, "portmap_tab", portmap_tab, sizeof(portmap_tab));
            if (err == ESP_OK) {
                err = nvs_commit(nvs);
                if (err == ESP_OK) {
                    LOGI("New portmap table stored.");
                }
            }
            nvs_close(nvs);

            ip_portmap_add(proto, my_ip, mport, daddr, dport);

            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

esp_err_t del_portmap(u8_t proto, u16_t mport) {
    esp_err_t err;
    nvs_handle_t nvs;

    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid && portmap_tab[i].mport == mport && portmap_tab[i].proto == proto) {
            portmap_tab[i].valid = 0;

            err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
            if (err != ESP_OK) {
                return err;
            }
            err = nvs_set_blob(nvs, "portmap_tab", portmap_tab, sizeof(portmap_tab));
            if (err == ESP_OK) {
                err = nvs_commit(nvs);
                if (err == ESP_OK) {
                    LOGI("New portmap table stored.");
                }
            }
            nvs_close(nvs);

            ip_portmap_remove(proto, mport);
            return ESP_OK;
        }
    }
    return ESP_OK;
}

static void initialize_console(void)
{
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

#if CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    uart_vfs_dev_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    uart_vfs_dev_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
            .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            #if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
                .source_clk = UART_SCLK_REF_TICK,
            #else
                .source_clk = UART_SCLK_XTAL,
            #endif
    };
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
            256, 0, 0, NULL, 0) );
    ESP_ERROR_CHECK( uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config) );

    /* Tell VFS to use UART driver */
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
#endif

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, O_NONBLOCK);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_CR);

    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };

    /* Install USB-SERIAL-JTAG driver for interrupt-driven reads and writes */
    usb_serial_jtag_driver_install(&usb_serial_jtag_config);

    /* Tell vfs to use usb-serial-jtag driver */
    esp_vfs_usb_serial_jtag_use_driver();
#endif

    /* Initialize the console */
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
            .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);

#if CONFIG_STORE_HISTORY
    /* Load command history from filesystem */
    linenoiseHistoryLoad(HISTORY_PATH);
#endif
}

void * led_status_thread(void * p)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (true)
    {
        gpio_set_level(BLINK_GPIO, ap_connect);

        for (int i = 0; i < connect_count; i++)
        {
            gpio_set_level(BLINK_GPIO, 1 - ap_connect);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            gpio_set_level(BLINK_GPIO, ap_connect);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    esp_netif_dns_info_t dns;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        LOGI("wifi station start");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t* d = (wifi_event_sta_disconnected_t*)event_data;
        LOGI("disconnected (reason %d) - retrying", d->reason);
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        LOGI("got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        ap_connect = true;
        //get mac of actual device
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        if (!has_started) {
            ZC_LOG_SENS(TAG, "Start of the program, mac: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            printf("\n");
            LOGI("App Version: %d.%d.%d-TX-pres (Git: %s)",
                    APP_VER_MAJOR, APP_VER_MINOR, APP_VER_PATCH, APP_VER_GIT_HASH);
            printf("\n");
        }
        has_started = true;
        my_ip = event->ip_info.ip.addr;
        delete_portmap_tab();
        apply_portmap_tab();
        if (esp_netif_get_dns_info(wifiSTA, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK)
        {
            esp_netif_set_dns_info(wifiAP, ESP_NETIF_DNS_MAIN, &dns);
            LOGI("set dns to:" IPSTR, IP2STR(&(dns.ip.u_addr.ip4)));
        }
        atomic_store(&got_ip, 1);
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        connect_count++;
        LOGI("%d. Station connected: ", connect_count);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        connect_count--;
        LOGI("station disconnected - %d remain", connect_count);
    }
}

const int CONNECTED_BIT = BIT0;
#define JOIN_TIMEOUT_MS (2000)



void wifi_init(const uint8_t* mac, const char* ssid, const char* ent_username, const char* ent_identity, const char* passwd, const char* static_ip, const char* subnet_mask, const char* gateway_addr, const uint8_t* ap_mac, const char* ap_ssid, const char* ap_passwd, const char* ap_ip)
{
    esp_netif_dns_info_t dnsserver;

    wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifiAP = esp_netif_create_default_wifi_ap();
    wifiSTA = esp_netif_create_default_wifi_sta();

    esp_netif_ip_info_t ipInfo_sta;
    if ((strlen(ssid) > 0) && (strlen(static_ip) > 0) && (strlen(subnet_mask) > 0) && (strlen(gateway_addr) > 0)) {
        has_static_ip = true;
        ipInfo_sta.ip.addr = esp_ip4addr_aton(static_ip);
        ipInfo_sta.gw.addr = esp_ip4addr_aton(gateway_addr);
        ipInfo_sta.netmask.addr = esp_ip4addr_aton(subnet_mask);
        esp_netif_dhcpc_stop(wifiSTA); // Don't run a DHCP client
        esp_netif_set_ip_info(wifiSTA, &ipInfo_sta);
        apply_portmap_tab();
    }

    my_ap_ip = esp_ip4addr_aton(ap_ip);

    esp_netif_ip_info_t ipInfo_ap;
    ipInfo_ap.ip.addr = my_ap_ip;
    ipInfo_ap.gw.addr = my_ap_ip;
    esp_netif_set_ip4_addr(&ipInfo_ap.netmask, 255,255,255,0);
    esp_netif_dhcps_stop(wifiAP); // stop before setting ip WifiAP
    esp_netif_set_ip_info(wifiAP, &ipInfo_ap);
    esp_netif_dhcps_start(wifiAP);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));


    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* ESP WIFI CONFIG */
    wifi_config_t wifi_config = { 0 };
        wifi_config_t ap_config = {
        .ap = {
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_WPA3_PSK,
            .ssid_hidden = 0,
            .max_connection = 8,
            .beacon_interval = 100,
        }
    };

    strlcpy((char*)ap_config.sta.ssid, ap_ssid, sizeof(ap_config.sta.ssid));
    if (strlen(ap_passwd) < 8) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
	    strlcpy((char*)ap_config.sta.password, ap_passwd, sizeof(ap_config.sta.password));
    }

    if (strlen(ssid) > 0) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA) );

        //Set SSID
        strlcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        //Set passwprd
        if(strlen(ent_username) == 0) {
            LOGI("STA regular connection");
            strlcpy((char*)wifi_config.sta.password, passwd, sizeof(wifi_config.sta.password));
        }
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        if(strlen(ent_username) != 0 && strlen(ent_identity) != 0) {
            LOGI("STA enterprise connection");
            if(strlen(ent_username) != 0 && strlen(ent_identity) != 0) {
                esp_eap_client_set_identity((uint8_t *)ent_identity, strlen(ent_identity)); //provide identity
            } else {
                esp_eap_client_set_identity((uint8_t *)ent_username, strlen(ent_username));
            }
            esp_eap_client_set_username((uint8_t *)ent_username, strlen(ent_username)); //provide username
            esp_eap_client_set_password((uint8_t *)passwd, strlen(passwd)); //provide password
            esp_wifi_sta_enterprise_enable();
        }

        if (mac != NULL) {
            ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_STA, mac));
        }
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP) );
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config) );

    if (ap_mac != NULL) {
        ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_AP, ap_mac));
    }

    // Enable DNS (offer) for dhcp server
    dhcps_offer_t dhcps_dns_value = OFFER_DNS;
    esp_netif_dhcps_option(wifiAP,ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &dhcps_dns_value, sizeof(dhcps_dns_value));

    // Set custom dns server address for dhcp server
    dnsserver.ip.u_addr.ip4.addr = esp_ip4addr_aton(DEFAULT_DNS);
    dnsserver.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(wifiAP, ESP_NETIF_DNS_MAIN, &dnsserver);

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
        pdFALSE, pdTRUE, JOIN_TIMEOUT_MS / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(esp_wifi_start());

    if (strlen(ssid) > 0) {
        LOGI("wifi_init_apsta finished.");
        ZC_LOG_SENS(TAG, "connect to ap SSID: %s ", ssid);
    } else {
        LOGI("wifi_init_ap with default finished.");
    }

    int8_t power;
    LOGW("%s", esp_err_to_name(esp_wifi_set_max_tx_power(8)));
    esp_wifi_get_max_tx_power(&power);
    printf("%d\n", power);

    LOGW("%s", esp_err_to_name(esp_wifi_config_80211_tx_rate(WIFI_IF_AP, WIFI_PHY_RATE_MCS0_LGI)));
}

char* param_set_default(const char* def_val) {
    char * retval = malloc(strlen(def_val)+1);
    strcpy(retval, def_val);
    return retval;
}


void app_main(void)
{
    ZC_ERR_SETUP_BACKTRACE_IN_TLS(1);

    printf("\n");
    LOGI("App Version: %d.%d.%d-TX (Git: %s)",
            APP_VER_MAJOR, APP_VER_MINOR, APP_VER_PATCH, APP_VER_GIT_HASH);
    printf("\n");

    esp_task_wdt_init(&wdt_config);

    initialize_nvs();
    ota_tx_check_partition_layout();

    zc_zf_tx_cfg_load();
    zc_zf_tx_cfg_print();

    zc_zf_tx_err_init();

    zc_logs_cfg.enable         = true;
    zc_logs_cfg.enable_error   = true;
    zc_logs_cfg.enable_warning = true;
    zc_logs_cfg.enable_info    = true;
    zc_logs_cfg.enable_debug   = false;
    zc_logs_cfg.enable_verbose = false;

#if CONFIG_STORE_HISTORY
    initialize_filesystem();
    LOGI("Command history enabled");
#else
    LOGI("Command history disabled");
#endif

    get_config_param_blob("mac", &mac, 6);
    get_config_param_str("ssid", &ssid);
    if (ssid == NULL) {
        ssid = param_set_default(zc_zf_tx_cfg.upstream_ssid);
    }
    get_config_param_str("ent_username", &ent_username);
    if (ent_username == NULL) {
        ent_username = param_set_default("");
    }
    get_config_param_str("ent_identity", &ent_identity);
    if (ent_identity == NULL) {
        ent_identity = param_set_default("");
    }
    get_config_param_str("passwd", &passwd);
    if (passwd == NULL) {
        passwd = param_set_default(zc_zf_tx_cfg.upstream_password);
    }
    get_config_param_str("static_ip", &static_ip);
    if (static_ip == NULL) {
        static_ip = param_set_default("");
    }
    get_config_param_str("subnet_mask", &subnet_mask);
    if (subnet_mask == NULL) {
        subnet_mask = param_set_default("");
    }
    get_config_param_str("gateway_addr", &gateway_addr);
    if (gateway_addr == NULL) {
        gateway_addr = param_set_default("");
    }
    get_config_param_blob("ap_mac", &ap_mac, 6);
    get_config_param_str("ap_ssid", &ap_ssid);
    if (ap_ssid == NULL) {

        //create a string with ZoeFall and the last 4 char of mac address
        uint8_t *mac_addr = malloc(6 * sizeof(uint8_t));
        esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);

        char *mac_str = malloc(13);
        if (mac_str == NULL) {
            LOGE("Failed to allocate memory for mac_str");
            return;
        }
        sprintf(mac_str, "%02X%02X", mac_addr[4], mac_addr[5]);
        char *name = malloc(strlen("ZoeFall")+5);
        strcpy(name, "ZoeFall");
        strcat(name, mac_str);
        ap_ssid = param_set_default(name);
        free(mac_str);
        free(name);
    }
    get_config_param_str("ap_passwd", &ap_passwd);
    if (ap_passwd == NULL) {
        ap_passwd = param_set_default("");
    }
    get_config_param_str("ap_ip", &ap_ip);
    if (ap_ip == NULL) {
        ap_ip = param_set_default(DEFAULT_AP_IP);
    }

    get_portmap_tab();

    // Setup WIFI
    wifi_init(mac, ssid, ent_username, ent_identity, passwd, static_ip, subnet_mask, gateway_addr, ap_mac, ap_ssid, ap_passwd, ap_ip);

    // Start staged credential switch if NVS contains pending upstream credentials
    tx_cred_switch_init(wifi_event_group);

    pthread_t t1;
    pthread_create(&t1, NULL, led_status_thread, NULL);

    ip_napt_enable(my_ap_ip, 1);
    LOGI("NAT is enabled");

    char* lock = NULL;
    get_config_param_str("lock", &lock);
    if (lock == NULL) {
        lock = param_set_default("0");
    }
    if (strcmp(lock, "0") ==0) {
        LOGI("Starting config web server");
        start_webserver();
    }
    free(lock);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO), // GPIO mask for the boot button
        .mode = GPIO_MODE_INPUT,                   // Set as input
        .pull_up_en = GPIO_PULLUP_ENABLE,          // Enable internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,     // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE             // No interrupts for now
    };
    gpio_config(&io_conf);

    static esp_http_client_config_t report_config = {
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };
    report_config.url = zc_zf_tx_cfg.tb_url;
    report_config.cert_pem = (const char *)ZC_CA_CERT_PEM_START;
    zc_report_init();
    xTaskCreate(zc_report_send_task, "report_send", 8192, &report_config, 4, NULL);

    xTaskCreate(send_transmission_task, "Send transmission", 4096, NULL, 8, NULL);
    xTaskCreate(csi_counter_task, "csi_counter_task", 4096, NULL, 2, NULL);
    xTaskCreate(isAliveTask, "isAliveTask", 8192, NULL, 5, NULL);

    ota_tx_start_task();
}
