#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "ap_wifi.h"

/* Maximum connection that Acces Point allows */
#define MAX_STA_CONN        (4u)
/* Maximum connection tries */
#define WIFI_MAX_CONN_TRIES (5u)

/* The lenght of "ssid=" string from HTTP POST method */
#define SSID_CONST_LEN          (5u)
/* The lenght of "pass=" string from HTTP POST method */
#define PASS_CONST_LEN          (5u)
/* The maximum length that a WiFi SSID can have */
#define WIFI_SSID_MAX_LEN       (32u)
/* The maximum length that a WiFi password can have */
#define WIFI_PASS_MAX_LEN       (64u)
/* The maximum length that a WiFi content can have (content used for POST method) */
#define WIFI_CONTENT_MAX_LEN    (100u)
/* The maximum payload length to send */
#define WIFI_MAX_PAYLOAD_LEN    (64u)

/* Used for variables initialization */
#define INIT_USGN_VAL       (0u)
/* Index 1 */
#define IDX_1               (1u)
/* Ticks between wifi connection tries */
#define TICKS_BETWEEN_TRIES (1000u)

/* The name of the storage space allocated for WiFi configuration */
#define STORAGE_NAMESPACE "wifi_config"

/* Terminal messages TAG */
static const char *TAG = "ap_wifi";

/**
 * @brief Extracts WiFi SSID and password from the given content buffer.
 *
 * @param[in] content       Pointer to the input buffer containing the raw data.
 * @param[in] content_len   Length of the input buffer.
 * @param[out] ssid         Output buffer to store the extracted SSID.
 * @param[out] pass         Output buffer to store the extracted password.
 * 
 * @returns
 *  - true  if credentials were successfully extracted  
 *  - false otherwise
 */
static bool get_wifi_info(const uint8_t *content, const uint32_t content_len, uint8_t *ssid, uint8_t *pass)
{
    /* Used for saving '&' (right after SSID last character position) character position in content string */
    uint32_t ssid_last_pos = INIT_USGN_VAL;
    /* Flag for when SSID is found */
    bool ssid_found = false;
    bool ret = true;

    /* Check if there are WiFi connection info */
    if (content == NULL)
    {
        ESP_LOGE(TAG, "No WiFi connection information");
    }
    else
    {
        /* Find SSID and PASS in content string and parse them */
        for (uint32_t idx = SSID_CONST_LEN; idx < content_len; idx++)
        {
            /* SSID terminate at '&' character */
            if ((ssid_found == false) && content[idx] == '&')
            {
                ssid_found = true;
                /* Save '&' character position */
                ssid_last_pos = idx;
                /* Set index to first character position for PASS */
                idx += PASS_CONST_LEN;
            }
            else if (((idx - SSID_CONST_LEN) < WIFI_SSID_MAX_LEN) && (ssid_found != true))
            {
                /* Parse SSID */
                ssid[idx - SSID_CONST_LEN] = content[idx];
            }
            else if ((idx - ssid_last_pos - PASS_CONST_LEN - IDX_1) < WIFI_PASS_MAX_LEN)
            {
                /* Parse PASS */
                pass[idx - ssid_last_pos - PASS_CONST_LEN - IDX_1] = content[idx];
            }
            else
            {
                ESP_LOGE(TAG, "SSID/PASS length error");
                ret = false;
            }
        }
    }

    return ret;
}

/**
 * @brief Handler for HTML web page.
 *
 * @param[in] req   HTTP Request Data Structure.
 * 
 * @returns
 *  - ESP_OK if OK
 *  - error code if not OK
 */
static esp_err_t root_get_handler(httpd_req_t *req) 
{
    esp_err_t ret = ESP_OK;

    /* HTML Web Page*/
    const char html[] = "<!DOCTYPE html><html><body>"
    "<h2>Config Wi-Fi</h2>"
    "<form id='wifi-form'>"
    "SSID:<br><input type='text' name='ssid'><br>"
    "Parola:<br><input type='password' name='pass'><br><br>"
    "<input type='submit' value='Connect'>"
    "</form>"
    "<p id='status'></p>"
    "<script>"
    "document.getElementById('wifi-form').addEventListener('submit', async function(e) {"
        "e.preventDefault();"
        "const form = new FormData(e.target);"
        "const params = new URLSearchParams();"
        "for (const pair of form.entries()) { params.append(pair[0], pair[1]); }"
        "try {"
            "const response = await fetch('/wifi', {"
                "method: 'POST',"
                "headers: { 'Content-Type': 'application/x-www-form-urlencoded' },"
                "body: params"
            "});"
            "document.getElementById('status').innerText = 'Sent!';"
        "} catch (err) {"
            "document.getElementById('status').innerText = 'Send error!';"
        "}"
    "});"
    "</script>"
    "<br><a href='/data'>Test /data</a>"
    "</body></html>";

    /* Send a HTTP response*/
    ret = httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);

    return ret;
}

/**
 * @brief Handler for saving WiFi SSID and password in NVS memory.
 *
 * @param[in] req   HTTP Request Data Structure.
 * 
 * @returns
 *  - ESP_OK if OK
 *  - error code if not OK
 */
static esp_err_t wifi_post_handler(httpd_req_t *req)
{
    /* Buffer for parsing the content */
    uint8_t content[WIFI_CONTENT_MAX_LEN];
    /* Buffer for parsing the WiFi password */
    uint8_t pass[WIFI_PASS_MAX_LEN] = {INIT_USGN_VAL};
    /* Buffer for parsing the WiFi SSID */
    uint8_t ssid[WIFI_SSID_MAX_LEN] = {INIT_USGN_VAL};
    /* The total length of the content */
    uint32_t total_len = req->content_len;
    /* Used to check if all bytes are received */
    uint32_t received = INIT_USGN_VAL;
    /* Save bytes read */
    uint32_t bytes_read = INIT_USGN_VAL;
    esp_err_t ret = ESP_OK;

    /* Read all received bytes */
    while ((ret == ESP_OK) && (received < total_len))
    {
        /* Read bytes from HTTP request */
        bytes_read = httpd_req_recv(req, (char *)(content + received), sizeof(content) - received);

        /* Check if all bytes was readed */
        if (bytes_read <= INIT_USGN_VAL) 
        {
            ret = ESP_FAIL;
        }
        else
        {
            /* Save number of bytes readed succesfully */
            received += bytes_read;
        }
    }
    /* End the content string */
    content[received] = '\0';

    /* If the content was readed succesfully, save data in NVS memory */
    if (ret == ESP_OK)
    {
        /* Parse WiFi SSID and password */
        if (get_wifi_info(content, total_len, ssid, pass) == true)
        {
            ESP_LOGI(TAG, "SSID: %s, PASS: %s", ssid, pass);
            
            /* Save WiFi SSID and password in NVS memory */
            nvs_handle_t nvs;
            ret = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs);
            if (ret == ESP_OK)
            {
                ret = nvs_set_str(nvs, "ssid", (char *)ssid);
                if (ret == ESP_OK)
                {
                    ret = nvs_set_str(nvs, "pass", (char *)pass);
                    if (ret == ESP_OK)
                    {
                        ret = nvs_commit(nvs);
                        if (ret == ESP_OK)
                        {
                            nvs_close(nvs);

                            /* Send response */
                            ret = httpd_resp_send(req, "Data saved. Connecting...", HTTPD_RESP_USE_STRLEN);
                            if (ret == ESP_OK)
                            {
                                /* Restart device for connecting to WiFi*/
                                esp_restart();
                            }
                        }
                    }
                }
            }
        }
        else
        {
            ret = ESP_FAIL;
        }   
    }
    
    return ret;
}

/**
 * @brief Starts the HTTP server and registers URI callbacks.
 * 
 * @return:
 *  - ESP_OK if OK
 *  - error code if not OK
 */
static esp_err_t start_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    esp_err_t ret = ESP_OK;

    /* Start HTTP server and register callbacks for every web page */
    ret = httpd_start(&server, &config);
    if (ret == ESP_OK)
    {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t wifi_uri = {
            .uri = "/wifi",
            .method = HTTP_POST,
            .handler = wifi_post_handler
        };
        httpd_register_uri_handler(server, &wifi_uri);
    }
    
    return ret;
}

/**
 * @brief Init as a Acces Point.
 * 
 * @return:
 *  - ESP_OK if OK
 *  - error code if not OK
 */
static esp_err_t start_ap_mode()
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Starting Acces Point...");

    /* Creates default WIFI AP */
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    /* Initialize WiFi */
    ret = esp_wifi_init(&cfg);
    if (ret == ESP_OK)
    {
        wifi_config_t wifi_config = {
            .ap = {
                .ssid = AP_SSID,
                .ssid_len = strlen(AP_SSID),
                .password = AP_PASS,
                .max_connection = MAX_STA_CONN,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK
            },
        };

        if (strlen(AP_PASS) == INIT_USGN_VAL) 
        {
            wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        }
        
        /* Set mode to Acces Point */
        ret = esp_wifi_set_mode(WIFI_MODE_AP);
        if (ret == ESP_OK)
        {
            /* Set WiFi configurations */
            ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
            if (ret == ESP_OK)
            {
                /* Start WiFi */
                ret = esp_wifi_start();    
                if (ret == ESP_OK)
                {
                    /* Start server with web pages */
                    ret = start_server();        
                }            
            }
        }
    }    
    
    return ret;
}

/**
* @brief Connect to STA as a client.
* 
* @return:
*  - true if OK
*  - false if not OK
*/
static bool connect_to_saved_wifi()
{
   /* Buffer to save WiFi SSID from NVS memory */
   uint8_t ssid[WIFI_SSID_MAX_LEN];
   /* Buffer to save WiFi password from NVS memory */
   uint8_t pass[WIFI_PASS_MAX_LEN];
   /* NVS memory handler */
   nvs_handle_t nvs;
   /* WiFi SSID length */
   uint32_t ssid_len = sizeof(ssid);
   /* WiFi SSID length */
   uint32_t pass_len = sizeof(pass);
   bool ret = true;

   /* Open allocated NVS for WiFi configuration info and get SSID and password*/
   if (nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) 
   {
       ret = false;
   }
   else if ((nvs_get_str(nvs, "ssid", (char *)ssid, (size_t *)&ssid_len) != ESP_OK) || (nvs_get_str(nvs, "pass", (char *)pass, (size_t *)&pass_len) != ESP_OK)) 
   {
       ret = false;
   }
   else
   {
       /* Close allocated NVS for WiFi */
       nvs_close(nvs);

       ESP_LOGI(TAG, "Saved SSID: %s", ssid);

       /* Creates default WIFI STA */
       esp_netif_create_default_wifi_sta();
       wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

       /* Initialize WiFi */
       if (esp_wifi_init(&cfg) == ESP_OK)
       {
           wifi_config_t wifi_config = { INIT_USGN_VAL };
           strcpy((char *)wifi_config.sta.ssid, (char *)ssid);
           strcpy((char *)wifi_config.sta.password, (char *)pass);

           /* WiFi set STA mode */
           if (esp_wifi_set_mode(WIFI_MODE_STA) == ESP_OK)
           {
               /* WiFi set configuration */
               if (esp_wifi_set_config(WIFI_IF_STA, &wifi_config) == ESP_OK)
               {
                   /* Start WiFi */
                   if (esp_wifi_start() == ESP_OK)
                   {
                       /* Connect to saved SSID from NVS */
                       esp_wifi_connect();

                       ESP_LOGI(TAG, "Connecting to %s...", ssid);

                       uint16_t tries = INIT_USGN_VAL;

                       wifi_ap_record_t ap_info;

                       bool connected = false;

                       /* Try to connect to SSID */
                       while ((connected == false ) && (tries < WIFI_MAX_CONN_TRIES))
                       {
                           /* Delay between tries*/
                           vTaskDelay(pdMS_TO_TICKS(TICKS_BETWEEN_TRIES));

                           /* Check if connection was made */
                           if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
                           {
                               ESP_LOGI(TAG, "Connected to %s", ap_info.ssid);

                               /* Make connection flag true */
                               connected = true;
                           }

                           tries++;
                       }

                       if (connected == false)
                       {
                           ESP_LOGW(TAG, "Connection failed. Starting AP");

                           ret = false;
                       }
                   }
               }
           }
       }
   }
   
   return ret;
}

/**
 * @brief Initialize WiFi and NVS Flash.
 * 
 * @return:
 *  - ESP_OK if OK
 *  - error code if not OK
 */
esp_err_t wifi_init()
{
    esp_err_t ret = ESP_OK;

    ret = nvs_flash_init();
    if (ret == ESP_OK)
    {
        ret = esp_netif_init();
        if (ret == ESP_OK)
        {
            ret = esp_event_loop_create_default();
        }
    }
    
    return ret;
}

/**
 * @brief Connect ESP to STA or AP.
 * 
 * @return:
 *  - ESP_OK if OK
 *  - error code if not OK
 */
esp_err_t wifi_connect()
{
    esp_err_t ret = ESP_OK;

    /* Check if is any saved WiFi in NVS flash */
    if (!connect_to_saved_wifi())
    {   
        /* Start AP mode if there are no saved WiFi info */
        ret = start_ap_mode();
    }
    else
    {
        /* Start STA server if there are saved WiFi info */
        ret = start_server();
    }

    return ret;
}