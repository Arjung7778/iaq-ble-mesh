#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "access.h"
#include "access_config.h"
#include "access_reliable.h"
#include "nrf_mesh_defines.h"
#include "nrf_mesh.h"
#include "log.h"

#include "mesh_vendor_model.h"
#include "device_state_manager.h"
#include "nrf_strerror.h"

#define VENDOR_COMPANY_ID   0x0059
#define VENDOR_MODEL_ID     0x1234
#define VENDOR_OPCODE_SENSOR_VALUES  0xC1
#define VENDOR_PAYLOAD_MAX  8

/* Default group address for publishing - configure this or use the one set via app */
#define DEFAULT_PUBLISH_ADDRESS  0xC000

static bool s_vendor_model_ready = false;
static bool s_publish_configured = false;
static uint16_t s_publish_address = DEFAULT_PUBLISH_ADDRESS;
static dsm_handle_t s_appkey_handle = DSM_HANDLE_INVALID;
static dsm_handle_t s_publish_addr_handle = DSM_HANDLE_INVALID;

static access_model_handle_t m_vendor_model_handle = ACCESS_HANDLE_INVALID;

static void vendor_model_rx_cb(access_model_handle_t handle,
                               const access_message_rx_t * p_message,
                               void * p_args);

static const access_opcode_handler_t m_vendor_opcode_handlers[] =
{
    {
        .opcode = { VENDOR_OPCODE_SENSOR_VALUES, VENDOR_COMPANY_ID },
        .handler = vendor_model_rx_cb
    }
};

uint32_t mesh_vendor_model_init(void)
{
    access_model_add_params_t add_params;
    memset(&add_params, 0, sizeof(add_params));
    add_params.model_id.model_id = VENDOR_MODEL_ID;
    add_params.model_id.company_id = VENDOR_COMPANY_ID;
    add_params.element_index = 0;
    add_params.p_opcode_handlers = m_vendor_opcode_handlers;
    add_params.opcode_count = ARRAY_SIZE(m_vendor_opcode_handlers);
    add_params.p_args = NULL;
    add_params.publish_timeout_cb = NULL;

    uint32_t status = access_model_add(&add_params, &m_vendor_model_handle);
    if (status != NRF_SUCCESS)
    {
        m_vendor_model_handle = ACCESS_HANDLE_INVALID;
        s_vendor_model_ready = false;
        return status;
    }

    s_vendor_model_ready = true;
    s_publish_configured = false;
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Vendor model added (company=0x%04X, model=0x%04X), handle=%u\n",
          VENDOR_COMPANY_ID, VENDOR_MODEL_ID, (unsigned)m_vendor_model_handle);
    return NRF_SUCCESS;
}

bool mesh_vendor_model_is_ready(void)
{
    return s_vendor_model_ready;
}

void mesh_vendor_model_publication_set(void)
{
    s_publish_configured = true;
    
    uint32_t status;
    
    // Get the configured publish address handle
    status = access_model_publish_address_get(m_vendor_model_handle, &s_publish_addr_handle);
    if (status == NRF_SUCCESS && s_publish_addr_handle != DSM_HANDLE_INVALID)
    {
        nrf_mesh_address_t addr;
        status = dsm_address_get(s_publish_addr_handle, &addr);
        if (status == NRF_SUCCESS)
        {
            s_publish_address = addr.value;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publish address: 0x%04X\n", s_publish_address);
        }
    }
    
    // Get the first bound AppKey
    dsm_handle_t appkey_handles[DSM_APP_MAX];
    uint16_t appkey_count = DSM_APP_MAX;
    
    status = access_model_applications_get(m_vendor_model_handle, appkey_handles, &appkey_count);
    if (status == NRF_SUCCESS && appkey_count > 0)
    {
        s_appkey_handle = appkey_handles[0];
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Using bound AppKey handle: %u\n", s_appkey_handle);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "No AppKey bound to model\n");
        s_appkey_handle = DSM_HANDLE_INVALID;
    }
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication configured for vendor model\n");
}

static void vendor_model_rx_cb(access_model_handle_t handle,
                               const access_message_rx_t * p_message,
                               void * p_args)
{
    (void)handle;
    (void)p_args;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "Vendor RX opcode=0x%04X company=0x%04X len=%u\n",
          p_message->opcode.opcode,
          p_message->opcode.company_id,
          p_message->length);

    if (p_message->length > 0 && p_message->p_data != NULL)
    {
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO,
                 "Vendor RX payload", p_message->p_data, p_message->length);
    }
}

static void pack_payload(uint8_t iaq_level, float iaq_float, uint16_t tvoc_x100, uint16_t eco2,
                         uint8_t * buf, uint8_t * out_len)
{
    buf[0] = iaq_level;                           // IAQ Level: 1-5
    buf[1] = (uint8_t)(tvoc_x100 & 0xFF);         // TVOC low byte
    buf[2] = (uint8_t)((tvoc_x100 >> 8) & 0xFF);  // TVOC high byte
    buf[3] = (uint8_t)(eco2 & 0xFF);              // eCO2 low byte
    buf[4] = (uint8_t)((eco2 >> 8) & 0xFF);       // eCO2 high byte
    
    // Store IAQ float × 10 (e.g., 1.2 → 12, 4.5 → 45)
    uint8_t iaq_x10 = (uint8_t)(iaq_float * 10.0f + 0.5f);
    buf[5] = iaq_x10;

    *out_len = 6;
}


void mesh_publish_sensor_values(float iaq, float tvoc, float eco2)
{
    static uint32_t s_warn_count = 0;
    
    if (m_vendor_model_handle == ACCESS_HANDLE_INVALID)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Vendor model not initialized\n");
        return;
    }

    // NaN check
    if (iaq != iaq || tvoc != tvoc || eco2 != eco2)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "NaN values detected\n");
        return;
    }

    if (!s_publish_configured)
    {
        s_warn_count++;
        if (s_warn_count % 10 == 1)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN,
                  "Publish not configured (attempt %u)\n", s_warn_count);
        }
        return;
    }
    
    if (s_appkey_handle == DSM_HANDLE_INVALID)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "AppKey handle invalid\n");
        return;
    }

    // Clamp and convert IAQ to 1-5 rating
    uint8_t iaq_level;
    if (iaq < 2.0f) 
        iaq_level = 1;      // Level 1: Very Good
    else if (iaq < 3.0f) 
        iaq_level = 2;      // Level 2: Good
    else if (iaq < 4.0f) 
        iaq_level = 3;      // Level 3: Medium
    else if (iaq < 5.0f) 
        iaq_level = 4;      // Level 4: Poor
    else 
        iaq_level = 5;      // Level 5: Bad
    
    // Clamp TVOC
    if (tvoc < 0.0f) tvoc = 0.0f;
    if (tvoc > 655.35f) tvoc = 655.35f;
    
    // Clamp eCO2
    if (eco2 < 0.0f) eco2 = 0.0f;
    if (eco2 > 65535.0f) eco2 = 65535.0f;

    uint8_t payload[VENDOR_PAYLOAD_MAX];
    uint8_t payload_len;

    uint16_t tvoc_x100 = (uint16_t)(tvoc * 100.0f + 0.5f);  // Round to nearest
    uint16_t eco2_i = (uint16_t)(eco2 + 0.5f);

    pack_payload(iaq_level, iaq, tvoc_x100, eco2_i, payload, &payload_len);

    access_message_tx_t tx;
    memset(&tx, 0, sizeof(tx));

    tx.opcode.opcode = VENDOR_OPCODE_SENSOR_VALUES;
    tx.opcode.company_id = VENDOR_COMPANY_ID;
    tx.p_buffer = payload;
    tx.length = payload_len;
    tx.force_segmented = false;
    tx.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    tx.access_token = nrf_mesh_unique_token_get();

    uint32_t status = access_model_publish(m_vendor_model_handle, &tx);

    if (status == NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "Published: IAQ_Level=%u, TVOC_x100=%u, eCO2=%u\n",
              iaq_level, tvoc_x100, eco2_i);
    }
    else
    {
        s_publish_configured = false;
        const char *err_str = nrf_strerror_get(status);
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "Publish failed: 0x%08X (%s)\n", status, (err_str ? err_str : "unknown"));
    }
}

access_model_handle_t mesh_vendor_model_handle_get(void)
{
    return m_vendor_model_handle;
}
