#pragma once

typedef struct MSEED_CONFIG
{
    uint16_t sample_count;
    uint32_t sequence_number;
    char station[6];
    char location[3];
    char channel[4];
    char network[3];
    uint64_t start_time;
    uint16_t time_fract;
} mseed_config_t;

uint8_t *mseed_record(float *p_data, mseed_config_t *sp_cfg);
uint8_t *mseed_datalink(uint8_t *p_record, mseed_config_t *sp_cfg);