#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// #include "lwip/sockets.h"
// #include "lwip/dns.h"
// #include "lwip/netdb.h"
// #include "lwip/igmp.h"

#include "esp_system.h"
#include "bootloader_random.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "esp_log.h"
#include "esp_spi_flash.h"

#include "nts1_iface.h"

void psts(char *message, uint8_t sts)
{
    printf(message);
    printf(";; status: %d\n", sts);
}

// RX Event handlers, weakly defined in C++ NTS1 object.
void nts1_handle_note_off_event(const nts1_rx_note_off_t *note_off)
{
    printf("note_off %d\n", note_off->note);
}
void nts1_handle_note_on_event(const nts1_rx_note_on_t *note_on)
{
    printf("note_on %d\n", note_on->note);
}
void nts1_handle_step_tick_event(void)
{
    printf("tick\n");
}
void nts1_handle_unit_desc_event(const nts1_rx_unit_desc_t *unit_desc)
{
    printf("unit_desc %s %d %d %d\n",
           unit_desc->name,
           unit_desc->main_id,
           unit_desc->sub_id,
           unit_desc->param_count);
}
void nts1_handle_edit_param_desc_event(const nts1_rx_edit_param_desc_t *param_desc)
{
    printf("edit_param %s %d %d %d %d %d\n",
           param_desc->name,
           param_desc->main_id,
           param_desc->sub_id,
           param_desc->value_type,
           param_desc->min,
           param_desc->max);
}
void nts1_handle_value_event(const nts1_rx_value_t *value)
{
    printf("value %d %d %d %d %d\n",
           value->req_id,
           value->main_id,
           value->sub_id,
           value->padding,
           value->value);
}
void nts1_handle_param_change(const nts1_rx_param_change_t *param_change)
{
    printf("param_change %d %d %d %d\n",
           param_change->param_id,
           param_change->param_subid,
           param_change->msb,
           param_change->lsb);
}

void app_main(void)
{
    uint8_t init_status = nts1_init();
    bootloader_random_enable();

    uint8_t note = 0;
    uint8_t increase = 1;
    uint8_t velo = 100;
    uint8_t param_val = 0;

    int n = 0;
    while (1)
    {
        n++;
        // psts("idle", nts1_idle());
        nts1_idle();
        if (n != 10000)
        {
            continue;
        }
        n = 0;

        //psts("note on", nts1_note_on(note, velo));

        // if we get to the max number, change the direction
        if (note >= 127)
        {
            increase = 0;
        }
        else if (note <= 20)
        {
            increase = 1;
        }
        else
        {
            nts1_param_change(k_param_id_osc_base, k_param_subid_osc_edit1, param_val);
            nts1_param_change(k_param_id_osc_base, k_param_subid_osc_edit2, param_val);
            nts1_param_change(k_param_id_osc_base, k_param_subid_osc_edit3, param_val);
            nts1_param_change(k_param_id_osc_base, k_param_subid_osc_edit4, param_val);
            nts1_param_change(k_param_id_osc_base, k_param_subid_osc_edit5, param_val);
            nts1_param_change(k_param_id_osc_base, k_param_subid_osc_edit6, param_val);
            param_val = esp_random() % 126;

            nts1_note_on(note, velo);
        }
        // increase or decrease the note number
        note = increase ? note + 1 : note - 1;
    }
}
