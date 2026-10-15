#ifndef RC433_RECEIVER_H
#define RC433_RECEIVER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/gpio_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// маркер пакета. В принципе мертвый код, оставлен как маркер структуры события
typedef enum { IDS_RX433 = 1 } data_source_t;
// сам пакет - 4 байта
typedef uint32_t rx_raw_packet_t;
// структура отдаваемая в очередь для обработки конечной логикой
typedef struct {
    rx_raw_packet_t value; // все 4 байта проверенные на валидность
    data_source_t source;  // маркер структуры события
    uint8_t protocol;      // маркер протокола радио
} rx433_data_t;

// СТРУКТУРА НАСТРОЕК
// очередь для данных от радиомодуля должна быть создана в вызывающем модуле
// в этой реализации пакет это 32бита данных
// размер очереди теоритически должен вместить все пакеты с валидным CRC битом
// В этой реализации чтение происходит по одному протоколу, жестко установленному на передатчике и
// приемнике при компиляции. Номер протокола возможно изменить  в заголовочном файле
// "rc433_protocol.h" Соответствует спецификации RCSwitch (Arduino libary for remote control outlet
// switches) version=2.6.4
typedef struct {
    QueueHandle_t output_queue; // Ссылка на внешнюю очередь
    gpio_num_t gpio_num;        // Номер пина (GPIO_NUM_XX)
    gpio_pullup_t pull_up;      // GPIO_PULLUP_DISABLE или GPIO_PULLUP_ENABLE
    gpio_pulldown_t pull_down;  // GPIO_PULLDOWN_DISABLE или GPIO_PULLDOWN_ENABLE
    bool inverted_logic;        // true - если приемник инвертирует физический уровень
                                // (реверсивный)
} rc433_config_t;

esp_err_t rc433_receiver_init(const rc433_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // RC433_RECEIVER_H
