#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include <stdint.h>

#include "rc433_protocol.h"
#include "rc433_receiver.h"

static const char *TAG = "MAIN_RADIO";

#define RX433_RECEIVE_TOLERANCE 60
#define RX433_MAX_CHANGES 67
#define RX433_SEPARATION_LIMIT 4300
// separationLimit: минимальный интервал (в микросекундах) между принятыми кодами;
// коды, следующие друг за другом с меньшим интервалом, игнорируются.
// Согласно спецификации RCSwitch (Arduino libary for remote control outlet switches) целесообразно
// установить этот предел равным длительности низкого уровня синхросигнала для текущего протокола.
// version=2.6.4

static DRAM_ATTR unsigned int rxTimings[RX433_MAX_CHANGES];
static DRAM_ATTR volatile uint32_t rxReceivedValue = 0;
static DRAM_ATTR volatile uint16_t rxReceivedBitLength = 0;
static DRAM_ATTR volatile unsigned int rxChangeCount = 0;
static DRAM_ATTR volatile int64_t rxLastTime = 0;
static DRAM_ATTR volatile unsigned int rxRepeatCount = 0;

static gpio_num_t active_gpio = GPIO_NUM_MAX;
static bool hardware_inverted = false; // Локальный флаг инверсии

// Промежуточная структура для кольцевого буфера (из ISR в задачу-фильтр)
typedef struct {
    rx_raw_packet_t raw_value; // Полное 32-битное значение (включая CRC внутри)
} radio_buffer_item_t;

// Кольцевой буфер для внутреннего сортировщика пакетов
RingbufHandle_t xRadioRingBuffer = NULL;
// Настройки размеров (Теория: буфер должен вмещать всплески шума)
#define RADIO_RING_BUF_ITEMS 32 // Вместимость промежуточного буфера (в пакетах)
#define BUSINESS_QUEUE_ITEMS 10 // Вместимость финальной очереди чистых данных

void vRadioFilterTask(void *pvParameters);

/**
 * @brief быстрый подсчет CRC8  по трем байтам полезной нагрузки
 * @param data 3 байта полезной нагрузки + байт CRC.
 *
 * @return валидность пакта данных.
 */
FORCE_INLINE_ATTR uint8_t calc_crc8_fast(uint32_t data) {
    uint8_t crc = 0x00;
    crc = crc8_table[crc ^ (data & 0xFF)];
    crc = crc8_table[crc ^ ((data >> 8) & 0xFF)];
    crc = crc8_table[crc ^ ((data >> 16) & 0xFF)];
    return crc;
}

/**
 * @brief Вычисляет абсолютную разницу между двумя беззнаковыми значениями.
 * @param a Первое значение.
 * @param b Второе значение.
 * @return Абсолютная разница между a и b.
 */
static IRAM_ATTR unsigned int rxDiff(unsigned int a, unsigned int b) {
    return (a > b) ? (a - b) : (b - a);
}

/**
 * @brief Распознаёт протокол на основе таймингов и извлекает закодированное значение.
 * @param pro массив протокола для проверки.
 * @param changeCount Количество изменений сигнала в буфере таймингов.
 * @return true — протокол распознан, значение и длина записаны в глобальные переменные;
 *         false — протокол не распознан.
 * @note Результат (code) записывается в rxReceivedValue, длина — в rxReceivedBitLength.
 */
static bool IRAM_ATTR rxDetectProtocol(const rxProtocol_t *pro, unsigned int changeCount) {
    if (changeCount <= 7 || changeCount > RX433_MAX_CHANGES)
        return false;

    unsigned long code = 0;
    const unsigned int syncLengthInPulses =
        (pro->syncFactor.low > pro->syncFactor.high) ? pro->syncFactor.low : pro->syncFactor.high;

    if (syncLengthInPulses == 0)
        return false;
    const unsigned int delay = rxTimings[0] / syncLengthInPulses;
    if (delay == 0)
        return false;

    const unsigned int delayTolerance = (delay * RX433_RECEIVE_TOLERANCE) / 100;

    // УЧИТЫВАЕМ ИНВЕРСИЮ ПРИЕМНИКА: Сдвигаем стартовый индекс парсинга, если логика
    // инвертирована
    bool final_invert = pro->invertedSignal ^ hardware_inverted;
    const unsigned int firstDataTiming = final_invert ? 2U : 1U;

    for (unsigned int i = firstDataTiming; i < changeCount - 1; i += 2U) {
        code <<= 1;
        const unsigned int zeroHigh = delay * pro->zero.high;
        const unsigned int zeroLow = delay * pro->zero.low;
        const unsigned int oneHigh = delay * pro->one.high;
        const unsigned int oneLow = delay * pro->one.low;

        if (rxDiff(rxTimings[i], zeroHigh) < delayTolerance &&
            rxDiff(rxTimings[i + 1], zeroLow) < delayTolerance) {
            // Бит 0
        } else if (rxDiff(rxTimings[i], oneHigh) < delayTolerance &&
                   rxDiff(rxTimings[i + 1], oneLow) < delayTolerance) {
            code |= 1UL;
        } else {
            return false;
        }
    }

    rxReceivedValue = (uint32_t)code;
    return true;
}

/**
 * @brief Обработчик прерывания по изменению уровня на GPIO 433 МГц.
 * @param arg Не используется.
 * @note Обрабатывает тайминги импульсов, распознаёт протокол (после второго повторения)
 *       и отправляет распознанный пакет в кольцевой буфер через xRingbufferSendFromISR.
 */
static void IRAM_ATTR rxIsrHandler(void *arg) {

    const int64_t time = esp_timer_get_time();
    const unsigned int duration = (unsigned int)(time - rxLastTime);

    // Если длительность больше лимита (то есть прилетела пауза-разделитель), мы делаем следующее:

    if (duration > RX433_SEPARATION_LIMIT) {
        // Вызываем проверку ОДИН раз для ОДНОГО протокола
        if (rxDetectProtocol(&active_proto, rxChangeCount) && xRadioRingBuffer != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            radio_buffer_item_t item;
            item.raw_value = rxReceivedValue;
            // Бросаем структуру в буфер
            xRingbufferSendFromISR(xRadioRingBuffer, &item, sizeof(item),
                                   &xHigherPriorityTaskWoken);

            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        }
        rxChangeCount = 0;
    }

    // ПРЕДОХРАНИТЕЛЬ: если буфер импульсов переполнился шумом — сбрасываем его в 0
    if (rxChangeCount >= RX433_MAX_CHANGES) {
        rxChangeCount = 0;
    }

    rxTimings[rxChangeCount++] = duration;
    rxLastTime = time;
}

/**
 * @brief Инициализирует приёмник 433 МГц: настраивает GPIO, прерывания,
 *        создаёт кольцевой буфер и задачу фильтрации.
 * @param config Указатель на структуру с параметрами инициализации.
 *               Не должен быть NULL; output_queue внутри также не должен быть NULL.
 * @return ESP_OK — успешная инициализация;
 *         ESP_ERR_INVALID_ARG — некорректные параметры;
 *         ESP_ERR_NO_MEM — не удалось выделить память под буфер или задачу;
 *         иная ошибка ESP-компонента при сбое GPIO/ISR-сервиса.
 * @note Создаётся задача vRadioFilterTask с приоритетом configMAX_PRIORITIES - 2,
 *       привязанная к ядру tskNO_AFFINITY.
 */
esp_err_t rc433_receiver_init(const rc433_config_t *config) {
    if (config == NULL || config->output_queue == NULL)
        return ESP_ERR_INVALID_ARG;

    esp_log_level_set(TAG, LOG_LOCAL_LEVEL);

    active_gpio = config->gpio_num;
    hardware_inverted = config->inverted_logic; // Запоминаем статус реверсивности

    rxReceivedValue = 0;
    rxReceivedBitLength = 0;

    // ====================================================================
    // Создание промежуточного кольцевого буфера (ISR -> Задача-Фильтр)
    // ====================================================================
    // Для RINGBUF_TYPE_NOSPLIT размер должен учитывать накладные расходы (8 байт на элемент)
    // и быть кратным 4 или 8 байтам для выравнивания в памяти.
    size_t item_size_with_overhead = sizeof(radio_buffer_item_t) + 8;
    size_t total_ring_buffer_size = item_size_with_overhead * RADIO_RING_BUF_ITEMS;

    // Округляем до ближайшего кратного 4 значения для безопасного выравнивания
    total_ring_buffer_size = (total_ring_buffer_size + 3) & ~3;

    xRadioRingBuffer = xRingbufferCreate(total_ring_buffer_size, RINGBUF_TYPE_NOSPLIT);
    if (xRadioRingBuffer == NULL) {
        ESP_LOGE(TAG, "Не удалось создать кольцевой буфер радио!");
        return ESP_ERR_NO_MEM; // Система не может работать
    }

    // Создаем переменную для хэндла задачи
    TaskHandle_t filter_task_handle = NULL;

    // Задача-Фильтр: Высокий приоритет (configMAX_PRIORITIES - 2).
    // Должна мгновенно вытеснять прикладной код при появлении данных в буфере.
    // Выделяем 3072 байт стека (достаточно для вычислений CRC и логики истории)
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        vRadioFilterTask, "radio_filter_tsk", 3072, (void *)config->output_queue,
        configMAX_PRIORITIES - 2, &filter_task_handle, tskNO_AFFINITY);

    if (xReturned != pdPASS) {
        // КАТАСТРОФА: Задача не создалась!
        // Обычно здесь выводят критическую ошибку в лог и останавливают систему
        ESP_LOGE("MAIN", "Критическая ошибка: Не удалось выделить память под vRadioFilterTask!");
        vRingbufferDelete(xRadioRingBuffer); // Буфер тоже удаляем
        xRadioRingBuffer = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Сбрасываем пин в исходное состояние, отключая его от аппаратных
    // периферийных матриц
    gpio_reset_pin(active_gpio);

    // НАСТРОЙКИ ЖЕЛЕЗА
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << active_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = config->pull_up,     // Берем из структуры
        .pull_down_en = config->pull_down, // Берем из структуры
        .intr_type =
            GPIO_INTR_ANYEDGE // Настройка фронта прерывания (ANYEDGE - стандарт для rc-switch)
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        // Если GPIO сбоит — чистим ресурсы и выходим
        vTaskDelete(filter_task_handle);
        vRingbufferDelete(xRadioRingBuffer);
        xRadioRingBuffer = NULL;
        return err;
    }

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    // Внимание: ESP_ERR_INVALID_STATE означает, что сервис ISR уже установлен кем-то другим.
    // Это НЕ ошибка, это нормальная ситуация, работу можно продолжать!
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // Если GPIO сбоит — чистим ресурсы и выходим
        vTaskDelete(filter_task_handle);
        vRingbufferDelete(xRadioRingBuffer);
        xRadioRingBuffer = NULL;
        return err;
    }

    err = gpio_isr_handler_add(active_gpio, rxIsrHandler, NULL);
    if (err != ESP_OK) {
        // Если не удалось привязать обработчик прерывания
        vTaskDelete(filter_task_handle);
        vRingbufferDelete(xRadioRingBuffer);
        xRadioRingBuffer = NULL;
        return err;
    }
    return ESP_OK;
}

// Настройки фильтра дедупликации
#define DEBOUNCE_MS 300 // Время (в мс), в течение которого игнорируются дубликаты
#define HISTORY_SIZE 4  // Сколько последних уникальных кодов храним в памяти

// Структура для хранения истории пакетов
typedef struct {
    rx_raw_packet_t raw_value; // Сырое значение для сравнения
    TickType_t last_seen_tick; // Время последнего обнаружения
} packet_history_t;

/**
 * @brief Проверяет, является ли пакет дубликатом в рамках временного окна.
 * @param raw_value Принятое 32-битное значение пакета для сравнения.
 * @return true — пакет является дубликатом (прошло менее DEBOUNCE_MS с последнего появления);
 *         false — пакет уникален или время с последнего появления превысило порог.
 * @note Использует статическую историю из HISTORY_SIZE последних пакетов с методом кольца.
 */
static bool is_duplicate_packet(rx_raw_packet_t raw_value) {
    // Выделяем память статически — она создается один раз при старте программы
    static packet_history_t packet_history[HISTORY_SIZE] = {0};
    static int history_index = 0;

    TickType_t current_tick = xTaskGetTickCount();

    // Ищем, есть ли уже этот пакет в нашей истории
    for (int i = 0; i < HISTORY_SIZE; i++) {
        if (packet_history[i].raw_value == raw_value) {
            // Пакет найден. Проверяем временное окно
            if ((current_tick - packet_history[i].last_seen_tick) < pdMS_TO_TICKS(DEBOUNCE_MS)) {
                // Время не вышло — это 100% дубликат из пачки. Обновляем время и отбрасываем.
                packet_history[i].last_seen_tick = current_tick;
                return true;
            }
            // Время вышло (прошло > 300мс), значит это новое нажатие кнопки/событие
            packet_history[i].last_seen_tick = current_tick;
            return false;
        }
    }

    // Если пакета в истории нет — записываем его на место самого старого (метод кольца)
    packet_history[history_index].raw_value = raw_value;
    packet_history[history_index].last_seen_tick = current_tick;

    history_index = (history_index + 1) % HISTORY_SIZE;

    return false;
}

/**
 * @brief Задача фильтрации: извлекает сырые пакеты из кольцевого буфера,
 *        проверяет CRC8, фильтрует дубликаты и отправляет валидные данные
 *        в очередь бизнес-логики.
 * @param pvParameters Указатель на очередь вывода (QueueHandle_t), переданный из инициализации.
 * @note Приоритет задачи — configMAX_PRIORITIES - 2. Работает на свободном ядре.
 *       Блокируется на xRingbufferReceive до появления новых данных.
 */
void vRadioFilterTask(void *pvParameters) {
    size_t itemSize;
    ESP_LOGI(TAG, "Задача фильтрации радиоэфира запущена.");

    while (1) {
        // Задача засыпает ТУТ. Она не тратит ни одного такта CPU, пока буфер пуст.
        // Как только прерывание выполнит xRingbufferSendFromISR, FreeRTOS мгновенно разбудит эту
        // задачу.
        radio_buffer_item_t *item =
            (radio_buffer_item_t *)xRingbufferReceive(xRadioRingBuffer, &itemSize, portMAX_DELAY);

        // Защитная проверка (на случай сбоев буфера)
        if (item == NULL) {
            continue;
        }

        // ====================================================================
        // Верификация контрольной суммы (CRC8)
        // ====================================================================
        // Вытаскиваем прилетевшую CRC8 из старшего байта (биты 24-31)
        uint8_t received_crc = (item->raw_value >> 24) & 0xFF;
        // Считаем контрольную сумму по трем байтам полезной нагрузки
        uint8_t calculated_crc = calc_crc8_fast(item->raw_value & 0x00FFFFFF);

        if (calculated_crc == received_crc) {

            // ====================================================================
            // Фильтрация временных дубликатов (Дедупликация)
            // ====================================================================
            if (!is_duplicate_packet(item->raw_value)) {

                // Пакет полностью валиден и уникален!
                // Собираем чистую структуру для бизнес-логики.
                rx433_data_t clean_data;
                clean_data.source = IDS_RX433;
                clean_data.protocol = RC433_TARGET_PROTOCOL;
                clean_data.value = item->raw_value;

                QueueHandle_t output_q = (QueueHandle_t)pvParameters;
                if (output_q != NULL) {
                    // Отправляем данные в финальную очередь прикладного уровня.
                    // Так как задача имеет высокий приоритет, мы делаем это без ожидания (таймаут =
                    // 0).
                    if (xQueueSend(output_q, &clean_data, 0) != pdPASS) {
                        ESP_LOGW(TAG, "Очередь бизнес-логики переполнена! Данные утеряны.");
                    } else {
                        ESP_LOGD(TAG, "Чистый пакет отправлен в бизнес-логику: 0x%08X",
                                 (unsigned int)item->raw_value);
                    }
                }

            } else {
                // Лог отладки (включается через menuconfig при необходимости)
                ESP_LOGD(TAG, "Дубликат пакета 0x%08X проигнорирован.",
                         (unsigned int)item->raw_value);
            }

        } else {
            // Сюда мы попадем, только если алгоритм rxDetectProtocol в прерывании ошибся
            // из-за идеального совпадения шума с таймингами протокола.
            ESP_LOGW(TAG, "Ошибка CRC8! Шум прорвался через парсер протоколов.");
        }

        // ====================================================================
        //  Обязательное освобождение памяти в Ring Buffer
        // ====================================================================
        // Если не вызвать эту функцию, буфер заполнится один раз и навсегда заблокируется.
        vRingbufferReturnItem(xRadioRingBuffer, (void *)item);
    }
}
