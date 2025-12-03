#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>
#include "i2cdev.h"
#include "pca9685.h"
#include "esp_log.h"
#include "pcf8574.h"
#include "pcf8563.h"
//#include "core.h"
#include "cJSON.h"
#include "utils.h"
#include "hardware.h"
#include "config.h"

#define SDA 32
#define SCL 33

#define IO_CLK   32
#define IO_LA595 33
#define IO_DI    34
#define IO_DO    12
#define IO_LA165 16
#define IO_EN    0
#define IO_REN   16
#define I2CPORT  0
#define MAXFREQ  1526

static const char *TAG = "HARDWARE";

static uint16_t relayValues = 0; // значения для реле
static uint16_t relayPrepareBits = 0; // флаг снижения скважности
static uint16_t relayBits = 0; // флаг снижения скважности
static bool i2c = false;
static SemaphoreHandle_t xMutex;
static bool clockPresent = false;
static uint16_t relPWM = 2000;
//i2c_dev_t dev_out1, dev_out2;

typedef struct {
    i2c_dev_t device;
    uint8_t address;
} device_t;
static device_t devices[9];

void sendTo595(uint8_t *values, uint8_t count) {
    // Функция просто отправит данные в 595 
    uint8_t value;
    uint8_t mask;
    for (uint8_t c=0;c<count;c++) {
        value = values[c];
        mask = 0b10000000; // старший бит
        //mask = 0b00000001; // сначала младший бит
        for (uint8_t i=0;i<8;i++) {
            gpio_set_level(IO_CLK, 0);                        
            gpio_set_level(IO_DO, 0);
            if (value & mask) {                
                gpio_set_level(IO_DO, 1);
            }
            gpio_set_level(IO_CLK, 1);            
            mask >>= 1;
            //mask <<= 1;
        }
    }
    gpio_set_level(IO_CLK, 0);
    gpio_set_level(IO_LA595, 1);
    gpio_set_level(IO_LA595, 0);    
    // #ifdef IO_EN
    gpio_set_level(IO_EN, 0); // OE enable                                       
    // #endif
}

void readFrom165(uint8_t *values, uint8_t count)
{
    gpio_set_level(IO_LA165, 0);
    gpio_set_level(IO_LA165, 1);
    for (uint8_t d=count; d>0; d--) {        
        values[d-1] = 0;
        for (uint8_t i=0; i<8; ++i) {
            gpio_set_level(IO_CLK, 0);
            //values[d-1] |= gpio_get_level(IO_DI) << (7 - i);            
            values[d-1] |= gpio_get_level(IO_DI) << i;            
            gpio_set_level(IO_CLK, 1);
        }        
    }
}

void setGPIOOut(uint8_t gpio) {
    gpio_pad_select_gpio(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
}

void setGPIOIn(uint8_t gpio) {
    gpio_pad_select_gpio(gpio);
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
}

void initHardware(SemaphoreHandle_t sem) {
    //xMutex = sem;
    xMutex = xSemaphoreCreateMutex();
    setGPIOOut(IO_EN);
    setGPIOOut(IO_REN);
    gpio_set_level(IO_EN, 1);    
    gpio_set_level(IO_REN, 1);     
    if (initI2Cdevices(&controllerType) == ESP_OK) {
        i2c = true;
        ESP_LOGI(TAG, "I2C inited. %s", getControllerTypeText(controllerType));
    } else {
        ESP_LOGI(TAG, "I2C not inited.");
        setGPIOOut(IO_CLK);
        setGPIOOut(IO_LA595);
        setGPIOOut(IO_LA165);
        setGPIOOut(IO_DO);
        setGPIOIn(IO_DI);        
        // для старых контроллеров нужно определить тип контроллера из конфигурации
        controllerType = UNKNOWN;        
    }
}

esp_err_t initPCA9685hw(i2c_dev_t dev, bool setValues) {
    esp_err_t err = ESP_FAIL;
    uint16_t freq = MAXFREQ;
    uint16_t nFreq = getConfigValueInt("hw/freq");    
    if (nFreq > 0) //  TODO : && nFreq < MAXFREQ
        freq = nFreq;
    uint16_t nRelPWM = getConfigValueInt("hw/pwm");    
    if (nRelPWM > 0)
        relPWM = nRelPWM;
    //if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {        
        err = pca9685_init(&dev);
        if (err == ESP_OK) {
            ESP_ERROR_CHECK(pca9685_restart(&dev));
            pca9685_set_pwm_frequency(&dev, freq);
            // set all to off
            if (setValues) {
                for (int i=0;i<16;i++) {
                    pca9685_set_pwm_value(&dev, i, 0);    	
                }
            }
        }        
    //    xSemaphoreGive(xMutex); 
    //}   
    return err;
    // err = pca9685_set_prescaler(&dev_out1, 10);
    // if (err != ESP_OK) {
    // 	ESP_LOGE(TAG, "Can't set prescaler");
    // 	return err;
    // }
}

void setI2COut(uint8_t adr, uint8_t num, uint16_t value) {
    if (!i2c) return;
//return;    
    if (value > 4096)
        value = 4096;
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {        
        pca9685_set_pwm_value(&devices[adr].device, num, value);
        xSemaphoreGive(xMutex); 
    }   
}

uint8_t readFrom8574(uint8_t adr) {
    if (!i2c) return 0;
//return 0;    
    uint8_t inputs;
    //adr 1,2,5,6
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {        
        pcf8574_port_read(&devices[adr].device, &inputs);
        xSemaphoreGive(xMutex); 
    }       
    return inputs;
}

void setRGBFace(char* color) {
    if (!i2c) return;
    uint16_t min = 1500;  
    uint8_t dev = 0;
    uint8_t r = 0, g = 0, b = 0;  
    if (controllerType == RCV2S) {
        dev = 4;
        r = 10;
        g = 11;
        b = 12;
    } else if (controllerType == RCV2B) {
        dev = 7;
        r = 13;
        g = 14;
        b = 15;    
    }

    if (!strcmp(color, "red")) {            
        setI2COut(dev, r, 4096);        
        setI2COut(dev, g, 4096);        
        setI2COut(dev, b, min);                    
    } else if (!strcmp(color, "yellow")) {            
        setI2COut(dev, r, 4096);        
        setI2COut(dev, g, min);        
        setI2COut(dev, b, min);                    
    } else if (!strcmp(color, "green")) {            
        setI2COut(dev, r, 4096);
        setI2COut(dev, g, min);        
        setI2COut(dev, b, 4096);                        
    } else {
        setI2COut(dev, r, 4096);        
        setI2COut(dev, g, 4096);        
        setI2COut(dev, b, 4096);
    }
    
}

// нужен таск для выставления и проверки значений PWM для реле.
// т.е. прилетает просто сигнал о необходимости включить реле, а таск уже сам должен выставить снижение тока
// если прилетает значение вкл для реле и при этом реле уже вкл, то не надо второй раз давать импульс,
// только если ранее реле было выключено
void relayTask(void *pvParameter) {    
    // SemaphoreHandle_t sem = getSemaphore();
    //cJSON *actionChild = (cJSON*)pvParameter;
    
    while (1) {
        if (1) {//(xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE) {
            for (uint8_t i=0;i<16;i++) {
                if (testbit(relayValues, i) && !testbit(relayPrepareBits, i)) {                
                    setbit(relayPrepareBits, i);
                } else if (testbit(relayValues, i) && !testbit(relayBits, i)) {
                    setI2COut(3, i, relPWM); // снижаем скважность
                    setbit(relayBits, i);
                }
            }
            // xSemaphoreGive(sem);
        }
        vTaskDelay(100 / portTICK_RATE_MS);        
    }    
    vTaskDelete(NULL);
}

void i2cScan(uint8_t found[], uint8_t *size) {
    i2c_dev_t dev = { 0 };
    dev.cfg.sda_io_num = SDA;
    dev.cfg.scl_io_num = SCL;
    dev.cfg.master.clk_speed = 100000;
    // uint8_t found[20];
    // uint8_t fCnt = 0;
    esp_err_t res;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    for (uint8_t addr = 3; addr < 0x78; addr++)
    {
        if (addr % 16 == 0)
            printf("\n%.2x:", addr);

        dev.addr = addr;
        res = i2c_dev_probe(&dev, I2C_DEV_WRITE);

        if (res == 0) {
            found[(*size)++] = addr;
            printf(" %.2x", addr);
        }
        else
            printf(" --");
    }
    printf("\n");
}

bool isMatched(uint8_t *array, uint8_t arraySize, uint8_t *pattern, uint8_t patternSize) {
    for (uint8_t i = 0; i < patternSize; i++) {
        bool found = false;
        for (uint8_t j = 0; j < arraySize; j++) {
            if (array[j] == pattern[i]) {
                found = true;
                break;
            }
        }
        if (!found)
            return false;
    }
    return true;
}

bool isInArray(uint8_t *array, uint8_t arraySize, uint8_t element) {
    for (uint8_t i = 0; i < arraySize; i++) {
        if (array[i] == element)
            return true;        
    }
    return false;
}

esp_err_t initPCA9685(uint8_t devNum, uint8_t *foundDevices, uint8_t devicesCount) {
    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (isInArray(foundDevices, devicesCount, devices[devNum].address)) {
        pca9685_init_desc(&devices[devNum].device, devices[devNum].address, I2CPORT, SDA, SCL);
        err = initPCA9685hw(devices[devNum].device, true);
        if (err == ESP_OK)
            ESP_LOGI(TAG, "PCA9685 with address 0x%x inited OK", devices[devNum].address);
        else
            ESP_LOGE(TAG, "Can't init PCA9685 address 0x%x", devices[devNum].address);
    }
    return err;
}

esp_err_t initI2Cdevices(enum controllerTypes *ctrlType) {    
    esp_err_t err = ESP_FAIL;  
    *ctrlType = UNKNOWN; 

    i2cdev_init();
    uint8_t foundDevices[20];
    uint8_t devicesCount = 0;
    i2cScan(foundDevices, &devicesCount);

    uint8_t aRCV2S[] = {0x20, 0x21, 0x40, 0x41};
    uint8_t aRCV2M[] = {0x20, 0x21, 0x22, 0x23, 0x27, 0x40, 0x41, 0x42};
    uint8_t aRCV2B[] = {0x20, 0x21, 0x22, 0x23, 0x40, 0x41, 0x42};
        
    if (isMatched(foundDevices, devicesCount, aRCV2M, sizeof(aRCV2M)/sizeof(uint8_t)))
        *ctrlType = RCV2M;
    else if (isMatched(foundDevices, devicesCount, aRCV2B, sizeof(aRCV2B)/sizeof(uint8_t)))
        *ctrlType = RCV2B;
    else if (isMatched(foundDevices, devicesCount, aRCV2S, sizeof(aRCV2S)/sizeof(uint8_t)))
        *ctrlType = RCV2S;    

    if (!devicesCount || *ctrlType == UNKNOWN) {
        ESP_LOGW(TAG, "No i2c device found or controllerType is unknown");
        return err;
    }
    
    /*
    // должна вернуть успех если есть минимальный набор
	RCV2S
	0x51 - clock
	0x20 - 8574 (relay)
    0x21 - 8574 (face)
	0x40 - 9685 (relay) 15 bit to 7 bit 8574
    0x41 - 9685 (face)	15 bit to 7 bit 8574
    ---
    RCV2B
	0x51 - clock
	0x20 - 8574 (relay)
    0x21 - 8574 (face)
    0x22 - 8574 (relay)
    0x23 - 8574 (face)
	0x40 - 9685 (relay) 
    0x41 - 9685 (face)	
    0x42 - 9685 (face)	
    ---
    RCV2M
    0x20 - 8574 (relay)
    0x21 - 8574 (face)
    0x22 - 8574 (relay)
    0x23 - 8574 (face)
    0x27 - 8574 (face rgb)  
    0x40 - 9685 (relay)  
    0x41 - 9685 (face)  
    0x42 - 9685 (face)   

    ---
    0x18 - i2c-ow DS2484
	*/
    for (uint8_t i=0; i<9;i++)
        memset(&devices[i].device, 0, sizeof(i2c_dev_t));
    // addresses
    devices[0].address = 0x51; // 8563 (clock)
    devices[1].address = 0x20; // 8574 (relay board)
    devices[2].address = 0x21; // 8574 (face board)
    devices[3].address = 0x40; // 9685 (relay board)
    devices[4].address = 0x41; // 9685 (face board)
    // big
    devices[5].address = 0x22; // 8574 (relay board)
    devices[6].address = 0x23; // 8574 (face board)
    devices[7].address = 0x42; // 9685 (face board)
    // medium
    devices[8].address = 0x27; // 8574 (face board)

    // у маленького и на плате реле и на плате индикации 15 bit 9685 соединен с 7 bit 8574

    // TODO : часы сделать
    if (isInArray(foundDevices, devicesCount, devices[0].address)) {
        pcf8563_init_desc(&devices[0].device, 0, SDA, SCL);        
        struct tm timeinfo;
        bool valid;
        if (pcf8563_get_time(&devices[0].device, &timeinfo, &valid) == ESP_OK) {
            clockPresent = true;
            ESP_LOGI(TAG, "%04d-%02d-%02d %02d:%02d:%02d, %s\n", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                     timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, valid ? "VALID" : "NOT VALID");
            if (valid && (timeinfo.tm_year + 1900 >= 2025)) {
                time_t now;
                //struct tm timeinfo;
                time(&now);
                
                //setenv("TZ", getSettingsValueString("ntpTZ"), 1);
                //tzset();
                localtime_r(&now, &timeinfo);
                ESP_LOGI(TAG, "Clock set");
            }
            // if (!valid) {
            //         struct tm time = {
            //         .tm_year = 120, // years since 1900
            //         .tm_mon  = 3,   // months since January
            //         .tm_mday = 3,
            //         .tm_hour = 12,
            //         .tm_min  = 35,
            //         .tm_sec  = 10,
            //         .tm_wday = 0    // days since Sunday
            //     };
            //     ESP_ERROR_CHECK(pcf8563_set_time(&dev, &time));

            // }
        } else {
            ESP_LOGI(TAG, "No clock chip present");
        }
    }

    pcf8574_init_desc(&devices[1].device, devices[1].address, I2CPORT, SDA, SCL);
    pcf8574_init_desc(&devices[2].device, devices[2].address, I2CPORT, SDA, SCL);
    pcf8574_init_desc(&devices[5].device, devices[5].address, I2CPORT, SDA, SCL);
    pcf8574_init_desc(&devices[6].device, devices[6].address, I2CPORT, SDA, SCL);
    //pcf8574_init_desc(&devices[8].device, devices[8].address, I2CPORT, SDA, SCL);

    //3,4,7    
    initPCA9685(4, foundDevices, devicesCount);
    initPCA9685(7, foundDevices, devicesCount);
    initPCA9685(3, foundDevices, devicesCount);
 
    gpio_set_level(IO_EN, 0);
    setRGBFace("yellow");  
    gpio_set_level(IO_REN, 0);
 
    xTaskCreate(&relayTask, "relayTask", 4096, NULL, 5, NULL);
    return ESP_OK;
}

void setRelayValues(uint16_t values) {
    for (uint8_t i=0;i<16;i++) {
        if (!testbit(values, i) && testbit(relayValues, i)) {
            // выключение
            setI2COut(3, i, 0);            
            clrbit(relayValues, i);            
        } else if (testbit(values, i) && !testbit(relayValues, i)) {
            // включение
            setI2COut(3, i, 4096);            
            setbit(relayValues, i);
            clrbit(relayBits, i);
            clrbit(relayPrepareBits, i);
        }
    }
    relayValues = values;
}

esp_err_t setClock() {
    if (!clockPresent) return ESP_ERR_NOT_FOUND;
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "setClock is %s", asctime(&timeinfo));    
    return pcf8563_set_time(&devices[0].device, &timeinfo);
}

esp_err_t getClock(struct tm time) {
    bool valid;
    esp_err_t ret = pcf8563_get_time(&devices[0].device, &time, &valid);     
    if (!valid)
        return ESP_FAIL;
    return ret;
}

void updateStateHW(uint16_t outputs, uint16_t inputsLeds, uint16_t outputsLeds) {
    // обновление состояния выходов, индикации
    // outputs - выходы реле, inputsLeds - индикация входов, outputsLeds - индикация выходов
    uint8_t values[6] = {0};
    if (controllerType == RCV1S) {
        values[1] = revByte(outputs << 2);    // TODO : проверить не перепутал ли я выходы и индикацию    
        values[0] = (inputsLeds & 0xF0) >> 4;        
        values[0] |= outputsLeds << 4;
        values[0] = revByte(values[0]);
        sendTo595(values, 2);       
    } else if (controllerType == RCV1B) {       
        values[5] = outputs >> 8;       // TODO : проверить не перепутал ли я выходы и индикацию    
        values[4] = outputs;
        values[3] = inputsLeds;
        values[2] = inputsLeds >> 8;
        values[1] = outputsLeds;
        values[0] = outputsLeds >> 8;
        sendTo595(values, 6);               
    } else if (controllerType == RCV2S) {       
        // TODO : сделать для новых
        // отдельно реле, индикация
        setRelayValues(outputs);

        for (uint8_t i=0; i<6; i++) {
            setI2COut(4, i, (inputsLeds & (0x1 << i) ) > 0 ? 0 : 4096);        
        }
        for (uint8_t i=0; i<4; i++) {
            setI2COut(4, i+6, (inputsLeds & (0x1 << i) ) > 0 ? 0 : 4096);        
        }
    } else if (controllerType == RCV2B) {   
        setRelayValues(outputs);

        for (uint8_t i=0; i<16; i++) {
            setI2COut(4, i, (inputsLeds & (0x1 << i) ) > 0 ? 0 : 4096);        
        }
        for (uint8_t i=0; i<12; i++) {
            setI2COut(7, i, (outputsLeds & (0x1 << i) ) > 0 ? 0 : 4096);        
        }
    }   
}

void readInputs(uint8_t *values, uint8_t count) {
    // чтение входов
    //ESP_LOGI(TAG, "readInputs cnt %d type %d", count, controllerType);
    if (controllerType == RCV1S || controllerType == RCV1B) {    
        readFrom165(values, count);        
    } else if (controllerType == RCV2S || controllerType == RCV2B || controllerType == RCV2M) {
        if (count == 2) {
            values[0] = readFrom8574(1) & 0x3F;
            values[1] = readFrom8574(2) & 0x0F;
            //ESP_LOGI(TAG, "Inputs2 %d %d", values[0], values[1]);
        } else if (count == 4) {
            values[0] = readFrom8574(1); // relay board inputs
            values[1] = readFrom8574(5); // relay board inputs
            values[2] = readFrom8574(2); // face
            values[3] = readFrom8574(6); // face
        }
    } else {
        // заглушка для неизвестных типов
        for (uint8_t i=0;i<count;i++)
            values[i] = 0xFF;
    }   
}

uint16_t readServiceButtons() { 
    // чтение сервисных кнопок (только при включении)
    // TODO : должен читать только сервисные кнопки
    // вернет 2 байта с соответствующими значениями, справа 0 бит. 1 кнопка нажата, 0 нет
    uint8_t inputs[BINPUTS];
    uint16_t buttons = 0x0;
    if (controllerType == RCV1S) {    
        readFrom165(inputs, 2);
        //ESP_LOGI(TAG, "readFrom165 %d %d", inputs[0], inputs[1]);
        buttons = inputs[0]>>4;
        buttons |= 0xFFF0;
        buttons = ~buttons;        
    } else if (controllerType == RCV1B) {
        readFrom165(inputs, 4);
        buttons = inputs[0]; // первые 8 левых кнопок байт 0
        buttons<<=2;
        buttons |= (inputs[1] & 0xC0) >> 6; // две кнопки справа. маска т.к. входы в воздухе
        buttons |= 0xFFC0; // чтобы заполнить 6 первых бит единицами после сдвига на 6
        buttons = ~buttons;
    } else if (controllerType == RCV2B) {
        inputs[0] = readFrom8574(2); // face
        inputs[1] = readFrom8574(6); // face
        buttons = inputs[1];
        buttons<<=8;
        buttons |= inputs[0];
        buttons = ~buttons;          
    } else {
        // TODO : добавить чтение значений кнопок для новых контроллеров
    }
    ESP_LOGI(TAG, "readServiceButtons inputs 0x%02x%02x buttons 0x%02x", inputs[1], inputs[0], buttons);    
    return buttons;
}

char* getControllerTypeText(uint8_t type) {
    switch (type) {
        case RCV1S:
            return "small";
        case RCV1B:    
            return "big";
        case RCV2S:    
            return "RCV2S";
        case RCV2M:    
            return "RCV2M";
        case RCV2B:    
            return "RCV2B";        
    }
    return "UNKNOWN";
}




/*
bool isDevicePresent(uint8_t address) {
    ESP_LOGI(TAG, "isDevicePresent begin %x", address);
    esp_err_t err = ESP_FAIL;
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {        
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = SDA,
            .scl_io_num = SCL,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000,        
        };
        i2c_param_config(I2CPORT, &conf);
        i2c_driver_install(I2CPORT, I2C_MODE_MASTER, 0, 0, 0);

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 0x1);
        i2c_master_stop(cmd);
        err = i2c_master_cmd_begin(I2CPORT, cmd, 50 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);        
        //esp_err_t err2 = i2c_driver_delete(I2CPORT);    
        ESP_LOGI(TAG, "isDevicePresent 0x%x %s", address, esp_err_to_name(err));
        xSemaphoreGive(xMutex); 
    }
    return err == ESP_OK;
}
*/