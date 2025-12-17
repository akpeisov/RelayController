#include "esp_log.h"
#include "esp_system.h"
#include "storage.h"
#include "webServer.h"
#include "utils.h"
#include "freertos/semphr.h"
#include "ota.h"
#include "core.h"
#include "network.h"
#include "ws.h"
#include "cJSON.h"
#include "hardware.h"
#include "time.h"
#include "config.h"
#include "modbus.h"
#include "mqtt.h"
#include "ftp.h"

static const char *TAG = "CORE";
static SemaphoreHandle_t sem_busy;
static cJSON *IOConfig;
static cJSON *jScheduler;
static cJSON *jMQTTTopics;
static cJSON *mbSlaves;
static bool mbSlave = false;
static bool mqttConnected = false;
static bool wsConnected = false;
static bool wsSendLogs = false;
static uint8_t mbSlaveId = 0;
static char* mbMode = "";
static esp_reset_reason_t resetReason;
static bool gWSTimeSet = false;

extern const char jwt_start[] asm("_binary_jwt_pem_start");
extern const char jwt_end[] asm("_binary_jwt_pem_end");

typedef struct {
	char name[10];
	uint8_t outputs;
	uint8_t inputs;
	uint8_t buttons;
} ControllerData;
ControllerData controllersData[] = {
  {"UNKNOWN", 0, 0, 0},    
  {"RCV1S", 4, 4, 4},    
  {"RCV1B", 10, 16, 10},  
  {"RCV2S", 4, 6, 4},   // D4MG
  {"RCV2M", 6, 8, 6},   // D5MG
  {"RCV2B", 12, 16, 12}  
};
uint32_t serviceButtonsTime[16] = {0}; // in ms
uint32_t inputButtonsTime[32] = {0}; // in ms
void processScheduler();
static bool reboot = false;

void determinateControllerType() {
    if (controllerType == UNKNOWN) {
        char *cType = getConfigValueString("controllerType");
        ESP_LOGI(TAG, "getConfigValueString %s", cType == NULL ? "NULL" : cType);
        if (cType != NULL) {
            if (!strcmp(cType, "small") || !strcmp(cType, "RCV1S")) {
                controllerType = RCV1S;
            } else if (!strcmp(cType, "big") || !strcmp(cType, "RCV1B")) {
                controllerType = RCV1B;
            }            
        } else {
            controllerType = RCV1S; // by default
        }
        // write to config
        setConfigValueString("controllerType", controllersData[controllerType].name);
    }        
}

SemaphoreHandle_t getSemaphore() {
    return sem_busy;
}

void createSemaphore() {
    sem_busy = xSemaphoreCreateMutex();
}

esp_err_t createIOConfig() {
    #define VARBUFFER 20
    ESP_LOGI(TAG, "Creating default IO config");
    IOConfig = cJSON_CreateObject(); // overwrite existing object
    // outputs
    char *name = NULL;
    name = malloc(VARBUFFER);
    
	cJSON *outputs = cJSON_CreateArray();    
    for (uint8_t i=0; i<controllersData[controllerType].outputs; i++) {
        cJSON *output = cJSON_CreateObject();
        cJSON_AddNumberToObject(output, "id", i);
        snprintf(name, VARBUFFER, "Out %d", i);    
        cJSON_AddStringToObject(output, "name", name);
        cJSON_AddStringToObject(output, "type", "s");
        cJSON_AddStringToObject(output, "state", "off");        
        cJSON_AddItemToArray(outputs, output);        
    }
    cJSON_AddItemToObject(IOConfig, "outputs", outputs);

	cJSON *inputs = cJSON_CreateArray();    
    // service buttons
    for (uint8_t i=0; i<controllersData[controllerType].buttons; i++) {
        cJSON *input = cJSON_CreateObject();
        cJSON_AddNumberToObject(input, "id", i+16);
        snprintf(name, VARBUFFER, "Svc %d", i);
        cJSON_AddStringToObject(input, "name", name);
        cJSON_AddStringToObject(input, "type", "BTN");
                
        cJSON *actions = cJSON_CreateArray();
        cJSON *action = cJSON_CreateObject();
        cJSON_AddNumberToObject(action, "order", 0);
        cJSON_AddNumberToObject(action, "output", i);
        cJSON_AddStringToObject(action, "action", "toggle");
        cJSON_AddItemToArray(actions, action);
        
        cJSON *events = cJSON_CreateArray();
        cJSON *event = cJSON_CreateObject();
        cJSON_AddStringToObject(event, "event", "toggle");
        cJSON_AddItemToObject(event, "actions", actions);
        cJSON_AddItemToArray(events, event);

        cJSON_AddItemToObject(input, "events", events);
        cJSON_AddItemToArray(inputs, input);
    }
    // external inputs    
    for (uint8_t i=0; i<controllersData[controllerType].inputs; i++) {
        cJSON *input = cJSON_CreateObject();
        cJSON_AddNumberToObject(input, "id", i);
        snprintf(name, VARBUFFER, "In %d", i);
        cJSON_AddStringToObject(input, "name", name);
        cJSON_AddStringToObject(input, "type", "SW");
        
        cJSON *actionsOn = cJSON_CreateArray();
        cJSON *actionOn = cJSON_CreateObject();
        cJSON_AddNumberToObject(actionOn, "order", 0);
        cJSON_AddNumberToObject(actionOn, "output", i>controllersData[controllerType].outputs-1?controllersData[controllerType].outputs-1:i);
        cJSON_AddStringToObject(actionOn, "action", "on");
        cJSON_AddItemToArray(actionsOn, actionOn);
        
        cJSON *eventsOn = cJSON_CreateArray();
        cJSON *eventOn = cJSON_CreateObject();
        cJSON_AddStringToObject(eventOn, "event", "on");
        cJSON_AddItemToObject(eventOn, "actions", actionsOn);
        cJSON_AddItemToObject(input, "events", eventsOn);

        cJSON *actionsOff = cJSON_CreateArray();
        cJSON *actionOff = cJSON_CreateObject();
        cJSON_AddNumberToObject(actionOff, "order", 0);
        cJSON_AddNumberToObject(actionOff, "output", i>controllersData[controllerType].outputs-1?controllersData[controllerType].outputs-1:i);
        cJSON_AddStringToObject(actionOff, "action", "off");
        cJSON_AddItemToArray(actionsOff, actionOff);
        
        cJSON *eventsOff = cJSON_CreateArray();
        cJSON *eventOff = cJSON_CreateObject();
        cJSON_AddStringToObject(eventOff, "event", "off");
        cJSON_AddItemToObject(eventOff, "actions", actionsOff);
        cJSON_AddItemToObject(input, "events", eventsOff);

        cJSON_AddItemToArray(inputs, input);
    }   
    cJSON_AddItemToObject(IOConfig, "inputs", inputs);
    
    free(name);
    // TODO : outputs & other
    return ESP_OK;
}

const char* esp_reset_reason_to_string(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_UNKNOWN:
            return "Unknown reset reason";
        case ESP_RST_POWERON:
            return "Power on reset";
        case ESP_RST_EXT:
            return "External pin reset";
        case ESP_RST_SW:
            return "Software reset";
        case ESP_RST_PANIC:
            return "Panic/exception reset";
        case ESP_RST_INT_WDT:
            return "Internal watchdog reset";
        case ESP_RST_TASK_WDT:
            return "Task watchdog reset";
        case ESP_RST_WDT:
            return "Reset due to other watchdogs";
        case ESP_RST_DEEPSLEEP:
            return "Reset after exiting deep sleep mode";    
        case ESP_RST_BROWNOUT:
            return "Brownout reset";
        case ESP_RST_SDIO:
            return "SDIO reset";
        default:
            return "Unknown/Invalid reset reason";
    }
}

void serviceTask(void *pvParameter) {
    ESP_LOGI(TAG, "Creating service task");
    // uint32_t minMem = getConfigValueInt("watchdog", "wdtmemsize");
    while(1) {   
        // every 1 second     
        // if ((minMem > 0) && (esp_get_free_heap_size() < minMem)) {
        //     ESP_LOGE(TAG, "HEAP memory WDT triggered. Actual free memory is %d. Restarting...", esp_get_free_heap_size());
        //     esp_restart();
        // }
        if (reboot) {
            static uint8_t cntReboot = 0;
            if (cntReboot++ >= 3) {
                ESP_LOGI(TAG, "Reboot now!");
                esp_restart();
            }
        }        
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

cJSON* getDeviceInfoJson() {
    cJSON *status = cJSON_CreateObject();    
    char *uptime = getUpTime();
    char *curdate = getCurrentDateTime("%d.%m.%Y %H:%M:%S");
    char *hostname = getConfigValueString("name");
    char *description = getConfigValueString("description");
    char *version = getCurrentVersion();
    char *ethip = getETHIPStr();
    char *wifiip = getWIFIIPStr();
    char *mac = getMac();
    cJSON_AddItemToObject(status, "mac", cJSON_CreateString(mac));
    cJSON_AddItemToObject(status, "freeMemory", cJSON_CreateNumber(esp_get_free_heap_size()));
    cJSON_AddItemToObject(status, "uptime", cJSON_CreateString(uptime));
    cJSON_AddItemToObject(status, "uptimeRaw", cJSON_CreateNumber(getUpTimeRaw()));    
    cJSON_AddItemToObject(status, "curdate", cJSON_CreateString(curdate));
    cJSON_AddItemToObject(status, "name", cJSON_CreateString(hostname));
    cJSON_AddItemToObject(status, "description", cJSON_CreateString(description));          
    cJSON_AddItemToObject(status, "version", cJSON_CreateString(version));
    cJSON_AddItemToObject(status, "wifiRSSI", cJSON_CreateNumber(getRSSI()));
    cJSON_AddItemToObject(status, "ethIP", cJSON_CreateString(ethip));
    cJSON_AddItemToObject(status, "wifiIP", cJSON_CreateString(wifiip));      
    cJSON_AddItemToObject(status, "model", cJSON_CreateString(controllersData[controllerType].name));      
    cJSON_AddItemToObject(status, "resetReason", cJSON_CreateString(esp_reset_reason_to_string(resetReason)));          
    free(uptime);
    free(curdate);  
    free(version);
    free(ethip);
    free(wifiip);
    return status;
}

void sendInfo() {
    if (!wsConnected && !mqttConnected) {
        return;
    }
    cJSON *payload = getDeviceInfoJson();
    if (!cJSON_IsObject(payload)) {
        cJSON_Delete(payload);
        return;
    }
    if (mqttConnected) {
        char topic[50] = {0};
        strcpy(topic, getConfigValueString("name"));
        if (strlen(topic) == 0) {
            strcpy(topic, "unknown");
        }
        strcat(topic, "/info\0");
        char *data = cJSON_PrintUnformatted(payload);
        MQTTPublish(topic, data);        
        free(data);
    }
    if (wsConnected) {
        cJSON *info = cJSON_CreateObject();
        cJSON_AddStringToObject(info, "type", "INFO");
        cJSON_AddItemToObject(info, "payload", payload);
        char *infoBuf = cJSON_PrintUnformatted(info);
        if (infoBuf != NULL) {      
            WSSendMessageForce(infoBuf);
            free(infoBuf);
            cJSON_Delete(info);     
        }   
    }  
    //cJSON_Delete(payload);  
}

void sendHello() {
    cJSON *hello = NULL;
    cJSON *payload = NULL;  
    char *token = WSgetToken(getMac());
    if (token == NULL) {
        ESP_LOGE(TAG, "sendHello error, token is null");        
        token = malloc(5);
        strcpy(token, "NULL\0");
    }
    hello = cJSON_CreateObject();
    payload = cJSON_CreateObject();

    if (!payload || !hello) {
        free(token);
        ESP_LOGE(TAG, "sendHello error. payload or hello object is null");        
        return;
    }
    
    cJSON_AddStringToObject(hello, "type", "HELLO");
    if (strcmp(token, "NULL")) {
        cJSON_AddStringToObject(payload, "token", token);
    } else {
        cJSON_AddStringToObject(payload, "mac", getMac());
    }
    cJSON_AddStringToObject(payload, "type", "relayController");
    if (strcmp(mbMode, "")) {
        cJSON_AddStringToObject(payload, "mbMode", mbMode);
        if (!strcmp(mbMode, "slave")) {
            cJSON_AddNumberToObject(payload, "mbSlaveId", mbSlaveId);
        }
    }
    //cJSON_AddItemToObject(payload, "info", getDeviceInfoJson());
    // add slaveId 
    cJSON_AddItemToObject(hello, "payload", payload);
    char *hello_str = cJSON_PrintUnformatted(hello);    
    cJSON_Delete(hello);
    WSSendMessageForce(hello_str);
    free(hello_str);
    free(token);
}

void updateValues() {
    uint16_t outputs = 0;
    uint16_t inputsLeds = 0;
    uint16_t outputsLeds = 0;

    if (!cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "outputs")) ||
        !cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "inputs"))) {
        ESP_LOGE(TAG, "bad config");
        return;
    }

    cJSON *childOutput = cJSON_GetObjectItem(IOConfig, "outputs")->child;
    while (childOutput) {
        uint8_t slaveId = 0;
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "slaveId")))
            slaveId = cJSON_GetObjectItem(childOutput, "slaveId")->valueint;        
        if (!cJSON_IsString(cJSON_GetObjectItem(childOutput, "state"))) {
            cJSON_AddStringToObject(childOutput, "state", "off");
        }
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "id")) &&
            slaveId == 0) {
            //ESP_LOGI(TAG, "id %d, state %s", cJSON_GetObjectItem(childOutput, "id")->valueint, )
            if (!strcmp(cJSON_GetObjectItem(childOutput, "state")->valuestring, "on")) {
                setbit(outputs, cJSON_GetObjectItem(childOutput, "id")->valueint);
                setbit(outputsLeds, cJSON_GetObjectItem(childOutput, "id")->valueint);
            }
        }
        childOutput = childOutput->next;
    }
    cJSON *childInput = cJSON_GetObjectItem(IOConfig, "inputs")->child;
    while (childInput) {  
        uint8_t slaveId = 0;
        if (cJSON_IsNumber(cJSON_GetObjectItem(childInput, "slaveId")))
            slaveId = cJSON_GetObjectItem(childInput, "slaveId")->valueint;        
        if (!cJSON_IsString(cJSON_GetObjectItem(childInput, "state"))) {
            cJSON_AddStringToObject(childInput, "state", "off");
        }
        if (cJSON_IsNumber(cJSON_GetObjectItem(childInput, "id")) &&
            slaveId == 0) {
            if (!strcmp(cJSON_GetObjectItem(childInput, "state")->valuestring, "on"))
                setbit(inputsLeds, cJSON_GetObjectItem(childInput, "id")->valueint);
        }
        childInput = childInput->next;
    }
    
    updateStateHW(outputs, inputsLeds, outputsLeds);   

    // текущие значения для modbus
    MBUpdateData(outputs, inputsLeds);     
}

void showError(uint8_t err) {
    updateStateHW(0, err & 0x0F, 0);
}

void initInputs() {
    // по умолчанию все входы должны быть выключены, иначе отображается неправильно на индикации
    cJSON *childInput = cJSON_GetObjectItem(IOConfig, "inputs")->child;
    while (childInput) {  
        if (cJSON_IsString(cJSON_GetObjectItem(childInput, "state"))) {
            cJSON_ReplaceItemInObject(childInput, "state", cJSON_CreateString("off"));        
        } else {
            cJSON_AddItemToObject(childInput, "state", cJSON_CreateString("off"));        
        }
        childInput = childInput->next;
    }
}

void initOutputs() {
    // инициализация выходов при включении
    // значение default может быть off или on
    cJSON *childOutput = cJSON_GetObjectItem(IOConfig, "outputs")->child;
    while (childOutput) {
        if (!cJSON_IsString(cJSON_GetObjectItem(childOutput, "type"))) {
            cJSON_AddItemToObject(childOutput, "type", cJSON_CreateString("s"));
        }
        if (!cJSON_IsString(cJSON_GetObjectItem(childOutput, "default"))) {
            cJSON_AddItemToObject(childOutput, "default", cJSON_CreateString("off"));
        }

        if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "default"))) {
            if (!strcmp(cJSON_GetObjectItem(childOutput, "default")->valuestring, "on")) {            
                cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString("on"));
            } else if (!strcmp(cJSON_GetObjectItem(childOutput, "default")->valuestring, "off")) {            
                cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString("off"));
            }
        }
        childOutput = childOutput->next;
    }
	// выключаем все выходы
	updateStateHW(0x0, 0x0, 0x0);
}

void sendWSUpdateOutput(uint8_t pSlaveId, uint8_t pOutput, char* pState, uint16_t pTimer) {
    cJSON *payload = cJSON_CreateObject();
    cJSON_AddStringToObject(payload, "mac", getMac());  
    cJSON_AddItemToObject(payload, "output", cJSON_CreateNumber(pOutput));
    cJSON_AddStringToObject(payload, "state", pState);
    if (pSlaveId > 0) {
        cJSON_AddItemToObject(payload, "slaveId", cJSON_CreateNumber(pSlaveId));
    }
    cJSON_AddItemToObject(payload, "timer", cJSON_CreateNumber(pTimer));
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "type", "UPDATE");
    cJSON_AddItemToObject(json, "payload", payload);
    char* message = cJSON_PrintUnformatted(json);
    WSSendMessage(message);
    free(message);
    cJSON_Delete(json);    
}

void sendWSUpdateInput(uint8_t pSlaveId, uint8_t pInput, char* pState) {
    cJSON *payload = cJSON_CreateObject();      
    cJSON_AddStringToObject(payload, "mac", getMac());
    cJSON_AddItemToObject(payload, "input", cJSON_CreateNumber(pInput));
    cJSON_AddStringToObject(payload, "state", pState);
    if (pSlaveId > 0) {
        cJSON_AddItemToObject(payload, "slaveId", cJSON_CreateNumber(pSlaveId));
    }
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "type", "UPDATE");
    cJSON_AddItemToObject(json, "payload", payload);
    char* message = cJSON_PrintUnformatted(json);
    WSSendMessage(message);
    free(message);
    cJSON_Delete(json);
}

void publishOutput(uint8_t pSlaveId, uint8_t pOutput, char* pValue, uint8_t pTimer) {
    sendWSUpdateOutput(pSlaveId, pOutput, pValue, pTimer);

    if (mqttConnected) {
        char topic[50] = {0};
        // hostname/outputs/slaveId/output
        sprintf(topic, "%s/outputs/%d/%d", getConfigValueString("name"), pSlaveId, pOutput);
        /*
        char buf[5];
        strcpy(topic, getConfigValueString("name"));
        if (strlen(topic) == 0) {
            strcpy(topic, getMac());
        }
        strcat(topic, "/outputs/");
        itoa(pSlaveId, buf, 10);
        strcat(topic, buf);
        strcat(topic, "/");
        itoa(pOutput, buf, 10);
        strcat(topic, buf); 
        strcat(topic, "\0");   
        */
        // напрямую toUpper вызывает ошибку
        char actionUpper[10];
        strcpy(actionUpper, pValue);    
        strcat(actionUpper, "\0");           
        MQTTPublish(topic, toUpper(actionUpper));        
    }
}

void publishInput(uint8_t pInput, char* pState, uint8_t pSlaveId) {
    sendWSUpdateInput(pSlaveId, pInput, pState);
    if (mqttConnected) {
        char topic[50] = {0};
        // hostname/inputs/slaveId/output
        sprintf(topic, "%s/inputs/%d/%d", getConfigValueString("name"), pSlaveId, pInput);
        char actionUpper[15];
        strcpy(actionUpper, pState);    
        strcat(actionUpper, "\0");           
        MQTTPublish(topic, toUpper(actionUpper));        
    }
}

char* getOutputState(uint8_t pOutput) {
    if (!cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "outputs"))) {        
        return "off";
    }    
    cJSON *childOutput = cJSON_GetObjectItem(IOConfig, "outputs")->child;
    while (childOutput) {  
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "slaveId")) &&
            (cJSON_GetObjectItem(childOutput, "slaveId")->valueint > 0)) {            
            childOutput = childOutput->next;
            continue;
        }
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "id")) &&
            (cJSON_GetObjectItem(childOutput, "id")->valueint == pOutput)) {            
            if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "state")))
                return cJSON_GetObjectItem(childOutput, "state")->valuestring;            
        }
        childOutput = childOutput->next;
    }
    return "off";
}

char* getInputState(uint8_t pInput) {
    if (!cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "inputs"))) {        
        return "off";
    }    
    cJSON *childInput = cJSON_GetObjectItem(IOConfig, "inputs")->child;
    while (childInput) {  
        if (cJSON_IsNumber(cJSON_GetObjectItem(childInput, "id")) &&
            (cJSON_GetObjectItem(childInput, "id")->valueint == pInput)) {            
            if (cJSON_IsString(cJSON_GetObjectItem(childInput, "state")))
                return cJSON_GetObjectItem(childInput, "state")->valuestring;            
        }
        childInput = childInput->next;
    }
    return "off";
}

void setOutput(uint8_t pOutput, char* pValue) {
    // установка значений для выхода        
    ESP_LOGI(TAG, "setOutput output %d, value %s", pOutput, pValue);
    if (!cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "outputs"))) {
        ESP_LOGE(TAG, "IOConfig outputs isn't array");
        return;
    }        
    if (!strcmp(pValue, "toggle")) {
        pValue = !strcmp(getOutputState(pOutput), "on") ? "off" : "on";
    }
    // касательно длительности. Приоритет длительности из правала. Т.е. если на входе стоит длительность 5, а в правиле 10, то выход включится на 10 сек
    uint16_t timer = 0;
    uint16_t oLimit = 0;
    cJSON *childOutput = cJSON_GetObjectItem(IOConfig, "outputs")->child;
    while (childOutput) {
        // skip all slaves
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "slaveId")) &&
            (cJSON_GetObjectItem(childOutput, "slaveId")->valueint > 0)) {
            childOutput = childOutput->next;
            continue;
        }
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "id")) &&
            (cJSON_GetObjectItem(childOutput, "id")->valueint == pOutput)) {
            cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString(pValue));

            // получить ограничение по времени для выхода
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "limit")))
                oLimit = cJSON_GetObjectItem(childOutput, "limit")->valueint;
            
            if (oLimit > 0) {
                // если есть ограничение запускаем таймер
                timer = oLimit;                
            }

            // если это тепличный таймер то взять значение длительности устанавливаемого состояния (on/off)
            if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "type"))  &&
                !strcmp(cJSON_GetObjectItem(childOutput, "type")->valuestring, "t")) {
                if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, pValue))) {
                    timer = cJSON_GetObjectItem(childOutput, pValue)->valueint;    
                }                
            }

            // если это one-shot таймер, то выставить 3 (0.3 секунды) для таймера
            if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "type"))  &&
                !strcmp(cJSON_GetObjectItem(childOutput, "type")->valuestring, "os")) {
                timer = 3;
            }
            
            // установить новое значение таймера для выхода
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "timer")))
                cJSON_ReplaceItemInObject(childOutput, "timer", cJSON_CreateNumber(timer));
            else
                cJSON_AddItemToObject(childOutput, "timer", cJSON_CreateNumber(timer));
            break;            
        }
        childOutput = childOutput->next;
    }          
    publishOutput(0, pOutput, pValue, timer);    
}

void setRemoteOutput(uint8_t pSlaveId, uint8_t pOutput, char* pValue) {
    // nothing
    ESP_LOGI(TAG, "setRemoteOutput. slaveId %d, output %d, action %s",
             pSlaveId, pOutput, pValue);
    MBSetRemoteOutput(pSlaveId, pOutput, pValue);
    //publishOutput(pSlaveId, pOutput, pValue);
}

void setAllOff() {
    // выставить все выходы в состояние выкл, включая тепличные таймеры и слейвы модбаса
    if (!cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "outputs"))) {
        ESP_LOGE(TAG, "IOConfig outputs isn't array");
        return;
    }
    char *action = "off";
    
    uint16_t timer = 0;
    cJSON *childOutput = cJSON_GetObjectItem(IOConfig, "outputs")->child;
    while (childOutput) {  
        uint8_t slaveId = 0;
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "slaveId")))
            slaveId = cJSON_GetObjectItem(childOutput, "timer")->valueint;
        if (slaveId > 0) {
            // для слейвов
            setRemoteOutput(slaveId, 
                            cJSON_GetObjectItem(childOutput, "id")->valueint, 
                            action);
        } else {
            // для самого устройства        
            cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString(action));
            // установить новое значение таймера для выхода
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "timer")))
                cJSON_ReplaceItemInObject(childOutput, "timer", cJSON_CreateNumber(timer));
            else
                cJSON_AddItemToObject(childOutput, "timer", cJSON_CreateNumber(timer));
        }
        childOutput = childOutput->next;
    }
    // TODO: MQTT publish one for all or for each
}

void actionsTask(void *pvParameter) {    
    // таск для обработки событий    
    SemaphoreHandle_t sem = getSemaphore();
    cJSON *actionChild = (cJSON*)pvParameter;
    uint16_t duration = 0;
    while (1) {
        // сначала проверяется текущее значение duration. Если больше нуля, то просто ждем
        if (duration) {
            duration--;            
            vTaskDelay(1000 / portTICK_RATE_MS);
        } else {
            if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE) {
                if (actionChild) {
                    // предполагаю что массив будет правильно отсортирован на стороне фронта
                    if (cJSON_IsString(cJSON_GetObjectItem(actionChild, "action")) &&
                       (!strcmp(cJSON_GetObjectItem(actionChild, "action")->valuestring, "wait"))) {      
                        // wait
                        if (cJSON_IsNumber(cJSON_GetObjectItem(actionChild, "duration"))) {
                            duration = cJSON_GetObjectItem(actionChild, "duration")->valueint;                        
                        }
                    } else if (cJSON_IsString(cJSON_GetObjectItem(actionChild, "action")) &&
                       (!strcmp(cJSON_GetObjectItem(actionChild, "action")->valuestring, "allOff"))) {
                        setAllOff();
                    } else if (cJSON_IsNumber(cJSON_GetObjectItem(actionChild, "output")) &&
                               cJSON_IsString(cJSON_GetObjectItem(actionChild, "action"))) {
                        // action                        
                        if (cJSON_IsNumber(cJSON_GetObjectItem(actionChild, "slaveId")) &&
                            cJSON_GetObjectItem(actionChild, "slaveId")->valueint > 0) {
                            setRemoteOutput(cJSON_GetObjectItem(actionChild, "slaveId")->valueint, 
                                            cJSON_GetObjectItem(actionChild, "output")->valueint, 
                                            cJSON_GetObjectItem(actionChild, "action")->valuestring);
                        } else {                        
                            setOutput(cJSON_GetObjectItem(actionChild, "output")->valueint, 
                                      cJSON_GetObjectItem(actionChild, "action")->valuestring);
                        }
                    }
                    actionChild = actionChild->next;
                } else {
                    // закончился массив, выходим и убиваем таск                    
                    break;
                }
            } 
            xSemaphoreGive(sem);
        }        
    } 
    xSemaphoreGive(sem);   
    vTaskDelete(NULL);
}

bool checkACL(cJSON *acls) {    
    if (!cJSON_IsArray(acls)) {
        ESP_LOGE(TAG, "acls not array");
        return false;
    }
    cJSON *childACL = acls->child;
    while (childACL) {                                    
        if (cJSON_IsString(cJSON_GetObjectItem(childACL, "state")) &&
            cJSON_IsString(cJSON_GetObjectItem(childACL, "io")) &&
            cJSON_IsNumber(cJSON_GetObjectItem(childACL, "id")) &&
            cJSON_IsString(cJSON_GetObjectItem(childACL, "type"))) {
            // acl is valid
            // если правило allow, то проверить вход/выход на соответствие, если не сойдется то НЕЛЬЗЯ    
            // если правило deny, то проверить вход/выход на соответствие, если сойдется то НЕЛЬЗЯ
            if ((!strcmp(cJSON_GetObjectItem(childACL, "type")->valuestring, "deny") &&
                ((!strcmp(cJSON_GetObjectItem(childACL, "io")->valuestring, "input") &&
                  !strcmp(getInputState(cJSON_GetObjectItem(childACL, "id")->valueint), 
                          cJSON_GetObjectItem(childACL, "state")->valuestring)) || 
                 (!strcmp(cJSON_GetObjectItem(childACL, "io")->valuestring, "output") &&
                  !strcmp(getOutputState(cJSON_GetObjectItem(childACL, "id")->valueint), 
                          cJSON_GetObjectItem(childACL, "state")->valuestring))
                )) || 
                (!strcmp(cJSON_GetObjectItem(childACL, "type")->valuestring, "allow") &&    
                ((!strcmp(cJSON_GetObjectItem(childACL, "io")->valuestring, "input") &&        
                  strcmp(getInputState(cJSON_GetObjectItem(childACL, "id")->valueint),     
                          cJSON_GetObjectItem(childACL, "state")->valuestring)) || 
                 (!strcmp(cJSON_GetObjectItem(childACL, "io")->valuestring, "output") &&
                  strcmp(getOutputState(cJSON_GetObjectItem(childACL, "id")->valueint), 
                          cJSON_GetObjectItem(childACL, "state")->valuestring))
                ))) {                                                                                        
                return true;                
            }                                        
        } else {
            ESP_LOGE(TAG, "Non valid ACL");
            return false;
        }
        childACL = childACL->next;
    }
    return false;
}

void processInputEvents(uint8_t pSlaveId, uint8_t pInput, char* pEvent, uint8_t i) {
    // TODO : обработать i
	// обработка события на входе/кнопке    
    ESP_LOGI(TAG, "processInputEvents. Input %d, event %s, slaveId %d",
             pInput, pEvent, pSlaveId);
    cJSON *childInput = cJSON_GetObjectItem(IOConfig, "inputs")->child;
    while (childInput) {
        uint8_t slaveId = 0; // определяем slaveId. Для мастера будет 0
        if (cJSON_IsNumber(cJSON_GetObjectItem(childInput, "slaveId")))
            slaveId = cJSON_GetObjectItem(childInput, "slaveId")->valueint;

        if (cJSON_IsNumber(cJSON_GetObjectItem(childInput, "id")) && 
            cJSON_GetObjectItem(childInput, "id")->valueint == pInput &&
            slaveId == pSlaveId) { 
            if (cJSON_IsArray(cJSON_GetObjectItem(childInput, "events"))) {
                // find event
                cJSON *childEvent;
                childEvent = cJSON_GetObjectItem(childInput, "events")->child;
                while (childEvent) {
                    // поиск нужного события
                    if (!strcmp(cJSON_GetObjectItem(childEvent, "event")->valuestring, pEvent)) {
                        // проверка ACL
                        if (cJSON_IsArray(cJSON_GetObjectItem(childEvent, "acls"))) {
                            if (checkACL(cJSON_GetObjectItem(childEvent, "acls"))) {
                                ESP_LOGE(TAG, "ACL denied");
                                // выйти из обработки события
                                // TODO : publish to websocket
                                break;
                            }
                        }

                        // обработка действий
                        if (cJSON_IsArray(cJSON_GetObjectItem(childEvent, "actions"))) {
                            // если одно действие то сразу выполнить, а если несколько то создать отдельный таск?
                            if (cJSON_GetArraySize(cJSON_GetObjectItem(childEvent, "actions")) == 1) {
                                // обычное правило  
                                cJSON *child = cJSON_GetObjectItem(childEvent, "actions")->child;
                                char *action = cJSON_GetObjectItem(child, "action")->valuestring;
                                uint8_t slaveId = 0;
                                if (cJSON_IsNumber(cJSON_GetObjectItem(child, "slaveId")))
                                    slaveId = cJSON_GetObjectItem(child, "slaveId")->valueint;
                                if (pSlaveId > 0 && slaveId == pSlaveId && !getActionOnSameSlave()) {
                                    // если это правило для слейва и действие для слейва и установлен параметр, то игнор
                                    break;
                                }
                                if (!strcmp(action, "allOff")) {
                                    setAllOff();                                    
                                } else if (slaveId > 0) {
                                    setRemoteOutput(slaveId, cJSON_GetObjectItem(child, "output")->valueint, action);
                                } else {
                                    setOutput(cJSON_GetObjectItem(child, "output")->valueint, action);
                                }
                            } else {
                                // цепочка действий, запускаем таск...
                                char taskName[14];
                                sprintf(taskName, "actiontask_%d", pInput);
                                TaskHandle_t xHandle = xTaskGetHandle(taskName);
                                if (xHandle != NULL) {
                                    // если по данному входу уже работает таск то удалить его
                                    vTaskDelete(xHandle);
                                }                                
                                xTaskCreate(&actionsTask, taskName, 4096, (void *)cJSON_GetObjectItem(childEvent, "actions")->child, 5, NULL);
                            }
                        }
                        break;
                    }                    
                    childEvent = childEvent->next;   
                }
                break;
            }
        }
        childInput = childInput->next;
    }

    // publish input    
    if (!strcmp(pEvent, "on") || !strcmp(pEvent, "off")) {
        publishInput(pInput, pEvent, 0);    
    }   

    // for link controller
    if (pInput == 16 && !strcmp(pEvent, "longpress")) {
        publishInput(pInput, pEvent, 0);    
    }
}

uint8_t correctInput(uint8_t pInput) {
    if (controllerType == RCV1S) {
        if ((pInput >= 4) && (pInput <= 7)) {
            return pInput+12;
        }
        if ((pInput >= 12) && (pInput <= 15)) {
            return pInput-12;
        }
    } else if (controllerType == RCV1B) {
        if ((pInput >= 16) && (pInput <= 31)) {
            return pInput-16;
        }
        if (pInput <= 7) { 
            return pInput+18;
        }
        if ((pInput >= 14) && (pInput <= 15)) {
            return pInput+2;
        }
    } else if ((controllerType == RCV2S) && pInput >= 8) {
        return pInput+8;
    } else if ((controllerType == RCV2B) && 1==1) {    // TODO : ???
        return pInput;
    }
    return 255;
}

void processInput(uint8_t pInput, uint8_t pEvent) {
    // найти в конфиге указанный вход и посмотреть его правила 
    // пока так - если кнопка и вкл то игнорим и ждем пока кнопка станет выкл
    uint8_t tmp = pInput;
    pInput = correctInput(pInput);
    if (pInput == 0xFF) {
        ESP_LOGW(TAG, "processInput. Ignoring trash");
        return;    
    }    
    ESP_LOGW(TAG, "processInput. orig %d, corrected %d, event %d", tmp, pInput, pEvent);
    //ESP_LOGW(TAG, "processInput #%d, event %d", pInput, pEvent);
    
    char *event = "off";
    uint8_t i = 255;

    // find input and event    
    if (!cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "inputs"))) {
        ESP_LOGE(TAG, "IOConfig inputs isn't array");
        return;
    }    
    cJSON *childInput = cJSON_GetObjectItem(IOConfig, "inputs")->child;
    while (childInput) {       
        if (cJSON_IsNumber(cJSON_GetObjectItem(childInput, "id")) && 
            cJSON_GetObjectItem(childInput, "id")->valueint == pInput) { 
            // update value
            char *state = "off";
            if (pEvent == 1) {
                state = "on";
            }
            if (cJSON_IsString(cJSON_GetObjectItem(childInput, "state")))
                cJSON_ReplaceItemInObject(childInput, "state", cJSON_CreateString(state));
            
            if (!strcmp(cJSON_GetObjectItem(childInput, "type")->valuestring, "INVSW")) {
                // переключатель
                if ((cJSON_IsNumber(cJSON_GetObjectItem(childInput, "ci"))) &&
                    (cJSON_GetObjectItem(childInput, "ci")->valueint > 0)) {
					// переключатель со счетчиком. для Саши делали. 
                    i = cJSON_GetObjectItem(childInput, "i")->valueint;
                    uint8_t ci = cJSON_GetObjectItem(childInput, "ci")->valueint;
                    ESP_LOGI(TAG, "i=%d  ci=%d", i, ci);
                    if (i+1 < ci) {
                        cJSON_ReplaceItemInObject(childInput, "i", cJSON_CreateNumber(i+1));
                    } else {
                        cJSON_ReplaceItemInObject(childInput, "i", cJSON_CreateNumber(0));
                    }
                }
                event = "toggle";                
            } else if (!strcmp(cJSON_GetObjectItem(childInput, "type")->valuestring, "SW")) {
                // выключатель
                //ESP_LOGI(TAG, "found input %d as switch", pInput);
                if (pEvent == 1)
                    event = "on";
                else
                    event = "off";
            } else {
                // кнопка
                if (pEvent == 1) {
                    // button is pressed, save timer
                    inputButtonsTime[pInput] = esp_timer_get_time() / 1000 & 0xFFFFFFFF;					
                    return;
                } else {
                    // button is released, get time
					if ((esp_timer_get_time() / 1000 & 0xFFFFFFFF) - inputButtonsTime[pInput] < 1000) {
						// click
						event = "toggle";
					} else {
						// long press
						event = "longpress";
					}					
                    ESP_LOGI(TAG, "Button %d event %s", pInput, event);                    
                }                
            }
            processInputEvents(0, pInput, event, i);
            // для слейва нужно записать событие для последующей передачи на мастер
            // если это сам мастер, то ничего страшного если он в свои регистры запишет, их все равно никто не прочитает
            MBAddInputEvent(pInput, event);
            return;            
        }
        childInput = childInput->next;
    }
}

void outputsTimer() {
    // check all active outputs for timeout
    // run every 1 second
    if (!cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "outputs"))) {        
        return;
    }    
    cJSON *childOutput = cJSON_GetObjectItem(IOConfig, "outputs")->child;
    while (childOutput) {  
        if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "type")) &&
            !strcmp(cJSON_GetObjectItem(childOutput, "type")->valuestring, "t")) {
            // это триггер, тепличный таймер
            uint16_t timer = 0;
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "timer"))) {
                timer = cJSON_GetObjectItem(childOutput, "timer")->valueint;
                if (timer > 0)
                    timer--; 
                if (timer == 0) {
                    if (!strcmp(cJSON_GetObjectItem(childOutput, "state")->valuestring, "on")) {
                        cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString("off"));
                        timer = cJSON_GetObjectItem(childOutput, "off")->valueint;
                    } else {
                        cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString("on"));
                        timer = cJSON_GetObjectItem(childOutput, "on")->valueint;
                    }                    
                    // publish(cJSON_GetObjectItem(childOutput, "id")->valueint,
                    //         cJSON_GetObjectItem(childOutput, "state")->valuestring, 0);                           
                }
                cJSON_ReplaceItemInObject(childOutput, "timer", cJSON_CreateNumber(timer));
                publishOutput(0, cJSON_GetObjectItem(childOutput, "id")->valueint, 
                              cJSON_GetObjectItem(childOutput, "state")->valuestring, timer); 
            } else {
                cJSON_AddItemToObject(childOutput, "timer", cJSON_CreateNumber(1));
            }            
        } else if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "state")) &&
            !strcmp(cJSON_GetObjectItem(childOutput, "state")->valuestring, "on")) {
            // это обычные выходы, которые сейчас активны
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "timer"))) {                
                uint16_t timer = 0;
                if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "timer")))
                    timer = cJSON_GetObjectItem(childOutput, "timer")->valueint;
                if (timer) {
                    // есть текущий таймер
                    timer--;
                    cJSON_ReplaceItemInObject(childOutput, "timer", cJSON_CreateNumber(timer));
                    if (timer == 0) {
                        // switch off output
                        cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString("off"));
                        ESP_LOGI(TAG, "outputTimer set output %d to off", cJSON_GetObjectItem(childOutput, "id")->valueint);
                        //publish(cJSON_GetObjectItem(childOutput, "id")->valueint, "OFF", 0);
                    }
                    publishOutput(0, cJSON_GetObjectItem(childOutput, "id")->valueint, 
                              cJSON_GetObjectItem(childOutput, "state")->valuestring, timer);                     
                }
            }
        } 
        
        childOutput = childOutput->next;
    }
}

void outputsTimerShot() {
    // test for shooter
    // every 100 ms
    if (!cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "outputs"))) {        
        return;
    }    
    cJSON *childOutput = cJSON_GetObjectItem(IOConfig, "outputs")->child;
    while (childOutput) {  
        if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "type")) &&
            !strcmp(cJSON_GetObjectItem(childOutput, "type")->valuestring, "shooter")) {
            uint16_t timer = 0;
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "timer"))) {
                timer = cJSON_GetObjectItem(childOutput, "timer")->valueint;
                if (timer > 0) {                    
                    timer--;                
                }
                if (timer == 0) {
                    if (!strcmp(cJSON_GetObjectItem(childOutput, "state")->valuestring, "on")) {
                        cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString("off"));
                        timer = cJSON_GetObjectItem(childOutput, "off")->valueint;
                    } else {
                        cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString("on"));
                        timer = cJSON_GetObjectItem(childOutput, "on")->valueint;
                    }                                        
                }
                //sendWSUpdateOutputTimer(0, cJSON_GetObjectItem(childOutput, "id")->valueint, timer);
                cJSON_ReplaceItemInObject(childOutput, "timer", cJSON_CreateNumber(timer));
            } else {
                cJSON_AddItemToObject(childOutput, "timer", cJSON_CreateNumber(1));
            }            
        } else if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "type")) &&
            !strcmp(cJSON_GetObjectItem(childOutput, "type")->valuestring, "os")) {
            // one shot for 0.5 sec
            uint16_t timer = 0;
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "timer"))) {
                timer = cJSON_GetObjectItem(childOutput, "timer")->valueint;
                if (timer > 0)
                    timer--;                
                if (timer == 0) {
                    cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString("off"));                    
                }
                cJSON_ReplaceItemInObject(childOutput, "timer", cJSON_CreateNumber(timer));
                //sendWSUpdateOutputTimer(0, cJSON_GetObjectItem(childOutput, "id")->valueint, timer);
            } else {
                cJSON_AddItemToObject(childOutput, "timer", cJSON_CreateNumber(1));
            }            
        }
        childOutput = childOutput->next;
    }
}

void inputsTask(void *pvParameter) {    
	// игнорировать нажатые ранее входы
	SemaphoreHandle_t sem = getSemaphore(); 
    uint8_t cnt_timer = 0;
    uint8_t sch_timer = 0;
	uint8_t inputsNew[BINPUTS];    
    uint8_t inputsOld[BINPUTS] = {0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t diff[BINPUTS];
	uint8_t inputsCnt = 2; // for RCV1S, RCV2S
	if ((controllerType == RCV1B) || (controllerType == RCV2B))
		inputsCnt = 4;
  //   else if (controllerType == RCV1S)
  //       inputsCnt = 2;
  //   else if (controllerType == RCV2S)
		// inputsCnt = 2;
	while (1) {
		// опрос изменения входов и кнопок
        if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE) {
			// анализ входов
			readInputs(inputsNew, inputsCnt);
            //ESP_LOGI(TAG, "Inputs %d  %d %d ctl %d", inputsCnt, inputsNew[0], inputsNew[1], controllerType);
            for (uint8_t i=0; i<inputsCnt; i++) {
                diff[i] = inputsNew[i] ^ inputsOld[i];
                if (diff[i] > 0) {                                        
                    for (uint8_t b=0; b<8; b++) {
                        if (diff[i] >> b & 0x01) {                        
                            // ESP_LOGI(TAG, "i %d b %d", i, b);
                            processInput(i*8 + b, inputsOld[i] >> b & 0x01);
                        }
                    }
                    inputsOld[i] = inputsNew[i];                    
                }
            }
			
            outputsTimerShot();
            if (++cnt_timer >= 10) {
                cnt_timer = 0;
                outputsTimer();
                if (++sch_timer >= 60) {
                    sch_timer = 0;
                    processScheduler();
                    sendInfo();
                }
            }
            // выставление значений на платах      
            updateValues();

			xSemaphoreGive(sem);
        } else {
            ESP_LOGI(TAG, "inputsTask task semaphore is busy");
        }

        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void resetDefaultConfigs() {
    ESP_LOGW(TAG, "Resetting device config");    
    if (createIOConfig() == ESP_OK) {
        setConfigValueObject("io", IOConfig);     
        saveConfig();
    }
}

bool checkServiceButtons() {
    uint16_t btns = readServiceButtons() & 0x0F;
    if (btns) {
        ESP_LOGI(TAG, "Something pressed while boot... %d", btns);
        showError(0x0F);
        //xTaskCreate(&buttonsStartUpTask, "buttonsStartUpTask", 4096, btns, 5, NULL);
        updateStateHW(0, 0, 0);
        switch (btns) {
            case 0b1001: // правая и левая кнопки
              startSoftAP();
              // no need reboot
              break;
            case 0b0101: // правая и через одну
              resetDefaultConfigs();
              reboot = true;
              // need reboot
              break;
            case 0b0011:
              resetNetworkConfig();
              reboot = true;
              // need reboot
              break;
            default:
              ESP_LOGI(TAG, "switch default");
              return false;
        }
        return true;
    }
    return false;
}

void startInputTask() {
	ESP_LOGI(TAG, "Starting input task");
    xTaskCreate(&inputsTask, "inputsTask", 4096, NULL, 5, NULL);    
}

esp_err_t getDeviceInfo(char **response) {
    cJSON *info = getDeviceInfoJson();
    if (cJSON_IsObject(info)) {
        *response = cJSON_Print(info);
    }
    cJSON_Delete(info);
    return ESP_OK;    
}

esp_err_t mbtest(char **response, char *content) {
    ESP_LOGI(TAG, "mbtest");

    cJSON *parent = cJSON_Parse(content);
    if (!cJSON_IsObject(parent)) {
        setErrorTextJson(response, "Is not a JSON object");
        cJSON_Delete(parent);
        return ESP_FAIL;
    }
    
    if (cJSON_IsNumber(cJSON_GetObjectItem(parent, "input")) &&
        cJSON_IsString(cJSON_GetObjectItem(parent, "event"))) {
        ESP_LOGI(TAG, "mbtest Input %d. Event %s", 
                 cJSON_GetObjectItem(parent, "input")->valueint, 
                 cJSON_GetObjectItem(parent, "event")->valuestring);
        MBAddInputEvent(cJSON_GetObjectItem(parent, "input")->valueint, 
                        cJSON_GetObjectItem(parent, "event")->valuestring);
        setTextJson(response, "OK");
        cJSON_Delete(parent);
        return ESP_OK;
    } else {
        setErrorTextJson(response, "Only input/event allowed");
        cJSON_Delete(parent);
    }
    return ESP_OK;
}

esp_err_t ioservice(char **response, char *content) {
    ESP_LOGI(TAG, "ioservice");

    cJSON *parent = cJSON_Parse(content);
    if (!cJSON_IsObject(parent))
    {
        setErrorTextJson(response, "Is not a JSON object");
        cJSON_Delete(parent);
        return ESP_FAIL;
    }
    
    if (cJSON_IsNumber(cJSON_GetObjectItem(parent, "input")) &&
        cJSON_IsString(cJSON_GetObjectItem(parent, "event"))) {
        ESP_LOGI(TAG, "ioservice Input %d. Event %s", 
                 cJSON_GetObjectItem(parent, "input")->valueint, 
                 cJSON_GetObjectItem(parent, "event")->valuestring);
        processInputEvents(0, cJSON_GetObjectItem(parent, "input")->valueint, 
                           cJSON_GetObjectItem(parent, "event")->valuestring, 255);
        setTextJson(response, "OK");    
        cJSON_Delete(parent);
        return ESP_OK;
    }

    if (cJSON_IsNumber(cJSON_GetObjectItem(parent, "output")) &&
        cJSON_IsString(cJSON_GetObjectItem(parent, "action"))) {
        ESP_LOGI(TAG, "ioservice Output %d. Action %s.", 
                 cJSON_GetObjectItem(parent, "output")->valueint, 
                 cJSON_GetObjectItem(parent, "action")->valuestring);
        char *action = cJSON_GetObjectItem(parent, "action")->valuestring;
        if (!strcmp(action, "on") || !strcmp(action, "off") || !strcmp(action, "toggle")) {
            setTextJson(response, "OK");                
            setOutput(cJSON_GetObjectItem(parent, "output")->valueint, action);
        } else {
            setErrorTextJson(response, "Action not found!");
        }        
        cJSON_Delete(parent);
        return ESP_OK;    
    }

    setErrorTextJson(response, "Unknown data");        
    return ESP_FAIL;    
}

char* getDeviceIOStates() {
    char *response;
    cJSON *json = cJSON_CreateObject();
    cJSON_AddItemToObject(json, "type", cJSON_CreateString("IOSTATES"));
    // io states
    cJSON *jOuts = cJSON_CreateArray();
    cJSON *childOutput = cJSON_GetObjectItem(IOConfig, "outputs")->child;
    while (childOutput) {
        if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "state"))) {
            cJSON *jOut = cJSON_CreateObject();
            cJSON_AddNumberToObject(jOut, "id", cJSON_GetObjectItem(childOutput, "id")->valueint);
            cJSON_AddStringToObject(jOut, "state", cJSON_GetObjectItem(childOutput, "state")->valuestring); 
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "slaveId")))
                cJSON_AddNumberToObject(jOut, "slaveId", cJSON_GetObjectItem(childOutput, "slaveId")->valueint);
            cJSON_AddItemToArray(jOuts, jOut);  
        }
        childOutput = childOutput->next;    
    }
    cJSON *jIns = cJSON_CreateArray();
    cJSON *childInput = cJSON_GetObjectItem(IOConfig, "inputs")->child;
    while (childInput) { 
        if (cJSON_IsString(cJSON_GetObjectItem(childInput, "type")) &&
            strcmp(cJSON_GetObjectItem(childInput, "type")->valuestring, "BTN") && 
            cJSON_IsString(cJSON_GetObjectItem(childInput, "state"))) {
            // only for inputs
            cJSON *jIn = cJSON_CreateObject();
            cJSON_AddNumberToObject(jIn, "id", cJSON_GetObjectItem(childInput, "id")->valueint);
            cJSON_AddStringToObject(jIn, "state", cJSON_GetObjectItem(childInput, "state")->valuestring);   
            if (cJSON_IsNumber(cJSON_GetObjectItem(childInput, "slaveId")))
                cJSON_AddNumberToObject(jIn, "slaveId", cJSON_GetObjectItem(childInput, "slaveId")->valueint);
            cJSON_AddItemToArray(jIns, jIn);        
        }
        childInput = childInput->next;  
    }   
    cJSON *jObj = cJSON_CreateObject();
    cJSON_AddItemToObject(jObj, "outputs", jOuts);
    cJSON_AddItemToObject(jObj, "inputs", jIns);    
    cJSON_AddItemToObject(json, "payload", jObj);
    response = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    return response;
}

void initScheduler() {
    ESP_LOGI(TAG, "Initiating scheduler");
    // сбрасываем у всех задач признак выполнения
    jScheduler = getConfigValueObject("scheduler"); 
    if (!cJSON_IsObject(jScheduler)) {
        jScheduler = cJSON_CreateObject();
        cJSON_AddArrayToObject(jScheduler, "tasks");
    }   
    cJSON *tasks = cJSON_GetObjectItem(jScheduler, "tasks");
    cJSON *childTask = tasks->child;
    while (childTask) {       
        if (cJSON_IsBool(cJSON_GetObjectItem(childTask, "done")))
            cJSON_ReplaceItemInObject(childTask, "done", cJSON_CreateFalse());
        else
            cJSON_AddItemToObject(childTask, "done", cJSON_CreateFalse());        
        if (!cJSON_IsString(cJSON_GetObjectItem(childTask, "name"))) {
            cJSON_AddItemToObject(childTask, "name", cJSON_CreateString("Noname task"));        
        }
        if (!cJSON_IsBool(cJSON_GetObjectItem(childTask, "enabled")))            
            cJSON_AddItemToObject(childTask, "enabled", cJSON_CreateFalse());        
        childTask = childTask->next;    
    }
}

void processScheduler() {
    // TODO : for modbus??
    // run minutely    
    // time_t rawtime;
    // struct tm *info;
    // time(&rawtime);
    // info = localtime(&rawtime);
    //struct tm *info;
    if (!getConfigValueBool("scheduler/enabled")) {
        return;
    }
    struct tm *info = getTime();
    uint16_t currentTime = info->tm_hour*60 + info->tm_min;
    uint16_t grace = 0; // период, в который еще можно запустить задачу
    static uint16_t lastSchedulerTime = 0;
    char *curdate = getCurrentDateTime("%d.%m.%Y %H:%M:%S");
    ESP_LOGI(TAG, "Scheduler time %d %s. Day of week %d. Free memory %d", currentTime, curdate, info->tm_wday, esp_get_free_heap_size());
    free(curdate);
    if (currentTime < lastSchedulerTime) {        
        // когда перевалит за 0.00
        initScheduler();
    }
    lastSchedulerTime = currentTime;
    // currentTime - minutes from 0.00
    // пройти все задачи, время которых еще не настало отметить как done = false
    // время которых прошло или наступило проверить на грейс период, по умолчанию он 5 мин
    // если задача со статусом done = false - выполнить ее и пометить как выполненная
    cJSON *tasks = cJSON_GetObjectItem(jScheduler, "tasks");
    cJSON *childTask = tasks->child;
    while (childTask) {       
        ESP_LOGD(TAG, "Scheduler task %s done is %d", 
                 cJSON_GetObjectItem(childTask, "name")->valuestring, 
                 cJSON_IsTrue(cJSON_GetObjectItem(childTask, "done")));
        if (cJSON_IsTrue(cJSON_GetObjectItem(childTask, "enabled")) &&
            cJSON_IsNumber(cJSON_GetObjectItem(childTask, "time")) &&
            (currentTime >= cJSON_GetObjectItem(childTask, "time")->valueint) &&
            !cJSON_IsTrue(cJSON_GetObjectItem(childTask, "done"))) {
            ESP_LOGD(TAG, "Scheduler inside loop");
            // если время настало
            // не будет работать в 0.00
            if (cJSON_IsNumber(cJSON_GetObjectItem(childTask, "grace")))
                grace = cJSON_GetObjectItem(childTask, "grace")->valueint;
            else
                grace = 1; // default grace time
            ESP_LOGD(TAG, "Scheduler task %s, time %d, grace %d",
                     cJSON_GetObjectItem(childTask, "name")->valuestring,
                     cJSON_GetObjectItem(childTask, "time")->valueint,
                     grace);
            // day of week
            if (cJSON_IsArray(cJSON_GetObjectItem(childTask, "dow"))) {
                // есть дни недели
                ESP_LOGD(TAG, "dow exists");
                bool taskDow = false;
                cJSON *iterator = NULL;    
                cJSON_ArrayForEach(iterator, cJSON_GetObjectItem(childTask, "dow")) {
                    if (cJSON_IsNumber(iterator)) {
                        if (iterator->valueint == info->tm_wday) {
                            taskDow = true;
                            break;
                        }                        
                    }       
                } 
                if (!taskDow) {
                    ESP_LOGD(TAG, "task %s dow not today", cJSON_GetObjectItem(childTask, "name")->valuestring);
                    childTask = childTask->next; 
                    continue;
                }
                ESP_LOGD(TAG, "task %s dow today, processing task...", cJSON_GetObjectItem(childTask, "name")->valuestring);
            }
            if (currentTime - cJSON_GetObjectItem(childTask, "time")->valueint <= grace) {
                // выполнить задачу
                // получить действия
                ESP_LOGD(TAG, "Scheduler. Task %s. Processing actions...",
                         cJSON_GetObjectItem(childTask, "name")->valuestring);
                cJSON *childAction = cJSON_GetObjectItem(childTask, "actions")->child;
                while (childAction) {
                    if (cJSON_IsString(cJSON_GetObjectItem(childAction, "type")) &&
                        !strcmp(cJSON_GetObjectItem(childAction, "type")->valuestring, "svc") &&
                        !cJSON_IsNumber(cJSON_GetObjectItem(childAction, "output")) &&
                        cJSON_IsString(cJSON_GetObjectItem(childAction, "action")) &&
                        !strcmp(cJSON_GetObjectItem(childAction, "action")->valuestring, "reboot")) {
                        // reboot
                        reboot = true;
                    } else if (cJSON_IsString(cJSON_GetObjectItem(childAction, "type")) &&
                               !strcmp(cJSON_GetObjectItem(childAction, "type")->valuestring, "out") &&
                               cJSON_IsNumber(cJSON_GetObjectItem(childAction, "output")) &&
                               cJSON_IsString(cJSON_GetObjectItem(childAction, "action"))) {
                        ESP_LOGI(TAG, "Scheduler output %d, action %s",
                                 cJSON_GetObjectItem(childAction, "output")->valueint, 
                                 cJSON_GetObjectItem(childAction, "action")->valuestring);
                        setOutput(cJSON_GetObjectItem(childAction, "output")->valueint, 
                                  cJSON_GetObjectItem(childAction, "action")->valuestring);
                    } else if (cJSON_IsString(cJSON_GetObjectItem(childAction, "type")) &&
                               !strcmp(cJSON_GetObjectItem(childAction, "type")->valuestring, "in") &&
                               cJSON_IsNumber(cJSON_GetObjectItem(childAction, "input")) &&
                               cJSON_IsString(cJSON_GetObjectItem(childAction, "action"))) {                        
                        ESP_LOGI(TAG, "Scheduler input %d, action %s", 
                                 cJSON_GetObjectItem(childAction, "input")->valueint, 
                                 cJSON_GetObjectItem(childAction, "action")->valuestring);
                        // вызов события, привязанного ко входу. Выполнить соответствующие правила                        
                        processInputEvents(0, cJSON_GetObjectItem(childAction, "input")->valueint, 
                                           cJSON_GetObjectItem(childAction, "action")->valuestring, 255);
                    } else {
                        ESP_LOGW(TAG, "Scheduler Something Wrong. %s", cJSON_Print(childAction));
                    }
                    childAction = childAction->next;    
                }
                // выставить признак выполнения (done уже точно есть) 
                cJSON_ReplaceItemInObject(childTask, "done", cJSON_CreateTrue());                
            }            
        }
        childTask = childTask->next;    
    }
    //mqttScheduler(currentTime);
}

esp_err_t ota(char **response, char *content) {
    ESP_LOGI(TAG, "ota service");

    cJSON *parent = cJSON_Parse(content);
    if (!cJSON_IsObject(parent)) {
        setErrorTextJson(response, "Is not a JSON object");
        cJSON_Delete(parent);
        return ESP_FAIL;
    }
    
    if (cJSON_IsString(cJSON_GetObjectItem(parent, "url"))) {
        startOTA(cJSON_GetObjectItem(parent, "url")->valuestring);
        setTextJson(response, "OK");
        cJSON_Delete(parent);        
    } else {
        setErrorTextJson(response, "No url provided");
        cJSON_Delete(parent);
    }
    return ESP_OK;
}

//
esp_err_t getTest(char **response) {
    ESP_LOGW(TAG, "Before %d", esp_get_free_heap_size());      
    *response = getDeviceIOStates();
    ESP_LOGW(TAG, "After %d", esp_get_free_heap_size());      
    return ESP_OK;    
}

esp_err_t uiRouter(httpd_req_t *req) {    
    //ESP_LOGI(TAG, "%d %s", req->method, req->uri);
    char *uri = getClearURI(req->uri);
    char *response = NULL;
    char *content = NULL;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    httpd_resp_set_type(req, "application/json");
	if (!strcmp(uri, "/service/config")) {
        if (req->method == HTTP_GET) {            
            err = getConfig(&response);
        } else if (req->method == HTTP_POST) {
            err = getContent(&content, req);
            if (err == ESP_OK) {
                SemaphoreHandle_t sem = getSemaphore();
                if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE) {
                    err = setConfig(&response, content); 
                    // IO config    
                    IOConfig = getConfigValueObject("io");           
                    xSemaphoreGive(sem);
                }
            }
        }        
    } else if (!strcmp(uri, "/service/file")) {
        if (req->method == HTTP_POST) {
            err = setFileWeb(req);            
        } else if (req->method == HTTP_GET) {
            err = getFileWebRaw(req);   
        }
    } else if ((!strcmp(uri, "/ui/deviceInfo")) && (req->method == HTTP_GET)) {
        err = getDeviceInfo(&response);   
    } else if (!strcmp(uri, "/ui/ioservice")) {
        if (req->method == HTTP_POST) {
            httpd_resp_set_type(req, "application/json");
            err = getContent(&content, req);
            if (err == ESP_OK) {
                err = ioservice(&response, content);    
            }
        }          
    } else if (!strcmp(uri, "/ui/mbtest")) {
        if (req->method == HTTP_POST) {
            httpd_resp_set_type(req, "application/json");
            err = getContent(&content, req);
            if (err == ESP_OK) {
                err = mbtest(&response, content);    
            }
        }
    } else if (!strcmp(uri, "/service/ota")) {
        if (req->method == HTTP_POST) {
            httpd_resp_set_type(req, "application/json");
            err = getContent(&content, req);
            if (err == ESP_OK) {
                err = ota(&response, content);                   
            }
        }               
    } else if (!strcmp(uri, "/service/reboot")) {
        if (req->method == HTTP_POST) {
            httpd_resp_set_type(req, "application/json");
            err = getContent(&content, req);
            if (err == ESP_OK) {                        
                ESP_LOGW(TAG, "Reboot!!!");            
                reboot = true;
                setTextJson(&response, "Reboot OK");
                err = ESP_OK;
            }
        }               
    }else if (!strcmp(uri, "/service/test")) {
        if (req->method == HTTP_GET) {            
            err = getTest(&response);
        }
    }
    //free(response);
    if (err == ESP_OK) {
        httpd_resp_set_status(req, "200");        
    } else if (err == ESP_ERR_NOT_FOUND) {
        httpd_resp_set_status(req, "404");        
        setErrorTextJson(&response, "Method not found!");        
        //httpd_resp_send(req, "Not found!"); //req->uri, strlen(req->uri));
    } else {
        httpd_resp_set_status(req, "400");
    }
    if (response != NULL) {
        httpd_resp_send(req, response, -1);
        free(response);
    }   
    if (content != NULL)
        free(content);
    free(uri);

    return ESP_OK;
}

void runWebServer() {
	if (webserverRegisterRouter(&uiRouter) != ESP_OK) {
		ESP_LOGE(TAG, "Can't register router");
	}
	
	initWebServer(1);	
}

// char* getIOConfigMsg() {
//     char *response;
//     cJSON *json = cJSON_CreateObject();
//     cJSON_AddItemToObject(json, "type", cJSON_CreateString("IOConfig"));
//     cJSON_AddItemToObject(json, "payload", IOConfig);
//     response = cJSON_PrintUnformatted(json);    
//     return response;
// }

// bool mbSlaveIdExists(int slaveId) {
//     cJSON *slave = NULL;
//     cJSON_ArrayForEach(slave, getConfigValueObject2("modbus", "slaves")) {
//         cJSON *idItem = cJSON_GetObjectItem(slave, "slaveId");
//         if (cJSON_IsNumber(idItem) && idItem->valueint == slaveId) {
//             return true;
//         }
//     }
//     return false;
// }
bool isMbSlaveOutputExists(int pId, int pSlaveId) {
    cJSON *slave = NULL;
    cJSON_ArrayForEach(slave, mbSlaves) {
        if (cJSON_IsNumber(cJSON_GetObjectItem(slave, "slaveId")) &&
            cJSON_GetObjectItem(slave, "slaveId")->valueint == pSlaveId) {
            char* model = cJSON_GetObjectItem(slave, "model")->valuestring;
            // получить кол-во выходов у модели
            for (int i = 0; i < sizeof(controllersData) / sizeof(controllersData[0]); i++) {
                if (strcmp(controllersData[i].name, model) == 0)
                    return pId < controllersData[i].outputs;
            }

        }            
    }
    return false;
}

bool isMbSlaveInputExists(int pId, int pSlaveId) {
    cJSON *slave = NULL;
    cJSON_ArrayForEach(slave, mbSlaves) {
        if (cJSON_IsNumber(cJSON_GetObjectItem(slave, "slaveId")) &&
            cJSON_GetObjectItem(slave, "slaveId")->valueint == pSlaveId) {
            char* model = cJSON_GetObjectItem(slave, "model")->valuestring;
            // получить кол-во выходов у модели
            for (int i = 0; i < sizeof(controllersData) / sizeof(controllersData[0]); i++) {
                if (strcmp(controllersData[i].name, model) == 0)
                    return pId < controllersData[i].inputs;
            }

        }            
    }
    return false;
}

bool isMbSlaveExists(int pSlaveId) {
    cJSON *slave = NULL;
    cJSON_ArrayForEach(slave, mbSlaves) {
        if (cJSON_IsNumber(cJSON_GetObjectItem(slave, "slaveId")) &&
            cJSON_GetObjectItem(slave, "slaveId")->valueint == pSlaveId) {
            return true;
        }            
    }
    return false;
}

bool outputExists(int pId, int pSlaveId) {
    cJSON *output = NULL;
    cJSON_ArrayForEach(output, cJSON_GetObjectItem(IOConfig, "outputs")) {
        uint8_t slaveId = 0;
        uint8_t id = 255;
        if (cJSON_IsNumber(cJSON_GetObjectItem(output, "slaveId")))
            slaveId = cJSON_GetObjectItem(output, "slaveId")->valueint;                
        if (cJSON_IsNumber(cJSON_GetObjectItem(output, "id")))
            id = cJSON_GetObjectItem(output, "id")->valueint;
        if ((id == pId) && (slaveId == pSlaveId))
            return true;                
    }
    return false;
}

bool inputExists(int pId, int pSlaveId) {
    cJSON *input = NULL;
    cJSON_ArrayForEach(input, cJSON_GetObjectItem(IOConfig, "inputs")) {
        uint8_t slaveId = 0;
        uint8_t id = 255;
        if (cJSON_IsNumber(cJSON_GetObjectItem(input, "slaveId")))
            slaveId = cJSON_GetObjectItem(input, "slaveId")->valueint;                
        if (cJSON_IsNumber(cJSON_GetObjectItem(input, "id")))
            id = cJSON_GetObjectItem(input, "id")->valueint;
        if ((id == pId) && (slaveId == pSlaveId))
            return true;                
    }
    return false;
}

bool buttonExists(int pId) {
    cJSON *input = NULL;
    cJSON_ArrayForEach(input, cJSON_GetObjectItem(IOConfig, "inputs")) {
        uint8_t id = 255;
        if (cJSON_IsNumber(cJSON_GetObjectItem(input, "id")))
            id = cJSON_GetObjectItem(input, "id")->valueint;
        if (id == pId)
            return true;                
    }
    return false;
}

char *getMbMode() {
    if (getConfigValueString("modbus/mode") == NULL)
        return "none";
    if (!strcmp(getConfigValueString("modbus/mode"), "master")) {            
        return "master";
    }
    else if (!strcmp(getConfigValueString("modbus/mode"), "slave")) {
        return "slave";
    }    
    return "none";
}

void correctIOConfig(bool forceSave) {
    // корректировка конфига устройства
    // проверить наличие всех выходов, входов и кнопок относительно модели контроллера
    bool changed = false;
    if (forceSave)
        changed = true;
    // if (!getConfigValueBool("modbus/enabled")) {
    //     mbMode = "";
    // } else {
    //     mbMode = getConfigValueString("modbus/mode");
    // }

    mbMode = getMbMode();
    ESP_LOGI(TAG, "correctIOConfig mbMode %s. Max outputs %d. Max inputs %d", 
             mbMode, controllersData[controllerType].outputs,
             controllersData[controllerType].inputs);

    mbSlaves = getConfigValueObject("modbus/slaves");
    // удаление лишних выходов
    cJSON *outputs = cJSON_GetObjectItem(IOConfig, "outputs");    
    int size = cJSON_GetArraySize(outputs);
    for (int i = size - 1; i >= 0; i--) {
        cJSON *item = cJSON_GetArrayItem(outputs, i);
        uint8_t slaveId = 0;
        uint8_t id = 255;
        if (cJSON_IsNumber(cJSON_GetObjectItem(item, "slaveId")))
            slaveId = cJSON_GetObjectItem(item, "slaveId")->valueint;
        if (cJSON_IsNumber(cJSON_GetObjectItem(item, "id")))
            id = cJSON_GetObjectItem(item, "id")->valueint;
        //ESP_LOGI(TAG, "id %d slaveId %d", id, slaveId);

        if (((!strcmp(mbMode, "slave")) && slaveId > 0) || // если это слейв и у него есть не его выходы
            ((slaveId == 0) && (id > controllersData[controllerType].outputs)) || // если выход больше чем можно           
            ((!strcmp(mbMode, "master")) && slaveId > 0 && !isMbSlaveExists(slaveId)) || // если это мастер и не его слейв
            ((!strcmp(mbMode, "master")) && slaveId > 0 && !isMbSlaveOutputExists(id, slaveId)) // если это мастер и выходов больше чем надо у слейвов
            ) {
            ESP_LOGW(TAG, "Deleting output with id %d slaveId %d", id, slaveId);
            cJSON_DeleteItemFromArray(outputs, i);
            changed = true;
        }
    }
    ESP_LOGI(TAG, "correctIOConfig deleting orphan inputs");
    // удаление лишних входов
    cJSON *inputs = cJSON_GetObjectItem(IOConfig, "inputs");    
    size = cJSON_GetArraySize(inputs);
    for (int i = size - 1; i >= 0; i--) {
        cJSON *item = cJSON_GetArrayItem(inputs, i);
        uint8_t slaveId = 0;
        uint8_t id = 255;
        if (cJSON_IsNumber(cJSON_GetObjectItem(item, "slaveId")))
            slaveId = cJSON_GetObjectItem(item, "slaveId")->valueint;
        if (cJSON_IsNumber(cJSON_GetObjectItem(item, "id")))
            id = cJSON_GetObjectItem(item, "id")->valueint;
        //ESP_LOGI(TAG, "id %d slaveId %d", id, slaveId);

        if ((id < 16) && // кнопки не удаляем
            ((!strcmp(mbMode, "slave") && slaveId > 0) || // если это слейв и у него есть не его выходы
            (slaveId == 0 && id > controllersData[controllerType].inputs) || // если выход больше чем можно           
            ((!strcmp(mbMode, "master")) && slaveId > 0 && !isMbSlaveExists(slaveId)) || // если это мастер и не его слейв
            ((!strcmp(mbMode, "master")) && slaveId > 0 && !isMbSlaveInputExists(id, slaveId))) // если это мастер и выходов больше чем надо у слейвов
            ) {
            ESP_LOGW(TAG, "Deleting input with id %d slaveId %d", id, slaveId);
            cJSON_DeleteItemFromArray(inputs, i);
            changed = true;
        }
    }
 
    ESP_LOGI(TAG, "correctIOConfig adding new outputs");
    char *name = NULL;
    name = malloc(25);
    // Добавление недостающих outputs 
    for (uint8_t i = 0; i < controllersData[controllerType].outputs; i++) {
        if (!outputExists(i, 0)) {
            cJSON *newOutput = cJSON_CreateObject();
            cJSON_AddNumberToObject(newOutput, "id", i);
            sprintf(name, "Out %d", i);                
            cJSON_AddStringToObject(newOutput, "name", name);
            cJSON_AddStringToObject(newOutput, "type", "s");
            cJSON_AddStringToObject(newOutput, "state", "off");
            cJSON_AddItemToArray(outputs, newOutput);
            ESP_LOGW(TAG, "Adding new output with id %d", i);
            changed = true;
        }
    }
    ESP_LOGI(TAG, "correctIOConfig adding new inputs");
    // Добавление недостающих inputs
    for (uint8_t i = 0; i < controllersData[controllerType].inputs; i++) {
        if (!inputExists(i, 0)) {
            cJSON *newInput = cJSON_CreateObject();
            cJSON_AddNumberToObject(newInput, "id", i);
            sprintf(name, "In %d", i);                
            cJSON_AddStringToObject(newInput, "name", name);
            cJSON_AddStringToObject(newInput, "type", "SW");
            cJSON_AddStringToObject(newInput, "state", "off");
            cJSON_AddItemToArray(inputs, newInput);
            ESP_LOGW(TAG, "Adding new input with id %d", i);
            changed = true;
        }
    }
    ESP_LOGI(TAG, "correctIOConfig adding new buttons");
    // Добавление недостающих buttons
    for (uint8_t i = 16; i < 16 + controllersData[controllerType].buttons; i++) {
        if (!buttonExists(i)) {
            cJSON *newButton = cJSON_CreateObject();
            cJSON_AddNumberToObject(newButton, "id", i);
            sprintf(name, "Svc %d", i);                
            cJSON_AddStringToObject(newButton, "name", name);
            cJSON_AddStringToObject(newButton, "type", "BTN");            
            cJSON_AddItemToArray(inputs, newButton);
            ESP_LOGW(TAG, "Adding new button with id %d", i);
            changed = true;
        }
    }

    // modbus
    // добавить модбас выходы всех устройств
    ESP_LOGI(TAG, "correctIOConfig adding new modbus io");
    uint8_t inputsQty, outputsQty;
    cJSON *slave = NULL;    
    cJSON_ArrayForEach(slave, mbSlaves) {
        ESP_LOGI(TAG, "correctIOConfig cJSON_ArrayForEach mb slave");
        if (cJSON_IsNumber(cJSON_GetObjectItem(slave, "slaveId"))) {
            uint8_t slaveId = cJSON_GetObjectItem(slave, "slaveId")->valueint;
            ESP_LOGI(TAG, "correctIOConfig cJSON_ArrayForEach mb slave %d", slaveId);
            inputsQty = 0;
            outputsQty = 0;
            char* model = cJSON_GetObjectItem(slave, "model")->valuestring;
            // получить кол-во по модели
            for (int i = 0; i < sizeof(controllersData) / sizeof(controllersData[0]); i++) {
                if (strcmp(controllersData[i].name, model) == 0) {
                    inputsQty = controllersData[i].inputs;
                    outputsQty = controllersData[i].outputs;
                }
            }
            for (uint8_t i = 0; i < outputsQty; i++) {
                if (!outputExists(i, slaveId)) {
                    cJSON *newOutput = cJSON_CreateObject();
                    cJSON_AddNumberToObject(newOutput, "id", i);
                    sprintf(name, "Sl %d Out %d", slaveId, i);                
                    cJSON_AddStringToObject(newOutput, "name", name);
                    cJSON_AddNumberToObject(newOutput, "slaveId", slaveId);
                    cJSON_AddStringToObject(newOutput, "state", "off");
                    cJSON_AddItemToArray(outputs, newOutput);
                    ESP_LOGW(TAG, "Adding new output with id %d slaveId %d", i, slaveId);
                    changed = true;
                }
            }
            for (uint8_t i = 0; i < inputsQty; i++) {
                if (!inputExists(i, slaveId)) {
                    cJSON *newInput = cJSON_CreateObject();
                    cJSON_AddNumberToObject(newInput, "id", i);
                    sprintf(name, "Sl %d In %d", slaveId, i);                
                    cJSON_AddStringToObject(newInput, "name", name);
                    cJSON_AddNumberToObject(newInput, "slaveId", slaveId);
                    cJSON_AddStringToObject(newInput, "state", "off");
                    cJSON_AddItemToArray(inputs, newInput);
                    ESP_LOGW(TAG, "Adding new input with id %d slaveId %d", i, slaveId);
                    changed = true;
                }
            }
        }            
    }

    free(name);
    ESP_LOGI(TAG, "correctIOConfig done");
    if (changed) {
        ESP_LOGI(TAG, "Config changed. Saving");
        saveConfig();
    }
}

void setWSTime(char *datetime) {
    gWSTimeSet = true;
    ESP_LOGI(TAG, "WSTimeset. Current datetime %s", datetime);
    struct tm tm_time = {0};
    if (sscanf(datetime, "%2d-%2d-%4d %2d:%2d:%2d",
               &tm_time.tm_mday,
               &tm_time.tm_mon,
               &tm_time.tm_year,
               &tm_time.tm_hour,
               &tm_time.tm_min,
               &tm_time.tm_sec) != 6) {
        ESP_LOGE(TAG, "Invalid date format");
        return;
    }

    tm_time.tm_mon  -= 1;         // tm_mon: 0-11
    tm_time.tm_year -= 1900;      // tm_year: years since 1900

    time_t t = mktime(&tm_time);

    struct timeval now = {
        .tv_sec = t,
        .tv_usec = 0
    };

    settimeofday(&now, NULL);

    ESP_LOGI(TAG, "Time set to: %s", datetime);
}

void wsMsg(char *message) {
	ESP_LOGI(TAG, "wsMsg received. Size %d, Text %s", strlen(message), message);
    char *response;
    char *type;
    cJSON *json = cJSON_Parse(message); 
    cJSON *payload = NULL;
    if(!cJSON_IsObject(json)) {
        ESP_LOGE(TAG, "WS Message isn't json");
        WSSendMessageForce("{\"type\":\"ERROR\", \"payload\": {\"message\": \"WS Message isn't json\"}}"); // TODO : Serialize it
        return; 
    }
    //ESP_LOGI(TAG, "%s", message);
    if (cJSON_IsObject(cJSON_GetObjectItem(json, "payload"))) {
        payload = cJSON_GetObjectItem(json, "payload");
    }
    // у сообщения обязательно должен быть type и payload (опционально)
    if (cJSON_IsString(cJSON_GetObjectItem(json, "type"))) {
        type = cJSON_GetObjectItem(json, "type")->valuestring;
        if (!strcmp(type, "AUTHORIZED")) {
            WSSetAuthorized();
            sendInfo();
            response = getDeviceIOStates();
            WSSendMessage(response);  
            free(response);
        } else if (!strcmp(type, "TIME") && cJSON_IsString(cJSON_GetObjectItem(json, "payload"))) {
            // set time
            setWSTime(cJSON_GetObjectItem(json, "payload")->valuestring);
            // after time is set we can send hello and start session
            sendHello();
        } else if (!strcmp(type, "INFO")) {       
            // запрос информации   
            sendInfo();            
        } else if (!strcmp(type, "GETDEVICECONFIG")) {
            response = getConfigMsg();//getIOConfigMsg();
            WSSendMessageForce(response);
            free(response);        
        } else if (!strcmp(type, "SETDEVICECONFIG") && payload != NULL) {                         
            ESP_LOGW(TAG, "Updating device config");
            if (cJSON_IsObject(payload)) {
                SemaphoreHandle_t sem = getSemaphore();
                if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE) {
                    //cJSON_Delete(IOConfig);
                    cJSON_DetachItemFromObject(json, "payload");
                    replaceConfig(payload);       
                    if (getConfigValueString("model") != NULL)
                        setConfigValueString("controllerType", getConfigValueString("model"));
                    // IO config    
                    IOConfig = getConfigValueObject("io");    
                    correctIOConfig(true);
                    xSemaphoreGive(sem);                    
                }
        //         ESP_LOGI(TAG, "IOConfig is object %d", cJSON_IsObject(IOConfig));
                WSSendMessageForce("{\"type\":\"DEVICECONFIGRESPONSE\", \"payload\": {\"message\": \"OK\"}}");
            } else {        
                WSSendMessageForce("{\"type\":\"DEVICECONFIGRESPONSE\", \"payload\": {\"message\": \"Ne OK\"}}");
            }
        } else if (!strcmp(type, "ACTION") && payload != NULL) {
            if (cJSON_IsString(cJSON_GetObjectItem(payload, "mac")) &&
                strcmp(toUpper(cJSON_GetObjectItem(payload, "mac")->valuestring), getMac())) {
                ESP_LOGE(TAG, "Wrong mac %s", cJSON_GetObjectItem(payload, "mac")->valuestring);                
            } else if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "input"))) { 
                uint8_t slaveId = 0;
                if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "slaveId")))
                    slaveId = cJSON_GetObjectItem(payload, "slaveId")->valueint;
                char* pValue = NULL;
                if (cJSON_IsString(cJSON_GetObjectItem(payload, "action"))) {
                    pValue = cJSON_GetObjectItem(payload, "action")->valuestring;
                }               
                processInputEvents(slaveId, cJSON_GetObjectItem(payload, "input")->valueint, pValue, 255);
            } else if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "output"))) {
                uint8_t pOutput = cJSON_GetObjectItem(payload, "output")->valueint;
                            
                char* pValue = NULL;
                uint8_t pSlaveId = 0;
                
                if (cJSON_IsString(cJSON_GetObjectItem(payload, "action"))) {
                    pValue = cJSON_GetObjectItem(payload, "action")->valuestring;
                }               
                if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "slaveId"))) {
                    pSlaveId = cJSON_GetObjectItem(payload, "slaveId")->valueint;
                }         
                ESP_LOGI(TAG, "ACTION. output %d slaveId %d action %s", pOutput, pSlaveId, pValue);
                // TODO : new modbus process
                if (pValue != NULL) {
                    if (pSlaveId == 0) {
                        setOutput(pOutput, pValue);                        
                    } else {                                
                        setRemoteOutput(pSlaveId, pOutput, pValue);
                    }           
                }
            }
        } else if (!strcmp(type, "OTA") && payload != NULL) {
            if (cJSON_IsString(cJSON_GetObjectItem(payload, "url"))) {
                startOTA(cJSON_GetObjectItem(payload, "url")->valuestring);
            }
        } else if (!strcmp(type, "SENDLOGS") && payload != NULL) {
            wsSendLogs = cJSON_IsTrue(cJSON_GetObjectItem(payload, "send"));                 
            ESP_LOGI(TAG, "Sending logs to websocket %s", wsSendLogs ? "enabled" : "disabled");
        } else if (!strcmp(type, "REBOOT")) {            
            ESP_LOGW(TAG, "Reboot request");
            reboot = true;
        }
    }
    cJSON_Delete(json);
    //ESP_LOGI(TAG, "payload is object %d", cJSON_IsObject(payload));
}

void wsEvent(uint8_t event) {
    if (event == WEBSOCKET_EVENT_CONNECTED) {
        wsConnected = true;
        //sendInfo();
        //sendHello();
    } else if (event == WEBSOCKET_EVENT_DISCONNECTED) {
        wsConnected = false;
    }
}

void initWS() {
    if (getConfigValueBool("network/cloud/enabled")) {
        char *jwt = NULL;                        
        loadTextFile("/certs/jwt.pem", &jwt);
        if (jwt == NULL) {
            ESP_LOGI(TAG, "take JWT from firmware...");
            uint16_t len = jwt_end - jwt_start;
            jwt = (char*)malloc(len+1);
            strncpy(jwt, jwt_start, len);
        }
    	WSinit(getConfigValueString("network/cloud/address"), &wsMsg, &wsEvent, jwt, getConfigValueBool("network/cloud/log"));            
    }
}

//void modBusEvent(uint8_t slaveId, uint16_t input, char *event) {
void modBusEvent(mb_event_t data) {
    // обработчик событий слейвов для мастера
    // data.input, data.state, data.slaveId
    // data.output, data.state, data.slaveId
    // data.event, data.input, data.slaveId
    // TODO : process events 
    if (!strcmp(data.type, "input")) {
        ESP_LOGI(TAG, "Input %d changed to %s on slave %d", data.input, data.state, data.slaveId);  
        sendWSUpdateInput(data.slaveId, data.input, data.state);
    } else if (!strcmp(data.type, "output")) {
        ESP_LOGI(TAG, "Output %d changed to %s on slave %d", data.output, data.state, data.slaveId);  
        //sendWSUpdateOutput(data.slaveId, data.output, data.state, 0);
        publishOutput(data.slaveId, data.output, data.state, 0);
    } else if (!strcmp(data.type, "event")) {
        ESP_LOGI(TAG, "Event %s on input %d on slave %d", data.event, data.input, data.slaveId);  
        processInputEvents(data.slaveId, data.input, data.event, 255);
    }
}

void modBusAction(uint8_t output, char *action) {
    // обработчик действия для слейва
    setOutput(output, action);
}

void initModBus() {
    if (getConfigValueBool("modbus/enabled")) {
        if (!strcmp(getConfigValueString("modbus/mode"), "master")) {
            mbSlaves = getConfigValueObject("modbus/slaves");
            MBInitMaster(IOConfig, &modBusEvent, mbSlaves, controllerType > 2);
            //mbMode = "master";
        } else if (!strcmp(getConfigValueString("modbus/mode"), "slave")) {
            mbSlaveId = getConfigValueInt("modbus/slaveId");
            MBInitSlave(mbSlaveId, &modBusAction, controllerType > 2);
            mbSlave = true;
            //mbMode = "slave";            
        }
    }   
}

void mqttEvent(uint8_t event) {
    if (event == MQTT_EVENT_CONNECTED) {
        mqttConnected = true;
        // subscribe to others
        if (cJSON_IsArray(jMQTTTopics)) {
            cJSON *childTopic = jMQTTTopics->child;
            while (childTopic) { 
                if (cJSON_IsString(cJSON_GetObjectItem(childTopic, "topic"))) {
                    MQTTSubscribe(cJSON_GetObjectItem(childTopic, "topic")->valuestring); 
                }
                childTopic = childTopic->next;
            }    
        }
    } else if (event == MQTT_EVENT_DISCONNECTED)
        mqttConnected = false;
}

void mqttData(char* topic, char* data) {
    ESP_LOGW(TAG, "parseTopic %s %s", topic, data);
    char str[100];
    strcpy(str, getConfigValueString("name"));
    strcat(str, "/in/\0");
    if (!strstr(topic, str)) {
        // это не топики своего устройства. Проверить внешний список
        if (cJSON_IsArray(jMQTTTopics)) {
            cJSON *childTopic = jMQTTTopics->child;
            while (childTopic) { 
                if (cJSON_IsString(cJSON_GetObjectItem(childTopic, "topic")) &&
                    !strcmp(topic, cJSON_GetObjectItem(childTopic, "topic")->valuestring)) {
                    if (cJSON_IsArray(cJSON_GetObjectItem(childTopic, "events"))) {
                        cJSON *childEvent = cJSON_GetObjectItem(childTopic, "events")->child;
                        while (childEvent) { 
                            if (cJSON_IsString(cJSON_GetObjectItem(childEvent, "event")) &&
                                !strcasecmp(data, cJSON_GetObjectItem(childEvent, "event")->valuestring)) {
                                if (cJSON_IsString(cJSON_GetObjectItem(childEvent, "action")) &&
                                    cJSON_IsString(cJSON_GetObjectItem(childEvent, "type"))) {
                                    if (!strcmp("out", cJSON_GetObjectItem(childEvent, "type")->valuestring) &&
                                        cJSON_IsNumber(cJSON_GetObjectItem(childEvent, "output"))) {
                                        uint8_t output = cJSON_GetObjectItem(childEvent, "output")->valueint;
                                        char *action = cJSON_GetObjectItem(childEvent, "action")->valuestring;
                                        uint8_t slaveId = 0;
                                        if (cJSON_IsNumber(cJSON_GetObjectItem(childEvent, "slaveId"))) {
                                            slaveId = cJSON_GetObjectItem(childEvent, "slaveId")->valueint;
                                        }                                    
                                        if (slaveId) {
                                            setRemoteOutput(slaveId, output, action);            
                                        } else {
                                            setOutput(output, action);
                                        }    
                                    } else if (!strcmp("in", cJSON_GetObjectItem(childEvent, "type")->valuestring) &&
                                        cJSON_IsNumber(cJSON_GetObjectItem(childEvent, "input"))) {
                                        uint8_t input = cJSON_GetObjectItem(childEvent, "input")->valueint;
                                        char *action = cJSON_GetObjectItem(childEvent, "action")->valuestring;
                                        uint8_t slaveId = 0;
                                        if (cJSON_IsNumber(cJSON_GetObjectItem(childEvent, "slaveId"))) {
                                            slaveId = cJSON_GetObjectItem(childEvent, "slaveId")->valueint;
                                        }            
                                        processInputEvents(slaveId, input, action, 255);                                                        
                                    }
                                }                                
                            }
                            childEvent = childEvent->next;
                        }
                    }
                    break;
                }
                childTopic = childTopic->next;
            }    
        }
        /*
        if (cJSON_IsArray(jMQTTTopics)) {
            cJSON *childTopic = jMQTTTopics->child;
            while (childTopic) { 
                if (cJSON_IsString(cJSON_GetObjectItem(childTopic, "topic")) &&
                    !strcmp(topic, cJSON_GetObjectItem(childTopic, "topic")->valuestring)) {
                    if (cJSON_IsArray(cJSON_GetObjectItem(childTopic, "data"))) {
                        cJSON *childData = cJSON_GetObjectItem(childTopic, "data")->child;
                        while (childData) { 
                            if (cJSON_IsString(cJSON_GetObjectItem(childData, "value")) &&
                                !strcmp(data, cJSON_GetObjectItem(childData, "value")->valuestring)) {
                                if (cJSON_IsString(cJSON_GetObjectItem(childData, "action")) &&
                                    cJSON_IsNumber(cJSON_GetObjectItem(childData, "output"))) {
                                    uint8_t output = cJSON_GetObjectItem(childData, "output")->valueint;
                                    char *action = cJSON_GetObjectItem(childData, "action")->valuestring;
                                    uint8_t slaveId = 0;
                                    if (cJSON_IsNumber(cJSON_GetObjectItem(childData, "slaveId"))) {
                                        slaveId = cJSON_GetObjectItem(childData, "slaveId")->valueint;
                                    }                                    
                                    if (slaveId) {
                                        setRemoteOutput(slaveId, output, action);            
                                    } else {
                                        setOutput(output, action);
                                    }    
                                }
                            }
                            childData = childData->next;
                        }
                    }
                    break;
                }
                childTopic = childTopic->next;
            }    
        }
        */
        return;
        /*
        [{
            "topic": "Device1/outputs/0/0",
            "data": [{
                    "value": "ON",
                    "action": "on",
                    "output": 3,
                    "slaveId": 1
                },
                {
                    "value": "OFF",
                    "action": "off",
                    "output": 3,
                    "slaveId": 1
                }]    
        }]
        */
    }   

    // Далее обработка событий самого устройства
    /*
        Для JSON формата будет такой пример
        TOPIC=Big-1-in/json
        DATA={
         "slave": 1,
         "output": 2,
         "action": "on"
        }
    */        
    
    uint8_t slaveId = 0;
    uint8_t output = 0xFF;
    char *action = NULL;

    if (strstr(topic, "/json")) {
        cJSON *jData = cJSON_Parse(data);
        if (!cJSON_IsObject(jData)) {
            ESP_LOGE(TAG, "data is not a json");
            return;
        }
        if (cJSON_IsNumber(cJSON_GetObjectItem(jData, "slaveId")))
            slaveId = cJSON_GetObjectItem(jData, "slaveId")->valueint;
        if (cJSON_IsNumber(cJSON_GetObjectItem(jData, "output")))
            output = cJSON_GetObjectItem(jData, "output")->valueint;
        if (cJSON_IsString(cJSON_GetObjectItem(jData, "action")))
            action = cJSON_GetObjectItem(jData, "action")->valuestring;
        if ((output == 0xFF) || (action == NULL)) {
            ESP_LOGE(TAG, "Wrong json data!");
            return;
        }                
    } else {
        char *token = strtok(topic, "/"); // host
        if (token == NULL)
            return;
        token = strtok(NULL, "/"); // in
        if (token == NULL)
            return;
        token = strtok(NULL, "/"); // slaveId
        if (token == NULL)
            return;
        slaveId = atoi(token);    
        token = strtok(NULL, "/"); // output
        if (token == NULL)
            return;
        output = atoi(token); 
        action = toLower(data);                
    }

    ESP_LOGI(TAG, "slaveId %d, output %d, action %s", slaveId, output, action);

    if (xSemaphoreTake(sem_busy, portMAX_DELAY) == pdTRUE) {
        if (slaveId) {
            // remote action
            setRemoteOutput(slaveId, output, action);
        } else {
            // local
            setOutput(output, action);
        }
        xSemaphoreGive(sem_busy);
    }
}

int custom_vprintf(const char *fmt, va_list args) {
    if (wsSendLogs) {
        char log_buffer[256];    
        vsnprintf(log_buffer, sizeof(log_buffer), fmt, args);
        cJSON *json = cJSON_CreateObject();        
        cJSON_AddStringToObject(json, "type", "LOG");
        cJSON_AddStringToObject(json, "payload", log_buffer);
        char *jsonStr = cJSON_PrintUnformatted(json);
        WSSendMessage(jsonStr);
        free(jsonStr);
        cJSON_Delete(json);
    }    
    return vprintf(fmt, args);
}

void initMQTT() {
    if (getConfigValueBool("mqtt/enabled")) {
        jMQTTTopics = getConfigValueObject("mqtt/topics");
        MQTTInit(&mqttData, &mqttEvent, jMQTTTopics);
    }
}

void initFTP(uint32_t address) {
    if (getConfigValueBool("network/ftp/enabled")) { 
        FTPinit(getConfigValueString("network/ftp/user"), getConfigValueString("network/ftp/pass"), address);
    }
}

void sntpEvent() {
    if (gWSTimeSet)
        return;
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "Clock is %s", asctime(&timeinfo));
    ESP_LOGI(TAG, "Clock set result %s", esp_err_to_name(setClock()));        
}

esp_err_t initCore(SemaphoreHandle_t sem) {
	//createSemaphore();
    setRGBFace("yellow");
    resetReason = esp_reset_reason();
    char *hostname = getConfigValueString("name");
    char *description = getConfigValueString("description");    
    ESP_LOGI(TAG, "Hostname %s, description %s", SS(hostname), SS(description));

    sem_busy = sem;
    initHardware(sem);    
	determinateControllerType();
	ESP_LOGI(TAG, "Controllertype is %s", controllersData[controllerType].name);
    if (controllerType == UNKNOWN) {
        ESP_LOGE(TAG, "Unknown controller type!");        
        //return;
    }
    IOConfig = getConfigValueObject("io");
	if (IOConfig == NULL || 
        (cJSON_IsObject(IOConfig) && !cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "outputs"))) ||
        (cJSON_IsObject(IOConfig) && !cJSON_IsArray(cJSON_GetObjectItem(IOConfig, "inputs"))) ) {
        createIOConfig();
        bool res = setConfigValueObject("io", IOConfig);
        ESP_LOGI(TAG, "IOConfig set result %d", res);
        saveConfig();
    }    
    initModBus();
    correctIOConfig(false);
    initInputs();
	initOutputs();    
    xTaskCreate(&serviceTask, "serviceTask", 4096, NULL, 5, NULL);    
    if (checkServiceButtons()) {
        setRGBFace("yellow");
        ESP_LOGI(TAG, "Service button pressed while boot. Sevice mode...");
        return ESP_ERR_NOT_FINISHED;
        // чтобы сеть не стартовала и запустилась AP
    } 
    startInputTask();
    initScheduler();	    
    esp_log_set_vprintf(&custom_vprintf);
    setRGBFace("green"); // TODO : сделать зеленый когда все поднялось. И продумать цвета    
    return ESP_OK;
}

/* TODO :
не включать выходы по входам при старте, надо игнорить текущие состояния


* scheduler
* timer on/off
* shooter
* modbus
* publish mqtt
* publish mqtt info every 1 minute?
* publish ws
* ws
* mqtt взаимодействие с другими устройствами?. Сделать маппинг, например, /Device1/0/0/ON -> slaveId0 output1 on
+ hardware. на старом контроллере кнопки начинаются с 16 айди, а на новом почему-то с 8, надо где-то исправить
- разобраться с eth. Работает на другой плате
- часы надо сделать, пока не получилось
+ почему-то состояния входов не актуальны. Т.е. при вкючении горят входы, которые не должны гореть, а после однократного изменения состояния по ним все ок
+ надо проверить при закороченном входе включение
+ определение устройств i2c через detect i2c
* переделать type wait на action wait и на бэке тоже
* когда по умолчанию выход был активен то надо чтобы реле щелкнуло, иначе оно будет просто "петь"
*/