#include "esp_log.h"
#include "esp_system.h"
#include "storage.h"
#include "webServer.h"
#include "utils.h"
#include "freertos/semphr.h"
//#include "ota.h"
#include "core.h"
#include "network.h"
#include "ws.h"
#include "cJSON.h"
#include "hardware.h"
#include "ota.h"
#include "time.h"
#include "settings.h"
#include "modbus.h"
#include "mqtt.h"
#include "ftp.h"

static const char *TAG = "CORE";
static SemaphoreHandle_t sem_busy;
static cJSON *deviceConfig;
static cJSON *jScheduler;
static cJSON *jMQTTTopics;
static bool mbSlave = false;
static bool mqttConnected = false;
static bool wsConnected = false;
static uint8_t mbSlaveId = 0;
static char* mbMode = "";

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
  {"RCV2B", 0, 0, 0}  
};
uint32_t serviceButtonsTime[16] = {0}; // in ms
uint32_t inputButtonsTime[32] = {0}; // in ms
//TaskHandle_t iTaskHandle[32] = {NULL};
void processScheduler();
static bool reboot = false;

void determinateControllerType() {
    if (controllerType == UNKNOWN) {
        char *cType = getSettingsValueString("controllerType");
        ESP_LOGI(TAG, "getSettingsValueString %s", cType == NULL ? "NULL" : cType);
        if (cType != NULL) {
            if (!strcmp(cType, "small") || !strcmp(cType, "RCV1S")) {
                controllerType = RCV1S;
            } else if (!strcmp(cType, "big") || !strcmp(cType, "RCV1B")) {
                controllerType = RCV1B;
            }            
        }
    }    
    // if (cType == NULL || !strcmp(cType, "UNKNOWN")) {
    //     ESP_LOGI(TAG, "Saving controllerType in settings");
    //     setSettingsValueString("controllerType", getControllerTypeText(controllerType));
    //     saveSettings();
    // }
}

SemaphoreHandle_t getSemaphore() {
    return sem_busy;
}

void createSemaphore() {
    sem_busy = xSemaphoreCreateMutex();
}

esp_err_t createDeviceConfig() {
    // ESP_LOGI(TAG, "createDeviceConfig. %s controller", itsSmall ? "Small" : "Big");
    if (cJSON_IsObject(deviceConfig))
        cJSON_Delete(deviceConfig);
    deviceConfig = cJSON_CreateObject();    
    // outputs
    char *name = NULL;
    name = malloc(15);
    
	cJSON *outputs = cJSON_CreateArray();    
    for (uint8_t i=0; i<controllersData[controllerType].outputs; i++) {
        cJSON *output = cJSON_CreateObject();
        cJSON_AddItemToObject(output, "id", cJSON_CreateNumber(i));
        sprintf(name, "Out %d", i);    
        cJSON_AddItemToObject(output, "name", cJSON_CreateString(name));
        cJSON_AddItemToObject(output, "type", cJSON_CreateString("s"));
        cJSON_AddItemToObject(output, "state", cJSON_CreateString("off"));        
        cJSON_AddItemToArray(outputs, output);        
    }
    cJSON_AddItemToObject(deviceConfig, "outputs", outputs);

	cJSON *inputs = cJSON_CreateArray();    
    // service buttons
    for (uint8_t i=0; i<controllersData[controllerType].buttons; i++) {
        cJSON *input = cJSON_CreateObject();
        cJSON_AddItemToObject(input, "id", cJSON_CreateNumber(i+16));
        sprintf(name, "Svc %d", i);
        cJSON_AddItemToObject(input, "name", cJSON_CreateString(name));
        cJSON_AddItemToObject(input, "type", cJSON_CreateString("BTN"));
                
        cJSON *actions = cJSON_CreateArray();
        cJSON *action = cJSON_CreateObject();
        cJSON_AddItemToObject(action, "order", cJSON_CreateNumber(0));
        cJSON_AddItemToObject(action, "output", cJSON_CreateNumber(i));
        cJSON_AddItemToObject(action, "action", cJSON_CreateString("toggle"));
        cJSON_AddItemToArray(actions, action);
        
        cJSON *events = cJSON_CreateArray();
        cJSON *event = cJSON_CreateObject();
        cJSON_AddItemToObject(event, "event", cJSON_CreateString("toggle"));
        cJSON_AddItemToObject(event, "actions", actions);
        cJSON_AddItemToArray(events, event);

        cJSON_AddItemToObject(input, "events", events);
        cJSON_AddItemToArray(inputs, input);
    }
    // external inputs    
    for (uint8_t i=0; i<controllersData[controllerType].inputs; i++) {
        cJSON *input = cJSON_CreateObject();
        cJSON_AddItemToObject(input, "id", cJSON_CreateNumber(i));
        sprintf(name, "In %d", i);
        cJSON_AddItemToObject(input, "name", cJSON_CreateString(name));
        cJSON_AddItemToObject(input, "type", cJSON_CreateString("SW"));
        
        cJSON *actionsOn = cJSON_CreateArray();
        cJSON *actionOn = cJSON_CreateObject();
        cJSON_AddItemToObject(actionOn, "order", cJSON_CreateNumber(0));
        cJSON_AddItemToObject(actionOn, "output", cJSON_CreateNumber(i>controllersData[controllerType].outputs-1?controllersData[controllerType].outputs-1:i));
        cJSON_AddItemToObject(actionOn, "action", cJSON_CreateString("on"));
        cJSON_AddItemToArray(actionsOn, actionOn);
        
        cJSON *eventsOn = cJSON_CreateArray();
        cJSON *eventOn = cJSON_CreateObject();
        cJSON_AddItemToObject(eventOn, "event", cJSON_CreateString("on"));
        cJSON_AddItemToObject(eventOn, "actions", actionsOn);
        cJSON_AddItemToObject(input, "events", eventsOn);

        cJSON *actionsOff = cJSON_CreateArray();
        cJSON *actionOff = cJSON_CreateObject();
        cJSON_AddItemToObject(actionOff, "order", cJSON_CreateNumber(0));
        cJSON_AddItemToObject(actionOff, "output", cJSON_CreateNumber(i>controllersData[controllerType].outputs-1?controllersData[controllerType].outputs-1:i));
        cJSON_AddItemToObject(actionOff, "action", cJSON_CreateString("off"));
        cJSON_AddItemToArray(actionsOff, actionOff);
        
        cJSON *eventsOff = cJSON_CreateArray();
        cJSON *eventOff = cJSON_CreateObject();
        cJSON_AddItemToObject(eventOff, "event", cJSON_CreateString("off"));
        cJSON_AddItemToObject(eventOff, "actions", actionsOff);
        cJSON_AddItemToObject(input, "events", eventsOff);

        cJSON_AddItemToArray(inputs, input);
    }   
    cJSON_AddItemToObject(deviceConfig, "inputs", inputs);
    
    free(name);
    // TODO : outputs & other
    return ESP_OK;
}

esp_err_t saveDeviceConfig() {
    if (!cJSON_IsObject(deviceConfig)) {
        ESP_LOGE(TAG, "deviceConfig is not a json!");
        return ESP_FAIL;
    }
    char *cfg = NULL;
    cfg = cJSON_Print(deviceConfig);
    //ESP_LOGI(TAG, "config %s", cfg);
    if (cfg == NULL) {
        ESP_LOGE(TAG, "Failed to print deviceConfig");
        return ESP_FAIL;
    }
    esp_err_t err = saveTextFile("/config/deviceconfig.json", cfg);
    free(cfg);
    return err;
}

esp_err_t loadDeviceConfig() {
    char * buffer;
    if (loadTextFile("/config/deviceconfig.json", &buffer) == ESP_OK) {
        cJSON *parent = cJSON_Parse(buffer);
        if(!cJSON_IsObject(parent) && !cJSON_IsArray(parent))
        {
            free(buffer);
            return ESP_FAIL;
        }
        cJSON_Delete(deviceConfig);
        deviceConfig = parent;        
    } else {
        ESP_LOGI(TAG, "can't read deviceConfig. creating default deviceConfig");
        cJSON_Delete(deviceConfig);
        if (createDeviceConfig() == ESP_OK) {
            saveDeviceConfig();        
            return ESP_OK;
        } else {
            return ESP_FAIL;
        }

    }
    free(buffer);
    return ESP_OK;
}

esp_err_t saveScheduler() {
    char *data = cJSON_Print(jScheduler);    
    esp_err_t err = saveTextFile("/config/scheduler.json", data);
    free(data);
    return err;
}

esp_err_t loadScheduler() {
    char * buffer;
    if (loadTextFile("/config/scheduler.json", &buffer) == ESP_OK) {
        cJSON *parent = cJSON_Parse(buffer);
        if (!cJSON_IsArray(parent)) {
            free(buffer);
            return ESP_FAIL;
        }
        cJSON_Delete(jScheduler);
        jScheduler = parent;        
        free(buffer);
    } else {
        ESP_LOGI(TAG, "can't read scheduler");
        cJSON_Delete(jScheduler);
        jScheduler = cJSON_CreateArray();       
        saveScheduler();
    }    
    return ESP_OK;
}

esp_err_t saveMqttTopics() {
    char *data = cJSON_Print(jMQTTTopics);    
    esp_err_t err = saveTextFile("/config/mqtttopics.json", data);
    free(data);
    return err;
}

esp_err_t loadMqttTopics() {
    char *buffer;
    if (loadTextFile("/config/mqtttopics.json", &buffer) == ESP_OK) {
        cJSON *parent = cJSON_Parse(buffer);
        if (!cJSON_IsArray(parent)) {
            free(buffer);
            return ESP_FAIL;
        }
        cJSON_Delete(jMQTTTopics);
        jMQTTTopics = parent;        
        free(buffer);
    } else {
        ESP_LOGI(TAG, "can't read mqtttopics");
        cJSON_Delete(jMQTTTopics);
        jMQTTTopics = cJSON_CreateArray();       
        saveMqttTopics();
    }    
    return ESP_OK;
}

void serviceTask(void *pvParameter) {
    ESP_LOGI(TAG, "Creating service task");
    uint32_t minMem = getSettingsValueInt2("watchdog", "wdtmemsize");
    while(1) {   
        // every 1 second     
        if ((minMem > 0) && (esp_get_free_heap_size() < minMem)) {
            ESP_LOGE(TAG, "HEAP memory WDT triggered. Actual free memory is %d. Restarting...", esp_get_free_heap_size());
            esp_restart();
        }
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
    char *hostname = getSettingsValueString("hostname");
    char *description = getSettingsValueString("description");
    char *version = getCurrentVersion();
    char *ethip = getETHIPStr();
    char *wifiip = getWIFIIPStr();
    char *mac = getMac();
    cJSON_AddItemToObject(status, "mac", cJSON_CreateString(mac));
    cJSON_AddItemToObject(status, "freememory", cJSON_CreateNumber(esp_get_free_heap_size()));
    cJSON_AddItemToObject(status, "uptime", cJSON_CreateString(uptime));
    cJSON_AddItemToObject(status, "uptimeraw", cJSON_CreateNumber(getUpTimeRaw()));    
    cJSON_AddItemToObject(status, "curdate", cJSON_CreateString(curdate));
    cJSON_AddItemToObject(status, "devicename", cJSON_CreateString(hostname));
    cJSON_AddItemToObject(status, "version", cJSON_CreateString(version));
    cJSON_AddItemToObject(status, "rssi", cJSON_CreateNumber(getRSSI()));
    cJSON_AddItemToObject(status, "ethip", cJSON_CreateString(ethip));
    cJSON_AddItemToObject(status, "wifiip", cJSON_CreateString(wifiip));  
    cJSON_AddItemToObject(status, "description", cJSON_CreateString(description));          
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
        strcpy(topic, getSettingsValueString("hostname"));
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
}

void updateValues() {
    uint16_t outputs = 0;
    uint16_t inputsLeds = 0;
    uint16_t outputsLeds = 0;

    cJSON *childOutput = cJSON_GetObjectItem(deviceConfig, "outputs")->child;
    while (childOutput) {
        uint8_t slaveId = 0;
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "slaveId")))
            slaveId = cJSON_GetObjectItem(childOutput, "slaveId")->valueint;        
        if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "state")) &&
            cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "id")) &&
            slaveId == 0) {
            //ESP_LOGI(TAG, "id %d, state %s", cJSON_GetObjectItem(childOutput, "id")->valueint, )
            if (!strcmp(cJSON_GetObjectItem(childOutput, "state")->valuestring, "on")) {
                setbit(outputs, cJSON_GetObjectItem(childOutput, "id")->valueint);
                setbit(outputsLeds, cJSON_GetObjectItem(childOutput, "id")->valueint);
            }
        }
        childOutput = childOutput->next;
    }
    cJSON *childInput = cJSON_GetObjectItem(deviceConfig, "inputs")->child;
    while (childInput) {  
        uint8_t slaveId = 0;
        if (cJSON_IsNumber(cJSON_GetObjectItem(childInput, "slaveId")))
            slaveId = cJSON_GetObjectItem(childInput, "slaveId")->valueint;        
        if (cJSON_IsString(cJSON_GetObjectItem(childInput, "state")) &&
            cJSON_IsNumber(cJSON_GetObjectItem(childInput, "id")) &&
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
    cJSON *childInput = cJSON_GetObjectItem(deviceConfig, "inputs")->child;
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
    cJSON *childOutput = cJSON_GetObjectItem(deviceConfig, "outputs")->child;
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
        cJSON_AddItemToObject(payload, "slaveid", cJSON_CreateNumber(pSlaveId));
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
    // TODO: publish on websocket and mqtt    
    if (!mbSlave)
        sendWSUpdateOutput(pSlaveId, pOutput, pValue, pTimer);

    if (mqttConnected) {
        char topic[50] = {0};
        char buf[5];
        strcpy(topic, getSettingsValueString("hostname"));
        if (strlen(topic) == 0) {
            strcpy(topic, "unknown");
        }
        strcat(topic, "/outputs/");
        itoa(pSlaveId, buf, 10);
        strcat(topic, buf);
        strcat(topic, "/");
        itoa(pOutput, buf, 10);
        strcat(topic, buf); 
        strcat(topic, "\0");   
        char actionUpper[10];
        strcpy(actionUpper, pValue);    
        strcat(actionUpper, "\0");   
        MQTTPublish(topic, toUpper(actionUpper));        
    }
}

void publishInput(uint8_t pInput, char* pState, uint8_t pSlaveId) {
    // TODO: publish on websocket and mqtt
    if (!mbSlave)
        sendWSUpdateInput(pSlaveId, pInput, pState);
}

char* getOutputState(uint8_t pOutput) {
    if (!cJSON_IsArray(cJSON_GetObjectItem(deviceConfig, "outputs"))) {        
        return "off";
    }    
    cJSON *childOutput = cJSON_GetObjectItem(deviceConfig, "outputs")->child;
    while (childOutput) {  
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
    if (!cJSON_IsArray(cJSON_GetObjectItem(deviceConfig, "inputs"))) {        
        return "off";
    }    
    cJSON *childInput = cJSON_GetObjectItem(deviceConfig, "inputs")->child;
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

void setOutput(uint8_t pOutput, char* pValue, uint16_t pDuration) {
    // установка значений для выхода        
    ESP_LOGI(TAG, "setOutput output %d, value %s, duration %d",
             pOutput, pValue, pDuration);
    if (!cJSON_IsArray(cJSON_GetObjectItem(deviceConfig, "outputs"))) {
        ESP_LOGE(TAG, "deviceConfig outputs isn't array");
        return;
    }        
    if (!strcmp(pValue, "toggle")) {
        pValue = !strcmp(getOutputState(pOutput), "on") ? "off" : "on";
    }
    // касательно длительности. Приоритет длительности из правала. Т.е. если на входе стоит длительность 5, а в правиле 10, то выход включится на 10 сек
    uint16_t timer = 0;
    uint16_t oDuration = 0;
    cJSON *childOutput = cJSON_GetObjectItem(deviceConfig, "outputs")->child;
    while (childOutput) {
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "id")) &&
            (cJSON_GetObjectItem(childOutput, "id")->valueint == pOutput)) {
            cJSON_ReplaceItemInObject(childOutput, "state", cJSON_CreateString(pValue));

            // получить duration выхода
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "duration")))
                oDuration = cJSON_GetObjectItem(childOutput, "duration")->valueint;
            else
                cJSON_AddItemToObject(childOutput, "duration", cJSON_CreateNumber(oDuration));

            if ((pDuration > 0) && (pDuration < 0xFFFF)) {
                // если задана какая-то длительность, то выставить таймер
                timer = pDuration;                
            } else {                
                timer = oDuration;
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

void setRemoteOutput(uint8_t pSlaveId, uint8_t pOutput, char* pValue, uint16_t pDuration) {
    // nothing
    ESP_LOGI(TAG, "setRemoteOutput. slaveId %d, output %d, action %s, duration %d",
             pSlaveId, pOutput, pValue, pDuration);
    MBSetRemoteOutput(pSlaveId, pOutput, pValue, pDuration);
    //publishOutput(pSlaveId, pOutput, pValue, pDuration);
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
                    } else if (cJSON_IsNumber(cJSON_GetObjectItem(actionChild, "output")) &&
                               cJSON_IsString(cJSON_GetObjectItem(actionChild, "action"))) {
                        // action                        
                        if (cJSON_IsNumber(cJSON_GetObjectItem(actionChild, "slaveId"))) {                            
                            setRemoteOutput(cJSON_GetObjectItem(actionChild, "slaveId")->valueint, 
                                            cJSON_GetObjectItem(actionChild, "output")->valueint, 
                                            cJSON_GetObjectItem(actionChild, "action")->valuestring, 0);
                        } else {                        
                            setOutput(cJSON_GetObjectItem(actionChild, "output")->valueint, 
                                      cJSON_GetObjectItem(actionChild, "action")->valuestring, 0);
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

void setAllOff() {
    // выставить все выходы в состояние выкл, включая тепличные таймеры и слейвы модбаса
    if (!cJSON_IsArray(cJSON_GetObjectItem(deviceConfig, "outputs"))) {
        ESP_LOGE(TAG, "deviceConfig outputs isn't array");
        return;
    }
    char *action = "off";
    
    uint16_t timer = 0;
    cJSON *childOutput = cJSON_GetObjectItem(deviceConfig, "outputs")->child;
    while (childOutput) {  
        uint8_t slaveId = 0;
        if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "slaveId")))
            slaveId = cJSON_GetObjectItem(childOutput, "timer")->valueint;
        if (slaveId > 0) {
            // для слейвов
            setRemoteOutput(slaveId, 
                            cJSON_GetObjectItem(childOutput, "id")->valueint, 
                            action, 0);            
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
    cJSON *childInput = cJSON_GetObjectItem(deviceConfig, "inputs")->child;
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
                                uint8_t duration = 0;
                                if (cJSON_IsNumber(cJSON_GetObjectItem(child, "duration")))
                                    duration = cJSON_GetObjectItem(child, "duration")->valueint;
                                if (!strcmp(action, "allOff")) {
                                    setAllOff();                                    
                                } else if (slaveId > 0) {
                                    setRemoteOutput(slaveId, cJSON_GetObjectItem(child, "output")->valueint, action, duration);
                                } else {
                                    setOutput(cJSON_GetObjectItem(child, "output")->valueint, action, duration);
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
}

void processRemoteInputEvents(uint8_t slaveId, uint8_t pInput, char* pEvent) {

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
    if (!cJSON_IsArray(cJSON_GetObjectItem(deviceConfig, "inputs"))) {
        ESP_LOGE(TAG, "deviceConfig inputs isn't array");
        return;
    }    
    cJSON *childInput = cJSON_GetObjectItem(deviceConfig, "inputs")->child;
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
						event = "long";
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
    if (!cJSON_IsArray(cJSON_GetObjectItem(deviceConfig, "outputs"))) {        
        return;
    }    
    cJSON *childOutput = cJSON_GetObjectItem(deviceConfig, "outputs")->child;
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
            } else {
                cJSON_AddItemToObject(childOutput, "timer", cJSON_CreateNumber(1));
            }            
        } else if (cJSON_IsString(cJSON_GetObjectItem(childOutput, "state")) &&
            !strcmp(cJSON_GetObjectItem(childOutput, "state")->valuestring, "on")) {
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
                }
            }
        } 
        
        childOutput = childOutput->next;
    }
}

void outputsTimerShot() {
    // test for shooter
    // every 100 ms
    if (!cJSON_IsArray(cJSON_GetObjectItem(deviceConfig, "outputs"))) {        
        return;
    }    
    cJSON *childOutput = cJSON_GetObjectItem(deviceConfig, "outputs")->child;
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
    if (createDeviceConfig() == ESP_OK)
        saveDeviceConfig();  
}

void buttonsStartUpTask(void *pvParameter) {    
	// только для анализа нажатия при старте
    uint8_t cnt = 0;
    uint16_t buttons;
    while (1) {
        buttons = readServiceButtons();
        if ((buttons) == 0) // ничего не нажато
            break;
        if (cnt++ >= 5) {
            ESP_LOGI(TAG, "Startup buttons pressed. Buttons value %d", buttons);
            switch (buttons & 0x0F) {
                case 0b1001: // правая и левая кнопки
                  wifi_init_softap();
                  break;
                case 0b0101: // правая и через одну
                  resetDefaultConfigs();
                  break;
                case 0b0011:
                  resetNetworkConfig();
                default:
                  break;
            }            
            break;
        }
        vTaskDelay(1000 / portTICK_RATE_MS);    
    }

    xTaskCreate(&inputsTask, "inputsTask", 4096, NULL, 5, NULL);
    vTaskDelete(NULL);
}

void startInputTask() {
	// проверка нажатия кнопок при запуске
    if (readServiceButtons() && 0x0F) {
        ESP_LOGI(TAG, "Something pressed while boot... %d", readServiceButtons() && 0x0F);
		showError(0x0F);
        xTaskCreate(&buttonsStartUpTask, "buttonsStartUpTask", 4096, NULL, 5, NULL);
		updateStateHW(0, 0, 0);
    } else {                    
		// ничего не было нажато, можно сразу создавать таск опроса входов
        ESP_LOGI(TAG, "Starting input task");
        xTaskCreate(&inputsTask, "inputsTask", 4096, NULL, 5, NULL);
    }	
}

esp_err_t getDeviceConfig(char **response) {
    ESP_LOGI(TAG, "getDeviceConfig");
    if (!cJSON_IsObject(deviceConfig)) {      
        setErrorTextJson(response, "config is not a json");
        return ESP_FAIL;
    }    
    *response = cJSON_Print(deviceConfig);
    return ESP_OK;    
}

esp_err_t setDeviceConfig(char **response, char *content) {
    cJSON *parent = cJSON_Parse(content);
    
    if(!cJSON_IsObject(parent))
    {
        setErrorTextJson(response, "Is not a JSON array");    
        cJSON_Delete(parent);
        return ESP_FAIL;
    }
       
    cJSON_Delete(deviceConfig);
    deviceConfig = parent;    
    saveDeviceConfig();
    setTextJson(response, "OK");    
    return ESP_OK;
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
        uint16_t duration = 0;
        if (cJSON_IsNumber(cJSON_GetObjectItem(parent, "duration"))) {
            duration = cJSON_GetObjectItem(parent, "duration")->valueint;
        }        
        ESP_LOGI(TAG, "ioservice Output %d. Action %s. Duration %d", 
                 cJSON_GetObjectItem(parent, "output")->valueint, 
                 cJSON_GetObjectItem(parent, "action")->valuestring, duration);
        char *action = cJSON_GetObjectItem(parent, "action")->valuestring;
        if (!strcmp(action, "on") || !strcmp(action, "off") || !strcmp(action, "toggle")) {
            setTextJson(response, "OK");                
            setOutput(cJSON_GetObjectItem(parent, "output")->valueint, action, duration);
        } else {
            setErrorTextJson(response, "Action not found!");
        }        
        cJSON_Delete(parent);
        return ESP_OK;    
    }

    setErrorTextJson(response, "Unknown data");        
    return ESP_FAIL;    
}

esp_err_t getScheduler(char **response) {
    if (!cJSON_IsArray(jScheduler)) {      
        setErrorTextJson(response, "Is not a JSON array");    
        return ESP_FAIL;
    }    
    *response = cJSON_Print(jScheduler);
    return ESP_OK;    
}

esp_err_t setScheduler(char **response, char *content) {
    cJSON *parent = cJSON_Parse(content);
    
    if(!cJSON_IsArray(parent)) {
        setErrorTextJson(response, "Is not a JSON array");    
        cJSON_Delete(parent);
        return ESP_FAIL;
    }       
    cJSON_Delete(jScheduler);
    jScheduler = parent;    
    saveScheduler();
    setTextJson(response, "OK");    
    return ESP_OK;
}

void initScheduler() {
    ESP_LOGI(TAG, "Initiating scheduler");
    // сбрасываем у всех задач признак выполнения
    cJSON *childTask = jScheduler->child;
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
    struct tm *info = getTime();
    uint16_t currentTime = info->tm_hour*60 + info->tm_min;
    uint16_t duration = 0;
    uint16_t grace = 0; // период, в который еще можно запустить задачу
    static uint16_t lastSchedulerTime = 0;
    ESP_LOGI(TAG, "Scheduler time %d. Day of week %d", currentTime, info->tm_wday);
    if (currentTime < lastSchedulerTime) {        
        // когда перевалит за 0.00
        initScheduler();
    }
    lastSchedulerTime = currentTime;
    // currentTime - minutes from 0.00
    // пройти все задачи, время которых еще не настало отметить как done = false
    // время которых прошло или наступило проверить на грейс период, по умолчанию он 5 мин
    // если задача со статусом done = false - выполнить ее и пометить как выполненная
    cJSON *childTask = jScheduler->child;
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
                        if (cJSON_IsNumber(cJSON_GetObjectItem(childAction, "duration")))
                            duration = cJSON_GetObjectItem(childAction, "duration")->valueint;
                        else 
                            duration = 0;
                        ESP_LOGI(TAG, "Scheduler output %d, action %s, duration %d",
                                 cJSON_GetObjectItem(childAction, "output")->valueint, 
                                 cJSON_GetObjectItem(childAction, "action")->valuestring, 
                                 duration);
                        setOutput(cJSON_GetObjectItem(childAction, "output")->valueint, 
                                  cJSON_GetObjectItem(childAction, "action")->valuestring, 
                                  duration);
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

esp_err_t getMqttTopics(char **response) {
    if (!cJSON_IsArray(jMQTTTopics)) {      
        setErrorTextJson(response, "jMQTTTopics is not a JSON array");    
        return ESP_FAIL;
    }    
    *response = cJSON_Print(jMQTTTopics);
    return ESP_OK;    
}

esp_err_t setMqttTopics(char **response, char *content) {
    cJSON *parent = cJSON_Parse(content);
    
    if(!cJSON_IsArray(parent)) {
        setErrorTextJson(response, "jMQTTTopics is not a JSON array");    
        cJSON_Delete(parent);
        return ESP_FAIL;
    }       
    cJSON_Delete(jMQTTTopics);
    jMQTTTopics = parent;    
    saveMqttTopics();
    setTextJson(response, "OK");    
    return ESP_OK;
}

esp_err_t uiRouter(httpd_req_t *req) {    
    //ESP_LOGI(TAG, "%d %s", req->method, req->uri);
    char *uri = getClearURI(req->uri);
    char *response = NULL;
    char *content = NULL;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    httpd_resp_set_type(req, "application/json");
	if (!strcmp(uri, "/service/config/network")) {
        if (req->method == HTTP_GET) {            
            err = getSettings(&response);
        } else if (req->method == HTTP_POST) {
            err = getContent(&content, req);
            if (err == ESP_OK) {
                err = setSettings(&response, content);                
            }
        }        
    } else if (!strcmp(uri, "/service/config/device")) {
        if (req->method == HTTP_GET) {
            err = getDeviceConfig(&response);
        } else if (req->method == HTTP_POST) {
            err = getContent(&content, req);
            if (err == ESP_OK) {
                err = setDeviceConfig(&response, content);    
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
    } else if (!strcmp(uri, "/service/config/scheduler")) {
        if (req->method == HTTP_GET) {            
            err = getScheduler(&response);
        } else if (req->method == HTTP_POST) {            
            err = getContent(&content, req);
            if (err == ESP_OK) {
                err = setScheduler(&response, content);    
            }
        }
    } else if (!strcmp(uri, "/service/config/mqtttopics")) {
        if (req->method == HTTP_GET) {            
            err = getMqttTopics(&response);
        } else if (req->method == HTTP_POST) {            
            err = getContent(&content, req);
            if (err == ESP_OK) {
                err = setMqttTopics(&response, content);    
            }
        }
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

char* getDeviceIOStates() {
    char *response;
    cJSON *json = cJSON_CreateObject();
    cJSON_AddItemToObject(json, "type", cJSON_CreateString("IOSTATES"));
    // io states
    cJSON *jOuts = cJSON_CreateArray();
    cJSON *childOutput = cJSON_GetObjectItem(deviceConfig, "outputs")->child;
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
    cJSON *childInput = cJSON_GetObjectItem(deviceConfig, "inputs")->child;
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
    return response;
}

char* getDeviceConfigMsg() {
    char *response;
    cJSON *json = cJSON_CreateObject();
    cJSON_AddItemToObject(json, "type", cJSON_CreateString("DEVICECONFIG"));
    cJSON_AddItemToObject(json, "payload", deviceConfig);
    response = cJSON_PrintUnformatted(json);    
    return response;
}

void updateOutput(cJSON *payload) {
    // обновление выхода в конфиге. Прилетает из сокета
    if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "id"))) {
        uint8_t id = cJSON_GetObjectItem(payload, "id")->valueint;
        // ищем необходимый выход по айди
        cJSON *childOutput = cJSON_GetObjectItem(deviceConfig, "outputs")->child;
        while (childOutput) { 
            if (cJSON_IsNumber(cJSON_GetObjectItem(childOutput, "id")) &&
                cJSON_GetObjectItem(childOutput, "id")->valueint == id) {
                if (cJSON_IsString(cJSON_GetObjectItem(payload, "name"))) {
                    cJSON_AddStringToObject(childOutput, "name", cJSON_GetObjectItem(payload, "name")->valuestring);
                }
                if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "duration"))) {
                    cJSON_AddNumberToObject(childOutput, "duration", cJSON_GetObjectItem(payload, "duration")->valueint);
                }
                if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "on"))) {
                    cJSON_AddNumberToObject(childOutput, "on", cJSON_GetObjectItem(payload, "on")->valueint);
                }
                if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "off"))) {
                    cJSON_AddNumberToObject(childOutput, "off", cJSON_GetObjectItem(payload, "off")->valueint);
                }
                if (cJSON_IsString(cJSON_GetObjectItem(payload, "type"))) {
                    cJSON_AddStringToObject(childOutput, "type", cJSON_GetObjectItem(payload, "type")->valuestring);
                }
                if (cJSON_IsString(cJSON_GetObjectItem(payload, "default"))) {
                    cJSON_AddStringToObject(childOutput, "default", cJSON_GetObjectItem(payload, "default")->valuestring);
                }
                if (cJSON_IsBool(cJSON_GetObjectItem(payload, "alice"))) {
                    cJSON_AddBoolToObject(childOutput, "alice", cJSON_IsTrue(cJSON_GetObjectItem(payload, "alice")));
                }                
                break;
            }   
            childOutput = childOutput->next;
        }    
    }
}

void updateInput(cJSON *payload) {
    // обновление входа в конфиге. Прилетает из сокета
    if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "id"))) {
        uint8_t id = cJSON_GetObjectItem(payload, "id")->valueint;
        // ищем необходимый выход по айди
        cJSON *childInput = cJSON_GetObjectItem(deviceConfig, "inputs")->child;
        while (childInput) { 
            if (cJSON_IsNumber(cJSON_GetObjectItem(childInput, "id")) &&
                cJSON_GetObjectItem(childInput, "id")->valueint == id) {
                // нашли нужный вход
                if (cJSON_IsString(cJSON_GetObjectItem(payload, "name"))) {
                    cJSON_AddStringToObject(childInput, "name", cJSON_GetObjectItem(payload, "name")->valuestring);
                }
                if (cJSON_IsString(cJSON_GetObjectItem(payload, "type"))) {
                    cJSON_AddStringToObject(childInput, "type", cJSON_GetObjectItem(payload, "type")->valuestring);
                }
                if (cJSON_IsArray(cJSON_GetObjectItem(payload, "events"))) {
                    cJSON_ReplaceItemInObject(childInput, "events", cJSON_GetObjectItem(payload, "events"));                    
                }
                break;
            }   
            childInput = childInput->next;
        }    
    }
}

void wsMsg(char *message) {
	// ESP_LOGI(TAG, "wsMsg %s", message);
    //ESP_LOG_BUFFER_HEXDUMP("message", message, strlen(message)+1, CONFIG_LOG_DEFAULT_LEVEL);
    char *response;
    cJSON *json = cJSON_Parse(message); 
    cJSON *payload = NULL;
    if(!cJSON_IsObject(json)) {
        ESP_LOGE(TAG, "Can't parse message.");
        return;
    }
    if (cJSON_IsObject(cJSON_GetObjectItem(json, "payload"))) {
        payload = cJSON_GetObjectItem(json, "payload");
    }
    char *type;
    if (cJSON_IsString(cJSON_GetObjectItem(json, "type"))) {
        type = cJSON_GetObjectItem(json, "type")->valuestring;
        if (!strcmp(type, "ALERT") && cJSON_IsString(cJSON_GetObjectItem(payload, "message"))) {
            // если сообщение READY то он прилинкован и готов общаться
            // если сообщение INFO то значит просто отправить информацию и ждать последующей линковки
            if (!strcmp(cJSON_GetObjectItem(payload, "message")->valuestring, "READY")) { 
                WSSetAuthorized();
                response = getDeviceIOStates();
                WSSendMessage(response);  
                free(response);
            }
        } else if (!strcmp(type, "GETDEVICECONFIG")) {
            response = getDeviceConfigMsg();            
            WSSendMessageForce(response);              
            free(response);                     
        } else if (!strcmp(type, "SETDEVICECONFIG") && payload != NULL) {
            if (cJSON_IsObject(payload)) {
                cJSON_Delete(deviceConfig);
                deviceConfig = payload;
            }
        } else if (!strcmp(type, "UPDATEOUTPUT") && payload != NULL) {
            updateOutput(payload);
        } else if (!strcmp(type, "UPDATEINPUT") && payload != NULL) {
            updateInput(payload);    
        } else if (!strcmp(type, "ACTION") && payload != NULL) {
            if (cJSON_IsString(cJSON_GetObjectItem(payload, "mac")) &&
                strcmp(toUpper(cJSON_GetObjectItem(payload, "mac")->valuestring), getMac())) {
                ESP_LOGE(TAG, "Wrong mac %s", cJSON_GetObjectItem(payload, "mac")->valuestring);                
            } else if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "input"))) { 
                uint8_t slaveId = 0;
                if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "slaveId")))
                    slaveId = cJSON_GetObjectItem(payload, "slaveId")->valueint;
                processInputEvents(slaveId, cJSON_GetObjectItem(payload, "input")->valueint, "toggle", 255);                                
            } else if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "output"))) {
                uint8_t pOutput = cJSON_GetObjectItem(payload, "output")->valueint;
                            
                char* pValue = NULL;
                uint16_t pDuration = 0;
                uint8_t pSlaveId = 0;
                
                if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "duration"))) {
                    pDuration = cJSON_GetObjectItem(payload, "duration")->valueint;
                }
                if (cJSON_IsString(cJSON_GetObjectItem(payload, "action"))) {
                    pValue = cJSON_GetObjectItem(payload, "action")->valuestring;
                }               
                if (cJSON_IsNumber(cJSON_GetObjectItem(payload, "slaveid"))) {
                    pSlaveId = cJSON_GetObjectItem(payload, "slaveid")->valueint;
                }           
                // TODO : new modbus process
                if (pValue != NULL) {
                    if (pSlaveId > 0) {
                        setRemoteOutput(pSlaveId, pOutput, pValue, pDuration);
                    } else {                                
                        setOutput(pOutput, pValue, pDuration);
                    }           
                }
            }
        } 
    }
    cJSON_Delete(json);

}

void wsEvent(uint8_t event) {
    if (event == WEBSOCKET_EVENT_CONNECTED) {
        wsConnected = true;
    } else if (event == WEBSOCKET_EVENT_DISCONNECTED) {
        wsConnected = false;
    }
}

void initWS() {
    if (getSettingsValueBool2("cloud", "enabled")) {
        char *jwt = NULL;                
        char *wsURL = getSettingsValueString2("cloud", "address");
        loadTextFile("/certs/jwt.pem", &jwt);
    	WSinit(wsURL, "relayController", &wsMsg, &wsEvent, jwt, 
               getSettingsValueBool2("cloud", "log"), mbMode, mbSlaveId);            
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

void modBusAction(uint8_t output, char *action, uint8_t duration) {
    // обработчик действия для слейва
    setOutput(output, action, duration);
}

void initModBus() {
    if (getSettingsValueBool2("modbus", "enabled")) {
        if (!strcmp(getSettingsValueString2("modbus", "mode"), "master")) {
            MBInitMaster(deviceConfig, &modBusEvent, getSettingsValueObject2("modbus", "slaves"), controllerType > 2);
            mbMode = "master";
        } else if (!strcmp(getSettingsValueString2("modbus", "mode"), "slave")) {
            mbSlaveId = getSettingsValueInt2("modbus", "slaveid");
            MBInitSlave(mbSlaveId, &modBusAction, controllerType > 2);
            mbSlave = true;
            mbMode = "slave";            
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
    strcpy(str, getSettingsValueString("hostname"));
    strcat(str, "/in/\0");
    if (!strstr(topic, str)) {
        // это не топики своего устройства. Проверить внешний список
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
                                        setRemoteOutput(slaveId, output, action, 0);            
                                    } else {
                                        setOutput(output, action, 0);
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
    uint16_t duration = 0;

    if (strstr(topic, "/json")) {
        cJSON *jData = cJSON_Parse(data);
        if (!cJSON_IsObject(jData)) {
            ESP_LOGE(TAG, "data is not a json");
            return;
        }
        if (cJSON_IsNumber(cJSON_GetObjectItem(jData, "slaveid")))
            slaveId = cJSON_GetObjectItem(jData, "slaveid")->valueint;
        if (cJSON_IsNumber(cJSON_GetObjectItem(jData, "output")))
            output = cJSON_GetObjectItem(jData, "output")->valueint;
        if (cJSON_IsString(cJSON_GetObjectItem(jData, "action")))
            action = cJSON_GetObjectItem(jData, "action")->valuestring;
        if (cJSON_IsNumber(cJSON_GetObjectItem(jData, "duration")))
            duration = cJSON_GetObjectItem(jData, "duration")->valueint;
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
        token = strtok(NULL, "/"); // slaveid
        if (token == NULL)
            return;
        slaveId = atoi(token);    
        token = strtok(NULL, "/"); // output
        if (token == NULL)
            return;
        output = atoi(token); 
        action = toLower(data);                
    }

    ESP_LOGI(TAG, "slaveid %d, output %d, action %s", slaveId, output, action);

    if (xSemaphoreTake(sem_busy, portMAX_DELAY) == pdTRUE) {
        if (slaveId) {
            // remote action
            setRemoteOutput(slaveId, output, action, duration);            
        } else {
            // local
            setOutput(output, action, duration);
        }
        xSemaphoreGive(sem_busy);
    }
}

void initMQTT() {
    if (getSettingsValueBool2("mqtt", "enabled")) {
        MQTTInit(&mqttData, &mqttEvent);
    }
}

void initFTP(uint32_t address) {
    if (getSettingsValueBool2("ftp", "enabled")) { 
        FTPinit(getSettingsValueString2("ftp", "user"), getSettingsValueString2("ftp", "pass"), address);
    }
}

void sntpEvent() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "Clock is %s", asctime(&timeinfo));
    ESP_LOGI(TAG, "Clock set result %s", esp_err_to_name(setClock()));    
}

void initCore(SemaphoreHandle_t sem) {
	//createSemaphore();    
    ESP_LOGI(TAG, "Hostname %s, description %s", getSettingsValueString("hostname"), getSettingsValueString("description"));
    sem_busy = sem;
    initHardware(sem);    
	determinateControllerType();
	ESP_LOGI(TAG, "Controllertype is %s", controllersData[controllerType].name);
    if (controllerType == UNKNOWN) {
        ESP_LOGE(TAG, "Unknown controller type!");
        setRGBFace("yellow");
        //return;
    }
	if (loadDeviceConfig() != ESP_OK) {
        ESP_LOGE(TAG, "Can't read config!");
        setRGBFace("red");
        return;
    }
    if (loadScheduler() != ESP_OK) {
        ESP_LOGI(TAG, "Can't read scheduler config.");
    }
    if (loadMqttTopics() != ESP_OK) {
        ESP_LOGI(TAG, "Can't read mqtttopics config.");
    }    
    initInputs();
	initOutputs();    
    startInputTask();
    initScheduler();
	xTaskCreate(&serviceTask, "serviceTask", 4096, NULL, 5, NULL);    
    initModBus();    
    setRGBFace("green"); // TODO : сделать зеленый когда все поднялось. И продумать цвета
}

/* TODO :
* scheduler
* timer on/off
* shooter
* modbus
* publish mqtt
* publish mqtt info every 1 minute?
* publish ws
* ws
* mqtt взаимодействие с другими устройствами?. Сделать маппинг, например, /Device1/0/0/ON -> slaveid0 output1 on
+ hardware. на старом контроллере кнопки начинаются с 16 айди, а на новом почему-то с 8, надо где-то исправить
- разобраться с eth. Работает на другой плате
- часы надо сделать, пока не получилось
+ почему-то состояния входов не актуальны. Т.е. при вкючении горят входы, которые не должны гореть, а после однократного изменения состояния по ним все ок
+ надо проверить при закороченном входе включение
+ определение устройств i2c через detect i2c
* переделать type wait на action wait и на бэке тоже
* когда по умолчанию выход был активен то надо чтобы реле щелкнуло, иначе оно будет просто "петь"
*/

/* modbus
+ на мастере в основной конфиг добавляются входы и выходы слейвов
- при редактировании входов и выходов через WS или рест необходимо будет отправлять конфигурацию на слейвы
+ события со всех слейвов приходят на мастер
+ события улетают в ws/mqtt
+ слейвы отрабатывают свои правила сами
- придумать как передавать конфиг от мастера и обратно (Может это делать через UI?)
- слейвы могут быть подключены к интернету, но тогда они не шлют сообщения WS. Шлет только мастер
- на бэкэнде это как будет выглядеть?

[
{
    "slaveId": 1,
    "enabled": true,
    "outputsState": 0,
    "inputsState": 0,
    "outputs": [

    ],
    "inputs": [
    ]
}
]
*/

