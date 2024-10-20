#define BOUTPUTS 6
#define BINPUTS 4

void sendTo595(uint8_t *values, uint8_t count);
void readFrom165(uint8_t *values, uint8_t count);
void setGPIOOut(uint8_t gpio);
void setGPIOIn(uint8_t gpio);
void initHardware(SemaphoreHandle_t sem);
esp_err_t initI2Cdevices();
void setI2COut(uint8_t adr, uint8_t num, uint16_t value);
uint8_t readFrom8574(uint8_t adr);
void setRelayValues(uint16_t values);
enum controllerTypes {
		UNKNOWN = 0,
		RCV1S = 1,
		RCV1B = 2,
		RCV2S = 3,
		RCV2M = 4,
		RCV2B = 5
	};
enum controllerTypes controllerType;
// enum controllerTypes determinateControllerTypeHW();
char* getControllerTypeText(uint8_t type);
void updateStateHW(uint16_t outputs, uint16_t inputsLeds, uint16_t outputsLeds);
void readInputs(uint8_t *values, uint8_t count);
uint16_t readServiceButtons();
void setRGBFace(char* color);
esp_err_t setClock();
esp_err_t getClock(struct tm time);