void runWebServer();
void initWS();
void initMQTT();
void initCore(SemaphoreHandle_t sem);
void initFTP(uint32_t address);
//SemaphoreHandle_t getSemaphore();
//void sntpEvent(struct tm timeinfo);
void sntpEvent();
