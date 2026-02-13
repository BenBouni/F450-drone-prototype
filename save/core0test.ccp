#include <atomic>
#include <Arduino.h>
#include <SPI.h>

std::atomic <float> blinkdelay;
unsigned long delay_ajout = 0;


class blink {
    private : 
    unsigned long blinkdel = 0 ;
    bool led_state = LOW ;
    int LED;
    public : 
    blink(int led) : LED(led)  {pinMode(LED, OUTPUT); };
    void uBlink() {
        if ( millis() - blinkdel > blinkdelay) {
         led_state = !led_state; 
         blinkdel = millis();
        }
        digitalWrite( LED, led_state);
    }
};

blink B(3);

void test(void * pvParameters) {
    for(;;){
        B.uBlink();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    xTaskCreatePinnedToCore(
        test,
        "blinkTEST",
        10000,
        NULL,
        1,
        NULL,
        0
    );
}

void loop() { 
    if (millis()-delay_ajout > 5000)
    blinkdelay = blinkdelay + 20;
}

