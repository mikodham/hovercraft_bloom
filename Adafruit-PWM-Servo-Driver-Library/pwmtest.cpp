// #include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

int main(){
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(1600);  // This 
    while(true){
        // Drive each PWM in a 'wave'
        for (uint16_t i=0; i<4096; i += 8) {
            for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) {
                pwm.setPWM(pwmnum, 0, (i + (4096/16)*pwmnum) % 4096 );
            }
        }
    }
};