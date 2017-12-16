#include "mbed.h"

DigitalOut led(PC_13);

int main2() {
	led = 1;
    while(1) {
    	wait(0.2);
    	led = !led;
    }
}
