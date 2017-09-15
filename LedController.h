#pragma once
#include <unistd.h>
#include <fcntl.h>

class LedController {
public:
    LedController(char * gpio_path){
       // fd2led = open("/sys/class/gpio/gpio158/value", O_WRONLY);
        fd2led = open(gpio_path, O_WRONLY);
    }
    void ledON(){
        write(fd2led, "1", 2);
    }
    void ledOFF(){
        write(fd2led, "0", 2);
    }
    void flash(int time){
        if (time > flash_cnt){
            led_status = !led_status;
            flash_cnt = 0;
        }
        if (led_status)
            ledON();
        else
            ledOFF();
        ++flash_cnt;
    }
	int readled(){
		int fd2ledin=open("/sys/class/gpio/gpio157/value",O_RDONLY);
		char value;
		read(fd2ledin, &value,1);
		//printf("-----%c------",value);
		close(fd2ledin);
		if (value=='1')
			return 1;
		else
			return 0;
	}

public:
    bool led_status;
    int flash_cnt;
    int fd2led;
};
