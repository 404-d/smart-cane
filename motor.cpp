#include<iostream>
#include<wiringPi.h> //type=ignored
#include<stdio.h>

//定义引脚
#define INT1 0 //GPIO. 0
#define INT2 1 //GPIO. 1
#define INT3 2 //GPIO. 2
#define INT4 3 //GPIO. 3

int main(){
    //初始化GPIO    
    wiringPiSetup();
    pinMode(INT1, OUTPUT);git config --global --unset https.proxy

    pinMode(INT2, OUTPUT);
    pinMode(INT3, OUTPUT);
    pinMode(INT4, OUTPUT);

    //电机旋转
    digitalWrite(INT1,HIGH);
    digitalWrite(INT2,LOW);
    digitalWrite(INT3,HIGH);
    digitalWrite(INT4,LOW);

    delay(10000);

    //停止
    digitalWrite(INT1,LOW);
    digitalWrite(INT2,LOW);
    digitalWrite(INT3,LOW);
    digitalWrite(INT4,LOW);

    return 0;
}