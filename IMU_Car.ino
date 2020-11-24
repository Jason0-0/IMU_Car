#include <arduino.h>
#include <inttypes.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include "ITG3200.h"
//#include <MsTimer2.h>

//wheel 1
#define IN1 9
#define IN2 10

//wheel 2
#define IN3 6
#define IN4 5

//servo
//const int ServoPin=3;

//ultra sonic
//const int TRIG=12;
//const int ECHO=13;

//LCD
#define LCD_RS 2
#define LCD_EN 12
#define LCD_D4 14
#define LCD_D5 15
#define LCD_D6 16 
#define LCD_D7 17

//gyro pid params 3.0 0.0
#define G_KP 3.0f
#define G_KD 0.6f
#define G_KI 0.0F
#define OUTPUT_MAX 100.0f
#define ACCEPTABLE_ERR 0//0.2F

//timer
#define SAMPLETIME 100    //每次采样的时间间隔（ms）

//given distance running
#define DISTANCE 7 //M
#define TIME_USED 15 //S

typedef struct
{
    float x;
    float y;
    float z;
    float yawAngle;
    float err[3];
}Axis_g;
//Servo srTurn;

//class & structs defines
//LCD 4-dataline connected to the Arduino
LiquidCrystal lcd(LCD_RS,LCD_EN,LCD_D4,LCD_D5,LCD_D6,LCD_D7);

//gyro
ITG3200 myGyro;    //陀螺仪类
Axis_g gyroData={0.0f,0.0f,0.0f,0.0f,{0.0f,0.0f,0.0f}}; //三轴角速度和累计转角

double temp=0;  //温度
//初始速度
int leftSpeed=150;  
int rightSpeed=150;

bool lcdUpdate=false;
bool serialUpdate=false;
long nowTime=0;
long beginTime=0;
const float v=(float)DISTANCE/TIME_USED;
float nowDist=0.0;

void setup()
{
    Serial.begin(115200);
  //  pinMode(ServoPin,OUTPUT);
  //  srTurn.attach(ServoPin); 
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);

    lcd.begin(16,2);
    lcd.print("  NUO_NUO YYDS!");

    //gyro setup
    myGyro.init();  //初始化陀螺仪
    myGyro.zeroCalibrate(400,8); //零位置校准，采样400次，一共耗时8*400ms
    Serial.println("done");
    lcd.clear();
    lcd.print("RUSH B!");
    //MsTimer2::set(INTERVAL,GyroUpdate);
    //MsTimer2::start();
    nowTime=millis();
    beginTime=millis();
}

void loop()
{    
    if (millis()-nowTime>SAMPLETIME)
    {
        nowTime=millis();
        //serialUpdate=true;
        GyroUpdate();   //更新陀螺仪数据
    }
    
    if (serialUpdate)
    {
        serialUpdate=false;
        Serial.print(beginTime);
        Serial.print(" ");
        Serial.println(millis());
        // Serial.print(gyroData.x);
        // Serial.print(" ");
        // Serial.print(gyroData.y);
        // Serial.print(" ");
        //Serial.print(gyroData.z);
        //Serial.print(" ");
        //Serial.println(gyroData.yawAngle);
        //Serial.println(outputt);
        

    }
    delayDisplay();  //LCD显示更新
    Rush(leftSpeed,rightSpeed);
    MovementCorrect();  //运动参数更新
    
}

void Rush(int leftSpeed,int rightSpeed)
{
    if(millis()-beginTime>TIME_USED*1000+300)
    {
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,HIGH);
        
        while(1){}
    }
    if(abs(leftSpeed)>255)leftSpeed=255;
    if(abs(rightSpeed)>255)rightSpeed=255;
    if(abs(leftSpeed)<0)leftSpeed=0;
    if(abs(rightSpeed)<0)rightSpeed=0;
    //lcd.clear();
    if(leftSpeed>0)
    {
        analogWrite(IN1,leftSpeed);
        analogWrite(IN2,0);
    }
    else
    {
        analogWrite(IN1,0);
        analogWrite(IN2,-leftSpeed);
    }
    if(rightSpeed>0)
    {
        analogWrite(IN3,rightSpeed);
        analogWrite(IN4,0);
    }
    else
    {
        analogWrite(IN3,0);
        analogWrite(IN4,-rightSpeed);
    }
    
    
    //lcd.print(leftSpeed);lcd.print(rightSpeed);
    // Serial.print(leftSpeed);
    // Serial.print(" ");
    // Serial.println(rightSpeed);
    //delay(delaytime);
}

inline void MovementCorrect()
{   
    int dPWM=0;    //每次调整的pwm量 
    dPWM=(int)PIDCal(&gyroData);
    //dPWM;
    
    rightSpeed=rightSpeed+dPWM;
    leftSpeed=leftSpeed-dPWM;   
    //Serial.println(dYaw);
}
void delayDisplay()
{
    //static int nowTime=0;
    if (lcdUpdate)
    {
    //     lcd.clear();
    //     lcd.home();
    //     lcd.print("YawAngle:");
    //     lcd.print(gyroData.yawAngle);
    //     //lcd.setCursor(0,4);
    //    // lcd.print(" err:");
        
    //     //lcd.print(dYaw);
    //     lcd.setCursor(1,2);
    //     lcd.print(myGyro.getTemperature());
    //     //nowTime=millis();
        nowDist=(millis()-beginTime)*v/1000;
        Serial.println(nowDist);
        lcd.home();
        lcd.print("X:");lcd.print(nowDist);lcd.print(" Y:0.0");
        lcd.setCursor(0,1);
        lcd.print("DIST:");lcd.print(DISTANCE-nowDist);
        lcdUpdate=false;
    }
    

    
}

//增量式PID
float PIDCal(Axis_g* gyroData)
{
    float output=0.0f;
    //Serial.println(gyroData->err[1]);
    gyroData->err[2]=gyroData->err[1];
    gyroData->err[1]=gyroData->err[0];
    gyroData->err[0]=gyroData->yawAngle;
    output=G_KP*(gyroData->err[0]-gyroData->err[1])+G_KI*(gyroData->err[0]-2.0f*gyroData->err[1]+gyroData->err[2])+G_KI*gyroData->err[0];
    //输出上限
    if (output>OUTPUT_MAX) output=OUTPUT_MAX;
    if(output<-(OUTPUT_MAX)) output=-(OUTPUT_MAX);
    return output;
    
    
    
}

void GyroUpdate()
{   
    serialUpdate=true;
    static int times=0;
    if(times>10)
    {
        times=0;
        lcdUpdate=true;
    }
    //获取陀螺仪数据
    myGyro.getAngularVelocity(&(gyroData.x),&(gyroData.y),&(gyroData.z));
    //差分积分
    if(gyroData.z>ACCEPTABLE_ERR||gyroData.z<-(ACCEPTABLE_ERR))
        gyroData.yawAngle+=gyroData.z*(float)SAMPLETIME/1000.0f;
    times++;
}
