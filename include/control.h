#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>


#define dir_frontRight PB7
#define pwm_frontRight PB6
#define encA_frontRight 0 
#define encB_frontRight 0
#define dir_frontLeft PA8 //PA0
#define pwm_frontLeft PB15 //PA1
#define encA_frontLeft 0 //PA2
#define encB_frontLeft 0 //PA3
#define dir_middleRight PB9 //PA8
#define pwm_middleRight PB8 //PB15
#define encA_middleRight 0 //PB14
#define encB_middleRight 0 //PB13
#define dir_middleLeft PB3 //PA0
#define pwm_middleLeft PA15 //PA1
#define encA_middleLeft 0 //PA2
#define encB_middleLeft 0 //PA3
#define dir_rearRight PB13 //PA7//PB9
#define pwm_rearRight PB14 //PB0//PB8
#define encA_rearRight 0//PB7
#define encB_rearRight 0//PB6
#define dir_rearLeft PB5 // PB9//PA7
#define pwm_rearLeft PB4 // PB8//PB0
#define encA_rearLeft  0//PB1
#define encB_rearLeft  0//PB2

float Kp=0;
float Ki=0;
float Kd=0;

class motor
{

  private:


  uint8_t direction, pwm, encA, encB; 
  double encVAl_a,encVAl_b;
  float error_sum = 0,integral=0,prev_error=0;
  int time_interval = 0.02;

  public:
    volatile long encoderValue;
    float vel,curr_vel;

  motor(int direction, int pwm,int encA, int encB)
  {
    this->direction = direction;
    this->pwm = pwm;
    this->encA = encA;
    this->encB = encB;
    encoderValue = 0;
    vel=0;
    curr_vel=0;
  }

  void rotateClockwise(int pwm_value)
  {
    digitalWrite(direction, 1);
    // if (curr_vel < pwm_value)
    // {
    //         Kp=0.08;
    //         Ki=0.00085;
    //         Kd=0.0009;
    //     while (curr_vel+1 < pwm_value)
    //     {
    //         float error = pwm_value - curr_vel;
    //         error_sum += error*time_interval;
    //         integral +=error_sum;
    //         curr_vel += Kp * error + Ki * integral + Kd * (error-prev_error) / time_interval;
    //         prev_error=error;
    //         analogWrite(pwm, curr_vel);
    //     }
    // } else if (curr_vel > pwm_value)
    // {
    //     Kp=0.05;
    //     Ki=0.000085;
    //     Kd=0.0009;
    //     while (curr_vel-1 > pwm_value)
    //     {
    //         float error = pwm_value - curr_vel;
    //         error_sum += error*time_interval;
    //         integral +=error_sum;
    //         curr_vel += Kp * error + Ki * integral + Kd * (error-prev_error) / time_interval;
    //         prev_error=error;
    //         analogWrite(pwm, curr_vel);
    //      }
    
    // }
            analogWrite(pwm, pwm_value);

  }

  void rotateAntiClockwise(int pwm_value)
  {
    digitalWrite(direction, 0);
    // if (curr_vel < pwm_value)
    // {
    //         Kp=0.08;
    //         Ki=0.00085;
    //         Kd=0.0009;
    //     while (curr_vel+1 < pwm_value)
    //     {
    //         float error = pwm_value - curr_vel;
    //         error_sum += error*time_interval;
    //         integral +=error_sum;
    //         curr_vel += Kp * error + Ki * integral + Kd * (error-prev_error) / time_interval;
    //         prev_error=error;
    //         analogWrite(pwm, curr_vel);
    //     }
    // } else if (curr_vel > pwm_value)
    // {
    //     Kp=0.05;
    //     Ki=0.000085;
    //     Kd=0.0009;
    //     while (curr_vel-1 > pwm_value)
    //     {
    //         float error = pwm_value - curr_vel;
    //         error_sum += error*time_interval;
    //         integral +=error_sum;
    //         curr_vel += Kp * error + Ki * integral + Kd * (error-prev_error) / time_interval;
    //         prev_error=error;
    //         analogWrite(pwm, curr_vel);
    //      }
    // }
            analogWrite(pwm, pwm_value);

  }
 
  
    void encoder1_1()
    {
      int b = digitalRead(encB);
      if (b==0)
      {
          encoderValue++;
      }
      else{
          encoderValue--;
      }
    }
    void encoder1_2()
    {
      int b = digitalRead(encA);
      if (b==0)
      {
          encoderValue--;
      }
      else{
          encoderValue++;
      }
    }

};


motor frontLeft(dir_frontLeft, pwm_frontLeft, encA_frontLeft, encB_frontLeft);
motor frontRight(dir_frontRight, pwm_frontRight, encA_frontRight, encB_frontRight);
motor middleLeft(dir_middleLeft, pwm_middleLeft, encA_middleLeft, encB_middleLeft);
motor middleRight(dir_middleRight, pwm_middleRight, encA_middleRight, encB_middleRight);
motor rearLeft(dir_rearLeft, pwm_rearLeft, encA_rearLeft, encB_rearLeft);
motor rearRight(dir_rearRight, pwm_rearRight, encA_rearRight, encB_rearRight);






void drive(int vel){
    if(vel>=0){
        frontLeft.rotateAntiClockwise(abs(vel));
        frontRight.rotateClockwise(abs(vel));
        middleLeft.rotateAntiClockwise(abs(vel));
        middleRight.rotateClockwise(abs(vel));   
        rearLeft.rotateAntiClockwise(abs(vel));
        rearRight.rotateClockwise(abs(vel));
    }else if(vel<0){
        frontLeft.rotateClockwise(abs(vel));
        frontRight.rotateAntiClockwise(abs(vel));
        middleLeft.rotateClockwise(abs(vel));
        middleRight.rotateAntiClockwise(abs(vel));   
        rearLeft.rotateClockwise(abs(vel));
        rearRight.rotateAntiClockwise(abs(vel));
    }
}

void spot_drive(int vel){
    if(vel>=0){
        frontLeft.rotateAntiClockwise(abs(vel));
        frontRight.rotateAntiClockwise(abs(vel));
        middleLeft.rotateAntiClockwise(abs(vel));
        middleRight.rotateAntiClockwise(abs(vel));   
        rearLeft.rotateAntiClockwise(abs(vel));
        rearRight.rotateAntiClockwise(abs(vel));
    }else if(vel<0){
        frontLeft.rotateClockwise(abs(vel));
        frontRight.rotateClockwise(abs(vel));
        middleLeft.rotateClockwise(abs(vel));
        middleRight.rotateClockwise(abs(vel));   
        rearLeft.rotateClockwise(abs(vel));
        rearRight.rotateClockwise(abs(vel));
    }
}

void initial_setup(){
pinMode(dir_frontRight,OUTPUT); 
pinMode(pwm_frontRight,OUTPUT);  
pinMode(encA_frontRight,INPUT_PULLUP);
pinMode(encB_frontRight,INPUT_PULLUP);
pinMode(dir_frontLeft,OUTPUT);  
pinMode(pwm_frontLeft,OUTPUT);  
pinMode(encA_frontLeft,INPUT_PULLUP); 
pinMode(encB_frontLeft,INPUT_PULLUP); 
pinMode(dir_middleRight,OUTPUT);  
pinMode(pwm_middleRight,OUTPUT);  
pinMode(encA_middleRight,INPUT_PULLUP); 
pinMode(encB_middleRight,INPUT_PULLUP); 
pinMode(dir_middleLeft,OUTPUT);  
pinMode(pwm_middleLeft,OUTPUT);  
pinMode(encA_middleLeft,INPUT_PULLUP); 
pinMode(encB_middleLeft,INPUT_PULLUP); 
pinMode(dir_rearRight,OUTPUT); 
pinMode(pwm_rearRight,OUTPUT);  
pinMode(encA_rearRight,INPUT_PULLUP); 
pinMode(encB_rearRight,INPUT_PULLUP); 
pinMode(dir_rearLeft,OUTPUT);  
pinMode(pwm_rearLeft,OUTPUT);  
pinMode(encA_rearLeft,INPUT_PULLUP);  
pinMode(encB_rearLeft,INPUT_PULLUP);  
}
