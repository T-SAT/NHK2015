#ifndef motor_h
#define motor_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <wiring_private.h>

//モーター制御ピン
//PD7(d7):dir_R (M1IN1)
//PD5(d5):PWM_R (M1IN2)
//PD6(d6):PWM_L (M2IN1)
//PB0(d8):dir_L (M2IN2)

namespace Motor {
void init(void);	//ピンとpwm_duty上限を与えて初期化
void run(int motor_L, int motor_R);	//pwm_dutyの設定
void stop(void);	          //現在のpwm_dutyを出力
};

void Motor::init(void)
{
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  sbi(TCCR0A, COM0A1); //5ピンでTimer0_PWMをするためのレジスタ設定
  sbi(TCCR0A, COM0B1); //60ピンでTimer0_PWMをするためのレジスタ設定
}

void Motor::run(int motorL, int motorR) //前と同じです。
{
  motorR = constrain(motorR, -254, 254);
  motorL = constrain(motorL, -254, 254);
  Serial.println("in motor::run");
  if (motorR < 0)
  {
    /*
    PORTD |= _BV(7);      //digitalWrite(in1Pin, HIGH);
    OCR0A = motorR + 255; //analogWrite(in2Pin, motorR+255);*/
    digitalWrite(7, HIGH);
    analogWrite(5, motorR + 255);
  }
  else
  {
    /*
    PORTD &= ~_BV(7);      //digitalWrite(in2Pin, LOW);
    OCR0A = motorR;       //analogWrite(in2Pin, motorR);*/
    digitalWrite(7, LOW);
    analogWrite(5, motorR);
  }

  if (motorL < 0)
  {
    /*
    PORTB |= _BV(0);      //digitalWrite(in3Pin, HIGH);
    OCR0B = motorL + 255; //analogWrite(in4Pin, motorL+255);*/
    digitalWrite(8, HIGH);
    analogWrite(6, motorL + 255);
  }
  else
  {
    /*
    PORTB &= ~_BV(0);     //digitalWrite(in3Pin, LOW);
    OCR0B = motorL;       //analogWrite(in4Pin, motorL);*/
    digitalWrite(8, LOW);
    analogWrite(6, motorL);
  }
  
  Serial.println("out motor::run");
}

void Motor::stop(void) //ブレーキ掛けます。
{
  digitalWrite(7, LOW);
  analogWrite(5, 0);
  digitalWrite(8, LOW);
  analogWrite(6, 0);
}

#endif
