#include <AccelStepper.h>
#define motorInterfaceType 1

const unsigned int receive_length = 19;
const int dir1Pin = 22, dir2Pin = 24, dir3Pin = 26, dir4Pin = 28, dir5Pin = 30,
          step1Pin = 3, step2Pin = 4, step3Pin = 5, step4Pin = 6, step5Pin = 7;

AccelStepper stepper1(motorInterfaceType, step1Pin, dir1Pin);
AccelStepper stepper2(motorInterfaceType, step2Pin, dir2Pin);
AccelStepper stepper3(motorInterfaceType, step3Pin, dir3Pin);
AccelStepper stepper4(motorInterfaceType, step4Pin, dir4Pin);
AccelStepper stepper5(motorInterfaceType, step5Pin, dir5Pin);

//AccelStepper* stepper_motor_ptr_arr[5] = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5};

void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  
  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);
  stepper3.setMaxSpeed(1000);
  stepper4.setMaxSpeed(1000);
  stepper5.setMaxSpeed(1000);
}

void loop() {
  static bool receive_flag = false, receive_flag2 = false;
  static int receive_counter, int_dir, int_speed;
  static char rxData[receive_length], inc_char, inc_char2; 
  static String str_dir, str_speed, inc_str, inc_str2;
  
  if(Serial1.available() > 0){
    inc_char = Serial1.read();
    delay(1);
    
    if(inc_char == 'S'){
      receive_flag=true;
      receive_counter = 0;
    }
    
    if(receive_flag && inc_char!='S' && inc_char!='F'){
      inc_str += inc_char;
      rxData[receive_counter] = inc_char;
      receive_counter++;
    }
    if(inc_char == 'F'){
      Serial.println("S" + inc_str + "F");
      
      str_dir = rxData[0]; 
      int_dir = str_dir.toInt();
      str_speed = String(rxData[1]) + String(rxData[2]) + String(rxData[3]); 
      int_speed = str_speed.toInt();
      
      if(rxData[4] == '1'){
        if(int_dir == 1)stepper1.setSpeed(-int_speed*3);  
        else if(int_dir == 0)stepper1.setSpeed(int_speed*3);
      }
      else if(rxData[4] == '0')stepper1.setSpeed(0);
      
      if(rxData[5] == '1'){
        if(int_dir == 1)stepper2.setSpeed(-int_speed*3);  
        else if(int_dir == 0)stepper2.setSpeed(int_speed*3);
      }
      else if(rxData[5] == '0')stepper2.setSpeed(0);
      
      if(rxData[6] == '1'){
        if(int_dir == 1)stepper3.setSpeed(-int_speed*3);  
        else if(int_dir == 0)stepper3.setSpeed(int_speed*3);
      }
      else if(rxData[6] == '0')stepper3.setSpeed(0);
      
      if(rxData[7] == '1'){
        if(int_dir == 1)stepper4.setSpeed(-int_speed*3);  
        else if(int_dir == 0)stepper4.setSpeed(int_speed*3);
      }
      else if(rxData[7] == '0')stepper4.setSpeed(0);
      
      if(rxData[8] == '1'){
        if(int_dir == 1)stepper5.setSpeed(-int_speed*3);  
        else if(int_dir == 0)stepper5.setSpeed(int_speed*3);
      }
      else if(rxData[8] == '0')stepper5.setSpeed(0);
      
      receive_flag=false;
      for(int i=0; i<receive_length-2; i++) rxData[i]= ""; 
      inc_str = "";
    }
            
  }
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepper4.runSpeed();
  stepper5.runSpeed();

}
