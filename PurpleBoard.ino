int DC1_RPWM = 2, DC1_LPWM = 3, 
    DC2_RPWM = 4, DC2_LPWM = 5, 
    DC3_RPWM = 6, DC3_LPWM = 7,
    DC4_RPWM = 8, DC4_LPWM = 9, 
    DC5_RPWM = 10, DC5_LPWM = 11;

const int receive_length = 20;
void setup() {
  Serial.begin(115200);
  
  pinMode(DC1_RPWM, OUTPUT); pinMode(DC1_LPWM, OUTPUT);
  pinMode(DC2_RPWM, OUTPUT); pinMode(DC2_LPWM, OUTPUT);
  pinMode(DC3_RPWM, OUTPUT); pinMode(DC3_LPWM, OUTPUT);
  pinMode(DC4_RPWM, OUTPUT); pinMode(DC4_LPWM, OUTPUT);
  pinMode(DC5_RPWM, OUTPUT); pinMode(DC5_LPWM, OUTPUT);
}

void loop() {
  static bool receive_flag = false;
  static int receive_counter, int_dir, int_speed,
             DC1, DC2, DC3, DC4, DC5;
  static char rxData[receive_length], inc_char; 
  static String str_dir, str_speed;
  float PWM;
  if(Serial.available() > 0){
    inc_char = Serial.read();
    delay(1);
    
    if(inc_char == 'S'){
      receive_flag=true;
      receive_counter = 0;
    }
    
    if(receive_flag && inc_char!='S' && inc_char!='F'){
      rxData[receive_counter] = inc_char;
      receive_counter++;
    }
    
    if(inc_char == 'F'){
      str_dir = rxData[4];
      str_speed = String(rxData[5]) + String(rxData[6]) + String(rxData[7]);
      
      int_dir = str_dir.toInt();
      int_speed = str_speed.toInt();
      PWM = int_speed/2;

      DC1 = rxData[13];
      DC2 = rxData[14];
      DC3 = rxData[15];
      DC4 = rxData[16];
      DC5 = rxData[17];
      
      receive_flag=false;
      for(int i=0; i<receive_length-2; i++) rxData[i]= ""; 
    }
  }

  if(DC1 == '1'){
    if(int_dir == 1){analogWrite(DC1_RPWM, PWM); analogWrite(DC1_LPWM, 0);}
    else if(int_dir == 0){analogWrite(DC1_LPWM, PWM); analogWrite(DC1_RPWM, 0);}
  }
  else if(DC1 == '0'){analogWrite(DC1_RPWM, 0); analogWrite(DC1_LPWM, 0);}
      
  if(DC2 == '1'){
    if(int_dir == 1){analogWrite(DC2_RPWM, PWM); analogWrite(DC2_LPWM, 0);}
    else if(int_dir == 0){analogWrite(DC2_LPWM, PWM); analogWrite(DC2_RPWM, 0);}
  }
  else if(DC2 == '0'){analogWrite(DC2_RPWM, 0); analogWrite(DC2_LPWM, 0);}

  if(DC3 == '1'){
    if(int_dir == 1){analogWrite(DC3_RPWM, int_speed); analogWrite(DC3_LPWM, 0);}
    else if(int_dir == 0){analogWrite(DC3_LPWM, int_speed); analogWrite(DC3_RPWM, 0);}
  }
  else if(DC3 == '0'){analogWrite(DC3_RPWM, 0); analogWrite(DC3_LPWM, 0);}
  
  if(DC4 == '1'){
    if(int_dir == 1){analogWrite(DC4_RPWM, int_speed); analogWrite(DC4_LPWM, 0);}
    else if(int_dir == 0){analogWrite(DC4_LPWM, int_speed); analogWrite(DC4_RPWM, 0);}
  }
  else if(DC4 == '0'){analogWrite(DC4_RPWM, 0); analogWrite(DC4_LPWM, 0);}

  if(DC5 == '1'){
    if(int_dir == 1){analogWrite(DC5_RPWM, int_speed); analogWrite(DC5_LPWM, 0);}
    else if(int_dir == 0){analogWrite(DC5_LPWM, int_speed); analogWrite(DC5_RPWM, 0);}
  }
  else if(DC5 == '0'){analogWrite(DC5_RPWM, 0); analogWrite(DC5_LPWM, 0);}

  
  delay(1);
}
