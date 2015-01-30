int LED_Pin1 = 11; 
int LED_Pin2 = 10; 
int LED_Pin3 = 6; 
int LED_Pin4 = 5; 
int howBright1;  
int howBright2;
int howBright3;
int howBright4;

void setup()
{ 
  pinMode(LED_Pin1, OUTPUT); 
  pinMode(LED_Pin2, OUTPUT); 
  pinMode(LED_Pin3, OUTPUT); 
  pinMode(LED_Pin4, OUTPUT); 
}
void loop() 
{ 
  howBright1 = random(0,255);     
  analogWrite(LED_Pin1, howBright1); 
         
  howBright2 = random(0,255);     
  analogWrite(LED_Pin2, howBright2); 
  
  howBright3 = random(0,255);     
  analogWrite(LED_Pin3, howBright3); 
  
  howBright4 = random(0,255);     
  analogWrite(LED_Pin4, howBright4); 
  delay(random(50,150)); 
}
