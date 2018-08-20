float RPM;
int Temperature_Adc = 4;
//PID
unsigned long lastTime = 0.0;//init 0.0?
double Input, Output, Setpoint;
double errSum=0;
double lastErr;
double kp, ki, kd;

void setup()
{ 
//PWM
pinMode(9, OUTPUT);
pinMode(10,OUTPUT);
TCCR1A = _BV(COM1A1)|_BV(COM1B1)|_BV(WGM11);
TCCR1B = _BV(WGM13)|_BV(WGM12)|_BV(CS10);
ICR1 = 0x031f;
//TCCR1A=0;//reset the register
//TCCR1B=0;//reset tthe register
//TCCR1A=0b10100011;// fast pwm mode
//TCCR1B=0b00000001;// prescaler CS1 CS0
OCR1A=300;//duty cycle for pin 9
OCR1B=400;//duty cycle for pin 5
//ICR1 = 0x00ff;

//Temperature sensor
pinMode(Temperature_Adc,INPUT);

//set PID param
SetTunings(50, 0.0, 100);
Setpoint = 33.0; //********Temp sensor 17.0C ********
Serial.begin(9600);
}

void Compute()
{
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);
  double error = - Setpoint + Input;
  errSum += -(error*timeChange);
  double dErr = -(error - lastErr) / timeChange;
  Output = kp * error + ki * errSum + kd * dErr;

 lastErr = error;
 lastTime = now;
}

void SetTunings(double Kp,double Ki, double Kd)
{
  kp = Kp, ki = Ki; kd = Kd;
}

void loop()
{
  float sumT=0;
  float T=0;
  for (int i=0; i<500; i++)
  {
    T = analogRead(Temperature_Adc) * (5.0/1024.0);
    sumT+=T;
    delay(1);
  }
//String str = "    Temperature: " + String(sumT/500.0/0.01);
//Serial.println(str);
Input = sumT/500.0/0.01;
Compute();

if(Output>799)
{Output = 799;}
if(Output<0)
{Output = 0;}
if(Output>0&&Output<150)
{Output = 150;}

OCR1A = int(Output);
OCR1B = int(Output);
String strr = "Output: "+String(int(Output))+" Input: " +String(Input) + " Setpoint: "+ String(Setpoint) + " errSum: "+ String(errSum);
Serial.println(strr);


delay(50);
}



