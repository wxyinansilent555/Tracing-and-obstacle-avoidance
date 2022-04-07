#define TR_left 19 //发送脉冲引脚
#define EC_left 35 //接收脉冲引脚
#define TR_middle 21
#define EC_middle 34
#define TR_right  2
#define EC_right  15
#define left_motor_1     33                             //IN1
#define left_motor_2     25                             //IN2
#define right_motor_1    26                             //IN3
#define right_motor_2    27                             //IN4
#define ENA              32                             //ENA
#define ENB              14                             //ENB
#define leftB_track_PIN   23
#define leftA_track_PIN   5                            //定义寻迹的左A引脚
#define middA_track_PIN   13
#define righA_track_PIN   22                             //定义寻迹的右A引脚
#define righB_track_PIN   18
int sensor [5] ;
int error;
int pwm1;
int pwm2;


float TM_us_left;//定义脉冲宽度储存变量和最终距离的变量
float TM_us_middle;
float TM_us_right;
float CJ[4];

void setup()
{
    pinMode(TR_left, OUTPUT);//设置TR为输出状态
    pinMode(EC_left, INPUT);// 设置EC为输入状态
    
    pinMode(TR_middle, OUTPUT);
    pinMode(EC_middle, INPUT);
    
    pinMode(TR_right, OUTPUT);
    pinMode(EC_right, INPUT);
    ledcSetup(0, 5000, 10);               //通道0， 5KHz，10位解析度
    ledcSetup(1, 5000, 10);               //通道1， 5KHz，10位解析度
    pinMode(left_motor_1,OUTPUT);
    pinMode(right_motor_1,OUTPUT);
    pinMode(left_motor_2,OUTPUT);
    pinMode(right_motor_2,OUTPUT);
    pinMode(ENA,OUTPUT);
    pinMode(ENB,OUTPUT);
    digitalWrite(left_motor_1,LOW);
    digitalWrite(left_motor_2,HIGH);
    digitalWrite(right_motor_1,LOW);
    digitalWrite(right_motor_2,HIGH);  
    ledcAttachPin(ENA, 0);
    ledcAttachPin(ENB, 1);
    
    Serial.begin(9600);
}
void read_sensor_values1()
{
    //读取寻迹传感器的数值左右各一个
    sensor[0] = digitalRead(leftB_track_PIN);            //八字形排布，左A
    sensor[1] = digitalRead(leftA_track_PIN);            //左B
    sensor[2] = digitalRead(middA_track_PIN); 
    sensor[3] = digitalRead(righA_track_PIN);
    sensor[4] = digitalRead(righB_track_PIN);
    Serial.print(sensor[0]);
    Serial.print(sensor[1]);
    Serial.print(sensor[2]);
    Serial.print(sensor[3]);
    Serial.print(sensor[4]);
    
    
    //根据传感器的数值判断是小左转、大左转、直行、小右转、大右转
    //然后返回pid控制算法的error值
    
     if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))
    {
        error = 2;                                  //          0 0 0 0 1  大右转
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
    {
        error = 1;                                  //          0 0 0 1 0  小右转
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    {
        error = 0;                                  //          0 0 1 0 0  直行
    }
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    {
        error = -1;                                 //         0 1 0 0 0  小左转
    }
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      error = -2;                                   //         1 0 0 0 0  大左转
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    {
        if (error == -2)   error =-3;                                                  // 0 0 0 0 0  原本是大左转 检测全是零 检测不到黑线 还需要继续转 加大error值                 
        else if(error==2) error = 3;
        
            
        
    }
}
void tracing()
{
    read_sensor_values1();        //获取2个寻迹模块的数值情况
    //写入电机的pwm波频率
    if(error==2){
      pwm1=1860;
      pwm2=1700;
    }
    else if(error==1){
      pwm1=1780;
      pwm2=1700;
    }
    else if(error==0){
      pwm1=1700;
      pwm2=1700;
    }
    else if(error==-1){
      pwm1=1700;
      pwm2=1780;
    }
    else if(error==-2){
      pwm1=1700;
      pwm2=1860;
    }
    else if(error==-3){
      pwm1=1700;
      pwm2=2050;
    }
    else if(error==3){
      pwm1=2050;
      pwm2=1700;
    }
    ledcWrite(0,pwm1);
    ledcWrite(1,pwm2);
}
void back_up()
{
    digitalWrite(left_motor_1,HIGH);
    digitalWrite(left_motor_2,LOW);
    digitalWrite(right_motor_1,HIGH);
    digitalWrite(right_motor_2,LOW);
    ledcWrite(0,0);
    ledcWrite(1,0);
    delay(1000);
    ledcWrite(0,1700);
    ledcWrite(1,1700);
    delay(2000);
    digitalWrite(left_motor_1,LOW);
    digitalWrite(left_motor_2,HIGH);
    digitalWrite(right_motor_1,LOW);
    digitalWrite(right_motor_2,HIGH);     
      
}
void read_sensor_values2()
{   
  
    digitalWrite(TR_left, HIGH);
    delayMicroseconds(10);//延时10微秒
    digitalWrite(TR_left, LOW);

    TM_us_left = pulseIn(EC_left,HIGH);//接受测量脉冲的宽度
    if((TM_us_left<60000) && (TM_us_left> 1)){
        //有效的脉冲宽度应该介于0-60000之间
        CJ[0] = TM_us_left * 0.034/2;//接收的脉冲宽度时间*0.034/2就是实际被测物体的距离
        Serial.print("左：");
        Serial.println(CJ[0]);
    }
    delay(10);
    
    
    digitalWrite(TR_middle, HIGH);
    delayMicroseconds(10);//延时10微秒
    digitalWrite(TR_middle, LOW);
    TM_us_middle = pulseIn(EC_middle,HIGH);//接受测量脉冲的宽度
    if((TM_us_middle<60000) && (TM_us_middle> 1)){
        //有效的脉冲宽度应该介于0-60000之间
        CJ[1] = TM_us_middle * 0.034/2;//接收的脉冲宽度时间*0.034/2就是实际被测物体的距离
        Serial.print("中：");
        Serial.println(CJ[1]);
    }
    delay(10);
    
    
    digitalWrite(TR_right, HIGH);
    delayMicroseconds(10);//延时10微秒
    digitalWrite(TR_right, LOW);
    TM_us_right = pulseIn(EC_right,HIGH);//接受测量脉冲的宽度
    if((TM_us_right<60000) && (TM_us_right> 1)){
        //有效的脉冲宽度应该介于0-60000之间
        CJ[2] = TM_us_right * 0.034/2;//接收的脉冲宽度时间*0.034/2就是实际被测物体的距离
        Serial.print("右：");
        Serial.println(CJ[2]);
    }
    delay(10);
    
}
void avoidance()
{
    read_sensor_values2();
    if(CJ[1]<=30&&CJ[1]>=10){
      if(CJ[0]>=CJ[2]){
        pwm1=1700;
        pwm2=2050;
        delay(1000);//左转  
      }
      else{
        pwm1=2050;
        pwm2=1700;
        delay(1000);//右转  
      }  
    }
    else if(CJ[1]<10){
        back_up();
        
    }
    else{
      if(CJ[0]-CJ[2]>=10){
        pwm1=1700;
        pwm2=1780;//小左偏转  
      }
      else if(CJ[2]-CJ[0]>=10){
        pwm1=1780;
        pwm2=1700;//小右偏转  
      }
      else{
        pwm1=1700;
        pwm2=1700;  
      }   
    }
    ledcWrite(0,pwm1);
    ledcWrite(1,pwm2);  
}
void loop()
{
    avoidance();
    if(CJ[1]>40){
      tracing();  
    }  
}
