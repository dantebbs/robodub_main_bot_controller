//#include <ros.h>
//#include <ros/time.h>
//#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/UInt8MultiArray.h>
//#include <std_msgs/String.h>
#include <stdint.h>
#include <Adafruit_NeoPixel.h>


#define ROBOT_ID "4"

// These pins need to be on the same port, with pairs going out to the motor controller A-B, A-B, etc.
#define ENCODER_PORT PINC

#define MOTOR_STOP 128

#define MASK B00000011
#define MASKBIT B00000001

#define PWM_PINR0 4
#define PWM_PINR1 5
#define PWM_PINL0 2
#define PWM_PINL1 3
#define PWM_PINRR0 6
#define PWM_PINRR1 7
#define PWM_PINRL0 8
#define PWM_PINRL1 9

#define LED_NUM 5
#define WS28_PIN 12
#define LED_MAX 25


#define MOTOR_ENABLE 11

//ros::NodeHandle_<ArduinoHardware, 5, 5, 1024, 1024>  n;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_NUM, WS28_PIN, NEO_GRB + NEO_KHZ800);

long lastMotorMessage = 0;
signed short ticks[4];
unsigned char last_encoders = 0;

#define SER_RX_BUF_LEN 100
#define SER_TX_BUF_LEN 100

void callback_leds(const uint8_t* Led_data__u8p) {
  for(int x = 0; x < LED_NUM ; x++ ) {
    strip.setPixelColor(x, Led_data__u8p[x * 3], Led_data__u8p[x * 3 + 1], Led_data__u8p[x * 3 + 2]);
  }
  
  strip.show();
}

void callback_w_vel( const std_msgs::UInt8MultiArray& w_vel) {
  if ( w_vel.data[1] >= 128 ) {
        analogWrite(PWM_PINR0, 0);
        analogWrite(PWM_PINR1, (abs(w_vel.data[1] - 128) * 2));
      } else {
        analogWrite(PWM_PINR1, 0);
        analogWrite(PWM_PINR0, (abs(w_vel.data[1] - 127) * 2));
      }
      
  if ( w_vel.data[0] <= 128 ) {
        analogWrite(PWM_PINL1, 0);
        analogWrite(PWM_PINL0, (abs(w_vel.data[0] - 127) * 2));
      } else {
        analogWrite(PWM_PINL0, 0);
        analogWrite(PWM_PINL1, (abs(w_vel.data[0] - 128) * 2));
      }
      
  if ( w_vel.data[3] <= 128 ) {
        analogWrite(PWM_PINRL0, 0);
        analogWrite(PWM_PINRL1, (abs(w_vel.data[3] - 127) * 2));
      } else {
        analogWrite(PWM_PINRL1, 0);
        analogWrite(PWM_PINRL0, (abs(w_vel.data[3] - 128) * 2));
      }  
  
    if ( w_vel.data[2] <= 128 ) {
        analogWrite(PWM_PINRR0, 0);
        analogWrite(PWM_PINRR1, (abs(w_vel.data[2] - 127) * 2));
      } else {
        analogWrite(PWM_PINRR1, 0);
        analogWrite(PWM_PINRR0, (abs(w_vel.data[2] - 128) * 2));
      }  
      
      lastMotorMessage = millis();
  
}

std_msgs::String str_imu_msg;
ros::Publisher imu_output("imu", &str_imu_msg);



void setLedStrip(int color) {
 
  for(int x = 0; x < LED_NUM ; x++ ) {
    strip.setPixelColor(x, color);
  }
  
  strip.show();
}

void setLedStrip(unsigned char r, unsigned char b, unsigned char g) {
 
  for(int x = 0; x < LED_NUM ; x++ ) {
    strip.setPixelColor(x, r, g, b);
  }
  
  strip.show();
}

String inputString = "";
char imuCharArray[128];
unsigned char outlength = 0;

void serialEvent1() {
  

  while (Serial1.available() ) {
    
    char in = (char) Serial1.read();
    inputString += in;
    outlength++;
    
    if( in == '/n' ) {
      inputString.toCharArray(imuCharArray, outlength);
      imuCharArray[outlength + 1] = '\0';
      imu_output.publish( &str_imu_msg );
      inputString = "";
      outlength = 0;
    }
  }
}



void callback_imu_input( const std_msgs::String& imu_input) {
  Serial1.print(imu_input.data);
}


ros::Subscriber<std_msgs::UInt8MultiArray> sub_w_vel("w_vel", &callback_w_vel );
ros::Subscriber<std_msgs::UInt8MultiArray> sub_leds("leds", &callback_leds );
std_msgs::Int16MultiArray msg_encoders;
ros::Publisher pub_encoders("enc", &msg_encoders);
ros::Subscriber<std_msgs::String> sub_imu_input("imu_input", &callback_imu_input);



void setup() {
  pinMode(MOTOR_ENABLE,OUTPUT);
  digitalWrite(MOTOR_ENABLE,LOW);
    
  
  pinMode(PWM_PINR0, OUTPUT);
  pinMode(PWM_PINR1, OUTPUT);
  pinMode(PWM_PINL0, OUTPUT);
  pinMode(PWM_PINL1, OUTPUT);
  pinMode(PWM_PINRR0, OUTPUT);
  pinMode(PWM_PINRR1, OUTPUT);
  pinMode(PWM_PINRL0, OUTPUT);
  pinMode(PWM_PINRL1, OUTPUT);

  analogWrite(PWM_PINR0, 0);
  analogWrite(PWM_PINR1, 0);
  analogWrite(PWM_PINL0, 0);
  analogWrite(PWM_PINL1, 0);
  analogWrite(PWM_PINRR0, 0);
  analogWrite(PWM_PINRR1, 0);
  analogWrite(PWM_PINRL0, 0);
  analogWrite(PWM_PINRL1, 0);

  strip.begin();
  setLedStrip(0, 0, 255);

  //TODO uncomment for IMU
  // If the IMU is not plugged in and the below line is uncommented, the floating pin
  // will make the interupt fire non stop, filling your buffers and eventuly
  // disconnecting your ROS connection
  //Serial1.begin(57600);

  ticks[0] = 0;
  ticks[1] = 0;
  ticks[2] = 0;
  ticks[3] = 0;

  msg_encoders.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  msg_encoders.layout.dim[0].label = "ticks";
  msg_encoders.layout.dim[0].size = sizeof(signed short) * 4;
  msg_encoders.layout.dim[0].stride = sizeof(signed short) * 4;
  msg_encoders.layout.data_offset = 0;
  msg_encoders.data = (int16_t*)malloc(sizeof(signed short) * 4);
  msg_encoders.data_length =  4;


  n.getHardware()->setBaud(115200);
  
  n.initNode();
  n.advertise(pub_encoders);
  n.advertise(imu_output);
  n.subscribe(sub_w_vel);
  n.subscribe(sub_leds);
  n.subscribe(sub_imu_input);
  
  while(!n.connected()) {
    setLedStrip(0, 0, 255);
    delay(10);
    n.spinOnce();
  }
  
  n.loginfo("Startup complete");

  
  // sub LED
  // sub W_vel done
  // adv enc done
  // sub servo
  // adv hit
  // sub shoot
}
char connect_toggle = 'd';
unsigned char index = 0;
unsigned short counter = 0;
bool justReconnected = true;


void loop() {
  // put your main code here, to run repeatedly:
  
  unsigned char encoders = ENCODER_PORT;
  index = 0;

  for (unsigned char x = 0; x < 8; x += 2) {
    
    if ( (encoders & (MASKBIT << x)) == (last_encoders & (MASKBIT << x))) {
      // Do nothing
    } else {

      if ( (((encoders & (MASKBIT << (x+1))) >> 1) ^ ((encoders & (MASKBIT << x)))) == 0) {
        ticks[index]--;
      } else {
        ticks[index]++;
      }
      
      // Update last encoder value
      last_encoders = (last_encoders & ~(MASK << x)) | (encoders & (MASK << x));
    }
    index++;
  }

  if ( counter > 10000 ) {
    
    msg_encoders.data[0] = ticks[0];
    msg_encoders.data[1] = ticks[1];
    msg_encoders.data[2] = ticks[2];
    msg_encoders.data[3] = ticks[3];
     

    pub_encoders.publish(&msg_encoders);
    //((unsigned int*)ticks)[0] = 0;
    
    ticks[0] = 0;
    ticks[1] = 0;
    ticks[2] = 0;
    ticks[3] = 0;
    
    
    counter = 0;
    
     if(!n.connected()) {
        setLedStrip(255, 0 ,0);
        digitalWrite(MOTOR_ENABLE,LOW);
        analogWrite(PWM_PINR0, 0);
        analogWrite(PWM_PINR1, 0);
        analogWrite(PWM_PINL0, 0);
        analogWrite(PWM_PINL1, 0);
        analogWrite(PWM_PINRR0, 0);
        analogWrite(PWM_PINRR1, 0);
        analogWrite(PWM_PINRL0, 0);
        analogWrite(PWM_PINRL1, 0);
        
    } else  {
      
      long time = millis();
      
      if( time - lastMotorMessage > 1000 ) {
        analogWrite(PWM_PINR0, 0);
        digitalWrite(MOTOR_ENABLE,HIGH);
        analogWrite(PWM_PINR1, 0);
        analogWrite(PWM_PINL0, 0);
        analogWrite(PWM_PINL1, 0);
        analogWrite(PWM_PINRR0, 0);
        analogWrite(PWM_PINRR1, 0);
        analogWrite(PWM_PINRL0, 0);
        analogWrite(PWM_PINRL1, 0);
        //Yellow indicates stall (no motor message in 1000ms)
        //TODO yellow is terrible
        setLedStrip(255, 0, 255);
      } else {
        setLedStrip(0, 255, 0);
      }
      
    } 
    

  } else {
    counter++;
  }  


  
  n.spinOnce();
}
