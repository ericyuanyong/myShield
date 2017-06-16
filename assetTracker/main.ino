

// This #include statement was automatically added by the Particle IDE.
#include "LIS3DH.h"

// This #include statement was automatically added by the Particle IDE.
#include "gpsLib.h"


#define START   D0
#define NONE    D1
#define PASS    D2
#define FAIL    D3



#define GPS_SWITCH  D6
#define START_KEY   D5
#define LED_PASS    D7
#define LED_FAIL    D4
#define ANTENNA_SIGNAL  D3      //double check the antenna control signal after send the command

#define TOTAL_SATELLATE 6       //make sure all the asset tracker can see more than 7 satellate during the test
#define STOP_TEST   120//240         //if the GPS cannot pass the test in 10minutes,then, it failed the test.
#define RETRY_TIMES 100           //retry time to get the satellite

#define DEBUG_MODE  1
#define NORMAL_MODE 2

#define ANT_EXTERNAL  1
#define ANT_INTERNAL   2



AmpleGPS GPS_M8Q;

uint8_t loadBuffer[LENGTH_CONFIG_ANT+8];

//uint8_t int_buffer[]={0xb5,0x62,0x06,0x13,0x04,0x00,0x00,0x00,0xf0,0x7d,0x8A,0x2A}; //the buffer that
//uint8_t ext_buffer[]={0xb5,0x62,0x06,0x13,0x04,0x00,0x01,0x00,0xf0,0x7d,0x8b,0x2e};

uint8_t int_buffer[]={0x00,0x00,0xf0,0x7d}; //data which will disable the active antenna
uint8_t ext_buffer[]={0x01,0x00,0xf0,0x7d}; //data which will enable the active antenna
uint8_t extall_buffer[]={0x1F,0X00,0XF0,0X7D};  //data which will enable all the antenna config for active antenna.

SYSTEM_MODE(MANUAL);



LIS3DHSPI accel(SPI, A2, WKP);


//return 1 if find the device id of the chip
bool testLIS3DH(void)
{
	LIS3DHConfig config;
	config.setAccelMode(LIS3DH::RATE_100_HZ);
	return accel.setup(config);
}


//return 1 if the start key had been pressed
uint8_t keyPress(void)
{
    if(digitalRead(START_KEY)==0)
    {
        delay(20);
        if(digitalRead(START_KEY)==0)
        {
            while(digitalRead(START_KEY)==0);
            Serial.println("Start button had pressed.Start the test now!!!");
            return 1;
        }
    }
    return 0;
}

void led_indicator(uint8_t status)
{
    if(status==PASS)
    {
        digitalWrite(LED_PASS,LOW);
        digitalWrite(LED_FAIL,HIGH);
    }
    else
    if(status==FAIL)
    {
        digitalWrite(LED_PASS,HIGH);
        digitalWrite(LED_FAIL,LOW);
    }
    else
    if(status==NONE)
    {
        digitalWrite(LED_PASS,HIGH);
        digitalWrite(LED_FAIL,HIGH);
    }
}



//return true if the command had successfully processed by M8Q.
bool antennaSelect(unsigned char antSource)
{
    uint8_t reTry = 0,reSend = 0;

    if(antSource==ANT_INTERNAL)
    {
        GPS_M8Q.loadSentence(CLASS_CFG,CFG_ANT,int_buffer,LENGTH_CONFIG_ANT,loadBuffer);
        GPS_M8Q.sendSentence(loadBuffer,LENGTH_CONFIG_ANT+8);
        delay(300);     //delay a little bit for M8Q to process the command
        while(1)
        {
            if(digitalRead(ANTENNA_SIGNAL)==0)  //the antenna signal should be 0 when using internal antenna
                return true;
            reTry++;
            if(reTry>=3)
            {
                reTry=0;
                reSend++;
                GPS_M8Q.loadSentence(CLASS_CFG,CFG_ANT,int_buffer,LENGTH_CONFIG_ANT,loadBuffer);
                GPS_M8Q.sendSentence(loadBuffer,LENGTH_CONFIG_ANT+8);
            }
            if(reSend >=3)
                return false;  //M8Q didnot process the antenna select command.
            delay(100);
            Serial.println("Retry internal antenna select");
        }
    }
    else
    if(antSource==ANT_EXTERNAL)         //the antenna signal should be 1 when using external antenna.
    {
        GPS_M8Q.loadSentence(CLASS_CFG,CFG_ANT,ext_buffer,LENGTH_CONFIG_ANT,loadBuffer);
        GPS_M8Q.sendSentence(loadBuffer,LENGTH_CONFIG_ANT+8);
        delay(300);
        while(1)
        {
            if(digitalRead(ANTENNA_SIGNAL)==1)
                return true;
            reTry++;
            if(reTry>=3)
            {
                reTry=0;
                reSend++;
                GPS_M8Q.loadSentence(CLASS_CFG,CFG_ANT,ext_buffer,LENGTH_CONFIG_ANT,loadBuffer);
                GPS_M8Q.sendSentence(loadBuffer,LENGTH_CONFIG_ANT+8);
            }
            if(reSend >=3)
                return false;  //M8Q didnot process the antenna select command.
            delay(200);
            Serial.println("Retry external antenna select");
        }
    }
    return false;
}

uint8_t workMode = 0;

void setup() {

    pinMode(GPS_SWITCH,OUTPUT);
    digitalWrite(GPS_SWITCH,HIGH);   //turn oFF the GPS module once it powered up.
    pinMode(START_KEY,INPUT_PULLUP);
    pinMode(ANTENNA_SIGNAL,INPUT_PULLUP);       //input mode to monitor the antenna signal port

    if(digitalRead(START_KEY)==0)       //press the key before it power up,and then it will enter debug mode
    {
        workMode = DEBUG_MODE;
        digitalWrite(GPS_SWITCH,LOW);
    }

    else
        workMode = NORMAL_MODE;

    pinMode(LED_PASS,OUTPUT);
    pinMode(LED_FAIL,OUTPUT);
    digitalWrite(LED_PASS,LOW);     //make sure all the led can work
    digitalWrite(LED_FAIL,LOW);
    delay(2000);
    led_indicator(NONE);

    Serial1.begin(9600);    //this is used for receiving the gps signal from asset tracker
    Serial.begin(9600);     //this is used for print out the info.

    GPS_M8Q.begin();    //initialization for GPS lib.
    Serial.println("Start the testing of new asset tracker,Press the start button and ready to go!!!!!");
}

char c=0;
uint8_t flag=0;
unsigned long interval =0;
unsigned long getFixTime = 0;
uint8_t testStart = 0;
uint8_t testResult = 0;

uint8_t retry = 0;  //retry times for get the correct satallites number.
uint8_t reSend = 0; //resend the command to M8Q

unsigned long runningIndicate = 0;
uint8_t ledStatus=0;
uint8_t antennaSource = 0;  //determine which antenna it will use
void loop() {
    if(workMode==NORMAL_MODE)
    {
        if(testStart==0)
        {
            if(keyPress())
                testStart=1;
            if(testStart==1)        //start to test accelerometer once the start button had been pressed.
            {
                led_indicator(NONE);    //clear all the led indication after press the key
                testResult = 0;
                if(testLIS3DH())    //if find the Accelerometer device
                {
                    digitalWrite(GPS_SWITCH,LOW);       //turn on GPS module only after it pass the Accelerometer test
                    GPS_M8Q.begin();    //initialization for GPS lib. need to begin everytime the putton had pressed
                    Serial.println("Accelerometer test passed,Now test the GPS module");
                    Serial.println("Start the test by using internal antenna");
                    delay(1500);        //need to delay a little bit then can send the command to M8Q
                    if(antennaSelect(ANT_INTERNAL))
                    {
                        Serial.println("command of using internal antenna had been processe successfully");
                        testStart = 1;  //start to test the gps module
                        antennaSource = ANT_INTERNAL;
                        runningIndicate = 0;    //start the running indicate led
                    }
                    else
                    {
                        Serial.println("send command failed.");
                        testStart = 0;
                        testResult = FAIL;
                        led_indicator(testResult);
                    }
                }
                else
                {
                    Serial.println("Accellerometer test failed!!!");
                    testStart = 0;      //go back to the inital stage and waiting for the user to start the test again
                    testResult = FAIL;
                    led_indicator(testResult);
                }
                retry = 0;
            }
        }

        if(testStart==1)        //ready to test the gps module
        {
            if(Serial1.available())
            {
                c=Serial1.read();
                GPS_M8Q.readSentence(c);
            }
            if((millis()-runningIndicate)>=500)
            {
                runningIndicate = millis();
                ledStatus = (ledStatus==1)?0:1;         //blink the LED during the test,to indicate it's still running the test.
                digitalWrite(LED_PASS,ledStatus);
            }
            if((millis()-interval)>=2500)       //check the fix status every 2.5seconds.
            {
                interval = millis();
                getFixTime++;
              //  Serial.println(getFixTime);         //this is just for debug-----------------------------------------------/
                if(getFixTime>STOP_TEST)        //if it cannot get the fix signal in 5 minutes, then it failed the GPS test process
                {
                    getFixTime=0;
                    testStart = 0;
                    digitalWrite(GPS_SWITCH,HIGH);  //turn off the gps module once failed the test
                    testResult = FAIL;
                    led_indicator(testResult);
                    Serial.println("GPS test failed because it had spent 10 minuts trying to get the fix signal");
                }
                if(GPS_M8Q.fix==true)
                {
                    GPS_M8Q.fix=false;      //clear the fix to avoid the legacy from last fix.
                    Serial.println("Get fix");
                    if((GPS_M8Q.fixquality==1)&&(retry<=RETRY_TIMES))
                    {
                        Serial.printf("Fix Quality is :%d,",GPS_M8Q.fixquality);
                        Serial.printf("SatellitesCount is %d\r\n",GPS_M8Q.satellitesCount);
                        if(GPS_M8Q.satellitesCount>=TOTAL_SATELLATE)    //find more than 6satellite in less then 10 minutes
                        {
                            if(antennaSource==ANT_INTERNAL)   //if more than 6 satellite was found by passive antenna,the test isn't finised yet, need to use active do the same test again.
                            {
                                antennaSource = ANT_EXTERNAL;
                                if(antennaSelect(ANT_EXTERNAL))
                                {
                                    retry=0;
                                    //getFixTime=0; make sure the total time must be in 10 minutes(include have antenna and didnot have antenna.)
                                    Serial.println("command of using active antenna had been processed successfully");
                                }
                                else
                                {
                                    Serial.println("Faild to select active antenna");
                                    retry=0;
                                    testStart=0;
                                    getFixTime=0;
                                    testResult = FAIL;                  //turn off the gps module once failed the test
                                    led_indicator(testResult);
                                    Serial.println("Failed at antenna select process");
                                }
                            }
                            else
                            if(antennaSource==ANT_EXTERNAL)     //the test will finish only after it passed the active antenna test.
                            {
                                ///////////////////////////////////////////////////////
                                //Still need to check whether it's ok to use it or not
                                if(GPS_M8Q.satellitesCount>=TOTAL_SATELLATE)
                                {
                                    retry = 0;
                                    testStart = 0;
                                    getFixTime=0;
                                    digitalWrite(GPS_SWITCH,HIGH);
                                    testResult = PASS;                  //turn off the gps module once failed the test
                                    led_indicator(testResult);
                                    Serial.println("GPS test passed!!!");
                                }
                                ///////////////////////////////////////////////////
                                /*
                                retry = 0;
                                testStart = 0;
                                getFixTime=0;
                                digitalWrite(GPS_SWITCH,HIGH);
                                testResult = PASS;                  //turn off the gps module once failed the test
                                led_indicator(testResult);
                                Serial.println("GPS test passed!!!");
                                */
                            }
                        }
                        else
                        {
                            retry++;
                            if(retry>RETRY_TIMES)        //which means it really cannot get more than 6 satellites
                            {
                                retry = 0;
                                testStart = 0;
                                getFixTime=0;
                                digitalWrite(GPS_SWITCH,HIGH);  //turn off the gps module once failed the test
                                testResult = FAIL;
                                led_indicator(testResult);
                                Serial.println("GPS test failed becasue it cannot see more than 6 satellites \r\n");
                            }
                            else
                                Serial.printf("Retry %d times to get the satellites count\r\n",retry);
                        }

                    }
                }
            }
        }
    }
    else
    if(workMode==DEBUG_MODE)        //this is the debug mode which will only be used for U-center
    {
        if(Serial1.available())
        {
            c=Serial1.read();
            Serial.print(c);
        }
    }
}
