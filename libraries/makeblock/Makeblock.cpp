#include "Makeblock.h"

#define MeBaseBoard


#if defined(__AVR_ATmega32U4__) //MeBaseBoard use ATmega32U4 as MCU

MePort_Sig mePort[11] = {{NC, NC}, {11, A8}, {13, A11}, {A10, A9}, {1, 0},
    {MISO, SCK}, {A0, A1}, {A2, A3}, {A4, A5}, {6, 7}, {5, 4}
};
#else // else ATmega328
MePort_Sig mePort[11] = {{NC, NC}, {11, 10}, {3, 9}, {12, 13}, {8, 2},
    {NC, NC}, {A2, A3}, {A6, A1}, {A7, A0}, {6, 7}, {5, 4}
};

#endif

union{
    byte b[4];
    float fVal;
    long lVal;
}u;

/*        Port       */
MePort::MePort(){
	s1 = mePort[0].s1;
    s2 = mePort[0].s2;
    _port = 0;
}
MePort::MePort(uint8_t port)
{
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
	//The PWM frequency is 976 Hz
#if defined(__AVR_ATmega32U4__) //MeBaseBoard use ATmega32U4 as MCU

TCCR1A =  _BV(WGM10);
TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

TCCR3A = _BV(WGM30);
TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);

TCCR4B = _BV(CS42) | _BV(CS41) | _BV(CS40);
TCCR4D = 0;

#else if defined(__AVR_ATmega328__) // else ATmega328

TCCR1A = _BV(WGM10);
TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

TCCR2A = _BV(WGM21) |_BV(WGM20);
TCCR2B = _BV(CS22);

#endif
}
uint8_t MePort::getPort(){
	return _port;
}
uint8_t MePort::getSlot(){
	return _slot;
}
bool MePort::dRead1()
{
    bool val;
    pinMode(s1, INPUT);
    val = digitalRead(s1);
    return val;
}

bool MePort::dRead2()
{
    bool val;
	pinMode(s2, INPUT);
    val = digitalRead(s2);
    return val;
}

void MePort::dWrite1(bool value)
{
    pinMode(s1, OUTPUT);
    digitalWrite(s1, value);
}

void MePort::dWrite2(bool value)
{
    pinMode(s2, OUTPUT);
    digitalWrite(s2, value);
}

int MePort::aRead1()
{
    int val;
    val = analogRead(s1);
    return val;
}

int MePort::aRead2()
{
    int val;
    val = analogRead(s2);
    return val;
}

void MePort::aWrite1(int value)
{   
    analogWrite(s1, value);  
}

void MePort::aWrite2(int value)
{
    analogWrite(s2, value); 
}
uint8_t MePort::pin1(){
	return s1;
}
uint8_t MePort::pin2(){
	return s2;
}
void MePort::reset(uint8_t port){
	s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
}
void MePort::reset(uint8_t port,uint8_t slot){
	s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
    _slot = slot;
}
/*             Wire               */
MeWire::MeWire(uint8_t address): MePort(){
	_slaveAddress = address + 1;
}
MeWire::MeWire(uint8_t port, uint8_t address): MePort(port)
{
    _slaveAddress = address + 1;
}
void MeWire::begin()
{
    delay(1000);
    Wire.begin();
    write(BEGIN_FLAG, 0x01);
}
bool MeWire::isRunning()
{
    return read(BEGIN_STATE);
}
void MeWire::setI2CBaseAddress(uint8_t baseAddress)
{
    byte w[2]={0};
    byte r[4]={0};
    w[0]=0x21;
    w[1]=baseAddress;
    request(w,r,2,4);
}

byte MeWire::read(byte dataAddress){
	byte *b={0};
	read(dataAddress,b,1);
	return b[0];
}

void MeWire::read(byte dataAddress,uint8_t *buf,int len)
{
	byte rxByte;
	Wire.beginTransmission(_slaveAddress); // transmit to device
	Wire.write(dataAddress); // sends one byte
	Wire.endTransmission(); // stop transmitting
	delayMicroseconds(1);
	Wire.requestFrom(_slaveAddress,len); // request 6 bytes from slave device
	int index =0;
	while(Wire.available()) // slave may send less than requested
	{
		rxByte = Wire.read(); // receive a byte as character
		buf[index] = rxByte;
		index++;
	}
}

void MeWire::write(byte dataAddress, byte data)
{
    Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(dataAddress); // sends one byte
    Wire.endTransmission(); // stop transmitting

    Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(data); // sends one byte
    Wire.endTransmission(); // stop transmitting
}
void MeWire::request(byte* writeData,byte*readData,int wlen,int rlen)
{
   
	uint8_t rxByte;
	uint8_t index =0;

	Wire.beginTransmission(_slaveAddress); // transmit to device

	Wire.write(writeData,wlen);

	Wire.endTransmission(); 
	delayMicroseconds(2);
	Wire.requestFrom(_slaveAddress,rlen); // request 6 bytes from slave device
	delayMicroseconds(2);
	while(Wire.available()) // slave may send less than requested
	{
		rxByte = Wire.read(); // receive a byte as character
        
		readData[index] = rxByte;
		index++; 
	}
}


/*             Serial                  */
MeSerial::MeSerial():MePort(),SoftwareSerial(NC,NC){
    _hard = true;
    _scratch = true;
    _polling = false;
}
MeSerial::MeSerial(uint8_t port):MePort(port),SoftwareSerial(mePort[port].s2,mePort[port].s1)
{
	_scratch = false;
    _hard = false;
    _polling = false;
    #if defined(__AVR_ATmega32U4__)
        _polling = getPort()>PORT_5;
        _hard = getPort()==PORT_4;
    #else
    	_hard = getPort()==PORT_5;    
    #endif
}
void MeSerial::setHardware(bool mode){
    _hard = mode;
}
void MeSerial::begin(long baudrate)
{
    _bitPeriod = 1000000/baudrate;
    if(_hard) {
		#if defined(__AVR_ATmega32U4__)
            _scratch?Serial.begin(baudrate):Serial1.begin(baudrate);
        #else
            Serial.begin(baudrate);
		#endif
    } else {
        SoftwareSerial::begin(baudrate);
    }
}

void MeSerial::end()
{
    if(_hard) {
		#if defined(__AVR_ATmega32U4__)
            Serial1.end();
        #else
            Serial.end();
		#endif
    } else {
        SoftwareSerial::end();
    }
}

size_t MeSerial::write(uint8_t byte)
{
    if(_isServoBusy == true)return -1;
    if(_hard){
    	#if defined(__AVR_ATmega32U4__)
            return (_scratch?Serial.write(byte):Serial1.write(byte));
        #else
            return Serial.write(byte);
		#endif
    }else return SoftwareSerial::write(byte);
}
int MeSerial::read()
{
    if(_isServoBusy == true)return -1;
    
    if(_polling){
        int temp = _byte;
        _byte = -1;
        return temp>-1?temp:poll();
    }
    if(_hard){
		#if defined(__AVR_ATmega32U4__)
        	return (_scratch?Serial.read():Serial1.read());
        #else
        	return Serial.read();
		#endif
    }else return SoftwareSerial::read();
}
int MeSerial::available()
{
    if(_polling){
        _byte = poll();
        return _byte>-1?1:0;
    }
    if(_hard){
    	#if defined(__AVR_ATmega32U4__)
        	return (_scratch?Serial.available():Serial1.available());
        #else
        	return Serial.available();
		#endif
    }else return SoftwareSerial::available();
}
bool MeSerial::listen()
{
    if(_hard)
        return true;
    else return SoftwareSerial::listen();
}
bool MeSerial::isListening()
{
    if(_hard)
        return true;
    else return SoftwareSerial::isListening();
}

int MeSerial::poll()
{
    int val = 0;
    int bitDelay = _bitPeriod - clockCyclesToMicroseconds(50);
    if (digitalRead(s2) == LOW) {
        for (int offset = 0; offset < 8; offset++) {
            delayMicroseconds(bitDelay);
            val |= digitalRead(s2) << offset;
        }
        delayMicroseconds(bitDelay);
        return val&0xff;
    }
    return -1;
}

/*             LineFinder              */
MeLineFollower::MeLineFollower(): MePort(0){
	
}
MeLineFollower::MeLineFollower(uint8_t port): MePort(port)
{

}
uint8_t MeLineFollower::readSensors()
{
    uint8_t state = S1_IN_S2_IN;
    bool s1State = MePort::dRead1();
    bool s2State = MePort::dRead2();
    state = ((1 & s1State) << 1) | s2State;
    return state;
}
bool MeLineFollower::readSensor1()
{
    return MePort::dRead1();
}
bool MeLineFollower::readSensor2()
{
    return MePort::dRead2();
}
/*             LimitSwitch              */
MeLimitSwitch::MeLimitSwitch(): MePort(0)
{
}
MeLimitSwitch::MeLimitSwitch(uint8_t port): MePort(port)
{
    _device = SLOT1;
    pinMode(s2,INPUT_PULLUP);
}
MeLimitSwitch::MeLimitSwitch(uint8_t port,uint8_t slot): MePort(port)
{
    reset(port,slot);
    if(getSlot()==SLOT1){
        pinMode(s1,INPUT_PULLUP);
    }else{
        pinMode(s2,INPUT_PULLUP);
    }
}
bool MeLimitSwitch::touched()                                                                                                                                                          
{
    // if(getSlot()==SLOT2){
        // pinMode(s1,INPUT_PULLUP);
    // }else{
        // pinMode(s2,INPUT_PULLUP);
    // }
    return !(getSlot()==SLOT1?digitalRead(s1):digitalRead(s2));
}

/*             MotorDriver              */
MeDCMotor::MeDCMotor(): MePort(0)
{

}
MeDCMotor::MeDCMotor(uint8_t port): MePort(port)
{

}
void MeDCMotor::run(int speed)
{
    speed = speed > 255 ? 255 : speed;
    speed = speed < -255 ? -255 : speed;

    if(speed >= 0) {
        MePort::dWrite2(HIGH);
        MePort::aWrite1(speed);
    } else {
        MePort::dWrite2(LOW);
        MePort::aWrite1(-speed);
    }
}
void MeDCMotor::stop()
{
    MeDCMotor::run(0);
}
/*           UltrasonicSenser                 */
MeUltrasonicSensor::MeUltrasonicSensor(): MePort(0)
{
}

MeUltrasonicSensor::MeUltrasonicSensor(uint8_t port): MePort(port)
{
}

long MeUltrasonicSensor::distanceCm()
{
    long distance = measure();
    return ((distance / 29) >> 1);
}

long MeUltrasonicSensor::distanceInch()
{
    long distance = measure();
    return ((distance / 74) >> 1);
}

long MeUltrasonicSensor::measure()
{
    long duration;
    MePort::dWrite2(LOW);
    delayMicroseconds(2);
    MePort::dWrite2(HIGH);	
    delayMicroseconds(10);
    MePort::dWrite2(LOW);
    pinMode(s2, INPUT);
    duration = pulseIn(s2, HIGH); 
    return duration;
}

/*          shutter       */
MeShutter::MeShutter(): MePort(0){
	
}
MeShutter::MeShutter(uint8_t port): MePort(port)
{
    MePort::dWrite1(LOW);
    MePort::dWrite2(LOW);
}
void MeShutter::shotOn()
{
    MePort::dWrite1(HIGH);
}
void MeShutter::shotOff()
{

    MePort::dWrite1(LOW);
}
void MeShutter::focusOn()
{
    MePort::dWrite2(HIGH);
}
void MeShutter::focusOff()
{
    MePort::dWrite2(LOW);
}

/*           Bluetooth                 */
MeBluetooth::MeBluetooth(): MeSerial(0)
{
}
MeBluetooth::MeBluetooth(uint8_t port): MeSerial(port)
{
}

/*           Infrared Receiver                 */
MeInfraredReceiver::MeInfraredReceiver(): MeSerial(0)
{
	
}
MeInfraredReceiver::MeInfraredReceiver(uint8_t port): MeSerial(port)
{
}
void MeInfraredReceiver::begin()
{
    MeSerial::begin(9600);
	pinMode(s1,INPUT);
}

int MeInfraredReceiver::read()
{
	int val;
	uint16_t i;
	do{
		i++;
		if(++i>2000)break;
		val = MeSerial::read();							//Read serial infrared data
		val &= 0xff;
	}while(val == 0x0 || val == 0xFF);	//0x0 and 0xff are the user code of BC7210A IC	
	delayMicroseconds(10);
	return  val;

}
bool MeInfraredReceiver::buttonState()        // Check button press

{
	bool val;
	if(_hard)
	MeSerial::end();
	val = MePort::dRead1();
	if(_hard)
	begin();
	
    return (!val);
}

MeRGBLed::MeRGBLed():MePort(0) {
	setNumber(4);
}
MeRGBLed::MeRGBLed(uint8_t port):MePort(port) {
	pinMask = digitalPinToBitMask(s2);
	ws2812_port = portOutputRegister(digitalPinToPort(s2));
	// ws2812_port_reg = portModeRegister(digitalPinToPort(s2));
	// *ws2812_port_reg |= pinMask; //set pinMode OUTPUT
	pinMode(s2,OUTPUT);
	setNumber(4);
}
MeRGBLed::MeRGBLed(uint8_t port,uint8_t slot):MePort(port){
	if(slot==SLOT2){
		pinMask = digitalPinToBitMask(s2);
		ws2812_port = portOutputRegister(digitalPinToPort(s2));
		pinMode(s2,OUTPUT);
		// ws2812_port_reg = portModeRegister(digitalPinToPort(s2));
	}else{
		pinMask = digitalPinToBitMask(s1);
		ws2812_port = portOutputRegister(digitalPinToPort(s1));
		pinMode(s1,OUTPUT);
		// ws2812_port_reg = portModeRegister(digitalPinToPort(s1));
	}
	// *ws2812_port_reg |= pinMask; // set pinMode OUTPUT
	setNumber(4);
}
void MeRGBLed::reset(uint8_t port){
	s2 = mePort[port].s2;
	s1 = mePort[port].s1;
	pinMask = digitalPinToBitMask(s2);
	ws2812_port = portOutputRegister(digitalPinToPort(s2));
	// ws2812_port_reg = portModeRegister(digitalPinToPort(s2));
}
void MeRGBLed::setNumber(uint8_t num_leds){
	count_led = num_leds;
	pixels = (uint8_t*)malloc(count_led*3);
}
cRGB MeRGBLed::getColorAt(uint8_t index) {
	
	cRGB px_value;
	
	if(index < count_led) {
		
		uint8_t tmp;
		tmp = index * 3;
		
		px_value.g = pixels[tmp];
		px_value.r = pixels[tmp+1];
		px_value.b = pixels[tmp+2];
	}
	
	return px_value;
}

uint8_t MeRGBLed::getNumber(){
	return count_led;
}
bool MeRGBLed::setColorAt(uint8_t index, uint8_t red,uint8_t green,uint8_t blue) {
	if(index < count_led) {
		uint8_t tmp = index * 3;
		pixels[tmp] = green;
		pixels[tmp+1] = red;
		pixels[tmp+2] = blue;
		
		return true;
	} 
	return false;
}
bool MeRGBLed::setColorAt(uint8_t index, long value) {
	if(index < count_led) {
		uint8_t tmp = index * 3;
		uint8_t red = (value&0xff0000)>>16;
		uint8_t green = (value&0xff00)>>8;
		uint8_t blue = value&0xff;
		pixels[tmp] = green;
		pixels[tmp+1] = red;
		pixels[tmp+2] = blue;
		return true;
	} 
	return false;
}
void MeRGBLed::clear(){
	for(int i=0;i<count_led;i++){
		setColorAt(i,0,0,0);
	}
	show();
}
/*
  This routine writes an array of bytes with RGB values to the Dataout pin
  using the fast 800kHz clockless WS2811/2812 protocol.
*/

// Timing in ns
#define w_zeropulse   350
#define w_onepulse    900
#define w_totalperiod 1250

// Fixed cycles used by the inner loop
#define w_fixedlow    3
#define w_fixedhigh   6
#define w_fixedtotal  10   

// Insert NOPs to match the timing, if possible
#define w_zerocycles    (((F_CPU/1000)*w_zeropulse          )/1000000)
#define w_onecycles     (((F_CPU/1000)*w_onepulse    +500000)/1000000)
#define w_totalcycles   (((F_CPU/1000)*w_totalperiod +500000)/1000000)

// w1 - nops between rising edge and falling edge - low
#define w1 (w_zerocycles-w_fixedlow)
// w2   nops between fe low and fe high
#define w2 (w_onecycles-w_fixedhigh-w1)
// w3   nops to complete loop
#define w3 (w_totalcycles-w_fixedtotal-w1-w2)

#if w1>0
  #define w1_nops w1
#else
  #define w1_nops  0
#endif

// The only critical timing parameter is the minimum pulse length of the "0"
// Warn or throw error if this timing can not be met with current F_CPU settings.
#define w_lowtime ((w1_nops+w_fixedlow)*1000000)/(F_CPU/1000)
#if w_lowtime>550
   #error "Light_ws2812: Sorry, the clock speed is too low. Did you set F_CPU correctly?"
#elif w_lowtime>450
   #warning "Light_ws2812: The timing is critical and may only work on WS2812B, not on WS2812(S)."
   #warning "Please consider a higher clockspeed, if possible"
#endif   

#if w2>0
#define w2_nops w2
#else
#define w2_nops  0
#endif

#if w3>0
#define w3_nops w3
#else
#define w3_nops  0
#endif

#define w_nop1  "nop      \n\t"
#define w_nop2  "rjmp .+0 \n\t"
#define w_nop4  w_nop2 w_nop2
#define w_nop8  w_nop4 w_nop4
#define w_nop16 w_nop8 w_nop8

void  MeRGBLed::rgbled_sendarray_mask(uint8_t *data,uint16_t datlen,uint8_t maskhi,uint8_t *port)
{
  	uint8_t curbyte,ctr,masklo;
	uint8_t oldSREG = SREG;
	cli();  //Disables all interrupts
	
  masklo = *port & ~maskhi;
  maskhi = *port | 	maskhi;
  
  while (datlen--) {
    curbyte=*data++;
    
    asm volatile(
    "       ldi   %0,8  \n\t"
    "loop%=:            \n\t"
    "       st    X,%3 \n\t"    //  '1' [02] '0' [02] - re
#if (w1_nops&1)
w_nop1
#endif
#if (w1_nops&2)
w_nop2
#endif
#if (w1_nops&4)
w_nop4
#endif
#if (w1_nops&8)
w_nop8
#endif
#if (w1_nops&16)
w_nop16
#endif
    "       sbrs  %1,7  \n\t"    //  '1' [04] '0' [03]
    "       st    X,%4 \n\t"     //  '1' [--] '0' [05] - fe-low
    "       lsl   %1    \n\t"    //  '1' [05] '0' [06]
#if (w2_nops&1)
  w_nop1
#endif
#if (w2_nops&2)
  w_nop2
#endif
#if (w2_nops&4)
  w_nop4
#endif
#if (w2_nops&8)
  w_nop8
#endif
#if (w2_nops&16)
  w_nop16 
#endif
    "       brcc skipone%= \n\t"    //  '1' [+1] '0' [+2] - 
    "       st   X,%4      \n\t"    //  '1' [+3] '0' [--] - fe-high
    "skipone%=:               "     //  '1' [+3] '0' [+2] - 

#if (w3_nops&1)
w_nop1
#endif
#if (w3_nops&2)
w_nop2
#endif
#if (w3_nops&4)
w_nop4
#endif
#if (w3_nops&8)
w_nop8
#endif
#if (w3_nops&16)
w_nop16
#endif

    "       dec   %0    \n\t"    //  '1' [+4] '0' [+3]
    "       brne  loop%=\n\t"    //  '1' [+5] '0' [+4]
    :	"=&d" (ctr)
//    :	"r" (curbyte), "I" (_SFR_IO_ADDR(ws2812_PORTREG)), "r" (maskhi), "r" (masklo)
    :	"r" (curbyte), "x" (port), "r" (maskhi), "r" (masklo)
    );
  }
  
  SREG = oldSREG;
}
void MeRGBLed::show() {
//	*ws2812_port_reg |= pinMask; // Enable DDR
	rgbled_sendarray_mask(pixels,3*count_led,pinMask,(uint8_t*) ws2812_port);	
}

MeRGBLed::~MeRGBLed() {
	
	
}

/*          EncoderMotor        */

MeEncoderMotor::MeEncoderMotor(uint8_t selector,uint8_t slot):MeWire(selector){
    _slot = slot;
}
void MeEncoderMotor::begin(){
	MeWire::begin();
	resetEncoder();
}
boolean MeEncoderMotor::setCounter(uint8_t counter){
    byte w[5]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x23;
    w[2]=0;
    w[3]=_slot;
    w[4]=counter;
    request(w,r,5,4);
    return r[3]==1;
}
boolean MeEncoderMotor::setRatio(float ratio){
    byte w[7]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x22;
    w[2]=_slot;
    u.fVal = ratio;
    w[3]=u.b[0];
    w[4]=u.b[1];
    w[5]=u.b[2];
    w[6]=u.b[3];
    request(w,r,7,4);
    return r[3]==1;
}
boolean MeEncoderMotor::setPID(float mp,float mi,float md,uint8_t mode){
    
    byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x24;
    w[2]=_slot;
    w[4]=mode;
    
    int i;
    
    w[3]=0;
    u.fVal = mp;
    for(i=0;i<4;i++){
        w[5+i]=u.b[i];
    }
    request(w,r,9,4);
    
    w[3]=1;
    u.fVal = mi;
    for(i=0;i<4;i++){
        w[5+i]=u.b[i];
    }
    request(w,r,9,4);
    
    w[3]=2;
    u.fVal = md;
    for(i=0;i<4;i++){
        w[5+i]=u.b[i];
    }
    request(w,r,9,4);
    return r[3]==1;
}
boolean MeEncoderMotor::moveTo(long degrees,float speed){
	byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x32;
    w[2]=_slot;
    int i;
    u.lVal = degrees;
    for(i=0;i<4;i++){
        w[3+i]=u.b[i];
    }
    u.fVal = speed;
    for(i=0;i<4;i++){
        w[7+i]=u.b[i];
    }
    request(w,r,11,4);
    return r[3]==1;
}
boolean MeEncoderMotor::move(long degrees,float speed){
    byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x31;
    w[2]=_slot;
    int i;
    u.lVal = degrees;
    for(i=0;i<4;i++){
        w[3+i]=u.b[i];
    }
    u.fVal = speed;
    for(i=0;i<4;i++){
        w[7+i]=u.b[i];
    }
    request(w,r,11,4);
    return r[3]==1;
}
boolean MeEncoderMotor::runTurns(float turns){
    return move(turns*360,10);
}
boolean MeEncoderMotor::runSpeed(float speed){
	return runSpeedAndTime(speed,0);
}
boolean MeEncoderMotor::runSpeedAndTime(float speed,long time){
    byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x33;
    w[2]=_slot;
    int i;
    u.fVal = speed;
    for(i=0;i<4;i++){
        w[3+i]=u.b[i];
    }
    u.lVal = time;
    for(i=0;i<4;i++){
        w[7+i]=u.b[i];
    }
    request(w,r,11,4);
    return r[3]==1;
}

boolean MeEncoderMotor::resetEncoder(){
    byte w[3]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x54;
    w[2]=_slot;
    request(w,r,11,3);
    return r[3]==1;
}
boolean MeEncoderMotor::enableDebug(){
    
    byte w[2]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x55;
    request(w,r,11,2);
    return r[3]==1;
}

boolean MeEncoderMotor::setCommandFlag(boolean flag){
    byte w[4]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x41;
    w[2]=_slot;
    w[3]=flag;
    request(w,r,4,4);
    return r[3]==1;
}
float MeEncoderMotor::getCurrentSpeed(){
    byte w[3]={0};
    byte r[7]={0};
    w[0]=0x91;
    w[1]=0x51;
    w[2]=_slot;
    request(w,r,3,7);
    int i;
    for (i=0; i<4; i++) {
        u.b[i]=r[3+i];
    }
    return u.fVal;
}
float MeEncoderMotor::getCurrentPosition(){
    byte w[3]={0};
    byte r[7]={0};
    w[0]=0x91;
    w[1]=0x52;
    w[2]=_slot;
    request(w,r,3,7);
    int i;
    for (i=0; i<4; i++) {
        u.b[i]=r[3+i];
    }
    return u.fVal;
}
float MeEncoderMotor::getPIDParam(uint8_t type,uint8_t mode){
    
    byte w[5]={0};
    byte r[9]={0};
    w[0]=0x91;
    w[1]=0x53;
    w[2]=_slot;
    w[3]=type;
    w[4]=mode;
    request(w,r,5,9);
    int i;
    for (i=0; i<4; i++) {
        u.b[i]=r[5+i];
    }
    return u.fVal;
    
}

/*Me4Button*/
Me4Button::Me4Button() : MePort(0){
	
}
Me4Button::Me4Button(uint8_t port) : MePort(port)
{
    _toggleState = NULL_KEY;
    _oldState = NULL_KEY_VALUE;
	_prevPressedState = 0;
    _pressedState = NULL_KEY;
    _releasedState = NULL_KEY;
    _heldState = NULL_KEY;
    _heldTime = millis();
}

bool Me4Button::update()
{
	if(millis()-_heldTime<10){
		return false;
	}
    uint8_t update_temp;
    uint16_t newState = 0;
	uint16_t t=0;
	uint16_t tmax = 0;
    for(uint8_t i = 0; i < 16; i++) {
		t = MePort::aRead2();
		if(i<4){
			continue;
		}
        newState += t;
		if(tmax<t){
			tmax = t;
		}
    }
	newState -=tmax;
    newState /= 11;
    if(abs(newState-_oldState) > 40)update_temp = 1;
	else update_temp = 0;	
    if (update_temp) {
		if(newState > KEY4_VALUE+50) { //released?
	            _toggleState = !_toggleState;
	            if(_oldState < KEY1_VALUE+5)_releasedState = KEY1;
	            else if(_oldState > KEY2_VALUE - 5 && _oldState < KEY2_VALUE + 5)_releasedState = KEY2;
	            else if(_oldState > KEY3_VALUE - 5 && _oldState < KEY3_VALUE + 5)_releasedState = KEY3;
	            else if(_oldState > KEY4_VALUE - 5 && _oldState < KEY4_VALUE + 5)_releasedState = KEY4;
				

				_pressedState = NULL_KEY;
	    }else{
	        if(newState < KEY1_VALUE+5)_pressedState = KEY1;
	        else if((newState > KEY2_VALUE - 5) && (newState < KEY2_VALUE + 5))_pressedState = KEY2;
	        else if((newState > KEY3_VALUE - 5) && (newState < KEY3_VALUE + 5))_pressedState = KEY3;
	        else if((newState > KEY4_VALUE - 5) && (newState < KEY4_VALUE + 5))_pressedState = KEY4;	
		}
        //delay(10); // debouncing
    } else {
       	if(_oldState < (KEY4_VALUE + 10))_pressedState = NULL_KEY;
		if(newState > KEY4_VALUE+5 && _oldState > KEY4_VALUE+5){
			_releasedState = NULL_KEY;
		}
    }
    _oldState = newState;
	_heldTime = millis();
	return true;
}


uint8_t Me4Button::pressed()
{
	update();
	uint8_t returnKey = _pressedState;
	_pressedState = NULL_KEY;
	return returnKey;
}

uint8_t Me4Button::released()
{
	update();
	uint8_t returnKey = _releasedState;
	_releasedState = NULL_KEY;
	return returnKey;
}

/*      Joystick        */
MeJoystick::MeJoystick() : MePort(0){}
MeJoystick::MeJoystick(uint8_t port) : MePort(port){}

int MeJoystick::readX()
{	
	return MePort::aRead1() ;
}

int MeJoystick::readY()
{
   
	return MePort::aRead2();
}

float MeJoystick::angle(){
	return atan2(readY(),readX())*180.0/PI;
}

float MeJoystick::strength(){
	long dx = abs(readX());
	long dy = abs(readY());
	long dist = dx*dx+dy*dy;
	return min(1.0,sqrt(dist)/255.0);
}

/*      Light Sensor        */
MeLightSensor::MeLightSensor() : MePort(0){}
MeLightSensor::MeLightSensor(uint8_t port) : MePort(port){}
int MeLightSensor::read()
{	
	return MePort::aRead2();
}

void MeLightSensor::lightOn()
{	
	MePort::dWrite1(HIGH);
}

void MeLightSensor::lightOff()
{	
	MePort::dWrite1(LOW);
}

float MeLightSensor::strength()
{
    
    return map(MePort::aRead2(),0,1023,0,1023);
}

/*      Sound Sensor        */
MeSoundSensor::MeSoundSensor() : MePort(0){}
MeSoundSensor::MeSoundSensor(uint8_t port) : MePort(port){}

int MeSoundSensor::strength()
{  
    return MePort::aRead2();
}

MeOneWire::MeOneWire(){
	
}
MeOneWire::MeOneWire(uint8_t pin)
{
	bitmask = MePIN_TO_BITMASK(pin);
	baseReg = MePIN_TO_BASEREG(pin);
//	reset_search();
}
void MeOneWire::reset(uint8_t pin)
{
	bitmask = MePIN_TO_BITMASK(pin);
	baseReg = MePIN_TO_BASEREG(pin);
//	reset_search();
}
bool MeOneWire::readIO(void)
{
	MeIO_REG_TYPE mask = bitmask;
	volatile MeIO_REG_TYPE *reg MeIO_REG_ASM = baseReg;
	uint8_t r;
	MeDIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(10);
	r = MeDIRECT_READ(reg, mask);
	return r;
}
// Perform the MeOneWire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a 0;
//
// Returns 1 if a device asserted a presence pulse, 0 otherwise.
//
uint8_t MeOneWire::reset(void)
{
	MeIO_REG_TYPE mask = bitmask;
	volatile MeIO_REG_TYPE *reg MeIO_REG_ASM = baseReg;
	uint8_t r;
	uint8_t retries = 125;

	noInterrupts();
	MeDIRECT_MODE_INPUT(reg, mask);
	interrupts();
	// wait until the wire is high... just in case
	do {
		if (--retries == 0) return 0;
		delayMicroseconds(2);
	} while ( !MeDIRECT_READ(reg, mask));

	noInterrupts();
	MeDIRECT_WRITE_LOW(reg, mask);
	MeDIRECT_MODE_OUTPUT(reg, mask);	// drive output low
	interrupts();
	delayMicroseconds(480);
	noInterrupts();
	MeDIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(70);
	r = !MeDIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(410);
	return r;
}

//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
void MeOneWire::write_bit(uint8_t v)
{
	MeIO_REG_TYPE mask=bitmask;
	volatile MeIO_REG_TYPE *reg MeIO_REG_ASM = baseReg;

	if (v & 1) {
		noInterrupts();
		MeDIRECT_WRITE_LOW(reg, mask);
		MeDIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(10);
		MeDIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(55);
	} else {
		noInterrupts();
		MeDIRECT_WRITE_LOW(reg, mask);
		MeDIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(65);
		MeDIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(5);
	}
}

//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
uint8_t MeOneWire::read_bit(void)
{
	MeIO_REG_TYPE mask=bitmask;
	volatile MeIO_REG_TYPE *reg MeIO_REG_ASM = baseReg;
	uint8_t r;

	noInterrupts();
	MeDIRECT_MODE_OUTPUT(reg, mask);
	MeDIRECT_WRITE_LOW(reg, mask);
	delayMicroseconds(3);
	MeDIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
	delayMicroseconds(10);
	r = MeDIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(53);
	return r;
}

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
//
void MeOneWire::write(uint8_t v, uint8_t power /* = 0 */) {
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	MeOneWire::write_bit( (bitMask & v)?1:0);
    }
    if ( !power) {
	noInterrupts();
	MeDIRECT_MODE_INPUT(baseReg, bitmask);
	MeDIRECT_WRITE_LOW(baseReg, bitmask);
	interrupts();
    }
}

void MeOneWire::write_bytes(const uint8_t *buf, uint16_t count, bool power /* = 0 */) {
  for (uint16_t i = 0 ; i < count ; i++)
    write(buf[i]);
  if (!power) {
    noInterrupts();
    MeDIRECT_MODE_INPUT(baseReg, bitmask);
    MeDIRECT_WRITE_LOW(baseReg, bitmask);
    interrupts();
  }
}

//
// Read a byte
//
uint8_t MeOneWire::read() {
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( MeOneWire::read_bit()) r |= bitMask;
    }
    return r;
}

void MeOneWire::read_bytes(uint8_t *buf, uint16_t count) {
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = read();
}

//
// Do a ROM select
//
void MeOneWire::select(const uint8_t rom[8])
{
    uint8_t i;

    write(0x55);           // Choose ROM

    for (i = 0; i < 8; i++) write(rom[i]);
}

//
// Do a ROM skip
//
void MeOneWire::skip()
{
    write(0xCC);           // Skip ROM
}

void MeOneWire::depower()
{
	noInterrupts();
	MeDIRECT_MODE_INPUT(baseReg, bitmask);
	interrupts();
}

void MeOneWire::reset_search()
{
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = FALSE;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--) {
    ROM_NO[i] = 0;
    if ( i == 0) break;
  }
}

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
//
void MeOneWire::target_search(uint8_t family_code)
{
   // set the search state to find SearchFamily type devices
   ROM_NO[0] = family_code;
   for (uint8_t i = 1; i < 8; i++)
      ROM_NO[i] = 0;
   LastDiscrepancy = 64;
   LastFamilyDiscrepancy = 0;
   LastDeviceFlag = FALSE;
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// MeOneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use MeOneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
uint8_t MeOneWire::search(uint8_t *newAddr)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number, search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = 0;

   // if the last call was not the last one
   if (!LastDeviceFlag)
   {
      // 1-Wire reset
      if (!reset())
      {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = FALSE;
         LastFamilyDiscrepancy = 0;
         return FALSE;
      }

      // issue the search command
      write(0xF0);

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = read_bit();
         cmp_id_bit = read_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1))
            break;
         else
         {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit)
               search_direction = id_bit;  // bit write value for search
            else
            {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy)
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               else
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);

               // if 0 was picked then record its position in LastZero
               if (search_direction == 0)
               {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            write_bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0)
            {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0)
            LastDeviceFlag = TRUE;

         search_result = TRUE;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0])
   {
      LastDiscrepancy = 0;
      LastDeviceFlag = FALSE;
      LastFamilyDiscrepancy = 0;
      search_result = FALSE;
   }
   for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   return search_result;
  }
  
// DS18B20 commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
//#define TEMP_LSB        0
//#define TEMP_MSB        1
//#define HIGH_ALARM_TEMP 2
//#define LOW_ALARM_TEMP  3
//#define CONFIGURATION   4
//#define INTERNAL_BYTE   5
//#define COUNT_REMAIN    6
//#define COUNT_PER_C     7
//#define SCRATCHPAD_CRC  8

// Device resolution
//#define TEMP_9_BIT  0x1F //  9 bit
//#define TEMP_10_BIT 0x3F // 10 bit
//#define TEMP_11_BIT 0x5F // 11 bit
//#define TEMP_12_BIT 0x7F // 12 bit

MeTemperature::MeTemperature():MePort(){
	
}
MeTemperature::MeTemperature(uint8_t port):MePort(port){
	_ts.reset(s2);
}
MeTemperature::MeTemperature(uint8_t port,uint8_t slot):MePort(port){
	MePort::reset(port, slot);
    _ts.reset( slot == SLOT2 ? s2 : s1);
}
void MeTemperature::reset(uint8_t port,uint8_t slot){
	MePort::reset(port, slot);
    _ts.reset( slot == SLOT2 ? s2 : s1);
}
float MeTemperature::temperature(){

	byte i;
	byte present = 0;
	byte type_s;
	byte data[12];
	byte addr[8];
	float celsius;
	long time;

	_ts.reset();
	_ts.skip();
	_ts.write(STARTCONVO);        // start conversion, with parasite power on at the end
	time = millis();
	while(!_ts.readIO() && (millis()-time)<750);
	
	present = _ts.reset();
	_ts.skip();    
	_ts.write(READSCRATCH);         
	for ( i = 0; i < 5; i++) {           // we need 9 bytes
	data[i] = _ts.read();
	}
	
	int16_t rawTemperature = (data[1] << 8) | data[0];

	return (float)rawTemperature * 0.0625;// 12 bit
}

static int8_t TubeTab[] = {0x3f,0x06,0x5b,0x4f,
                           0x66,0x6d,0x7d,0x07,
                           0x7f,0x6f,0x77,0x7c,
                           0x39,0x5e,0x79,0x71,
						   0xbf,0x86,0xdb,0xcf,
						   0xe6,0xed,0xfd,0x87,
						   0xff,0xef,0xf7,0xfc,
						   0xb9,0xde,0xf9,0xf1,0x40};//0~9,A,b,C,d,E,F,-                    
Me7SegmentDisplay::Me7SegmentDisplay():MePort()
{
}
Me7SegmentDisplay::Me7SegmentDisplay(uint8_t port):MePort(port)
{
  Clkpin = s2;
  Datapin = s1;
  pinMode(Clkpin,OUTPUT);
  pinMode(Datapin,OUTPUT);
  set();
  clearDisplay();
}
void Me7SegmentDisplay::reset(uint8_t port){
  reset(port);
  Clkpin = s2;
  Datapin = s1;
  pinMode(Clkpin,OUTPUT);
  pinMode(Datapin,OUTPUT);
  set();
  clearDisplay();
}
void Me7SegmentDisplay::init(void)
{
  clearDisplay();
}

void Me7SegmentDisplay::writeByte(int8_t wr_data)
{
  uint8_t i,count1;   
  for(i=0;i<8;i++)        //sent 8bit data
  {
    digitalWrite(Clkpin,LOW);      
    if(wr_data & 0x01)digitalWrite(Datapin,HIGH);//LSB first
    else digitalWrite(Datapin,LOW);
    wr_data >>= 1;      
    digitalWrite(Clkpin,HIGH);
      
  }  
  digitalWrite(Clkpin,LOW); //wait for the ACK
  digitalWrite(Datapin,HIGH);
  digitalWrite(Clkpin,HIGH);     
  pinMode(Datapin,INPUT);
  while(digitalRead(Datapin))    
  { 
    count1 +=1;
    if(count1 == 200)//
    {
     pinMode(Datapin,OUTPUT);
     digitalWrite(Datapin,LOW);
     count1 =0;
    }
    //pinMode(Datapin,INPUT);
  }
  pinMode(Datapin,OUTPUT);
  
}
//send start signal to TM1637
void Me7SegmentDisplay::start(void)
{
  digitalWrite(Clkpin,HIGH);//send start signal to TM1637
  digitalWrite(Datapin,HIGH); 
  digitalWrite(Datapin,LOW); 
  digitalWrite(Clkpin,LOW); 
} 
//End of transmission
void Me7SegmentDisplay::stop(void)
{
  digitalWrite(Clkpin,LOW);
  digitalWrite(Datapin,LOW);
  digitalWrite(Clkpin,HIGH);
  digitalWrite(Datapin,HIGH); 
}
void Me7SegmentDisplay::display(uint16_t value){
	display((float)value);
}
void Me7SegmentDisplay::display(int value){
	display((float)value);
}
void Me7SegmentDisplay::display(float value){
	
	int i=0;
	bool isStart = false;
	int index = 0;
	int8_t disp[]={
		0,0,0,0
	};
	bool isNeg = false;
	if(value<0){
		isNeg = true;
		value = -value;
		disp[0] = 0x20;
		index++;
	}
	for(i=0;i<7;i++){
		int n = checkNum(value,3-i);
		if(n>=1||i==3){
		 isStart=true; 
		}
		if(isStart){
		  if(i==3){
	 		disp[index]=n+0x10;
		  }else{
		 	disp[index]=n;
		  }
		  index++;
	  	}
		if(index>3){
		 break; 
		}
	}
	display(disp);
}
int Me7SegmentDisplay::checkNum(float v,int b){
 if(b>=0){
	return floor((v-floor(v/pow(10,b+1))*(pow(10,b+1)))/pow(10,b));
 }else{
	b=-b;
	int i=0;
 	for(i=0;i<b;i++){
  		v = v*10;
  	}
	return ((int)(v)%10);
 }
}

void Me7SegmentDisplay::display(int8_t DispData[])
{
  int8_t SegData[4];
  uint8_t i;
  for(i = 0;i < 4;i ++)
  {
    SegData[i] = DispData[i];
  }
  coding(SegData);
  start();          //start signal sent to TM1637 from MCU
  writeByte(ADDR_AUTO);//
  stop();           //
  start();          //
  writeByte(Cmd_SetAddr);//
  for(i=0;i < 4;i ++)
  {
    writeByte(SegData[i]);        //
  }
  stop();           //
  start();          //
  writeByte(Cmd_DispCtrl);//
  stop();           //
}
//******************************************
void Me7SegmentDisplay::display(uint8_t BitAddr,int8_t DispData)
{
  int8_t SegData;
  SegData = coding(DispData);
  start();          //start signal sent to TM1637 from MCU
  writeByte(ADDR_FIXED);//
  stop();           //
  start();          //
  writeByte(BitAddr|0xc0);//
  writeByte(SegData);//
  stop();            //
  start();          //
  writeByte(Cmd_DispCtrl);//
  stop();           //
}

void Me7SegmentDisplay::clearDisplay(void)
{
  display(0x00,0x7f);
  display(0x01,0x7f);
  display(0x02,0x7f);
  display(0x03,0x7f);  
}
//To take effect the next time it displays.
void Me7SegmentDisplay::set(uint8_t brightness,uint8_t SetData,uint8_t SetAddr)
{
  Cmd_SetData = SetData;
  Cmd_SetAddr = SetAddr;
  Cmd_DispCtrl = 0x88 + brightness;//Set the brightness and it takes effect the next time it displays.
}


void Me7SegmentDisplay::coding(int8_t DispData[])
{
  uint8_t PointData = 0; 
  for(uint8_t i = 0;i < 4;i ++)
  {
    if(DispData[i] == 0x7f)DispData[i] = 0x00;
    else DispData[i] = TubeTab[DispData[i]];
  }
}
int8_t Me7SegmentDisplay::coding(int8_t DispData)
{
  uint8_t PointData = 0; 
  if(DispData == 0x7f) DispData = 0x00 + PointData;//The bit digital tube off
  else DispData = TubeTab[DispData] + PointData;
  return DispData;
}
/*************Me Potentiometer****/
MePotentiometer::MePotentiometer():MePort(0){

}
MePotentiometer::MePotentiometer(uint8_t port):MePort(port){

}
uint16_t MePotentiometer::read(){
	return MePort::aRead2();
}

/*************Me PIR motion sensor****/
MePIRMotionSensor::MePIRMotionSensor():MePort(0){

}
MePIRMotionSensor::MePIRMotionSensor(uint8_t port):MePort(port){
	pinMode(s2,INPUT);
}
bool MePIRMotionSensor::isPeopleDetected(){
	return MePort::dRead2();
}

/***********Me GYRO*********/
MeGyro::MeGyro(){

}
void MeGyro::begin(){
	gSensitivity = 65.5; // for 500 deg/s, check data sheet
	gx = 0;
	gy = 0;
	gz = 0;
	gyrX = 0;
	gyrY = 0;
	gyrZ = 0;
	accX = 0;
	accY = 0;
	accZ = 0;
	gyrXoffs = -281.00;
	gyrYoffs = 18.00;
	gyrZoffs = -83.00;
	FREQ=30.0;
	Wire.begin();
	delay(1000);
	writeReg (0x6b, 0x00);
  // CONFIG:
  // Low pass filter samples, 1khz sample rate
	writeReg (0x1a, 0x01);
	writeReg(0x1b, 0x08);
	uint8_t sample_div = 1000 / FREQ - 1;
	writeReg (0x19, sample_div);
	calibrate();
}
double MeGyro::angleX(){
	if(accZ==0)return 0;
	return (atan((float)accY/(float)accZ))*180/3.1415926;
} 
double MeGyro::angleY(){
	if(accZ==0)return 0;
	return (atan((float)accX/(float)accZ))*180/3.1415926;
} 
double MeGyro::angleZ(){
	if(accY==0)return 0;
	return (atan((float)accX/(float)accY))*180/3.1415926;
} 
void MeGyro::calibrate(){
  int x;
  long xSum = 0, ySum = 0, zSum = 0;
  int num = 500;
  uint8_t error;
  for (x = 0; x < num; x++){
    error = readData(0x43, i2cData, 6);
	xSum += ((i2cData[0] << 8) | i2cData[1]);
    ySum += ((i2cData[2] << 8) | i2cData[3]);
    zSum += ((i2cData[4] << 8) | i2cData[5]);
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;
} 
void MeGyro::update(){
	 unsigned long start_time = millis();
  uint8_t error;
  // read imu data
  error = readData(0x3b, i2cData, 14);
  if(error!=0)
    return;

  double ax, ay, az;
  // assemble 16 bit sensor data
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);

  gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
  gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
  gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / (gSensitivity+1);
  
  ay = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;
  ax = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2))) * 180 / M_PI;

  // angles based on gyro (deg/s)
  gx = gx + gyrX / FREQ;
  gy = gy - gyrY / FREQ;
  gz += gyrZ / FREQ;

  // complementary filter
  // tau = DT*(A)/(1-A)
  // = 0.48sec
  gx = gx * 0.96 + ax * 0.04;
  gy = gy * 0.96 + ay * 0.04;

  //delay(((1/FREQ) * 1000) - (millis() - start_time)-time);
}
int MeGyro::readData(int start, uint8_t *buffer, int size)
{
	int i, n, error;
	Wire.beginTransmission(0x68);
	n = Wire.write(start);
	if (n != 1)
	return (-10);
	n = Wire.endTransmission(false);    // hold the I2C-bus
	if (n != 0)
	return (n);
	delayMicroseconds(1);
	// Third parameter is true: relase I2C-bus after data is read.
	Wire.requestFrom(0x68, size, true);
	i = 0;
	while(Wire.available() && i<size)
	{
	buffer[i++]=Wire.read();
	}
	delayMicroseconds(1);
	if ( i != size)
	return (-11);
	return (0);  // return : no error
}
int MeGyro::writeData(int start, const uint8_t *pData, int size)
{
  int n, error;
  Wire.beginTransmission(0x68);
  n = Wire.write(start);        // write the start address
  if (n != 1)
  return (-20);
  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
  return (-21);
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
  return (error);
  return (0);         // return : no error
}
int MeGyro::writeReg(int reg, uint8_t data)
{
  int error;
  error = writeData(reg, &data, 1);
  return (error);
}

