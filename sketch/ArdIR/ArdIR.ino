/* 
 * ArdIR sketch 
 */
 
#include <OneWire.h>
#include <EEPROM.h>
#define USE_SERIAL_EEPROM
#define BUFDIM 256 // buffer size to store IR stream; 250 X 8 bit = 2000 -> 200ms more than sufficient!
// note: if external eeprom is used, 'BUFDIM' must be a multiple of 'EEP_PAGE_SIZE'

#ifdef USE_SERIAL_EEPROM
#define EEP_BYTE_SIZE		32768					// size of eeprom
#define EEP_I2C_ADDRESS 	0x50					// i2c bus address
#define EEP_PAGE_SIZE 		16						// arduino buffer size / eeprom page size
#define EEP_BUFFER_PAGES 	BUFDIM/EEP_PAGE_SIZE	// number of pages to store the buffer
#include <Wire.h>
#endif

#define NCHANNELS 5  // <--- can be changed accordingly with homonym definition in html page
#define FRAME_TIME 30000 // listen time

//=== Arduino signal definitions ====//
#define BoardLED 			13   // Arduino/RandA on-board led
#define InterruptInputPin 	6 // INT0 / PD2
#define PushButton1 		8
#define PushButton2 		10
#define PWMOUT 				5
#define LED1 				3 // red
#define LED2 				7 // green
#define DS18B20 			11

// ==== variables definitions ====//
volatile uint8_t BitStream[BUFDIM];
uint16_t buf_ptr;
uint8_t BitMask;
uint16_t Frame_width_counter;
uint8_t SystemStateFlags;
#define P1_PUSHED   0x01
#define P2_PUSHED   0x02
#define START_ACQ   0x04
#define IR_DETECT   0x08
#define SEND_IR     0x10
#define MANUALE     0x20
char buffer[16];
static uint16_t TimerPostScaler=1000;
static uint16_t TimeoutCounter = 0;
static int isr_tps=0;
uint8_t mstate=0;
uint8_t ucPushCounter=0,ucPushCounter2=0;
int RxByte;
uint8_t ucTmp1, KeyNum;
uint16_t uiTmp1; 
// following is kept from example code to read DS18B20
byte i;
byte present = 0;
byte data[12];
byte addr[8];
char StrTemp[8];
// ==== end variables definitions ====//
OneWire ds(DS18B20); // init 1-wire communication with DS18B20
// ==== start function block ====//
void ManDelay (uint16_t ms) {
  volatile uint16_t uiTmp1, uiTmp2;
  for (uiTmp1 = 0; uiTmp1 < ms; uiTmp1 ++) {
      uiTmp2 = 700;
      while (uiTmp2 > 0) uiTmp2 --;
    }
}

void LedBlink(uint8_t Nblinks) { // useful to blink red led
  while (Nblinks > 0) {
    digitalWrite(LED1, 1); 
    ManDelay(500);
    digitalWrite(LED1, 0); 
    ManDelay(500);
    Nblinks--;
  }
}

/*****
 * ParseInput() reads serial characters stream (from servlet) in order to retrieve channel number (from 3rd character on)
 * It returns channel number (1..N if any), 0 otherwise
 * Parameters: none
 *****/
int ParseInput(void) {
	int retval;
	int n = Serial.readBytesUntil('\n',buffer,24);
	if(n>0) {
		buffer[n]='\0';
		if (n>=3) {
			retval = atoi(&buffer[2]);
			if (retval <= NCHANNELS + 1) return retval;
			else return 0;
		}
		else return 0;
	}
	return 0;
}

/*****
 * setup(): start of the world!
 *****/
void setup()
{ 
	pinMode(BoardLED, OUTPUT);
	pinMode(LED1, OUTPUT); pinMode(LED2, OUTPUT);
	pinMode(PushButton1, INPUT_PULLUP);
	pinMode(PushButton2, INPUT_PULLUP);
	pinMode(PWMOUT, OUTPUT);
	pinMode(InterruptInputPin, INPUT_PULLUP);
// open the serial port at 9600 bps:
	Serial.begin(9600);
#ifdef USE_SERIAL_EEPROM
// init eeprom comm bus
	Wire.begin();
#endif
    noInterrupts();           // disable all interrupts
 // T0 setup
	TCCR0A = 0x03; // PWM disconnected
	TCCR0B = 0x0A; // prescaler = 8
	OCR0A = 55;
	OCR0B = 25;
 // T1 setup
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 1600;
    TCCR1B |= (1 << WGM12); // CTC
    TCCR1B |= (1 << CS10); // prescaler 1
// enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A); 
// setup buffers
    buf_ptr = 0;
    BitStream[buf_ptr] = 0;
    BitMask = 0x01;
    SystemStateFlags=0;
	LedBlink(3);
    interrupts();             // enable all interrupts
}

/****
 * interrupt service routine
 * called every 100 microseconds
 ****/ 
ISR(TIMER1_COMPA_vect)    {   
	if (TimerPostScaler > 0) TimerPostScaler --;
	else {
		TimerPostScaler = 1000;
		if (TimeoutCounter > 0) TimeoutCounter --;
	}
	if (SystemStateFlags & START_ACQ) { // I.R. acquisition
		if ((Frame_width_counter < FRAME_TIME) && (!(SystemStateFlags & IR_DETECT)))
			Frame_width_counter ++;
		if (Frame_width_counter == FRAME_TIME)
			SystemStateFlags &= ~START_ACQ; // timeout
		if (digitalRead(InterruptInputPin)) {
			digitalWrite(BoardLED, 1);
		}
		else {
			SystemStateFlags |= IR_DETECT; // frame detected
			digitalWrite(BoardLED, 0);
			BitStream[buf_ptr] |= BitMask; // set masked bit if signal active (low)
		}
		if (SystemStateFlags & IR_DETECT) {
			if (BitMask < 0x80) BitMask <<=1;
			else {
				BitMask = 0x01;
				buf_ptr ++;
				if (buf_ptr < BUFDIM) 
					BitStream[buf_ptr] = 0;
				else 
					SystemStateFlags &= ~START_ACQ; // acq. finished
			}
		}
	} // acquisition 
	else if (SystemStateFlags & SEND_IR) { // I.R. trx
		if (isr_tps > 0) isr_tps --;
		else {
		if (BitStream[buf_ptr] & BitMask) {
			TCCR0A = 0x23; // PWM connected, clear OC0B on compare match
		}
		else {
			TCCR0A = 0x03; // PWM disconnected
			digitalWrite(PWMOUT, 0);
		}
		if (BitMask < 0x80) BitMask <<=1;
		else {
			BitMask = 0x01;
			buf_ptr ++;
			if (buf_ptr > BUFDIM-1) {
				SystemStateFlags &= ~SEND_IR; // trx finished
				TCCR0A = 0x03; // PWM disconnected
				digitalWrite(PWMOUT, 0);
			}
		}
	  }
	}
}

/****
 * StoreIrRecord() 
 * stores the (previously acquired) IR frame (BitStream) into non-volatile memory (either internal or external)
 * the memory block is selected by 'ChNum'
 * It returns true if operation succeed 
 ****/
bool StoreIrRecord(uint8_t ChNum) {
	uint16_t BaseAddress, EepAddress;
	boolean b_result = TRUE; int i, bytecnt;
	BaseAddress = (ChNum + 1)* BUFDIM; //1-idx channel
#ifdef USE_SERIAL_EEPROM
	if (BaseAddress > EEP_BYTE_SIZE - BUFDIM) return FALSE; // eeprom size exceeded
	for (i = 0; i < EEP_BUFFER_PAGES; i ++) { // loop to store the entire buffer by writing EEP_BUFFER_PAGES pages onto eeprom
		EepAddress = BaseAddress + (i * EEP_PAGE_SIZE);
		Wire.beginTransmission(EEP_I2C_ADDRESS);
		Wire.write((int)((EepAddress) >> 8));   // MSB
		Wire.write((int)((EepAddress) & 0xFF)); // LSB
		for (bytecnt = 0; bytecnt < EEP_PAGE_SIZE; bytecnt ++)
			Wire.write((byte)BitStream[bytecnt + (i * EEP_PAGE_SIZE)]);
		int res = Wire.endTransmission();
		if (res != 0) return FALSE;  // error
		ManDelay(6);  // needs 5ms minimum guaranteed for each page write
	}
// now for the test
	for (i = 0; i < EEP_BUFFER_PAGES; i ++) { // loop to read back all EEP_BUFFER_PAGES pages from eeprom
		EepAddress = BaseAddress + (i * EEP_PAGE_SIZE);
		Wire.beginTransmission(EEP_I2C_ADDRESS);
		Wire.write((int)(EepAddress >> 8));   // MSB
		Wire.write((int)(EepAddress & 0xFF)); // LSB
		int res = Wire.endTransmission();
		if (res != 0) return FALSE;  // error
		Wire.requestFrom(EEP_I2C_ADDRESS, EEP_PAGE_SIZE);
		bytecnt = 0; 
		while(Wire.available()) {
			if (BitStream[bytecnt + (i * EEP_PAGE_SIZE)] != Wire.read())
				b_result = FALSE;
			bytecnt ++;
		}
	}
#else
	if (ChNum > 3) b_result = FALSE; // internal eeprom exceeded
	else {
		for (uiTmp1 = 0; uiTmp1 < buf_ptr+1; uiTmp1 ++) {
			EEPROM.write(BaseAddress + uiTmp1, BitStream[uiTmp1]);
		}
		b_result = TRUE; // note = assume that internal eeprom works ok
	}
#endif
	return b_result;
}

/****
 * LoadIrRecord() 
 * Loads the IR frame into vector BitStream[], from non-volatile memory (either internal or external)
 * the memory block to be copied is selected by 'ChNum'
 * It returns true if operation succeed 
 ****/
bool LoadIrRecord(uint8_t ChNum) {
	uint16_t BaseAddress, EepAddress;
	int i, bytecnt;
	//BaseAddress = ChNum * BUFDIM;
	BaseAddress = (ChNum + 1)* BUFDIM; //1-idx channel
#ifdef USE_SERIAL_EEPROM
	if (BaseAddress > EEP_BYTE_SIZE - BUFDIM) return FALSE; // eeprom size exceeded
		for (i = 0; i < EEP_BUFFER_PAGES; i ++) { // loop to read all EEP_BUFFER_PAGES pages from eeprom
		EepAddress = BaseAddress + (i * EEP_PAGE_SIZE);
		Wire.beginTransmission(EEP_I2C_ADDRESS);
		Wire.write((int)(EepAddress >> 8));   // MSB
		Wire.write((int)(EepAddress & 0xFF)); // LSB
		int res = Wire.endTransmission();
		if (res != 0) return FALSE;  // error
		Wire.requestFrom(EEP_I2C_ADDRESS, EEP_PAGE_SIZE);
		bytecnt = 0;
		while(Wire.available()) {
			BitStream[bytecnt + (i * EEP_PAGE_SIZE)] = Wire.read();
			bytecnt ++;
		}
	}
#else
	if (ChNum > 3) return FALSE; // internal eeprom exceeded
	for (i = 0; i < BUFDIM; i ++)
		BitStream[i] = EEPROM.read(BaseAddress + i); // load buffer from proper EEprom block
#endif
	return TRUE;
}

/*
 * Endless loop...
 */
void loop() {
  switch (mstate) {
    case 0: // start
		digitalWrite(LED1, 0); digitalWrite(LED2, 0);
		mstate = 1;
    break;
    case 1: // wait cmd
		ucTmp1 = ParseInput();
		if (ucTmp1 > 0) { // something's arrived
			if (ucTmp1 <= NCHANNELS) { // transmit channel number specified
				KeyNum = ucTmp1 - 1; // 0-idx channel number
				ManDelay(100);
				mstate = 11;
			}
			else if (ucTmp1 == NCHANNELS+1) { // transmit temperature
				mstate = 8;
			}
		}
		else if (SystemStateFlags & P1_PUSHED) { // Acquisition required: go waiting for channel number
			mstate = 2; 
			TimeoutCounter = 100; // 10s timeout
		}
		else if (SystemStateFlags & P2_PUSHED) { // manual command
			SystemStateFlags |= MANUALE;
			LedBlink(2); // manual mode
			mstate = 7;
			TimeoutCounter = 50; // 5s timeout to return here 
		}
    break;
    case 2: // acquisition
		ucTmp1 = ParseInput();
		if ((ucTmp1 > 0) && (ucTmp1 <= NCHANNELS)) {
            KeyNum = ucTmp1 - 1; // 0-idx channel number
            mstate = 3;
			digitalWrite(LED1, 1); // led steady
		} // if rec char
		if (mstate == 2) { // nothing 
			digitalWrite(LED1, 1);
			ManDelay(500);
			digitalWrite(LED1, 0);
			ManDelay(500);
			if (TimeoutCounter == 0) { // timeout: abort
				mstate = 1;
			}
		}
    break;
    case 3: // prepare to start acquisition
		buf_ptr = 0;
        BitStream[buf_ptr] = 0;
        BitMask = 0x01;
        Frame_width_counter = 0;
        SystemStateFlags &= ~IR_DETECT;
        SystemStateFlags |= START_ACQ;
        mstate = 4;
    break;
    case 4: // wait for end of acquisition
		if (!(SystemStateFlags & START_ACQ))  { // finished
			digitalWrite(LED1, 0);
			if (buf_ptr == BUFDIM) { // acquisition succeeded
				if (StoreIrRecord((byte)KeyNum)) {
					digitalWrite(LED1, 0);  digitalWrite(LED2, 1); // green
					TimeoutCounter = 50; // 5s timeout to display green led
				}
				else {
					digitalWrite(LED2, 0); 
					LedBlink(3); TimeoutCounter = 0; // back asap to initial state after blink
				}
			}
			else { // failed
          // Serial.print("Error: acquisition failed; chars received: "); Serial.println(buf_ptr, DEC); 
				digitalWrite(LED2, 0); 
				LedBlink(3); TimeoutCounter = 0; // back asap to initial state after blink
			}
			mstate = 5;
		}
    break;
    case 5: // get back to initial state
      if (!(SystemStateFlags & (P1_PUSHED|P2_PUSHED))) {
        if (TimeoutCounter == 0) {
			digitalWrite(LED1, 0);  digitalWrite(LED2, 0); // leds off
			mstate = 1;
		}
      }
      break;
    case 7: // stand alone modality: acquire/transmit only by using local Keys
		KeyNum = 0;
		if (SystemStateFlags & P1_PUSHED) { // acquisition mode
			digitalWrite(LED1, 1);
			buf_ptr = 0;
			BitStream[buf_ptr] = 0;
			BitMask = 0x01;
			Frame_width_counter = 0;
			SystemStateFlags &= ~IR_DETECT;
			SystemStateFlags |= START_ACQ;
			mstate = 4;
		}
		else if (SystemStateFlags & P2_PUSHED) { // transmission mode
			LedBlink(3); // 3 blinks before transmit!
			LoadIrRecord((byte)KeyNum);
			buf_ptr = 0;
			BitMask = 0x01;
			SystemStateFlags |= SEND_IR;
			mstate = 12;
		}
		else if (TimeoutCounter == 0) {
			LedBlink(2); // manual mode: exit
			digitalWrite(LED1, 0);  digitalWrite(LED2, 0); // leds off
			mstate = 1;
		}
    break;
	case 8: // start cycle to transmit temperature
		if (TestDs18b20()) 
			Serial.println("NOK");
		else {
			Serial.println(StrTemp);
		}
		mstate = 1;
    break;
    case 11: // load buffer with channel data
		Serial.print("\n"); Serial.println("OK");Serial.print("\n");
		if (LoadIrRecord((byte)KeyNum)) {
			buf_ptr = 0;
			BitMask = 0x01;
			isr_tps = 100;
			SystemStateFlags |= SEND_IR;
			digitalWrite(LED2, 1);
			mstate = 12;
		}
		else {
			digitalWrite(LED1, 1); 
			TimeoutCounter = 30; // 3s red led
			mstate = 5;
		}
    break;
    case 12: // wait for trx end...
      if (!(SystemStateFlags & SEND_IR))  { // finished
        digitalWrite(LED2, 0);
        TimeoutCounter = 1;
		mstate = 5; // go wait for pushbuttons inactivity
      }
    break;
    default:
      mstate=0;
  } // switch (mstate) 
  if (!digitalRead(PushButton1)) {
    if (ucPushCounter < 10) ucPushCounter ++;
    else SystemStateFlags |= P1_PUSHED;
  }
  else {
    ucPushCounter = 0;
    SystemStateFlags &= ~P1_PUSHED;
  }
    if (!digitalRead(PushButton2)) {
    if (ucPushCounter2 < 10) ucPushCounter2 ++;
    else SystemStateFlags |= P2_PUSHED;
  }
  else {
    ucPushCounter2 = 0;
    SystemStateFlags &= ~P2_PUSHED;
  }
}

/* Test DS18 
 * just copied from Arduino repository and adapted to the application
 */
int TestDs18b20(void) { 
	int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
	ds.reset_search();
	if ( !ds.search(addr)) {
 //     Serial.print("No more addresses.\n");
      ds.reset_search();
	  return 1;
    }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
    //  Serial.print("CRC is not valid!\n");
      return 1;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(800);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
//  Serial.print("P="); Serial.print(present,HEX); Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
 //   Serial.print(data[i], HEX); Serial.print(" ");
  }
//  Serial.print(" CRC="); Serial.print( OneWire::crc8( data, 8), HEX); Serial.println();
	LowByte = data[0];
	HighByte = data[1];
	TReading = (HighByte << 8) + LowByte;
	SignBit = TReading & 0x8000;  // test most sig bit
	if (SignBit) { // negative
		TReading = (TReading ^ 0xffff) + 1; // 2's comp
	}
	Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25
	Whole = Tc_100 / 100;  // separate off the whole and fractional portions
	Fract = Tc_100 % 100;
	sprintf(&StrTemp[1], "%d,%d", Tc_100/100, Tc_100 % 100);
	if (SignBit) 
		StrTemp[0] = '-';
	else 
		StrTemp[0] = '+';
	return 0;
}
