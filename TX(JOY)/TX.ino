#define USE_MICRO_WIRE
//#define DEBUG
//#define DEBUG_NRF24
//#define FLAPS

#include <SPI.h>
#include <bits_macros.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <directADC.h>
#include <EEPROM.h>
#include <GyverOLED.h>

#define THROTTLE_STICK_AI ADC_A0
#define ROLL_STICK_AI ADC_A6
#define PITCH_STICK_AI ADC_A7
#define VCC_AI ADC_A1

#define LEFT_BUTT_1_PIN 7
#define RIGHT_BUTT_1_PIN 8 
#define LEFT_BUTT_2_PIN 17
#define RIGHT_BUTT_2_PIN 16 
#define CENTER_BUTT_PIN 5
#define SELECTOR_LEFT_PIN 4 
#define SELECTOR_RIGHT_PIN 3

#define BUZZER_PIN 6
#define CONNECT_OK_LED_PIN 2

#define SCREEN_NUM

#define BUZER_TONE 32										// ORC0A value for CTC mode PWM
#define SHORT_BEEP 7										// buzzer beep duration
#define MEDIUM_BEEP 20
#define LONG_BEEP 40

#define SYSTEM_DATA_ADDR 5 
#define CONFIG_DATA_ADDR 10  

#define TX_INT 2											  // 50 Hz   (100Hz/TX_INT)
#define VCC_INT 200											// 0.5 Hz
#define DISP_UPD_INT 50									// 2 Hz
#define BUTT_INT 5											// 20 Hz
#define FIRST_ON_DELAY 150

#define TRIM_SCALE 15
#define TRIM_PITCH 0
#define TRIM_ROLL 1

//#define VCC_INT_REF 1060UL  
//#define VCC_CALC_CONST (VCC_INT_REF*256UL)

#define VCC_PWR 5150                    // 5.15 V = 5150 mV
#define VCC_COEF (VCC_PWR/255.0)
#define VCC_INC 5 
#define BRIGHT_INC 10

#define BAT_LOW 3300      
#define BAT_FULL 4150  
#define BAT_LOW_REMOTE 33   						// == 3.3V 	  
#define BAT_FULL_REMOTE 42 							// == 4.2V

#define LEFT_BUTT_1_BIT 0
#define RIGHT_BUTT_1_BIT 1
#define LEFT_BUTT_2_BIT 2
#define RIGHT_BUTT_2_BIT 3 
#define CENTER_BUTT_BIT 4 

#define SELECTOR_LEFT 1
#define SELECTOR_CENTER 2
#define SELECTOR_RIGHT 3

#define FLAPS_SET_FULL_UP 0
#define FLAPS_SET_POS_1 1
#define FLAPS_SET_POS_2 2

#define PACKAGE_SIZE_TX 5
#define PACKAGE_SIZE_RX 2

#define NO_ACK_MAX 50

#define ON 1
#define OFF 0

#define TRUE 1
#define FALSE 0

#define NUM_OF_SCREENS 3

RF24 radio(9,10); 	
GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;

//--------------------0--------1-----2-------3---------4------
enum TX_data_pack {throttle, roll, pitch, buttons, switcher};
//----------------------0------------1------------2------
enum RX_data_pack {battery, flaps_trim_fb};

enum screnNames {SCREEN_MAIN = 1, SCREEN_TIMER, SCREEN_SETTINGS};

uint8_t address[][2] = {"1Node","2Node"};
uint8_t tx_data[PACKAGE_SIZE_TX];
uint8_t rx_data[PACKAGE_SIZE_RX];

uint8_t ADC_mux_switch = 0;
uint8_t throttle_val, roll_val, pitch_val;
uint8_t buttons_state, switcher_state, click_detect, butt_hold_state;
uint8_t count_to_tx, count_vcc, count_disp, count_butt, count_buzz, count_no_ACK, count_first_on;
uint8_t buzzDuration;
uint8_t screen = 1;

bool thr_init = 0;
bool battery_low = 0;
bool connectionOK;

uint16_t charge;

#ifdef DEBUG
	unsigned long loop_time;
#endif

struct {																				//system data structure
  uint8_t brightness;
  uint16_t internal_ref;																// 1049 - value
} systemData;

struct {
	 uint8_t thr_max;
	 uint8_t thr_min;
	 uint8_t roll_max;
	 uint8_t roll_min;
	 uint8_t pitch_max;
	 uint8_t pitch_min;  
	} stickLimits;

void pin_init(void){
	 pinMode(CONNECT_OK_LED_PIN, OUTPUT);	
	 pinMode(BUZZER_PIN, OUTPUT);	
	 
	 digitalWrite(CONNECT_OK_LED_PIN, HIGH);											//switch off connect OK LED	
	 digitalWrite(BUZZER_PIN, LOW);

	 pinMode(LEFT_BUTT_1_PIN, INPUT_PULLUP);
	 pinMode(RIGHT_BUTT_1_PIN, INPUT_PULLUP);
	 pinMode(LEFT_BUTT_2_PIN, INPUT_PULLUP);
	 pinMode(RIGHT_BUTT_2_PIN, INPUT_PULLUP);
	 pinMode(CENTER_BUTT_PIN, INPUT_PULLUP);
	 pinMode(SELECTOR_LEFT_PIN, INPUT_PULLUP);
	 pinMode(SELECTOR_RIGHT_PIN, INPUT_PULLUP);	 
	}

void NRF24_init(void){
	#ifdef DEBUG_NRF24
	 if (radio.begin()) printf("NRF24 init OK\n"); 
	 else printf("NRF24 init FAULT\n");
	#else 
	 radio.begin();
	#endif

	 radio.setAutoAck(1);                    			 	// Ensure autoACK is enabled
	 radio.enableAckPayload();               				// Allow optional ack payloads
	 radio.setRetries(7,15);                 				// 5us time between retries, max no. of retries (15)
	 radio.enableDynamicPayloads();
	 
	 radio.setPALevel(RF24_PA_MAX); 
	 radio.setDataRate(RF24_250KBPS); 
	 radio.setChannel(0x00);  

	//---to write data
     radio.openWritingPipe(address[0]);       				// Open different pipes when writing. Write on pipe 0, address 0
     radio.openReadingPipe(1,address[1]);     				// Read on pipe 1, as address 1 
	 radio.stopListening();                 				// Stop listening

	//---to read data
    // radio.openWritingPipe(address[1]);					// Since only two radios involved, both listen on the same addresses and pipe numbers in RX mode
    // radio.openReadingPipe(1,address[0]);					// then switch pipes & addresses to transmit. 
    // radio.startListening();								// Need to start listening after opening new reading pipes	
	
	 radio.powerUp();
	 
	#ifdef DEBUG_NRF24
	 radio.printDetails();                   				// Dump the configuration of the rf unit for debugging		
	#endif	
	 
	 delay(5);
	}

void timers_init (void){
	//-- timer 0 init (1953 Hz)
	 TCCR0A = (0 << COM0A0) | (1 << WGM01);												//
	 TCCR0B = (1 << CS02);																//
	 OCR0A = BUZER_TONE;																// = (16*10^6) / (i*256) - 1 (i must be <256)	 
	 
	//-- timer 2 init (100 Hz)
	 TCCR2A = (1 << WGM21);															
	 TCCR2B = (1 << CS20) | (1 << CS21)| (1 << CS22);									//prescaler - clk/1024
	 TIMSK2 = (1 << OCIE2A);															// 		clk	   / (Hz*prescaler) - 1
	 OCR2A = 156;																		// = (16*10^6) / (i*1024) - 1 (i must be <256)	 
	}

void _ADCInit(void){                  														
	 ADC_enable();		
	 ADC_setPrescaler(8);		
	 ADC_setReference(ADC_VCC);		
	 setAnalogMux(THROTTLE_STICK_AI);	
	 ADC_setResolution(8);		
	 delayMicroseconds(100);         														
	 ADC_startConvert();		
	}	

void oledEnable(void){                        								 
	 oled.init(); 
	 oled.setContrast(systemData.brightness);   											
	 oled.clear();
	 oled.update();
	}

void ADCReadData(void){
	 if(ADC_available()){
		 switch (ADC_mux_switch){
			 case 0:
				 throttle_val = 255-ADC_read8();
				 setAnalogMux(ROLL_STICK_AI);
				 ADC_mux_switch = 1;
				 delayMicroseconds(5);
				 break;
			 case 1:
				 roll_val = ADC_read8();				 
				 setAnalogMux(PITCH_STICK_AI);
				 ADC_mux_switch = 2;
				 delayMicroseconds(5);
				 break;
			 case 2:
				 pitch_val = ADC_read8();
				 
				 if (count_vcc >= VCC_INT) {							//if time to mesure VCC
           //setAnalogMux(ADC_1V1);
					 setAnalogMux(VCC_AI);
					 ADC_mux_switch = 3;
					 delayMicroseconds(300);
					}
				 else{
					 setAnalogMux(THROTTLE_STICK_AI);
					 ADC_mux_switch = 0;				 
					 delayMicroseconds(5);	
					}								
				 break;
			 case 3:
//				 charge = (uint16_t)(VCC_CALC_CONST / ADC_read8()); 	// read battery voltage	using internal 1.1V ref  
				 charge = (uint16_t)(ADC_read8() * VCC_COEF);
				 setAnalogMux(THROTTLE_STICK_AI);
				 ADC_mux_switch = 0;
				 delayMicroseconds(5);
				 count_vcc = 0;
				 break;
			}
		 ADC_startConvert();
		}
	}

uint16_t getVref(uint16_t vcc){    														  // read referance voltage
	 uint16_t buf = 0;                 	

	 for (uint8_t i = 0; i < 8; i++) { 	
		 ADC_startConvert(); 	             
		 while (!ADC_available());                                             
		 buf += ADC_read8();             
		}                                                                      
	
	 buf >>= 3;                        	
	 return ((uint32_t) vcc * buf) >> 8;  
	}

void throttle_init (void){
	 static uint8_t step = 0;
 
	 if (step == 0 && throttle_val >= stickLimits.thr_max){
		 buzzerBeep(SHORT_BEEP);
		 step = 1;
		}
	 if (step == 1 && throttle_val <= stickLimits.thr_min){	
		 buzzerBeep(LONG_BEEP);
		 thr_init = TRUE;
		 step = 0;
		}			 	
	}

static inline void signalQualityIndicator(void){
	 uint8_t retransmit_count;
	 
	 retransmit_count = radio.getARC();
	  
	 do {																				
		 if (retransmit_count < 15){
		 oled.setCursor(110,0);
		 oled.print(retransmit_count);
			}
		 else {
			 oled.circle(118, 7, 6, OLED_STROKE);
			 oled.line(115, 4, 122, 11);
			 break;
			}
			
		 if (retransmit_count < 14){	
			 oled.fastLineV(111, 15, 15);
			 oled.fastLineV(112, 15, 15);
			}
		 else break;
	
		 if (retransmit_count < 12){	
			 oled.fastLineV(114, 12, 15);
			 oled.fastLineV(115, 12, 15);
			}
		 else break;
		 
		 if (retransmit_count < 9){	
			 oled.fastLineV(117, 9, 15);
			 oled.fastLineV(118, 9, 15);
			}
		 else break;
		 
		 if (retransmit_count < 7){	
			 oled.fastLineV(120, 6, 15);
			 oled.fastLineV(121, 6, 15);
			}
		 else break;
			 
		 if (retransmit_count < 5){	
			 oled.fastLineV(123, 3, 15);
			 oled.fastLineV(124, 3, 15);
			}
		 else break;
		 
		 if (retransmit_count < 3){	
			 oled.fastLineV(126, 0, 15);
			 oled.fastLineV(127, 0, 15);
			}
		} while (false);	 
	}

static inline void batteriesIndicators(bool noACK, bool blink){
	 uint8_t  charge_indicator_remote, charge_indicator_local;
	 uint8_t charge_remote;
		
	 //---local battery indicator
	 charge_indicator_local = map(charge, BAT_LOW, BAT_FULL, 0, 14);
	 
	 oled.setScale(1);
	 oled.setCursor(0,1);
	 oled.print(charge / 1000);
	 oled.setCursor(5,1);
	 oled.print(",");
	 oled.setCursor(9,1);
	 oled.print((charge %1000) / 100);
	 oled.setCursor(17,1); 
	 oled.print("V"); 
	 
	 oled.rect(1, 0, 16, 6, OLED_STROKE);  															// indicator body
	 oled.fastLineV(17, 1, 5);  																	// "battery-look line"
	 for (uint8_t j = 0; j < charge_indicator_local; j++) oled.line(2 + j, 0, 2 + j, 6);			// filling batt indicator with vertical lines	
	 
	 //---remote battery indicator	
	 charge_remote = ((rx_data[battery]>>4)*10) + (rx_data[battery] & 0b0001111);
	 charge_indicator_remote = constrain(map(charge_remote, BAT_LOW_REMOTE, BAT_FULL_REMOTE, 0, 40), 0, 40);
	 
	 if (charge_remote <= BAT_LOW_REMOTE && connectionOK) {
		 battery_low = TRUE;
		 buzzerBeep(MEDIUM_BEEP);
		}
	 else battery_low = FALSE;
	 
	 if (!battery_low || (battery_low && blink)){
	 	 oled.rect(45, 0, 85, 6, OLED_STROKE);  										// indicator body
		 oled.fastLineV(86, 1, 5);  																// "battery-look" line 
		 for (uint8_t i = 0; i < charge_indicator_remote; i++) oled.line(46 + i, 0, 46 + i, 6);		// filling batt indicator with vertical lines
		}
	 
	 if (noACK) {
		 oled.setCursor(65,1);
		 oled.print("?"); 
		}
	 else{	 
		 oled.setCursor(56,1);
		 oled.print(rx_data[battery]>>4);
		 oled.setCursor(61,1);
		 oled.print(",");
		 oled.setCursor(65,1);
		 oled.print(rx_data[battery] & 0b0001111);
		 oled.setCursor(73,1); 
		 oled.print("V"); 
		}		 
	}

static inline void mainScreen (bool noACK, bool blink){
	 uint8_t throttle_percent_val;
	 uint8_t flaps;
	 int8_t trim;
	
	 switch(screen){
		 case SCREEN_MAIN:
			 throttle_percent_val = constrain(map(throttle_val, stickLimits.thr_min, stickLimits.thr_max, 0, 100), 0, 100);
		   //---throttle need init indicator	
			 if (!thr_init && blink) {
				 oled.setCursor(20,2);
				 oled.print("!!!NEED INIT!!!"); 						 
			 	}
			 else if (battery_low && blink){
				 oled.setCursor(15,2);
				 oled.print("!!!LOW BATTERY!!!");
				}				 
				
		   //---throttle indicator
			 oled.setScale(1);
			 oled.setCursor(8,4);
			 oled.print("THR"); 
			 oled.setScale(2);
			 if (throttle_percent_val >= 100) oled.setCursor(1,6);
			 else if (throttle_percent_val > 9) oled.setCursor(7,6);
				 else oled.setCursor(13,6);
			 oled.print(throttle_percent_val); 		 
			 
			#ifdef FLAPS 
		   //---flaps indicator	 
			 oled.setScale(1);
			 oled.setCursor(50,4);
			 oled.print("FLAPS"); 
			 oled.setCursor(55,6);	
			 oled.setScale(2);
			 
			 if (noACK) {
				 oled.setCursor(58,6);
				 oled.print("?"); 
				}
			 else{
				 flaps = rx_data[flaps_trim_fb] & 0b00000011;		 
				 if (flaps == FLAPS_SET_FULL_UP) oled.print("UP");
				 if (flaps == FLAPS_SET_POS_1) oled.print("F1");
				 if (flaps == FLAPS_SET_POS_2) oled.print("F2");
				}
      #else
       oled.setScale(1);
			 oled.setCursor(50,4);
			 oled.print("SENCE"); 
			 oled.setCursor(55,6);	
			 oled.setScale(2);
			 	 
			 if (switcher_state == SELECTOR_LEFT) oled.print("3X");
			 if (switcher_state == SELECTOR_CENTER) oled.print("2X");
			 if (switcher_state == SELECTOR_RIGHT) oled.print("1X");
			#endif	
			
		   //---trim indicator
			 oled.setScale(1);
			 oled.setCursor(99,4);
			 oled.print("TRIM"); 
			 oled.setCursor(119,5);
			 if ((rx_data[flaps_trim_fb] >> 7) == TRIM_PITCH) oled.print("P"); 
			 else oled.print("R");

			 oled.setScale(2);	 

			 if (noACK) {
				 oled.setCursor(108,6);	
				 oled.print("?"); 
				}
			 else{
				 trim = TRIM_SCALE - ((rx_data[flaps_trim_fb] >> 2) & 0b00011111);
				 if (trim >= 10){
					 oled.setCursor(91,6);	
					 oled.print("+");
					 oled.setCursor(103,6);					 
					 }
				 else if (trim >= 0){
					 oled.setCursor(96,6);	
					 oled.print("+");
					 oled.setCursor(108,6);	
					 }
				 else if (trim <= -10) oled.setCursor(90,6);	
				 else oled.setCursor(97,6);
				 
				 oled.print(trim); 
				}
		break;
		 case SCREEN_TIMER:
		break;
		 case SCREEN_SETTINGS:		 
		break;
		}	
	}

void displayUpdate (void){
	 bool noAckData = 0;
	 static bool blink = 0;
	 
	 blink = !blink;
	 
	 if (count_no_ACK >= NO_ACK_MAX) noAckData = 1;
		
	 oled.clear();
	 
	 batteriesIndicators(noAckData, blink);
	 signalQualityIndicator();
	 mainScreen(noAckData, blink);

	 oled.update();	 
	 count_disp = 0;
	}

void dataTransmitPrepare(void){
	 if (thr_init) tx_data[throttle] = constrain(map(throttle_val, stickLimits.thr_min, stickLimits.thr_max, 0, 255), 0,  255);
	 else tx_data[throttle] = 0;
	 
	 tx_data[roll] = roll_val;
	 tx_data[pitch] = pitch_val;
	 tx_data[buttons] = buttons_state;
	 tx_data[switcher] = switcher_state; 
	}
	
void dataSend(void){
	#ifdef DEBUG_NRF24	
	 unsigned long time = micros();
	#endif 
	 
	 if (radio.write(tx_data, PACKAGE_SIZE_TX)){
		 if(!radio.available()){ 
			 connection(true);
			 if (count_no_ACK < NO_ACK_MAX) count_no_ACK++;
			#ifdef DEBUG_NRF24
			 printf("Transmit OK. ACK response is blank. Round-trip delay: %lu us\n\r", micros()-time);
			 printf("Retransmit count: %d\n\r", radio.getARC());
			#endif 			 
			}
		 else{ 
			 connection(true);
             while(radio.available()){	
                 radio.read(rx_data, PACKAGE_SIZE_RX);
				 count_no_ACK = 0;	
				#ifdef DEBUG_NRF24				 
                 printf("Got response %d, %d, %d. Round-trip delay: %lu us\n\r", rx_data[0], rx_data[1], rx_data[2], micros()-time);
				 printf("Retransmit count: %d\n\r", radio.getARC());	
				#endif
				}
			} 
		}
	 else {
		#ifdef DEBUG_NRF24	
		 printf("Transmition FAULT\n");
		#endif
		 connection(false);
		 if (count_no_ACK < NO_ACK_MAX) count_no_ACK++;
		}
	}

void connection(bool state){
	 if (state) {
		 digitalWrite(CONNECT_OK_LED_PIN, LOW);
		 connectionOK = TRUE;
		}
	 else {
		 digitalWrite(CONNECT_OK_LED_PIN, HIGH);
		 connectionOK = FALSE;
		}
	}

void buttonsRead(void){
	 static uint8_t buttons_prev_state;//, hold;
	 uint8_t butt_curr_state = 0;
	 
	 if(digitalRead(LEFT_BUTT_1_PIN) == 0){
		 SetBit(butt_curr_state, LEFT_BUTT_1_BIT);
		 if (!BitIsSet(buttons_prev_state, LEFT_BUTT_1_BIT)) buzzerBeep(SHORT_BEEP);
    }	
	 if(digitalRead(RIGHT_BUTT_1_PIN) == 0){
		 SetBit(butt_curr_state, RIGHT_BUTT_1_BIT);
		 if (!BitIsSet(buttons_prev_state, RIGHT_BUTT_1_BIT)) buzzerBeep(SHORT_BEEP);
		}
	 if(digitalRead(LEFT_BUTT_2_PIN) == 0){
		 SetBit(butt_curr_state, LEFT_BUTT_2_BIT);
		 if (!BitIsSet(buttons_prev_state, LEFT_BUTT_2_BIT)) buzzerBeep(SHORT_BEEP);
		}
	 if(digitalRead(RIGHT_BUTT_2_PIN) == 0){
		 SetBit(butt_curr_state, RIGHT_BUTT_2_BIT);
		 if (!BitIsSet(buttons_prev_state, RIGHT_BUTT_2_BIT)) buzzerBeep(SHORT_BEEP);
		}
	 if(digitalRead(CENTER_BUTT_PIN) == 0){
		 SetBit(butt_curr_state, CENTER_BUTT_BIT);
		 if (!BitIsSet(buttons_prev_state, CENTER_BUTT_BIT)) buzzerBeep(SHORT_BEEP);
		}
	
	 if (buttons_prev_state != butt_curr_state) {
		 buttons_prev_state = butt_curr_state;
		 buttons_state = butt_curr_state;		 
		 click_detect = butt_curr_state;
		}
	
	 if (!digitalRead(SELECTOR_LEFT_PIN)) switcher_state = SELECTOR_LEFT;	 
	 if (digitalRead(SELECTOR_RIGHT_PIN) && digitalRead(SELECTOR_LEFT_PIN)) switcher_state = SELECTOR_CENTER; 
	 if (!digitalRead(SELECTOR_RIGHT_PIN)) switcher_state = SELECTOR_RIGHT;

	 count_butt = 0;
	}

static inline bool isButtClick (uint8_t bit){	 
	 if (BitIsSet(click_detect, bit)){
		 ClearBit(click_detect, bit);
		 return 1;
		}
	 return 0;	
	}

static inline void buzzer(bool state){
	 if (state == OFF) TCCR0A = 0b00000010;			// == (0 << COM0A0) | (1 << WGM01);
	 else TCCR0A = 0b01000010;						// == (1 << COM0A0) | (1 << WGM01);
	}

void buzzerBeep(uint8_t duration){
	 if (count_buzz >= buzzDuration) count_buzz = 0;	 
	 buzzer(ON);
	 buzzDuration = duration;
	}

void serviceMode(){    																		      // 
	 uint16_t vcc = 4200;  																	// 
	 uint8_t curr_selection;
	 
	 stickLimits.thr_max = 127;																//start values before adjustment
	 stickLimits.thr_min = 127;
	
	 while (1) {
		 buttonsRead();
		 switch (switcher_state){
			 case SELECTOR_LEFT:															//left position fo switcher
				 setAnalogMux(ADC_1V1);		
				 delayMicroseconds (50);
				  
				 if (isButtClick(LEFT_BUTT_1_BIT)) {  									
					 vcc = constrain(vcc - VCC_INC, 2000, 5500);								
					} 
				 if (isButtClick(RIGHT_BUTT_1_BIT)) {								
					 vcc = constrain(vcc + VCC_INC, 2000, 5500);								
					} 		
					
				 if (isButtClick(CENTER_BUTT_BIT)) {                         			
					 EEPROM.put(SYSTEM_DATA_ADDR, systemData);   									
					 digitalWrite(CONNECT_OK_LED_PIN, LOW);
					 delay(100);
					 digitalWrite(CONNECT_OK_LED_PIN, HIGH);
					}	
					
				 systemData.internal_ref = getVref(vcc);       										
				 
				 oled.clear();                                 									
				 oled.home();
				 oled.setScale(2);
				 oled.print(F("Vcc: "));
				 oled.println(vcc);
				 oled.print(F("Ref: "));
				 oled.println(systemData.internal_ref);
				 oled.setScale(1);
				 oled.print
				 (F(
				 "\n"
				 "Press OK to save"
				 ));
				 oled.setCursor(0,7);
				 oled.print("Press L/R butt to adj\n");				 
				break;	
				
			 case SELECTOR_CENTER:																	//center position fo switcher
				 ADCReadData();
				 delayMicroseconds (50);

				 if ((throttle_val) > stickLimits.thr_max) stickLimits.thr_max = throttle_val;
				 if ((throttle_val) < stickLimits.thr_min) stickLimits.thr_min = throttle_val;
				 
				 if ((pitch_val) > stickLimits.pitch_max) stickLimits.pitch_max = pitch_val;
				 if ((pitch_val) < stickLimits.pitch_min) stickLimits.pitch_min = pitch_val;
				 
				 if ((roll_val) > stickLimits.roll_max) stickLimits.roll_max = roll_val;
				 if ((roll_val) < stickLimits.roll_min) stickLimits.roll_min = roll_val;

				 
				 if (isButtClick(CENTER_BUTT_BIT)) { 
					 EEPROM.put(CONFIG_DATA_ADDR, stickLimits); 	
					 digitalWrite(CONNECT_OK_LED_PIN, LOW);
					 delay(100);
					 digitalWrite(CONNECT_OK_LED_PIN, HIGH);
					}
					
				 if (isButtClick(LEFT_BUTT_1_BIT)){ 
					 stickLimits.thr_max = 127;															//start values before adjustment
					 stickLimits.thr_min = 127;
					 
					 stickLimits.roll_max = 127;														//start values before adjustment
					 stickLimits.roll_max = 127;					 
					 
					 stickLimits.pitch_max = 127;														//start values before adjustment
					 stickLimits.pitch_max = 127;					 				 
					}
				 
				 oled.clear(); 
				 oled.setScale(1);
				 
				 oled.setCursor(0,0);				 
				 oled.print(F("T:"));
				 oled.println(throttle_val);
				 
				 oled.setCursor(48,0);				 
				 oled.print(F("P:"));
				 oled.println(pitch_val);

				 oled.setCursor(98,0);				 
				 oled.print(F("R:"));
				 oled.println(roll_val);
				 
				 oled.setCursor(35,1);
				 oled.print("MIN");				 
				 oled.setCursor(75,1);
				 oled.print("MAX");
				 
				 oled.setCursor(0,2);
				 oled.print("THR:");
				 oled.setCursor(0,3);
				 oled.print("PIT:");				 
				 oled.setCursor(0,4);
				 oled.print("ROL:");

				 oled.setCursor(35,2);
				 oled.print(stickLimits.thr_min);				 
				 oled.setCursor(75,2);
				 oled.print(stickLimits.thr_max);
				 
				 oled.setCursor(35,3);
				 oled.print(stickLimits.pitch_min);				 
				 oled.setCursor(75,3);
				 oled.print(stickLimits.pitch_max);
				 
				 oled.setCursor(35,4);
				 oled.print(stickLimits.roll_min);				 
				 oled.setCursor(75,4);
				 oled.print(stickLimits.roll_max);				 

				 oled.setCursor(0,6);
				 oled.print("Left butt - reset");
				 oled.setCursor(0,7);
				 oled.print("Center butt - save");
				break;	
				
			 case SELECTOR_RIGHT:																		 //right position of switcher				 
				 if (isButtClick(LEFT_BUTT_1_BIT)) { 
					 systemData.brightness = constrain(systemData.brightness - BRIGHT_INC, 0, 255);		
					 oled.setContrast(systemData.brightness);					 
					} 
				 if (isButtClick(RIGHT_BUTT_1_BIT)) {								
					 systemData.brightness = constrain(systemData.brightness + BRIGHT_INC, 0, 255);	
					 oled.setContrast(systemData.brightness);					 
					} 
          if (isButtClick(CENTER_BUTT_BIT)) {                         			
					 EEPROM.put(SYSTEM_DATA_ADDR, systemData); 
					 digitalWrite(CONNECT_OK_LED_PIN, LOW);
					 delay(100);
					 digitalWrite(CONNECT_OK_LED_PIN, HIGH);
					}					 
				 
				 oled.clear(); 
				 oled.setScale(2);
				 
				 oled.setCursor(0,0);				 
				 oled.print(F("BRIGHT:"));
				 oled.println(systemData.brightness);
				 
				 oled.setScale(1);
				 oled.setCursor(0,7);
				 oled.print("Center butt - save");				 
				break;
			}
		 oled.update();
		}
	}

ISR (TIMER2_COMPA_vect){
	 if (count_to_tx < TX_INT) count_to_tx++;
	 if (count_vcc < VCC_INT) count_vcc++;
	 if (count_disp < DISP_UPD_INT) count_disp++;
	 if (count_butt < BUTT_INT) count_butt++;
	 
	 if (count_buzz < buzzDuration) count_buzz++;
	 else buzzer(OFF);
	 
	 if (count_first_on < FIRST_ON_DELAY) count_first_on++;
	}
	
void setup(){
	#ifdef DEBUG || DEBUG_NRF24
     Serial.begin(115200); 																
	 printf_begin();	
	#endif

	 EEPROM.get(SYSTEM_DATA_ADDR, systemData);  			//get settings from EEPROM
	 EEPROM.get(CONFIG_DATA_ADDR, stickLimits); 
	 	
	 pin_init();
	 NRF24_init();											//
	 timers_init();
	 _ADCInit();
	 oledEnable();
	 
	 tx_data[roll] = 127;									// center position of roll-pitch stick
	 tx_data[pitch] = 127;
	 
	 charge = BAT_FULL;
	 
	 sei();
	 
	 buzzerBeep(SHORT_BEEP);
	 
	 if (!digitalRead(CENTER_BUTT_PIN)) serviceMode();  	// if button pressed enter service mode
	}	
	
void loop(void) {
	 if (!thr_init) throttle_init();						//throttle stick data will be accepted only after init procedure: thr stick full UP -> stick full DOWN (safety)

	 if (count_to_tx >= TX_INT){
		 
		#ifdef DEBUG  
		 loop_time = micros();
		#endif
		 
		 dataTransmitPrepare();
		 if(count_first_on >= FIRST_ON_DELAY) dataSend();
		  
		 count_to_tx = 0;	 
		 
		#ifdef DEBUG 
//		 printf("TX loop time: %lu us\n\r", micros() - loop_time);
		#endif 
		}
		
	 ADCReadData();
	 
	 if(count_first_on >= FIRST_ON_DELAY){
		 if (count_disp >= DISP_UPD_INT) displayUpdate();
		 if (count_butt >= BUTT_INT) buttonsRead();	 
		}
	}
















	
	