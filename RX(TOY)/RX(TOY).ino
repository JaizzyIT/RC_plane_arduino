//#define DEBUG
//#define DEBUG_NRF24
//#define FLAPS
//#define LIGHT

#include <directADC.h>
#include <EEPROM.h>
#include <SPI.h>
#include <bits_macros.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define LED_PIN 2
#define MOTORS_PIN 3
#define SERVO_1_ROLL_PIN 9
#define SERVO_2_ROLL_PIN 10 
#define SERVO_3_PITCH_PIN 5
#define BATTERY_AI ADC_A3

#define MOTORS_PWM OCR0B
#define SERVO_1_ROLL_PWM OCR1A					// left servo
#define SERVO_2_ROLL_PWM OCR1B					// right servo
#define SERVO_3_PITCH_PWM OCR3A					// pitch servo

//#define VCC_INT_REF 1080UL  
//#define VCC_CALC_CONST (VCC_INT_REF*1024UL)

#define VCC_PWR 5150                    // 5.15 V = 5150 mV
#define VCC_COEF (VCC_PWR/1023.0)

#define NO_DATA_RECEIVED_COUNT 122

#define ADC_INT 244								      // timer4 interrupt occur with 122 Hz -> ADC_INT/122 = ADC read freq
#define AUTO_SAVE_INT 2440

#define BAT_LOW 3300     
#define BAT_CRITICAL 3100
#define BAT_FULL 4100
 
#define THROTTLE_MIN_SENSE 13					  // 0-255

#define ON 1
#define OFF 0
#define TRUE 1
#define FALSE 0

/*values in discretes
999-4999 discretes = duty cycle = 0.5ms - 2.5ms = 0-180 degrees, PWM carrier frequency = 50 Hz
96 discretets - middle position*/
#define SERVO_ROLL_MIN_POS 1999
#define SERVO_ROLL_MAX_POS 3999
#define SERVO_PITCH_MIN_POS 1999
#define SERVO_PITCH_MAX_POS 3999

#define SERVO_ROLL_FLAPS_END_1_POS 500
#define SERVO_ROLL_FLAPS_END_2_POS 1000
#define SERVO_PITCH_FLAPS_1_POS 220
#define SERVO_PITCH_FLAPS_2_POS 380

#define ROLL_IDLE_VALUE 3300				            // 
#define PITCH_IDLE_VALUE 2550				          // 
#define THR_IDLE_POS 0                      // 0-255

#define PITCH_IDLE_STEP 50
#define ROLL_IDLE_STEP 50
#define THR_IDLE_STEP 5

#define PITCH_TRIM_SCALE 15						// -15...0...+15
#define ROLL_TRIM_SCALE 15						// -15...0...+15
#define TRIM_PITCH 0
#define TRIM_ROLL 1

#define SYSTEM_DATA_ADDR 5 
#define CONFIG_DATA_ADDR 10

#define BUTT_1_BIT 0							// left 1 button
#define BUTT_2_BIT 1  							// right 1 button
#define BUTT_3_BIT 2							// left 2 button
#define BUTT_4_BIT 3  							// right 2 button 
#define BUTT_5_BIT 4 							// center button

#define FLAPS_SET_FULL_UP 0
#define FLAPS_SET_POS_1 1
#define FLAPS_SET_POS_2 2
#define FLAPS_SET_NO_FLAPS 3

#define SELECTOR_LEFT 1							//
#define SELECTOR_CENTER 2						//
#define SELECTOR_RIGHT 3						//

#define PACKAGE_SIZE_RX 5
#define PACKAGE_SIZE_TX 2

#define TRIM_DATA_ADDR 5


RF24 radio(19,18);	

//--------------------0-------1------2-------3--------4--------5--------6------
enum RX_data_pack {throttle, roll, pitch, buttons, switcher};
//----------------------0------------1----------2------
enum TX_data_pack {battery, flaps_trim_fb};

uint8_t address[][2] = {"1Node","2Node"};											// Radio pipe addresses for the 2 nodes to communicate.

int8_t pitch_trim, roll_trim;
uint8_t tx_data[PACKAGE_SIZE_TX];
uint8_t rx_data[PACKAGE_SIZE_RX];

uint8_t flaps_set;

uint8_t no_data_packs_count;
uint8_t count_ADC;
uint8_t idle_thr_curr_pos;
uint16_t idle_pitch_curr_pos, idle_roll_curr_pos;

uint16_t count_EEPROM_auto_save;
uint16_t charge;
uint16_t flaps_dest_pos, flaps_curr_pos, pitch_flaps_dest_pos, pitch_flaps_curr_pos;
uint16_t roll_min, roll_max, pitch_min, pitch_max;

bool trim_select;
bool need_save_flag;
bool idle_state;

struct {																			          //trim data structure
  int8_t p_trim;
  int8_t r_trim;																	// 
} trimData;

void pin_init(void){
	 pinMode(MOTORS_PIN, OUTPUT);	 
	 pinMode(SERVO_1_ROLL_PIN, OUTPUT);
	#ifdef FLAPS
	 pinMode(SERVO_2_ROLL_PIN, OUTPUT); 
	#endif 
	 pinMode(SERVO_3_PITCH_PIN, OUTPUT);
	#ifdef LIGHT
	 pinMode(LED_PIN, OUTPUT);
	 digitalWrite(LED_PIN, HIGH);
	#endif	 
	}
	
void NRF24_init(void){																  // Setup and configure radio
	#ifdef DEBUG_NRF24
	 if (radio.begin()) printf("NRF24 init OK\n"); 
	 else printf("NRF24 init FAULT\n");
	#else 
	 radio.begin();
	#endif
	
	 radio.setAutoAck(1);                    			 	// Ensure autoACK is enabled
	 radio.enableAckPayload();               				// Allow optional ack payloads
	 radio.setRetries(7,15);                 				// Smallest time between retries, max no. of retries
	 radio.enableDynamicPayloads();
	 
	 radio.setPALevel(RF24_PA_MAX); 	 
	 radio.setDataRate(RF24_250KBPS); 
	 radio.setChannel(0x00);  
	 
	//---to write data
    // radio.openWritingPipe(address[0]);       			// Open different pipes when writing. Write on pipe 0, address 0
    // radio.openReadingPipe(1,address[1]);     			// Read on pipe 1, as address 1 
	// radio.stopListening();                 				// Stop listening
	 
	//---to read data
     radio.openWritingPipe(address[1]);						// Since only two radios involved, both listen on the same addresses and pipe numbers in RX mode
     radio.openReadingPipe(1,address[0]);					// then switch pipes & addresses to transmit. 
     radio.startListening();								// Need to start listening after opening new reading pipes	
	
	 radio.powerUp();
	 
	#ifdef DEBUG_NRF24
	 radio.printDetails();                   				// Dump the configuration of the rf unit for debugging		
	#endif
	 
	 delay(5);
	}

void _ADCInit(void){         
          															
	 ADC_enable();			
	 ADC_setPrescaler(8);			
	 ADC_setReference(ADC_VCC);

/*	 ADMUX &= ~(1 << MUX0);															//ADC_1V1 - internal Vref 
   ADMUX |= ((1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1));	*/

   ADMUX |= (1 << MUX2);

	 ADC_setResolution(10);			
	 delayMicroseconds(100);         															
	 ADC_startConvert();		
	}	

void timers_init (void){
	 //-- timer 0 - fast PWM, 8 bit, prescaler = 256 (~305 Hz) - motors
	 TCCR0A = 0b00000011;											 
	 TCCR0B = 0b00000011;
	 
	 //-- timer 1 - fast PWM, 10 bit, prescaler = 8 (50 Hz), TOP = ICR1 - servo 1, servo 2
	#ifdef FLAPS
	 TCCR1A = 0b10100010;                                            
	 TCCR1B = 0b00011010;
	 ICR1 = 39999;
	#else
	 TCCR1A = 0b10000010;                                            
	 TCCR1B = 0b00011010;
	 ICR1 = 39999;
	#endif
	
	 //timer 3 - fast PWM, 10 bit, prescaler = 8 (50 Hz), TOP = ICR1 - servo 3
	 TCCR3A = 0b10000010;                                            
	 TCCR3B = 0b00011010;
	 ICR3 = 39999;
	 
	 //timer 4 - fast PWM, 10 bit, prescaler = 128 (~122 Hz) - timer with interrupts for general purpose
	 TCCR4A = 0b00000000;                                            
	 TCCR4B = 0b00001000;
	 TIMSK4 = 0b00000100;	 
	}

void servo_sense_calc(void){														// calculating sense level of servos (to use full scale use divider == 1)
	 uint8_t divider;
	 
	 if(rx_data[switcher] == SELECTOR_LEFT) divider = 8;                  // 3/4 sense scale
	 else if(rx_data[switcher] == SELECTOR_CENTER) divider = 4;						// 1/2 sense scale			
	 else if(rx_data[switcher] == SELECTOR_RIGHT) divider = 3;						// 1/3 sense scale
	
	 roll_min = SERVO_ROLL_MIN_POS + ((SERVO_ROLL_MAX_POS - SERVO_ROLL_MIN_POS) / divider) + (roll_trim << 5);					
	 roll_max = SERVO_ROLL_MAX_POS - ((SERVO_ROLL_MAX_POS - SERVO_ROLL_MIN_POS) / divider) + (roll_trim << 5);	 
	 pitch_min = SERVO_PITCH_MIN_POS + ((SERVO_PITCH_MAX_POS - SERVO_PITCH_MIN_POS) / divider) + (pitch_trim << 5);
	 pitch_max = SERVO_PITCH_MAX_POS - ((SERVO_PITCH_MAX_POS - SERVO_PITCH_MIN_POS) / divider) + (pitch_trim << 5);	
	}	

uint16_t getVcc(void){             											// 
	 static uint16_t buf[4];// = {1023, 1023, 1023, 1023};
	 static uint8_t i = 0, j=0;

	 buf[i] = ADC_read();

	 if (i < 3) i++;
	 else i = 0;
	
	 ADC_startConvert();	 

	 return (uint16_t)(((buf[0] + buf[1] + buf[2] + buf[3]) >> 2) * VCC_COEF);
//	 return (uint16_t)(VCC_CALC_CONST / ((buf[0] + buf[1] + buf[2] + buf[3]) >> 2));
	}

void dataTransmitPrepare(void){
	 tx_data[battery] = (charge / 1000) << 4;
	 tx_data[battery] |= (charge %1000) / 100;	 
	 
	 if (trim_select == TRIM_PITCH) tx_data[flaps_trim_fb] = (TRIM_PITCH << 7) | (constrain((PITCH_TRIM_SCALE + pitch_trim), 0, 30) << 2) | flaps_set;
	 else tx_data[flaps_trim_fb] = (TRIM_ROLL << 7) | (constrain((ROLL_TRIM_SCALE + roll_trim), 0, 30) << 2) | flaps_set; 
	}

void buttonsProcessing(void){
	 static uint8_t buttons_prev_state, switcher_prev_state;

	 //--- buttons state
	 if (buttons_prev_state != rx_data[buttons]){
		#ifdef FLAPS
		 if (BitIsSet(rx_data[buttons], BUTT_2_BIT)){								// button 2
		 	 if (flaps_set < FLAPS_SET_POS_2) flaps_set++;			
			 servo_sense_calc();
		 	}			
		 if (BitIsSet(rx_data[buttons], BUTT_1_BIT)){								// button 1
		 	 if (flaps_set > FLAPS_SET_FULL_UP) flaps_set--;
			 servo_sense_calc();			 
		 	}			
		#endif
		#ifdef LIGHT
		 digitalWrite(LED_PIN, HIGH);
		 if (BitIsSet(rx_data[buttons], BUTT_2_BIT)){								// button 2
		 	 digitalWrite(LED_PIN, LOW);				 
		 	}
		#endif			
		 if (BitIsSet(rx_data[buttons], BUTT_3_BIT)){								// button 4
		 	 if (trim_select == TRIM_PITCH && pitch_trim < PITCH_TRIM_SCALE) {			
				 pitch_trim++;			
				 servo_sense_calc(); 			
				}	
			 else if (trim_select == TRIM_ROLL && roll_trim > -(ROLL_TRIM_SCALE)) {
				 roll_trim--;				 
				 servo_sense_calc(); 
				}			 
		 	}			
		 if (BitIsSet(rx_data[buttons], BUTT_4_BIT)){								// button 3 
			 if (trim_select == TRIM_PITCH && pitch_trim > -(PITCH_TRIM_SCALE)) {
				 pitch_trim--;				 
				 servo_sense_calc(); 
				}
			 else if (trim_select == TRIM_ROLL && roll_trim < ROLL_TRIM_SCALE) {
				 roll_trim++;			
				 servo_sense_calc();
				}					
		 	}	
		 if (BitIsSet(rx_data[buttons], BUTT_5_BIT)){								// button 5 (center/OK)
		 	 trim_select = !trim_select;
		 	}			
		 if ((BitIsSet(rx_data[buttons], BUTT_3_BIT)) && (BitIsSet(rx_data[buttons], BUTT_4_BIT))) {		// button 3 & 4 - trim reset
		 	 pitch_trim = 0;
			 roll_trim = 0;
		 	}					
			
		 buttons_prev_state = rx_data[buttons];	 	
		}
 
	 //--- switcher state
	 if(switcher_prev_state != rx_data[switcher]){
		 servo_sense_calc(); 
		 switcher_prev_state = rx_data[switcher];
		}	 
	}

void receivedDataProcessing(void){
	 //--- throttle parameters
	 if ((rx_data[throttle] > THROTTLE_MIN_SENSE) && (charge > BAT_CRITICAL)) {
		 TCCR0A = 0b00100011;
		 MOTORS_PWM = rx_data[throttle];
		}
	 else {
		 MOTORS_PWM = 0;
		 TCCR0A = 0b00000011;
		}
	
	 trimData.p_trim = pitch_trim;
	 trimData.r_trim = roll_trim;
	 
	 buttonsProcessing();	
	 
	 if ((trimData.p_trim != pitch_trim) || (trimData.r_trim != roll_trim)) need_save_flag = true;
	 
	 //--- servo positions	
	#ifdef FLAPS 
	 switch (flaps_set) {																//defining wich pos for flaps is need
		 case FLAPS_SET_FULL_UP:
		 	 flaps_dest_pos = 0;
			 pitch_flaps_dest_pos = 0;
		 	break;
		 case FLAPS_SET_POS_1:
		 	 flaps_dest_pos = SERVO_ROLL_FLAPS_END_1_POS;
			 pitch_flaps_dest_pos = SERVO_PITCH_FLAPS_1_POS;
			 break;
		 case FLAPS_SET_POS_2:
		 	 flaps_dest_pos = SERVO_ROLL_FLAPS_END_2_POS;
			 pitch_flaps_dest_pos = SERVO_PITCH_FLAPS_2_POS;
			break;
		}
	#endif  
	 
	 SERVO_1_ROLL_PWM = constrain((map(rx_data[roll], 0, 255, roll_min, roll_max) + flaps_curr_pos), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
   idle_roll_curr_pos = SERVO_1_ROLL_PWM;
	#ifdef FLAPS
	 SERVO_2_ROLL_PWM = constrain((map(rx_data[roll], 0, 255, roll_min, roll_max) - flaps_curr_pos), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
	#endif 
	 SERVO_3_PITCH_PWM = constrain((map(rx_data[pitch], 0, 255, pitch_min, pitch_max) + pitch_flaps_curr_pos), SERVO_PITCH_MIN_POS, SERVO_PITCH_MAX_POS);
   idle_pitch_curr_pos = SERVO_3_PITCH_PWM;

   idle_thr_curr_pos = rx_data[throttle];
	}

void idle_output_values(void){ 

	 MOTORS_PWM = idle_thr_curr_pos;
   if (idle_thr_curr_pos == THR_IDLE_POS){
	   TCCR0A = 0b00000011;
    }

	 SERVO_1_ROLL_PWM = idle_roll_curr_pos;
	#ifdef FLAPS
	 SERVO_2_ROLL_PWM = idle_roll_curr_pos;
	#endif
	 SERVO_3_PITCH_PWM = idle_pitch_curr_pos;	 

   #ifdef FLAPS
	 flaps_set = FLAPS_SET_FULL_UP;
   #endif
	}	

ISR (TIMER4_OVF_vect){
	 static uint16_t count_flaps, count_idle;
	 
	#ifdef FLAPS
	 if (count_flaps >= 10){
		 count_flaps = 0;
		 if (flaps_dest_pos != flaps_curr_pos){
			 if (flaps_dest_pos > flaps_curr_pos) flaps_curr_pos = flaps_curr_pos + 20;
			 else flaps_curr_pos = flaps_curr_pos - 20;
			}
		 if (pitch_flaps_dest_pos != pitch_flaps_curr_pos){
			 if (pitch_flaps_dest_pos > pitch_flaps_curr_pos) pitch_flaps_curr_pos = pitch_flaps_curr_pos + 10;
			 else pitch_flaps_curr_pos = pitch_flaps_curr_pos - 10;
			}			 
		}
	 else count_flaps++;
	#endif

 if (idle_state){
   if (count_idle >= 10){
  	 count_idle = 0;
  	 if (PITCH_IDLE_VALUE != idle_pitch_curr_pos){
  		 if (PITCH_IDLE_VALUE > idle_pitch_curr_pos) {
         idle_pitch_curr_pos = idle_pitch_curr_pos + PITCH_IDLE_STEP;
         if (PITCH_IDLE_VALUE < idle_pitch_curr_pos) idle_pitch_curr_pos = PITCH_IDLE_VALUE;
        }
  		 else {
         idle_pitch_curr_pos = idle_pitch_curr_pos - PITCH_IDLE_STEP;
         if (PITCH_IDLE_VALUE > idle_pitch_curr_pos) idle_pitch_curr_pos = PITCH_IDLE_VALUE;
        }
  		}	
  	 if (ROLL_IDLE_VALUE != idle_roll_curr_pos){
  		 if (ROLL_IDLE_VALUE > idle_roll_curr_pos) {
         idle_roll_curr_pos = idle_roll_curr_pos + ROLL_IDLE_STEP;
         if (ROLL_IDLE_VALUE < idle_roll_curr_pos) idle_roll_curr_pos = ROLL_IDLE_VALUE;
        }
  		 else {
         idle_roll_curr_pos = idle_roll_curr_pos - ROLL_IDLE_STEP;
         if (ROLL_IDLE_VALUE > idle_roll_curr_pos) idle_roll_curr_pos = ROLL_IDLE_VALUE;
        }
  		}	      
  	 if (THR_IDLE_POS != idle_thr_curr_pos){
       if (idle_thr_curr_pos >= THR_IDLE_STEP) idle_thr_curr_pos = idle_thr_curr_pos - THR_IDLE_STEP;
       else idle_thr_curr_pos = THR_IDLE_POS;
  		}		        
  	}
   else count_idle++;
  }

 if (count_ADC < ADC_INT) count_ADC++;
 if (no_data_packs_count < NO_DATA_RECEIVED_COUNT) no_data_packs_count++;
 if ((count_EEPROM_auto_save < AUTO_SAVE_INT) && need_save_flag) count_EEPROM_auto_save++;	


}
	
void setup(){
	#ifdef DEBUG
     Serial.begin(115200); 	
	 printf_begin();
	#endif
	
	 pin_init();	
	 NRF24_init();	
	 _ADCInit(); 
	 timers_init();
	 
	 EEPROM.get(TRIM_DATA_ADDR, trimData);  									//get trim settings from EEPROM
	 pitch_trim = trimData.p_trim;
	 roll_trim = trimData.r_trim;

   idle_pitch_curr_pos = PITCH_IDLE_VALUE;
   idle_roll_curr_pos = ROLL_IDLE_VALUE;
   idle_thr_curr_pos = THR_IDLE_POS;

	 idle_output_values();

	 servo_sense_calc();

   #ifndef FLAPS
   flaps_set = FLAPS_SET_NO_FLAPS;
	 #endif

	 sei();	 
	}
	
void loop(void) {
	 byte pipeNo;                          											  // Declare variable for the pipe
    if(radio.available(&pipeNo)){              									// Read all available payloads
		 radio.read(rx_data, PACKAGE_SIZE_RX);          
		 
		 /* Since this is a call-response. Respond directly with an ack payload.
		 Ack payloads are much more efficient than switching to transmit mode to respond to a call*/		 	
		 if (radio.writeAckPayload(pipeNo, &tx_data, PACKAGE_SIZE_TX)) {
			#ifdef DEBUG_NRF24 
			 printf("ACK data load OK\n");
			#endif		
			}
		 else {
			#ifdef DEBUG_NRF24 
			 Serial.print("ACK data load FAULT\n");
			#endif
			 radio.flush_tx(); 
			}
		 
		#ifdef DEBUG_NRF24
		 printf("Got data: %d, %d, %d, %d, %d\n\r", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4]); 
		 printf("Sent response %d, %d \n\n\r", tx_data[0], tx_data[1]); 
		#endif 
		
		 dataTransmitPrepare();
		 receivedDataProcessing();
		 
		 no_data_packs_count = 0;
		}

	 if ((count_ADC >= ADC_INT) && ADC_available()) {
     charge = getVcc();
     count_ADC = 0;
    }

	 if ((no_data_packs_count >= NO_DATA_RECEIVED_COUNT) || (charge <= BAT_CRITICAL)) {
     idle_state = TRUE;
     idle_output_values();   
    }
   else idle_state = FALSE;

	 if ((count_EEPROM_auto_save >= AUTO_SAVE_INT) && need_save_flag) {
		 EEPROM.put(TRIM_DATA_ADDR, trimData);
		 need_save_flag = false;
		 count_EEPROM_auto_save = 0;
		}
	}