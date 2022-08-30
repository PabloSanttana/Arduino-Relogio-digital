
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1


#define clr_bit(Y,bit_x) (Y &=~(1 << bit_x)) 
#define set_bit(Y,bit_x) (Y |= (1 << bit_x)) 
#define cpl_bit(Y,bit_x) (Y ^= (1<<bit_x))

#define AREF 0 // Tensão de Referencia = Aref
#define AVCC 1 // Tensão de Referencia = Avcc
#define VR11 2 // Tensão de Referencia = 1,1 V
#define ADC0 0 // Selecionar a entrada ADC0
#define ADC1 1 // Selecionar a entrada ADC1
#define ADC2 2 // Selecionar a entrada ADC2
#define ADC3 3 // Selecionar a entrada ADC3
#define ADC4 4 // Selecionar a entrada ADC4
#define ADC5 5 // Selecionar a entrada ADC5
#define Vtemp 6// Selecionar o Sensor de Temperatura interna
#define V11 7 // Selecionar a tensão(1,1 v)
#define Vgnd 8 // Selecionar a tensão GND (0v)


#define DG1 PD2
#define DG2 PD3
#define DG3 PD4
#define DG4 PD5
#define DG5 PC1
#define DG6 PC0
#define OnAlram PC3

//Estados do relogio
volatile int clock_digits[6]={0};
volatile bool clock_set_hours=false;

//Estados do alarme
volatile int alarm_digits[4]={0};
volatile bool alarm_set=false;
volatile bool alarm_saved=false;
volatile bool alarm_on=false;
volatile bool alarm_stop=false;

volatile bool alarm_toview=false;
volatile unsigned int toviewCount =0;


volatile bool set_config_hours_minutes=false;



volatile unsigned int TIMER = 0;
volatile unsigned int segundos = 100;
volatile unsigned int SETDISPLAY = 0;
volatile unsigned int FPSDISPLAY = 10;
volatile bool INITIAL = true;



void UART_Init(void){
  UBRR0H = (uint8_t) (MYUBRR >> 8); // ajusta a taxa de transmissão
  UBRR0L = (uint8_t) (MYUBRR);
  UCSR0A = 0; // desabilita velocidade dupla
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);// habilita o transmissor e o receptor
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // ajusta o formato do frame 
 // 8 bits de dados 1 bit de parada  
}


//verifica se novo dado pode ser enviado pela UART
//Retorna valor 32 se novo dado pode ser enviado ou zero caso não
uint8_t uartTxOk(void){
 return (UCSR0A & (1 << UDRE0)); 
}

//Envia um byte pela pota UART
void uart_Transmit(uint8_t data){
  UDR0 = data; // coloca o dad no registrador de transmissão e o envia
}

//Envia uma string pela porta UART
void uartString (char *c){
  for(;*c!=0; c++){
    while(!uartTxOk());
    uart_Transmit(*c);
  }
}

//Verifica se UART possui novo dado
//Retorna valor 128 se existir novo dado recebido. Zerpo se não
uint8_t uartRx0k(void){
 return (UCSR0A & (1<< RXC0));
}

// ler byte recebido na porta UART
uint8_t uartRx(void){
 return UDR0;
}

//habilitar ou desabilitar a interrupção de recepção da USART
// x = 0, desabilita, qualquer outro valor, habilita a interrupçao

void uartIntRx(uint8_t _hab){
  if(_hab){
   UCSR0B |= ( 1 << RXCIE0); // habilitar a interrupção de recep 
  }else{
    UCSR0B &=~ ( 1 << RXCIE0); // desabilita a interrupção de recep 
  }
}

//habilitar ou desabilitar a interrupção de transmissão da USART
// x = 0, desabilita, qualquer outro valor, habilita a interrupçao
void uartIntTx(uint8_t _hab){
  if(_hab){
   UCSR0B |= ( 1 << TXCIE0); // habilitar a interrupção de recep 
  }else{
    UCSR0B &=~ ( 1 << TXCIE0); // desabilita a interrupção de recep 
  }
}

//----------------------------------------------------------------------------------
//Configura o conversor ADC 
// ref = 0. Para usar a tensão de referencoa Aref
//ref = 1. Para usar a tensão de referencia Avcc - lembre-se do capacitor 100nf
//ref = 2. Para usar a tensão de referencia interna de 1,1 V
// did: valor para o registrador DIDR0

void adcBegin(uint8_t ref,uint8_t did){
  ADCSRA = 0; // configuração inicial
  ADCSRB = 0; // configurção inicial
  DIDR0 = did; // valor do did
  
  if(ref ==0){
  	ADMUX &=~((1<<REFS1) | (1<<REFS0)); //Aref
  }
  if((ref ==1) || (ref > 2)){
   	 ADMUX &=~(1<<REFS1); //Avcc
     ADMUX |=(1<<REFS0);//Avcc
  }
  if(ref ==2){
   	ADMUX |=((1<<REFS1) | (1<<REFS0)); //tensão interna 1.1v
  }
  
  ADMUX &=~(1<<ADLAR);// alinhamento a direita
  ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  // habilitar AD. Prescaler de 128 (clk_AD = F_cpu/128)
}

//-----------------------------------------------------------------------------
// Selecionar canal do ADC
// 0 <= channel <= 5 - ler dos pinos AD0 a AD5
// channel = 6 - leitura do sensir de temperatura 
// channel = 7 - 1,1v
// channel > 7 - GND
void adcChannel(uint8_t canal){
  if(canal <= 5){ // selescionar o canal no multiplex
   ADMUX = (ADMUX & 0xF0) | canal;
  }
  else if(canal == 6){// sensor de temperatura interno
     ADMUX = (ADMUX & 0xF0) | 0x08 ; 
  }
  else if(canal == 7){ // seleciona 1.1v
     ADMUX = (ADMUX & 0xF0) | 0x0E ; 
  }
  else if(canal > 7){// seleciona GND
      ADMUX = (ADMUX & 0xF0) | 0x0F ; 
  }

} 
 
//----------------------------------------------------------------------------
// iniciar a conversão
void adcSample(void){
  ADCSRA |= (1<<ADSC); // iniciar a conversão
}

//------------------------------------------------------
//verificar se a conversão foi concluida
//Retorna valor 0 se conversão concluida. 64 se não.

uint8_t adcOk(void){
 return (ADCSRA & (1<<ADSC)); 
}

//----------------------------------------------------------
//ler o ADC e retorna o Valor lido do ADC
uint16_t adcReadOnly(){
  return ((ADCH<<8) | ADCL); // retorna o valor do ADC
}

//------------------------------------------------------------
//converte, aguarda, ler e retorna valor lido do ADC
uint16_t adcRead(){
  adcSample(); // iniciar a conversão
  while(adcOk()); // aguardando a finalização da conversão ADSC =0
  return adcReadOnly(); // retorna o valor do ADC
}

//-------------------------------------------------------------------------
//habilitar ou desabilitar interrupções do ADC
// se X =0, desabilita interuupçao
// Caso contrario, habilitada
void adcIntEn(uint8_t x){
  if(x){
    ADCSRA |= (1<<ADIE); //habilitar interrupçao do ADC
  }
  else{
    ADCSRA &=~(1<<ADIE);
  }
}




//---------------------------------------------------------------------------
//Envia pela uart variavel de 2 bytes (16 bits) com digitos em decimal
void uartDec2B(uint16_t valor){ 
  int8_t disp;
  byte digitos[5]={0};
  int8_t conta = 0;
  
  //horas
  if(set_config_hours_minutes){
  	 valor = (23*valor)/1023;
   //minutos
  }else{
    valor = (59*valor)/1023;
  }
  
	do //converte o valor armazenando os algarismos no vetor digitos
	{ disp = (valor%10);//armazena o resto da divisao por 10 e soma 
		valor /= 10;
		digitos[conta]=disp;
		conta++;
	} while (valor!=0);
  
  if(clock_set_hours){
    if(set_config_hours_minutes){
      clock_digits[0] = digitos[1];
      clock_digits[1] = digitos[0];
    }else{
      clock_digits[2] = digitos[1];
      clock_digits[3] = digitos[0];
    }
  
  }else if(alarm_set){
     if(set_config_hours_minutes){
      alarm_digits[0] = digitos[1];
      alarm_digits[1] = digitos[0];
    }else{
      alarm_digits[2] = digitos[1];
      alarm_digits[3] = digitos[0];
    }
    
  }
 

}


void hanldeSetDisplay(){
  
  if(SETDISPLAY == (1*FPSDISPLAY)){
   set_bit(PORTC,DG6); 
    if(alarm_toview || alarm_set ){
    	PORTB = alarm_digits[0];
    }else{
     PORTB =clock_digits[0];
    }
  
   clr_bit(PORTD,DG1);   
  }
  else if(SETDISPLAY == (5*FPSDISPLAY)){
   set_bit(PORTD,DG1);   // desligar  o digitos display anterior
   if(alarm_toview || alarm_set ){
    	PORTB = alarm_digits[1];
    }else{
     PORTB =clock_digits[1];
    }
   clr_bit(PORTD,DG2);
  }
  else if(SETDISPLAY==(10*FPSDISPLAY)){
   set_bit(PORTD,DG2);   
   if(alarm_toview || alarm_set ){
    	PORTB = alarm_digits[2];
    }else{
     PORTB =clock_digits[2];
    }
   clr_bit(PORTD,DG3);
  }
  else if(SETDISPLAY == (15*FPSDISPLAY)){
   set_bit(PORTD,DG3);   // desligar todos os digitos
   if(alarm_toview || alarm_set ){
    	PORTB = alarm_digits[3];
    }else{
     PORTB =clock_digits[3];
    }
   clr_bit(PORTD,DG4);
  }
  else if(SETDISPLAY==(20*FPSDISPLAY)){
   set_bit(PORTD,DG4);   // desligar todos os digitos
    if(clock_set_hours|| alarm_set || alarm_toview){
      PORTB = 11;
    }else{
     PORTB =clock_digits[4];
    }
   clr_bit(PORTC,DG5);
  }
  else if(SETDISPLAY==(25*FPSDISPLAY)){
   set_bit(PORTC,DG5);   // desligar todos os digitos
    if(clock_set_hours|| alarm_set || alarm_toview){
      PORTB = 11;
    }else{
     PORTB =clock_digits[5];
    }
   clr_bit(PORTC,DG6);
  }
  
   else if(SETDISPLAY==(29*FPSDISPLAY)){
   SETDISPLAY=0;
  } 
  
}

void calculatedigits(){
     if(TIMER >= segundos){
            TIMER =0;
            clock_digits[5]++;
            if(clock_digits[5]>=10){
                clock_digits[5] =0;
                clock_digits[4]++;
                if(clock_digits[4] >=6){
                    clock_digits[4]=0;
                    clock_digits[3]++;
                     alarm_stop=false;
                    if(clock_digits[3]>=10){
                        clock_digits[3]=0;
                        clock_digits[2]++;
                        if(clock_digits[2]>=6){
                            clock_digits[2]=0;
                            clock_digits[1]++;
                            if(clock_digits[1]>=10){
                                clock_digits[1]=0;
                                clock_digits[0]++;
                            }else if(clock_digits[0]==2 &&clock_digits[1] == 4){
                                clock_digits[0] = 0;
                                clock_digits[1] = 0;
                            }
                        }
                    }
                }
            }
       
         if(alarm_toview){
           toviewCount++;
             if( toviewCount== 2){
            	alarm_toview=false;
             	toviewCount=0;
           }
             
         }
       }
}


void validateAlarm(){

  if((!alarm_saved) || alarm_set || alarm_on ||clock_set_hours || alarm_stop ){
    
  	return;
  }
  int count = 0;
  for(int i = 0; i<4 ; i++){
    if(alarm_digits[i] == clock_digits[i]){
     count++;
    }
  }
  if(count == 4){
   
    clr_bit(PORTC,OnAlram); //tocando alarme
    
    alarm_on = true; // tocando alarme
  }


}

void handleSet_hours(){
	if(!alarm_set && !alarm_on){
           clock_set_hours=!clock_set_hours ;
     	   set_config_hours_minutes=false;
      }

}

void handleSet_Alarm(){
  // colocar alarme
 	if(!clock_set_hours){
            alarm_set=!alarm_set;
            set_config_hours_minutes=false;
            alarm_saved=true;
            alarm_stop=false;
         }

}

void handleToglle_hours_minutes(){
  // horas ou alarme habilitados
		if(clock_set_hours || alarm_set){
           set_config_hours_minutes=!set_config_hours_minutes;
         }
  
}

void handleStop(){
  //para alarme
  if(!clock_set_hours && !alarm_set){
  	alarm_on=false;
    alarm_stop=true;
    set_bit(PORTC,OnAlram);
  }
  	
}

void handle_toview(){
  // não pode esta em modes
  // ver alarme
  if(!clock_set_hours && !alarm_set){
    alarm_toview=true;
  }

}

void potenciometro(){
  
  // habilitar a leitura do podenciomentro

if(clock_set_hours || alarm_set){
  	  uint16_t valorADC;
      valorADC = adcRead(); //ler o valor analogico
      uartDec2B(valorADC);
    }

}


void soneca(){
  // soneca
  int i=0;
  while(i<5){
    i++;
   alarm_digits[3]++;
     if(alarm_digits[3]>=10){
        alarm_digits[3]=0;
        alarm_digits[2]++;
        if(alarm_digits[2]>=6){
            alarm_digits[2]=0;
            alarm_digits[1]++;
            if(alarm_digits[1]>=10){
                alarm_digits[1]=0;
                alarm_digits[0]++;
            }else if(alarm_digits[0]==2 &&alarm_digits[1] == 4){
                alarm_digits[0] = 0;
                alarm_digits[1] = 0;
            }
        }
    }
  }

}



int main(){
  DDRD = 0xFF; // configurando como saida
  DDRB = 0b00111111;
  PORTD = 0b10111100;
  PORTB = 0b00110000;
  DDRC =  0b00001011;
  PORTC = 0b00001011;

 UART_Init(); //Inicalização UART
 uartIntRx(1); // habilitar recebimento UART interr.
  
  adcBegin(AVCC, 0x01); //Inicialização A/D
  adcChannel(ADC2);// porta A/d
  
  
  TCCR0A=(1<<COM0A0) | (1<<WGM01); // troca de estados
  TCCR0B = (1<<CS00) | (1<<CS01); // TC0 prescaler de 64
  OCR0A = 249; // maximo valor de contagem do registrador TCNT0
  TIMSK0 = 1<<OCIE0A; // interrupção no OCR0A
   

  sei(); 
  
  while(1){
    /*
    _delay_ms(1000);
      uart_Transmit('1'); 
    uartString("\r\n");
    */
    
    potenciometro();
 
  }
 return 0; 
}



ISR(TIMER0_COMPA_vect){ // interrupção acada 1ms
   TIMER += 1;
  SETDISPLAY +=1;
  if(INITIAL && TIMER>=segundos){
      INITIAL=false;
      FPSDISPLAY=1;
      TIMER =0;
      SETDISPLAY=0;
  }
  else if(!clock_set_hours){
        calculatedigits();
    }else if(clock_set_hours){
        clock_digits[5] =0;
        clock_digits[4] =0;
        TIMER=0;
    }
  hanldeSetDisplay();
  validateAlarm();
  
}




ISR(USART_RX_vect){
  
  uint8_t dado_rx; 
  dado_rx = uartRx();
  while(!uartTxOk());
  if(!INITIAL){
        switch(dado_rx){//testa o valor Lido
            //horas
            case '1':
                handleSet_hours(); 
                break;
            //horas ou minutos
            case '2':
                handleToglle_hours_minutes();
                break;
            //alarme
            case '3':
          		
                if(alarm_on){
                  soneca();
                  handleStop();
                  
                }else{
                  handleSet_Alarm();
                }

                break;
          	case '4':
          		// para ou ver alarme
                if(alarm_on){
					handleStop();
                }else{
                	handle_toview();
                }
                
                break;
          case 's':
          		/**/
                break;
        }
    }
}





