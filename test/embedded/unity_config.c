#include "unity_config.h"

#include <stm32f1xx.h>

//Asincrónico, 8 bits de datos, sin paridad, 1 bit de parada

void unityOutputStart(unsigned long const baudrate){
    SystemCoreClockUpdate(); //Revisa la configuracion de clock y suelta la frecuencia del mismo
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN; //Activa el clock tanto del puerto A como del USART1
    GPIOA->CRH = (GPIOA->CRH & ~(0xF << GPIO_CRH_MODE9_Pos)) | (0b1011 << GPIO_CRH_MODE9_Pos); //Configura el pin 9 como un pin de salida rapida
    USART1->BRR = (SystemCoreClock+1) / baudrate; //Ver archivo de familia -> Configuracion de USART
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE; //UE activa el USART, TE activa el transmisor
}
void unityOutputChar(unsigned int const c){
    while(!(USART1->SR & USART_SR_TXE)); //Ciclo que se ejecuta mientras los datos NO estan siendo transmitidos al registro de desplazamiento
    USART1->DR = c; //Coloca el caracter recibido en el registro DR (Caracteres recibidos o transmitidos)
}
void unityOutputFlush(void){
    while(!(USART1->SR & USART_SR_TC)); //Ciclo que se ejecuta mientras la transmisión NO ESTA COMPLETA
}
void unityOutputComplete(void){
    unityOutputFlush(); //Me aseguro que la transmisión este completa
    USART1->CR1 &= ~(USART_CR1_TE | USART_CR1_UE); //Desconfiguro los registros previamente configurados
    RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN);
}