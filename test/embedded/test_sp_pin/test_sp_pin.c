#include <unity.h>
#include <soporte_placa.h>
#include <stm32f1xx.h>

//Aqui se testearán las funciones definidas en el archivo sp_pin.c

void setUp(){
    __disable_irq();                        //Deshabilito interrupciones
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;     //Activo reloj de puerto B (para resetearlo)
    RCC->APB2RSTR = RCC_APB2RSTR_IOPBRST;   //Reseteo el puerto B
    RCC->APB2RSTR = 0;                      //Una vez reseteado, vuelvo a poner en 0 el registro de reset
    RCC->APB2ENR &= ~RCC_APB2ENR_IOPBEN;    //Deshabilito el reloj (una vez reseteado)
    __enable_irq();                         //Habilito interrupciones
}
void tearDown (void) {

}

static void TEST_CONFIGURAR_PIN_COMO_ENTRADA(void) {
//Preparo el entorno
    uint32_t OFFSET = 4U;                               //Defino el offset para configurar pin 9 (4 bits para el registro CRH)
    uint32_t MASK_PIN9_MODO = (0xF << OFFSET);          //Defino una mascara de 4 bits en 1 para los bits de CRH del Pin 9
    uint32_t MODO_ESPERADO = 0b0100;                    //El modo esperado es "01" (Entrada flotante), "00" (Entrada)
    SP_Pin_setModo(SP_PB9,SP_PIN_SALIDA);               //Lo configuro como salida (??)
    uint32_t CRL_ANTES = GPIOB->CRL;                    // Guardo los valores de CRL, CRH y ODR antes de ejecutar la funcion
    uint32_t CRH_ANTES = GPIOB->CRH;
    uint32_t ODR_ANTES = GPIOB->ODR;

//Ejecuto la funcion a probar
    SP_Pin_setModo(SP_PB9, SP_PIN_ENTRADA);         

//Recolecto datos
    uint32_t CRL_DESPUES = GPIOB->CRL;                  //Guardo los valores despues de la ejecucion
    uint32_t CRH_DESPUES = GPIOB->CRH;
    uint32_t ODR_DESPUES = GPIOB->ODR;
    uint32_t DELTA_CRL = CRL_ANTES ^ CRL_DESPUES;       //Los registros DELTA guardan en 1 los bits que cambiaron, y en 0 los que no (XOR)
    uint32_t DELTA_CRH = CRH_ANTES ^ CRH_DESPUES; 
    uint32_t DELTA_ODR = ODR_ANTES ^ ODR_DESPUES;
    uint32_t MODO_ACTUAL = (GPIOB->CRH & MASK_PIN9_MODO) >> OFFSET; //Extraigo los 4 bits de modo de CRH para el pin 9
    bool CLOCK_B_ENCENDIDO = RCC->APB2ENR & RCC_APB2ENR_IOPBEN;     //Sera verdadero si el clock del puerto B esta encendido
//Comparo datos
    TEST_ASSERT_TRUE(CLOCK_B_ENCENDIDO);                            //El clock deberia encenderse
    TEST_ASSERT_FALSE(DELTA_CRL);                                   //CRL no deberia cambiar
    TEST_ASSERT_FALSE(DELTA_ODR);                                   //ODR no deberia cambiar
    TEST_ASSERT_BITS_LOW(~MASK_PIN9_MODO,DELTA_CRH);                //Los bits de CRH que NO PERTENEZCAN al modo del pin9 no deberian cambiar
    TEST_ASSERT_TRUE(MODO_ESPERADO & MODO_ACTUAL);                  //Los 4 bits de modo extraidos deberian ser iguales a los del modo esperado
}

static void TEST_CONFIGURAR_PIN_COMO_ENTRADA_PULLUP(void) {
//Preparo el entorno
    uint32_t OFFSET = 4U;
    uint32_t MASK_CRH = (0xF << 4U);
    uint32_t MASK_ODR = (0b1 << 9U);                                //En pullup, ademas de los bits de modo, se escribe un 1 en el pin (reflejado en el bit 9 del ODR)
    uint32_t MODO_ESPERADO = 0b1000;                                //Modo esperado: Pullup/Pulldown
    SP_Pin_setModo(SP_PB9,SP_PIN_ENTRADA);
    uint32_t CRL_ANTES = GPIOB->CRL;
    uint32_t CRH_ANTES = GPIOB->CRH;
    uint32_t ODR_ANTES = GPIOB->ODR;

//Ejecuto la funcion a probar
    SP_Pin_setModo(SP_PB9, SP_PIN_ENTRADA_PULLUP);

//Recolecto datos
    uint32_t CRL_DESPUES = GPIOB->CRL;
    uint32_t CRH_DESPUES = GPIOB->CRH;
    uint32_t ODR_DESPUES = GPIOB->ODR;
    uint32_t DELTA_CRL = CRL_ANTES ^ CRL_DESPUES;
    uint32_t DELTA_CRH = CRH_ANTES ^ CRH_DESPUES; 
    uint32_t DELTA_ODR = ODR_ANTES ^ ODR_DESPUES;                   //El bit 9 del ODR SI DEBERIA CAMBIAR (De 0 a 1)
    uint32_t MODO_ACTUAL = (GPIOB->CRH & MASK_CRH) >> OFFSET;
    bool CLOCK_B_ENCENDIDO = RCC->APB2ENR & RCC_APB2ENR_IOPBEN;

//Comparo datos
    TEST_ASSERT_TRUE(CLOCK_B_ENCENDIDO);
    TEST_ASSERT_FALSE(DELTA_CRL);
    TEST_ASSERT_BITS_LOW(~MASK_ODR,DELTA_ODR);                      //Verifico que todos los bits del DELTA_ODR (Excepto el 9) permanezcan iguales
    TEST_ASSERT_BITS_LOW(~MASK_CRH,DELTA_CRH);
    TEST_ASSERT_TRUE(MODO_ESPERADO & MODO_ACTUAL);
    TEST_ASSERT_TRUE(MASK_ODR & ODR_DESPUES);                       //Verifico si el bit 9 del ODR esta en 1
}

static void TEST_CONFIGURAR_PIN_COMO_ENTRADA_PULLDOWN(void) {
//Preparo el entorno
    uint32_t OFFSET = 4U;
    uint32_t MASK_CRH = (0xF << OFFSET);
    uint32_t MASK_ODR = (0b1 << 9U);  
    uint32_t MODO_ESPERADO = 0b1000;
    SP_Pin_setModo(SP_PB9,SP_PIN_ENTRADA);
    uint32_t CRL_ANTES = GPIOB->CRL;
    uint32_t CRH_ANTES = GPIOB->CRH;
    uint32_t ODR_ANTES = GPIOB->ODR;

//Ejecuto la funcion a probar
    SP_Pin_setModo(SP_PB9, SP_PIN_ENTRADA_PULLDN);

//Recolecto datos
    uint32_t CRL_DESPUES = GPIOB->CRL;
    uint32_t CRH_DESPUES = GPIOB->CRH;
    uint32_t ODR_DESPUES = GPIOB->ODR;
    uint32_t DELTA_CRL = CRL_ANTES ^ CRL_DESPUES;
    uint32_t DELTA_CRH = CRH_ANTES ^ CRH_DESPUES; 
    uint32_t DELTA_ODR = ODR_ANTES ^ ODR_DESPUES;
    uint32_t MODO_ACTUAL = (GPIOB->CRH & MASK_CRH) >> OFFSET;
    bool CLOCK_B_ENCENDIDO = RCC->APB2ENR & RCC_APB2ENR_IOPBEN;

//Comparo datos
    TEST_ASSERT_TRUE(CLOCK_B_ENCENDIDO);
    TEST_ASSERT_FALSE(DELTA_CRL);
    TEST_ASSERT_BITS_LOW(~MASK_CRH,DELTA_CRH);
    TEST_ASSERT_BITS_LOW(~MASK_ODR,DELTA_ODR);  
    TEST_ASSERT_TRUE(MODO_ESPERADO & MODO_ACTUAL);
    TEST_ASSERT_FALSE(MASK_ODR & ODR_DESPUES);         //Igual que el PULLUP, solo que ahora verifico si hay un 0 en el bit 9 del ODR
}

static void TEST_CONFIGURAR_PIN_COMO_SALIDA(void) {
//Preparo el entorno
    uint32_t OFFSET = 4U;
    uint32_t MASK_CRH = (0xF << OFFSET);
    uint32_t MODO_ESPERADO = 0b0010;
    SP_Pin_setModo(SP_PB9,SP_PIN_ENTRADA);
    uint32_t CRL_ANTES = GPIOB->CRL;
    uint32_t CRH_ANTES = GPIOB->CRH;
    uint32_t ODR_ANTES = GPIOB->ODR;

//Ejecuto la funcion a probar
    SP_Pin_setModo(SP_PB9, SP_PIN_SALIDA);

//Recolecto datos
    uint32_t CRL_DESPUES = GPIOB->CRL;
    uint32_t CRH_DESPUES = GPIOB->CRH;
    uint32_t ODR_DESPUES = GPIOB->ODR;
    uint32_t DELTA_CRL = CRL_ANTES ^ CRL_DESPUES;
    uint32_t DELTA_CRH = CRH_ANTES ^ CRH_DESPUES; 
    uint32_t DELTA_ODR = ODR_ANTES ^ ODR_DESPUES;
    uint32_t MODO_ACTUAL = (GPIOB->CRH & MASK_CRH) >> OFFSET;
    bool CLOCK_B_ENCENDIDO = RCC->APB2ENR & RCC_APB2ENR_IOPBEN;
//Comparo datos
    TEST_ASSERT_TRUE(CLOCK_B_ENCENDIDO);
    TEST_ASSERT_FALSE(DELTA_CRL);
    TEST_ASSERT_FALSE(DELTA_ODR);
    TEST_ASSERT_BITS_LOW(~MASK_CRH,DELTA_CRH);
    TEST_ASSERT_TRUE(MODO_ESPERADO & MODO_ACTUAL);
}

static void TEST_CONFIGURAR_PIN_COMO_SALIDA_DRENADOR_ABIERTO(void) {
//Preparo el entorno
    uint32_t OFFSET = 4U;
    uint32_t MASK_PIN9_MODO = (0xF << OFFSET);
    uint32_t MODO_ESPERADO = 0b0110;
    SP_Pin_setModo(SP_PB9,SP_PIN_ENTRADA);
    uint32_t CRL_ANTES = GPIOB->CRL;
    uint32_t CRH_ANTES = GPIOB->CRH;
    uint32_t ODR_ANTES = GPIOB->ODR;

//Ejecuto la funcion a probar
    SP_Pin_setModo(SP_PB9, SP_PIN_SALIDA_OPEN_DRAIN);

//Recolecto datos
    uint32_t CRL_DESPUES = GPIOB->CRL;
    uint32_t CRH_DESPUES = GPIOB->CRH;
    uint32_t ODR_DESPUES = GPIOB->ODR;
    uint32_t DELTA_CRL = CRL_ANTES ^ CRL_DESPUES;
    uint32_t DELTA_CRH = CRH_ANTES ^ CRH_DESPUES; 
    uint32_t DELTA_ODR = ODR_ANTES ^ ODR_DESPUES;
    uint32_t MODO_ACTUAL = (GPIOB->CRH & MASK_PIN9_MODO) >> OFFSET;
    bool CLOCK_B_ENCENDIDO = RCC->APB2ENR & RCC_APB2ENR_IOPBEN;
//Comparo datos
    TEST_ASSERT_TRUE(CLOCK_B_ENCENDIDO);
    TEST_ASSERT_FALSE(DELTA_CRL);
    TEST_ASSERT_FALSE(DELTA_ODR);
    TEST_ASSERT_BITS_LOW(~MASK_PIN9_MODO,DELTA_CRH);
    TEST_ASSERT_TRUE(MODO_ESPERADO & MODO_ACTUAL);
} 

static void TEST_ESCRIBIR_1 (void) {
//Preparo el entorno
    SP_Pin_setModo(SP_PB9,SP_PIN_SALIDA_OPEN_DRAIN);
    SP_Pin_write(SP_PB9,0);
    uint32_t CRL_ANTES = GPIOB->CRL;
    uint32_t CRH_ANTES = GPIOB->CRH;
    uint32_t ODR_ANTES = GPIOB->ODR;
    uint32_t MASK_ODR = 0b1 << 9U;
//Ejecuto la funcion a probar
    SP_Pin_write(SP_PB9,1);
//Recolecto datos
    uint32_t CRL_DESPUES = GPIOB->CRL;
    uint32_t CRH_DESPUES = GPIOB->CRH;
    uint32_t ODR_DESPUES = GPIOB->ODR;
    uint32_t DELTA_CRL = CRL_ANTES ^ CRL_DESPUES;
    uint32_t DELTA_CRH = CRH_ANTES ^ CRH_DESPUES; 
    uint32_t DELTA_ODR = ODR_ANTES ^ ODR_DESPUES;
//Comparo datos
    TEST_ASSERT_FALSE(DELTA_CRL);
    TEST_ASSERT_FALSE(DELTA_CRH);
    TEST_ASSERT_BITS_LOW(~MASK_ODR,DELTA_ODR);
    TEST_ASSERT_TRUE(MASK_ODR & DELTA_ODR);             //Compruebo que el ODR cambio
    TEST_ASSERT_TRUE(MASK_ODR & ODR_DESPUES);           //Compruebo que ese cambio es de 0 a 1
}

static void TEST_ESCRIBIR_0 (void) {
//Preparo el entorno
    SP_Pin_setModo(SP_PB9,SP_PIN_SALIDA_OPEN_DRAIN);
    SP_Pin_write(SP_PB9,1);
    uint32_t CRL_ANTES = GPIOB->CRL;
    uint32_t CRH_ANTES = GPIOB->CRH;
    uint32_t ODR_ANTES = GPIOB->ODR;
    uint32_t MASK_ODR = 0b1 << 9U;
//Ejecuto la funcion a probar
    SP_Pin_write(SP_PB9,0);
//Recolecto datos
    uint32_t CRL_DESPUES = GPIOB->CRL;
    uint32_t CRH_DESPUES = GPIOB->CRH;
    uint32_t ODR_DESPUES = GPIOB->ODR;
    uint32_t DELTA_CRL = CRL_ANTES ^ CRL_DESPUES;
    uint32_t DELTA_CRH = CRH_ANTES ^ CRH_DESPUES; 
    uint32_t DELTA_ODR = ODR_ANTES ^ ODR_DESPUES;
//Comparo datos
    TEST_ASSERT_FALSE(DELTA_CRL);
    TEST_ASSERT_FALSE(DELTA_CRH);
    TEST_ASSERT_BITS_LOW(~MASK_ODR,DELTA_ODR);
    TEST_ASSERT_TRUE(MASK_ODR & DELTA_ODR);         //Compruebo que el ODR cambio
    TEST_ASSERT_FALSE(MASK_ODR & ODR_DESPUES);     //Compruebo que ese cambio sea de 1 a 0
}

static void TEST_LEER_1 (void) {                       //Consultar
//Preparo el entorno
    SP_Pin_setModo(SP_PB9,SP_PIN_SALIDA);
    SP_Pin_write(SP_PB9,1);
    SP_Pin_setModo(SP_PB9,SP_PIN_ENTRADA);
    SP_Pin_write(SP_PB9,0);

//Leo el valor del pin
    bool valor_1 = SP_Pin_read(SP_PB9);

//Comparo datos
TEST_ASSERT_EQUAL(1,valor_1);
}

static void TEST_LEER_0 (void) {                       //Consultar
//Preparo el entorno
    SP_Pin_setModo(SP_PB9,SP_PIN_SALIDA);
    SP_Pin_write(SP_PB9,0);
    SP_Pin_setModo(SP_PB9,SP_PIN_ENTRADA);
    SP_Pin_write(SP_PB9,1);

//Leo el valor del pin
    bool valor_0 = SP_Pin_read(SP_PB9);

//Comparo datos
    TEST_ASSERT_EQUAL(0,valor_0);                   //Ninguno de los 3 deberia cambiar
}

static void TEST_JTAG_PINES (void) {                                            //Consultar
    static SP_HPin const pinesJtag[] = {SP_PA15,SP_PB3,SP_PB4};                 //Arreglo que contiene 3 pines
    static size_t const NUM_PINES_JTAG = sizeof(pinesJtag)/sizeof(*pinesJtag);  //NUM_PINES_JTAG = 3
    for(size_t i=0;i<NUM_PINES_JTAG;++i){                                       // 0, 1, 2
        SP_HPin const hPin = pinesJtag[i];                                      //hPin = SP_PA15, SP_PB3, SP_PB4
        SP_Pin_setModo(hPin,SP_PIN_SALIDA);                                     //Configuro como salida (??)
        SP_Pin_write(hPin,0);                                                   //0 en el ODR
        SP_Pin_setModo(hPin,SP_PIN_ENTRADA);                                    //Configuro como entrada (??)
        bool const valor_0 = SP_Pin_read(hPin);                                 //valor_0 deberia ser 0
        SP_Pin_setModo(hPin,SP_PIN_SALIDA);                                     //Configuro como salida (??)
        SP_Pin_write(hPin,1);                                                   //1 en el ODR
        SP_Pin_setModo(hPin,SP_PIN_ENTRADA);                                    //Configura como entrada (??)
        bool const valor_1 = SP_Pin_read(hPin);                                 //valor_1 deberia ser 1
        TEST_ASSERT_FALSE(valor_0);                                             
        TEST_ASSERT_TRUE(valor_1);
    }
}


int main (void) {
    SP_init();
    UNITY_BEGIN();
    SP_Tiempo_delay(1000);
    RUN_TEST(TEST_CONFIGURAR_PIN_COMO_ENTRADA); //Llama a "RUN_TEST_AT_LINE", quien a su vez llama a "UnityDefaultTestRun" y le da la funcion a testear 
    RUN_TEST(TEST_CONFIGURAR_PIN_COMO_ENTRADA_PULLUP);
    RUN_TEST(TEST_CONFIGURAR_PIN_COMO_ENTRADA_PULLDOWN);
    RUN_TEST(TEST_CONFIGURAR_PIN_COMO_SALIDA);
    RUN_TEST(TEST_CONFIGURAR_PIN_COMO_SALIDA_DRENADOR_ABIERTO);
    RUN_TEST(TEST_ESCRIBIR_0);
    RUN_TEST(TEST_ESCRIBIR_1);
    RUN_TEST(TEST_LEER_1);
    RUN_TEST(TEST_LEER_0);
    RUN_TEST(TEST_JTAG_PINES);
    UNITY_END();
    return 0;
}