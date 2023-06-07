
#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#ifndef NULL
#ifndef __cplusplus
#define NULL (void*)0
#else
#define NULL 0
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @brief Configura el puerto USART 1 con el baudrate indicado (frecuencia de salida de datos)
 * 
 * @param baudrate Numero de simbolos por unidad de tiempo
 */
void unityOutputStart(unsigned long baudrate);
void unityOutputChar(unsigned int c);
void unityOutputFlush(void);
void unityOutputComplete(void);

#define UNITY_OUTPUT_START()    unityOutputStart((unsigned long) 115200)
#define UNITY_OUTPUT_CHAR(c)    unityOutputChar(c)
#define UNITY_OUTPUT_FLUSH()    unityOutputFlush()
#define UNITY_OUTPUT_COMPLETE() unityOutputComplete()

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* UNITY_CONFIG_H */

