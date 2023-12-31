/*
 * uart.c
 *
 *  Created on: Jul 18, 2023
 *      Author: picman
 */
#include "uart.h"

void sputc(char c)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, UART_TIMEOUT);
  return;
}

void sputs(char *str)
{
  int cnt = 0;
  for(cnt=0; cnt<UART_BUFFER_SIZE; cnt++){
    if(str[cnt]=='\0') break;
  }
  HAL_UART_Transmit(&huart2, (uint8_t *)str, cnt, UART_TIMEOUT);
  return;
}

void sputsln(char *str)
{
  sputs(str);
  HAL_UART_Transmit(&huart2, (uint8_t *)NEW_LINE, strlen(NEW_LINE), UART_TIMEOUT);
}

int sputsf(char *format_str, ...)
{
  char *tmp[UART_BUFFER_SIZE];

  va_list args;
  va_start( args, format_str );

  if(strlen(format_str) > UART_BUFFER_SIZE){
    va_end(args);
    return -1;
  }

  vsprintf(tmp, format_str, args);
  sputs(tmp);

  return 0;
}


char sgetc(void)
{
  char c[1] = {'\0'};
  while(1){
    HAL_StatusTypeDef res = HAL_UART_Receive(&huart2, (uint8_t *)c, 1, UART_TIMEOUT);
    if(res == HAL_OK) break;
  }
  return (char)c[0];
}

int sgets(char *buffer, int len)
{
  int idx = 0;

  while(1){
    char c = sgetc();
    if(c == RETURN_CHAR){
      buffer[idx++] = '\0';
      break;
    }else if(c == '\b'){
      buffer[--idx] = '\0';
    }else{
      buffer[idx++] = c;
    }

    if(idx >= UART_BUFFER_SIZE-1){
      buffer[UART_BUFFER_SIZE-1] = '\0';
      break;
    }
  }
  return idx;
}

