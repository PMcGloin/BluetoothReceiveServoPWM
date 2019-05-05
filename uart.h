/*******************************************************************************
* Function Name: UARTReadChar
*
* Input Parameters: void
*
* Returns: character
*
* Purpose of function: 	reads character from USART register
*******************************************************************************/
char UARTReadChar(){
    while(!RCIF);
    if (OERR == 1){ //overrun error
        CREN = 0;
        CREN = 1;   //Reset CREN
    }
    return RCREG;
}
/*******************************************************************************
* Function Name: UARTReadString
*
* Input Parameters: character array, unsigned character length
*
* Returns: void
*
* Purpose of function: 	reads string of characters into char array
*******************************************************************************/
void UARTReadString(char Output[], unsigned char length){
    unsigned char i;
	for(i=0;i<length;i++){
        Output[i] = UARTReadChar();
    }
}