char UARTReadChar(){
    while(!RCIF);
    if (OERR == 1){ //overrun error
        CREN = 0;
        CREN = 1;   //Reset CREN
    }
    return RCREG;
}
void UARTReadString(char Output[], unsigned char length){
    unsigned char i;
	for(i=0;i<length;i++){
        Output[i] = UARTReadChar();
    }
}