char UARTInitialise(const long int baudrate){
	unsigned int x;
	x = (_XTAL_FREQ - baudrate*64)/(baudrate*64);
	if(x>255){
		x = (_XTAL_FREQ - baudrate*16)/(baudrate*16);
		BRGH = 1;
	}
	if(x<256){
        SPBRG = x;
        SYNC = 0;
        SPEN = 1;
        TRISC7 = 1;
        TRISC6 = 1;
        CREN = 1;
        TXEN = 1;
        return 1;
	}
	return 0;
}
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