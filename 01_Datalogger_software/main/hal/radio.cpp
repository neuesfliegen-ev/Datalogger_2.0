#include "hal/radio.h"

/* Radio class methods */

// startUART
// Get UART port number from main,
// 2 ports are available, use one for radio other for GPS
void RadioClass::startUART(uart_port_t p)
{
	uart_port = p;
};
