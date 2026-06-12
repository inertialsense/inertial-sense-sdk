#include "IsSerialPort.h"

ISArduinoSerialPort serialPort(Serial, 115200);     // in Arduino land, Serial is global/static, so we can be to.
port_handle_t myPort = (port_handle_t)&serialPort;  // we need the pointer/address of serialPort, but cast to a port_handle_t

char myBuff[128];                     // a buffer to read data into
char myTxBuff[32] = "TX DATA\r\n";    // some data to send/write

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:

  if (portIsValid(myPort)) {                          // validate the port, and bind it to the device 
    if (!portIsOpened(myPort)) {
      portOpen(myPort);                               // open the port is its not already
    }

    int bytesAvailable = portAvailable(myPort);       // any bytes waiting to be received
    while (bytesAvailable > 0) {                      // as long as there is data to read... 
      int bytesToRead = (bytesAvailable > sizeof(myBuff) ? sizeof(myBuff) : bytesAvailable); // don't read more than our buffer
      int bytesRead = portRead(myPort, myBuff, (bytesAvailable);   // read tha data, return actually number of bytes read
      if (bytesRead > 0) {
        // do something with the data...
      }

      bytesAvailable = portAvailable(myPort);         // check again for any new data - ie, drain the buffer
    }
      
    portWrite(myPort, myTxBuff, strlen(myTxBuff));    // send some data out to the port
  }

  delay(10);                                          // wait some period to allow the device to respond,
}
