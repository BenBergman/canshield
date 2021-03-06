/*
****** NOTE ******
When communicating with the Arduino over serial, be sure that carriage returns are selected


for talking to dc/dc

AT H1           -enable display of reply headers
AT PP 2C SV 60  -sets (SV) extended message headers (60) on for protocol B (2C)
AT PP 2C ON     -activates modified settings
AT CP 0C        -sets first byte of extended address header
AT SH 70 82 01  -sets remainder of extended address header
01              -message

to listen to dc/dc broadcast

AT CRA 0C 7F 81 82   -listen to messages at address 0C 7F 81 82
// wait for at least .1s as this is the frequency of broadcast
// after message received:
AT AR                -return to mode that automatically sets receive address
*/




#include <math.h>
#include <NewSoftSerial.h>

#define SIZE 44
#define NUL '\0'
#define PROMPT '>'
#define RETURN '\r'
#define speedTimeout 75

int y;


void setup()
{

  char str[SIZE];

  Serial.begin(9600);
  Serial.flush();

  Serial1.begin(9600);
  Serial1.flush();


  //Restart ELM
  send_command("AT WS\r", str);

  delay(1000);

  //BCS requires 250kbaud.  Set baud rate divisor in PP 2D to 02.  Baud rate = 500/[02]
  send_command("AT PP 2D SV 02\r", str);

  //Enable the value for use
  send_command("AT PP 2D ON\r", str);

  //increase timeout time
  send_command("AT PP 03 SV A0\r", str);

  send_command("AT PP 03 ON\r", str);

  //Restart the ELM
  send_command("AT WS\r", str);
  delay(2500);

  //Turn echo off
  send_command("AT E0\r", str);  

  //Set protocol user1 CAN 11 bit ID, 250 kbaud
  send_command("AT SP B\r", str);   

  //wakeup();

  Serial.println("Starting main loop");

}

void stripAddress(char *str)
{
  for(int i = 0; i<29; i++)
  {
    str[i] = str[i+3];
  }
  str[29] = NUL;
}


void loop()
{
  char str[SIZE];
  str[0] = NUL;
  int b;
  byte i = 0;


  //Serial.println("\nSerial to CAN");
  if (Serial.available() > 0) {
    while( (b=Serial.read()) != RETURN && i<SIZE) {
      if(b>=' ')
        str[i++] = b;
    }
    str[i++] = RETURN; //bad
    str[i++] = NUL;

    // forward the command:
    //Serial.print("Sending to ELM: ");
    //Serial.println(str);
    Serial1.print(str);
  }
  delay(100);




  i = 0;
  str[0] = NUL;

  //Serial.println("\nCAN to Serial");
  if (Serial1.available()) {
    while( (b=Serial1.read()) != PROMPT && b != -1 && i<SIZE) {
      if(b>=' ')
        str[i++] = b;
    }
    if(b==PROMPT)
      str[i++] = b;
    str[i++] = NUL;
    // forward the character:
    Serial.println(str);
    
    //FOR TESTING
    //delay(300);
    //ELMserial.flush();
    //Serial.println(i, DEC);
    /*
    str[0]='a';
    str[1]=RETURN;
    str[2]=NUL;
    if (i >= SIZE) {
      delay(300);
      ELMserial.flush();
      ELMserial.print(str);
      ELMserial.flush();
    }
    */
  } 
  delay(100);



  /*
  char str1[SIZE];
  char str2[SIZE];
  char str3[SIZE];
  char str4[SIZE];

  Serial.println("Serial to CAN");

  serial_read_data(str1);
  if(str1 != 0) {
    Serial.print("Sending message to ELM:");
    Serial.println(str1);
    ELMserial.flush();
    ELMserial.print(str1);
  }

  delay(1000);

  Serial.println("CAN to Serial");

  read_data(str3);
  if(str3 != 0) {
    Serial.print("Message from ELM: ");
    Serial.println(str3);
  }

  delay(1000);
  */
}


/*
//gets short info from the BMS.  This includes the battery voltage and state of charge.
void get_shortInfo()
{
  int result;
  char msg[SIZE];

  //  Set CAN address to 100
  send_command("AT SH 100\r", msg);

  //  send request 0x07
  result = send_command("07\r", msg);
  if( result != 0)  
  {
    stripAddress(msg);

    //  char msg[] = "FB 1C 01 00 17 2C 00 00";    //fake data for testing

    char buffer1[10];
    char buffer2[10];
    char temp[3];
    long volts;
    int soc;

    strcpy(buffer1, "0x");
    substring(msg, temp, 3,4);
    strcat(buffer1, temp);
    substring(msg, temp, 0,1);
    strcat(buffer1, temp);
    sscanf(buffer1, "%li", &volts);

    dtostrf(volts/100.0, 2, 1, buffer1);
    strcpy(voltage, buffer1);
    strcat(voltage, v);  

    strcpy(buffer2, "0x");
    substring(msg, temp, 15,16);
    strcat(buffer2, temp);
    sscanf(buffer2, "%i", &soc);
    itoa(soc, buffer2,10);
    strcpy(stateOfCharge, buffer2);
    strcat(stateOfCharge, s);
  }
  else
  {
    strcpy(voltage, "err");
    strcpy(stateOfCharge, "err");
  }
}
*/

/*
//Get the minimum cell voltage from each slave
void getMinVolts()
{
  char temp[3];

  char msg[SIZE];
  char buffer0[10];
  char buffer1[10];

  long volts0;
  long volts1;

  //Set CAN address to 102
  send_command("AT SH 102\r", msg);

  //  char msg[] = "01 1D 01 F3 0C 00 00";  //fake data for testing

  //send request 0x1D 01
  if(send_command("1D 02\r", msg) != 0)
  {  
    strcpy(buffer0, "0x");
    substring(msg, temp, 9,10);
    strcat(buffer0, temp);
    substring(msg, temp, 6,7);
    strcat(buffer0, temp);
    sscanf(buffer0, "%li", &volts0);
    itoa(volts0, buffer0,10);
    strcpy(minVolts2, buffer0);
    strcat(minVolts2, mv);
  }
  else
  {
    strcpy(minVolts2, "err");
  }


  if(send_command("1D 01\r", msg) != 0)
  {
    strcpy(buffer1, "0x");
    substring(msg, temp, 9,10);
    strcat(buffer1, temp);
    substring(msg, temp, 6,7);
    strcat(buffer1, temp);
    sscanf(buffer1, "%li", &volts1);
    itoa(volts1, buffer1,10);
    strcpy(minVolts1, buffer1);
    strcat(minVolts1, mv);
  }
  else
  {
    strcpy(minVolts1, "err");
  }
}
*/


//Gets the BMS's software version
void get_software()
{
  char str[SIZE];

  //Set CAN address to 102
  send_command("AT SH 102\r", str);

  //send request 0x20
  send_command("20\r", str);  
  // stripAddress(str);
  delay(4000);
}


//Wakes up the BMS if it was asleep
void wakeup()
{
  char str[SIZE];

  //Set CAN address to 102
  send_command("AT SH 010\r", str);

  //send request 0x20
  send_command("01\r", str);  
}


//sends a CAN command
int send_command(char *cmd, char *result)
{
  Serial1.flush(); 
  Serial1.print(cmd);
  return read_data(result);
}


//Reads data from the serial port.  Returns 1 if data is as expected, or 0 if the data does not end with a prompt or nothing was received
int read_data(char *str)
{
  int b;
  byte i=0;

  //Only read if something is available to read
  if(Serial1.available() > 0)
  {
    // wait for something on com port
    while((b=Serial1.read())!=PROMPT && i<SIZE)
    {
      if(b>=' ')
        str[i++]=b;
    }

    if(b == PROMPT)  // we got a prompt
    {
      str[i]=NUL;
      return 1;
    }
    else
    {  
      //Message larger than size, or did not receive prompt
      return 0;
    }
  }
  else
  {
    return 0;
  }
}


//sends a CAN command
int serial_send_command(char *cmd, char *result)
{
  Serial.flush(); 
  Serial.println(cmd);
  return read_data(result);
}


//Reads data from the serial port.  Returns 1 if data is as expected, or 0 if the data does not end with a prompt or nothing was received
int serial_read_data(char *str)
{
  int b;
  byte i=0;

  //Only read if something is available to read
  if(Serial.available() > 0)
  {
    // wait for something on com port
    while((b=Serial.read())!=RETURN && i<SIZE)
    {
      if(b>=' ')
        str[i++]=b;
    }

    if(b == RETURN)  // we got a command
    {
      str[i]=NUL;
      return 1;
    }
    else
    {  
      //Message larger than size, or did not receive prompt
      return 0;
    }
  }
  else
  {
    return 0;
  }
}


//Copies a substring of the input to the output.
void substring(char *in, char *out, int begin, int end)
{
  for(int i = begin; i<=end; i++)
  {
    out[i-begin] = in[i];
  }
  out[end-begin+1] = NUL;
}
