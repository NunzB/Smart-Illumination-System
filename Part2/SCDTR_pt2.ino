#include <SPI.h>
#include <EEPROM.h>
#include "pid.h"
#include "mcp2515.h"
#include "can_frame_stream.h"
#include "Consensus.h"

boolean firstLoop = true;

int sensorValue = 0;
int sensorPin = A2;
int ledPin = 3;
//int resetPin = A5;

//double tempo_write;
//double tempo_atual;
//unsigned long serial_chrono;

float u_fb = 0;
int u = 0;

float Lth = 60;

float lux;
float lux_modelo;
float volt_modelo;
float erro_lux;
float e;
float luxInterfExt;

float kp = 0.2;
float ki = 1;
float kd = 1;
float a = 10;

float sensorVoltage;
float sensorResist;

float Gains[3];
int Vcc = 5;
int R1 = 10;

const byte maskFilter = B11111000; // mask bits that are not prescale
const byte prescale = B00000001; //fastest possible
volatile bool flag;
float fs = 100;   //amostras/s

volatile can_frame_stream cf_stream {};
volatile can_frame_stream cf_streamHub{};

uint32_t mask = 0x00000004;
uint32_t filt0 = 0x00000000; //accepts msg ID x0xx on RXB0
//uint32_t filt1 = 0x00000004; //accepts msg ID x1xx on RXB1

unsigned long counter = 0;
int counterCalibration;

enum States {CONSENSUS, PID, CALIBRATION, GET_INFO, SET_INFO};
States currentState = CONSENSUS;

float * auxConsensusResult;
float consensusResult[3];
int consensusInter = 0;
bool startEvent = true;

//EEPROM read
struct EEPROMVals {
  int rAddress = 0;
  uint32_t ID;
  float K;
  float Tau1;
  float Tau2;
  float m;
  float b;
} eepromVals;

volatile bool interrupt = false; //notification flag for ISR and loop()
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;

char Serial_ch[5];  //message received from serial
char Hub_ch[5];
bool sendInfo = false;
uint32_t sendBackID;
can_frame HubFrame;  //frame to identify which action is requested
uint32_t previousDeskID;
char previousInstruction;
bool Occupancy = true;
bool consensusOn = true;
uint32_t desk_ID;


// union - save memory by using the same memory region for storing different objects at different times
//aponta para o msm endereço, mas representa de forma diferente
union my_can_msg { //to pack/unpack long ints into bytes
  unsigned long value;
  unsigned char bytes[4];
};

MCP2515 mcp2515(10); //SS pin 10
Consensus consensus{};
pid pid{};                          // Inicia a classe do controlo pid

ISR(TIMER1_COMPA_vect) {            //Interrupção do timer1, notify main loop
  flag = 1;
}

void setup() {
  Serial.begin(115200);

  cli();        //disable interrupts
  TCCR1A = 0;   //reset registers
  TCCR1B = 0;

  TCNT1 = 0;    //reset counter

  // OCR1A = desired_period/clock_period – 1 = clock_freq/desired_freq - 1 = (16*10^6/1024)/(1)–1
  OCR1A = (int)((16 * pow(10, 6) / 1024) / (fs)) - 1; //must be <65536
  TCCR1B |= (1 << WGM12);                             //CTC On

  TCCR1B |= (1 << CS12) | (1 << CS10);               // Set prescaler for 1024
  TIMSK1 |= (1 << OCIE1A);                           // enable timer compare interrupt

  TCCR2B = (TCCR2B & mask) | prescale;
  sei();                                //enable interrupts

  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);

  //Insere mask na máscara dos buffers 0 e 1 do CAN
  mcp2515.setFilterMask(MCP2515::MASK0, 0, mask);
  //mcp2515.setFilterMask(MCP2515::MASK1, 0, mask);

  //filters related to RXB0
  mcp2515.setFilter(MCP2515::RXF0, 0, filt0);
  mcp2515.setFilter(MCP2515::RXF1, 0, filt0);

  /*//filters related to RXB1
    mcp2515.setFilter(MCP2515::RXF2, 0, filt1);
    mcp2515.setFilter(MCP2515::RXF3, 0, filt1);
    mcp2515.setFilter(MCP2515::RXF4, 0, filt1);
    mcp2515.setFilter(MCP2515::RXF5, 0, filt1);*/

  mcp2515.setNormalMode();
  // use mcp2515.setLoopbackMode() for local testing

  getValEEPROM();
  Serial.print("I'm arduino ");
  Serial.println(eepromVals.ID);
  delay(2000);
  Calibration();
  consensus.init(eepromVals.ID, Gains[0], Gains[1], Gains[2], luxInterfExt);
  
}

void loop() {
  if (firstLoop) {
    Serial.print("O: ");
    Serial.println(luxInterfExt);
    Serial.print("Gain for Arduino 0: ");
    Serial.println(Gains[0]);
    Serial.print("Gain for Arduino 1: ");
    Serial.println(Gains[1]);
    Serial.print("Gain for Arduino 2: ");
    Serial.println(Gains[2]);
    firstLoop = false;
  }

  //Serial.println("in loop");
 if(!consensusOn){
    ReadHubFrame();
    ReadSerial();
  }
  RunStateMachine();



  //delay(100);

  /*if (interrupt) {
    interrupt = false;
    if (mcp2515_overflow) {
      Serial.println("\t\tError: MCP2516 RX Buffers Overflow");
      mcp2515_overflow = false;
    }
    if (arduino_overflow) {
      Serial.println("\t\tError: Arduino Stream Buffers Overflow");
      arduino_overflow = false;
    }
    can_frame frame;
    while ( cf_stream.get(frame) ) {
      my_can_msg msg;
      for (int i = 0; i < 4; i++) {
        msg.bytes[i] = frame.data[i];
      }
      Serial.print("\tReceiving: ");
      Serial.println(msg.value);
    }

    while ( cf_streamHub.get(frame) ) {
      my_can_msg msg;
      for (int i = 0; i < 4; i++) {
        msg.bytes[i] = frame.data[i];
      }
      Serial.print("\tReceiving Hub: ");
      Serial.println(msg.value);
    }
    }*/
}

void RunStateMachine()
{
  switch (currentState)
  {
    case CALIBRATION:
      {
        Calibration();
        currentState = CONSENSUS;
        
        break;
      }

    case CONSENSUS:
      {
        if (startEvent)
        {
          startEvent = false;
          consensusInter = 0;
          Serial.println("Setting new Lth...");
          consensus.SetNewOperation(Lth);

          /*can_frame frame;

            if ( cf_stream.get(frame) )
            {
            Serial.print("I had messages to read");
            }*/
        }

        if (consensusInter == 50)
        {
          currentState = PID;
          consensusOn = false;
          break;
        }

        consensus.iterate();

        Serial.print("Iteration number: ");
        Serial.println(consensusInter);
        Serial.print("First duty: ");
        Serial.print(consensus.d[0]);
        Serial.print("  Second duty: ");
        Serial.print(consensus.d[1]);
        Serial.print("  Third duty: ");
        Serial.println(consensus.d[2]);

        int otherArduinoMessages = 0;
        can_frame consensusFrame;
        float avg0 = 0;
        float avg1 = 0;
        float avg2 = 0;

        
        if (write((eepromVals.ID | 0b001000), consensus.d[0]) != 0)
        {
          Serial.print("Error sending first consensus message, iter: ");
          Serial.println(consensusInter);
        }
        
        if (write((eepromVals.ID | 0b010000), consensus.d[1]) != 0)
        {
          Serial.print("Error sending second consensus message, iter: ");
          Serial.println(consensusInter);
        }
        
        if (write((eepromVals.ID | 0b100000), consensus.d[2]) != 0)
        {
          Serial.print("Error sending third consensus message, iter: ");
          Serial.println(consensusInter);
        }


        while (otherArduinoMessages < 6)
        {
          //Serial.println(analogRead(sensorPin));
          if ( cf_stream.get(consensusFrame) ) {
            if ((consensusFrame.can_id & 0b001000))
            {
              Serial.print("Receiving msg 0 from arduino: ");
              Serial.println(consensusFrame.can_id & 0x3);
              Serial.println(readFloatFromStream(consensusFrame));
              avg0 += readFloatFromStream(consensusFrame);

            }
            else
            {
              if (consensusFrame.can_id & 0b010000)
              {
                Serial.print("Receiving msg 1 from arduino: ");
                Serial.println(consensusFrame.can_id & 0x3);
                Serial.println(readFloatFromStream(consensusFrame));
                avg1 += readFloatFromStream(consensusFrame);
              }
              else
              {
                Serial.print("Receiving msg 2 from arduino: ");
                Serial.println(consensusFrame.can_id & 0x3);
                Serial.println(readFloatFromStream(consensusFrame));
                avg2 += readFloatFromStream(consensusFrame);
              }
            }
            otherArduinoMessages++;
          }
        }

        consensus.d_avg[0] = avg0 / 3.0;
        consensus.d_avg[1] = avg1 / 3.0;
        consensus.d_avg[2] = avg2 / 3.0;

        consensus.y[0] = consensus.y[0] + consensus.rho * (consensus.d[0] - consensus.d_avg[0]);
        consensus.y[1] = consensus.y[1] + consensus.rho * (consensus.d[1] - consensus.d_avg[1]);
        consensus.y[2] = consensus.y[2] + consensus.rho * (consensus.d[2] - consensus.d_avg[2]);

        consensusInter++;


        break;
      }
    case PID:
      {
        if (Occupancy) {  //caso false o led desta mesa está desligado
          if (flag) {
            callPID();
            flag = false;
          }
        }
        else {
          analogWrite(ledPin, 0);
        }
        break;
      }
    case GET_INFO:  //msg = g o <i>
      {
        Serial.println("get info");
        desk_ID = (uint32_t)((Serial_ch[4]) - '0');
        Serial.println(desk_ID);
        Serial.println(eepromVals.ID);

        if (desk_ID == eepromVals.ID) {     //a info é neste arduino
          Serial.println("desk_ID == eepromVals.ID");
          if (sendInfo) {   //entrei aqui pq recebi msg no hub
            sendInfo = false;
            if (write((sendBackID), InfoRequested(Serial_ch[2])) != 0) {
              Serial.print("Error sending HUB message back to original arduinos");
            }
          }
          else {  //Send info back to HUB, entrei aqui pq recebi msg no serial
            Serial.print(Serial_ch[2]);
            Serial.print(" ");
            Serial.print(eepromVals.ID);
            Serial.print(" ");
            Serial.print(InfoRequested(Serial_ch[2]));
          }
          currentState = PID;
        }
        else { //a info nao é neste arduino, mas recebi pedido no serial
          Serial.println("envia msg para outro arduino");
          previousInstruction = Serial_ch[2];
          previousDeskID = desk_ID;
          if (write((eepromVals.ID | 0b100), Serial_ch) != 0) {
            Serial.print("Error sending HUB message to other arduinos");
          }
          currentState = PID;
        }
        break;
      }

    case SET_INFO:    //msg = O <i> <val>
      {
        desk_ID = Serial_ch[2];
        if (desk_ID == eepromVals.ID) {     //a info é neste arduino
          if (sendInfo) {   //entrei aqui pq recebi msg no hub
            sendInfo = false;
            if (write((sendBackID), SetRequested(Serial_ch[0], Serial_ch[4])) != 0) {
              Serial.print("Error sending HUB message back to original arduinos");
            }
          }
          else {  //Send info back to HUB, entrei aqui pq recebi msg no serial
            Serial.print(Serial_ch[2]);
            Serial.print(eepromVals.ID);
            Serial.print(SetRequested(Serial_ch[0], Serial_ch[4]));
          }
        }
        else { //a info nao é neste arduino, mas recebi pedido no serial
          previousInstruction = Serial_ch[0];
          previousDeskID = desk_ID;
          if (write((eepromVals.ID ), Serial_ch) != 0) {
            Serial.print("Error sending HUB message to other arduinos");
          }
          currentState = PID;
        }
        break;
      }
    /*case GET_INFO:
      int desk_ID = Serial_ch[4];
      if(Serial_ch[2] == 'I'){    //get current measured illum at desk i
        if((uint32_t)desk_ID == eepromVals.ID){
          getIllum();
          if(sendInfo){
            sendInfo = false;
            if(write((sendBackID), lux) != 0){
              Serial.print("Error sending HUB message back to original arduinos");
            }
          }
          else{   //Send info back to HUB
            Serial.print("I");
            Serial.print(eepromVals.ID);
            Serial.print(lux);
          }
        }
        else{
          previousInstruction = Serial_ch[2];
          previousDeskID = desk_ID;
          if (write((eepromVals.ID | 0b1000100), Serial_ch) != 0){
            Serial.print("Error sending HUB message to other arduinos");
          }
          currentState = PID;
        }
      }
      else if(Serial_ch[2] == 'd'){    //get current duty cycle at desk i
        if((uint32_t)desk_ID == eepromVals.ID){

          if(sendInfo){
            sendInfo = false;
            if(write((sendBackID), ((u*100)/255) ) != 0){
              Serial.print("Error sending HUB message back to original arduinos");
            }
          }
          else{   //Send info back to HUB
            Serial.print("d");
            Serial.print(eepromVals.ID);
            Serial.print(lux);
          }
        }
        else{
          previousInstruction = Serial_ch[2];
          previousDeskID = desk_ID;
          if (write((eepromVals.ID), Serial_ch) != 0){
            Serial.print("Error sending HUB message to other arduinos");
          }
          currentState = PID;
        }
      }

      break;*/

    default:
      Serial.println("Wrong state");
      delay(1000);
      break;
  }
}

uint32_t InfoRequested(char Request) {
  my_can_msg msg;
  switch (Request)
  {
    case 'I':
      getIllum();
      msg.value = lux;
      return msg.bytes;

    case 'd':
      msg.value = ((u * 100) / 255);
      return msg.bytes;

    case 'o':
      msg.value = Occupancy;
      return msg.bytes;

    //case 'O':   //JOAO VÊ ESTA INSTRUÇÃO. SÃO OS BOUNDS DO CONSENSUS?
    //break;



    default:
      break;
  }
}

uint32_t SetRequested(char Request, char value) {
  my_can_msg msg;
  switch (Request)
  {
    case 'O':
      if (value = '1') {
        Occupancy = true;
        msg.value = 'ack';
        return msg.bytes;
      }
      else if (value = '0') {
        Occupancy = true;
        msg.value = 'ack';
        return msg.bytes;
      }
      else {
        msg.value = 'err';
        return msg.bytes;
      }
      break;

    default:
      break;
  }
}

void ReadSerial() {
  //check for input every 1/10th second
  //if( (millis() - serial_chrono) < 100)
  //return;
  if (Serial.available()) {
    int i = 0;
    while (Serial.available() > 0) {
      char inData = Serial.read();
      Serial_ch[i] = inData;
      i++;
      delay(100);
    }
    Serial_ch[i] = '\0';
    Serial.println(Serial_ch);
    if ( Serial_ch[0] == 'g' | Serial_ch[0] == 'd') {
      currentState = GET_INFO;
      //serial_chrono = millis();
    }
    else if ( Serial_ch[0] == 'o') {
      currentState = SET_INFO;
      //serial_chrono = millis();
    }
    //Serial.print(currentState);
  }

}

void ReadHubFrame() {

  float actionResult;
  if ( !cf_streamHub.get(HubFrame) )
    return;
  else { //há msg
    sendBackID = HubFrame.can_id;
    for (int i = 0; i < 5; i++) {
      Serial_ch[i] = HubFrame.data[i];
    }
    Serial.print("sendbackID: ");
    Serial.println(sendBackID);
    Serial.println(Serial_ch);
    if (((sendBackID & 0b11) == (eepromVals.ID))) { //indica que é mensagem de retorno
      Serial.println("HUB msg returning");
      actionResult = readFloatFromStream(HubFrame);

      //mandar info para o hub
      Serial.print(previousInstruction);
      Serial.print(previousDeskID);
      Serial.print(actionResult);
    }
    else if (((uint32_t)Hub_ch[4] != eepromVals.ID)) { // a msg é para o outro arduino, continua no PID
      Serial.println("HUB msg received but not for me");
      return;
    }
    else { //msg recebida para o arduino correto
      Serial.println("HUB msg received for me");
      if ( Serial_ch[0] == 'g' | Serial_ch[0] == 'd') {
        currentState = GET_INFO;
        sendInfo = true;
      }
    }

  }
}

MCP2515::ERROR write(uint32_t id, uint32_t val) {
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 4;
  my_can_msg msg;
  msg.value = val;
  for (int i = 0; i < 4; i++) {
    frame.data[i] = msg.bytes[i];
  }
  return mcp2515.sendMessage(&frame);
}

/*MCP2515::ERROR read(unsigned long &c) {
  can_frame frame;
  my_can_msg msg;
  MCP2515::ERROR err = mcp2515.readMessage(&frame);
  if (err == MCP2515::ERROR_OK) {
    for (int i = 0; i < 4; i++) {
      msg.bytes[i] = frame.data[i];
    }
    c = msg.value;
  }
  return err;
  }*/

float readFloatFromStream(can_frame frame)
{
  my_can_msg msg;
  for (int i = 0; i < 4; i++) {
    msg.bytes[i] = frame.data[i];
  }
  return msg.value;
}

/*void arduinoSync(int limit){
  for(int i = 0; i< limit/3 ; i++){
    write(eepromVals.ID, 0);
  }
  int msgReceived = 0;
  while (msgReceived < limit-1) {
    if ( cf_stream.get(frame) ) {
      msgReceived++;
    }
  }
  }*/

void callPID() {
  //tempo_atual = micros();

  //Serial.print("Sampling time: ");
  //Serial.println(tempo_atual - tempo1);

  getIllum();
  //volt_modelo = sim.GetSystemVoltage(tempo_atual * pow(10, -6));
  //lux_modelo = sim.GetSystemIlluminance(tempo_atual * pow(10, -6));
  lux_modelo = Gains[IDToArrayIndex(eepromVals.ID)] * consensus.d[eepromVals.ID];

  if ((abs(lux_modelo - lux) / lux_modelo) < 1) {
    u = pid.calc(lux_modelo, lux, u);
  }
  //Serial.println(pid.CONTROL); //variavel de controlo na classe pid
  analogWrite(ledPin, u);
  //tempo1 = tempo_atual;
}

void getIllum() {
  sensorValue = analogRead(sensorPin);                  // Leitura do valor medido pelo sensor
  sensorVoltage = sensorValue * ( Vcc / 1023.0);
  sensorResist = (( R1 * Vcc ) / sensorVoltage) - R1;
  lux = pow(10, (( log10(sensorResist) - eepromVals.b ) / (eepromVals.m)));
  return;
}

void Calibration()
{
  Serial.println("Starting calibration...");
  //// Calibrate external interference
  can_frame frame;
  MCP2515::ERROR err;
  int msgReceived = 0;
  int counterCalibration = 0;

  analogWrite(ledPin, 0);
  sensorValue = analogRead(sensorPin);  // Leitura do valor medido pelo sensor
  luxInterfExt = digitalToLux(sensorValue);
  if ((err = write(eepromVals.ID, 0)) != 0) {
    Serial.print("Error calibration external interferences write. nº: ");
    Serial.println(err);
  }

  Serial.println("Waiting for other arduinos to read external interference...");
  while (msgReceived < 2) {
    //write(eepromVals.ID, 0);
    if ( cf_stream.get(frame) ) {
      //if(frame.can_id ==
      msgReceived++;
    }
  }
  msgReceived = 0;

  Serial.println("Starting mutual interference calibration...");

  //// Calibrate other arduinos' interference
  while (counterCalibration < 3) {
    if (counterCalibration == eepromVals.ID) {

      Serial.println("Setting max duty cycle...");
      counterCalibration++;
      analogWrite(ledPin, 255);

      if ((err = write(eepromVals.ID, 0)) != 0) {
        Serial.print("Error calibration write 255. nº: ");
        Serial.println(err);
      }
      delay(2000);
      Serial.println("Waiting for other arduinos to get gains...");
      while (msgReceived < 2) {
        if ( cf_stream.get(frame) ) {
          msgReceived++;
        }
      }
      Serial.println("Reading gain...");
      for (int i = 0; i < 10; i++) {
        sensorValue += analogRead(sensorPin);  // Leitura do valor medido pelo sensor

        //Serial.println(sensorValue);
      }
      Gains[IDToArrayIndex(eepromVals.ID)] = (digitalToLux(sensorValue / 10) - luxInterfExt) / 255;
      Serial.print("Own sensor: ");
      Serial.println(sensorValue / 10);
      analogWrite(ledPin, 0);
      if ((err = write(eepromVals.ID, 0)) != 0) {
        Serial.print("Error calibration write 255. nº: ");
        Serial.println(err);
      }
      msgReceived = 0;
    }

    if (cf_stream.get(frame) ) {
      Serial.print("Getting mutual interference gain from arduino no. ");
      Serial.println(frame.can_id);
      counterCalibration++;
      delay(1000);
      sensorValue = analogRead(sensorPin);  // Leitura do valor medido pelo sensor
      Serial.println(sensorValue);
      Serial.println(digitalToLux(sensorValue) / 255, 4);
      Gains[IDToArrayIndex(frame.can_id)] = (digitalToLux(sensorValue) - luxInterfExt) / 255;

      if ((err = write(eepromVals.ID, 0)) != 0)
      {
        Serial.print("Error calibration write 255. nº: ");
        Serial.println(err);
      }

      Serial.println("Waiting for other arduinos to get gains...");
      while (msgReceived < 2) {
        if ( cf_stream.get(frame) ) {
          msgReceived++;
        }
      }
      msgReceived = 0;
    }

    if (counterCalibration == 3) {
      Serial.println("Waiting for other arduinos to exit calibration...");
      if ((err = write(eepromVals.ID, 0)) != 0) {
        Serial.print("Error calibration write 255. nº: ");
        Serial.println(err);
      }
      while (msgReceived < 2) {
        if ( cf_stream.get(frame) ) {
          //if(frame.can_id ==
          msgReceived++;
        }
      }
      msgReceived = 0;
    }
  }

  //Serial.println("Exiting calibration...");
}

void irqHandler()
{
  //Serial.println("hehey");
  int test;
  can_frame frame;
  uint8_t irq = mcp2515.getInterrupts(); //read CANINTF
  if (irq & MCP2515::CANINTF_RX0IF) {    //msg in receive buffer 0
    //Serial.println("Buffer 0");
    mcp2515.readMessage(MCP2515::RXB0, &frame); //also clears RX0IF
    if (!(cf_stream.put(frame))) arduino_overflow = true;
    //Serial.println(test);
  }
  if (irq & MCP2515::CANINTF_RX1IF) { //msg in receive buffer 1
    //Serial.println("Buffer 1");
    mcp2515.readMessage(MCP2515::RXB1, &frame); //also clears RX1IF
    if ((frame.can_id & 0x4)) {
      if (!(test = cf_streamHub.put(frame))) arduino_overflow = true;
    }
    else if (!(test = cf_stream.put(frame))) arduino_overflow = true;

    Serial.println(test);
  }
  irq = mcp2515.getErrorFlags(); //read EFLG
  if ( (irq & MCP2515::EFLG_RX0OVR) | (irq & MCP2515::EFLG_RX1OVR) ) {
    mcp2515_overflow = true;
    mcp2515.clearRXnOVRFlags();
  }
  mcp2515.clearInterrupts();
  interrupt = true; //notify loop()
}


int IDToArrayIndex(uint32_t id)
{
  if ((id & 0b11) == 0b00)
    return 0;
  else if ((id & 0b11) == 0b01)
    return 1;
  else if ((id & 0b11) == 0b10)
    return 2;
}


void getValEEPROM() {
  EEPROM.get( eepromVals.rAddress, eepromVals.ID );
  eepromVals.rAddress += sizeof(uint32_t);
  EEPROM.get(eepromVals.rAddress, eepromVals.K);
  eepromVals.rAddress += sizeof(float);
  EEPROM.get(eepromVals.rAddress, eepromVals.Tau1);
  eepromVals.rAddress += sizeof(float);
  EEPROM.get(eepromVals.rAddress, eepromVals.Tau2);
  eepromVals.rAddress += sizeof(float);
  EEPROM.get(eepromVals.rAddress, eepromVals.m);
  eepromVals.rAddress += sizeof(float);
  EEPROM.get(eepromVals.rAddress, eepromVals.b);
}


float digitalToLux(int sensorValue) {
  sensorVoltage = sensorValue * ( Vcc / 1023.0);
  sensorResist = (( R1 * Vcc ) / sensorVoltage) - R1;
  return pow(10, (( log10(sensorResist) - eepromVals.b ) / (eepromVals.m)));
}
