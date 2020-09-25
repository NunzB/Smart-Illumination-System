
// Iniciação de variáveis
#include <math.h>
#include "simulator.h"
#include "pid.h"

bool startCalibration = true, waiting = false, startLoop = false;

int sensorValue = 0, sensorPin = A0, ledPin = 3;  // valor medido pelo sensor (LUX) , pin de input no arduino para o LDR
int brightness = 0;

int Vcc = 5, R1 = 10;

float G = 2.006, b = 2.77, m = -0.76;

double tempo_write, tempo_atual, tempo1;

float sensorVoltage, sensorResist;

float u_fb = 0;
int u = 0, u_before;
float lux, x_modelo, lux_modelo, volt_modelo, erro_lux, e;

float kp = 0.3, ki = 1.9, kd = 0.1, a = 10;

const byte mask = B11111000; // mask bits that are not prescale
const byte prescale = B00000001; //fastest possible
volatile bool flag;
int fs = 10;   //amostras/s

int counter;

Simulator sim{m, b, G, Vcc};        // Inicia a classe do simulador
pid pid{};                          // Inicia a classe do controlo pid

ISR(TIMER1_COMPA_vect) {            //Interrupção do timer1, notify main loop
  flag = 1;
}

void setup() {
  pinMode(ledPin, OUTPUT); // seleção da porta de output digital PWM (porta 2)
  Serial.begin(500000); // comunicação de dados em série - 9600 bits/s;

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

}

void loop() {

  if (startCalibration)
  {
    Serial.println("Insert desired lux (0-500)");
    pid.init(kp, ki, kd, 1 / fs, a);
    startCalibration = false;
    waiting = true;
  }

  if (waiting == true && Serial.available() > 0)        //espera por input 
  {
    waiting = false;
    startLoop = true;
    x_modelo = Serial.parseInt();
    if (x_modelo <= 0 && x_modelo >= G * 255) {
      Serial.print("Valor Invalido.\n");
      x_modelo = 0;
    }
    Serial.println("DutyCycle\ttime\t\tlux\t\tlux modelo\tvolt\tvolt modelo");
    u = x_modelo / G;
    analogWrite(ledPin, u);
    tempo_write = micros();
    sim.SetTargetIlluminance(x_modelo, tempo_write * pow(10, -6));
  }

  if (startLoop)
  {
    analogWrite(ledPin, u);

    if (flag) {
      tempo_atual = micros();
      //Serial.print("Sampling time: ");
      //Serial.println(tempo_atual - tempo1);

      sensorValue = analogRead(sensorPin);                  // Leitura do valor medido pelo sensor
      sensorVoltage = sensorValue * ( Vcc / 1023.0);
      sensorResist = (( R1 * Vcc ) / sensorVoltage) - R1;
      lux = pow(10, (( log10(sensorResist) - b ) / (m)));

      volt_modelo = sim.GetSystemVoltage(tempo_atual * pow(10, -6)); 
      lux_modelo = sim.GetSystemIlluminance(tempo_atual * pow(10, -6));

      u_fb += pid.calc(lux_modelo, lux);

      if (u + u_fb < 0 | u + u_fb > 255) {        // limita o duty cycle aplicado ao led entre 0 e 255, previne wind-up
        if (u + u_fb < 0)
          u = 0;
        else
          u = 255;

        u_fb = 0;
      }
      else {
        if (abs(u_fb) >= 1) {
          u += u_fb;
          u_fb = 0;
        }
      }

      Serial.print(u);
      Serial.print("\t");
      Serial.print(tempo_atual);
      Serial.print("\t");
      Serial.print(lux, 4);
      Serial.print("\t\t");
      Serial.print(lux_modelo, 4);
      Serial.print("\t\t");
      Serial.print(sensorVoltage);
      Serial.print("\t");
      Serial.println(volt_modelo, 4);

      tempo1 = tempo_atual;
      flag = 0;
    }

    while (Serial.available() > 0)  {         // lê do buffer o dado recebido:
      x_modelo = Serial.parseInt();
      if (x_modelo <= 0 && x_modelo >= G * 255) {
        Serial.print("Valor Invalido.\n");
        x_modelo = 0;
      }
      Serial.println("time\t\tlux\t\tlux modelo\tvolt\tvolt modelo");

      u = x_modelo / G;
      tempo_write = micros();
      sim.SetTargetIlluminance(x_modelo, tempo_write * 1e-6);

    }

  }

}

/*if(sensorValue < 1000) { //define o valor de threshold [PODE-SE MUDAR PARA O VALOR QUE SE DESEJAR]
  //digitalWrite(ledPin,HIGH); //Liga o LED (ou dispositivo)

  if(brightness < dutyCycle[1]) {
    brightness = brightness +10;
  }
  else if(brightness > dutyCycle[1]){
    brightness = dutyCycle[1];
  }

  }

  else{ digitalWrite(ledPin,LOW); //Desliga o LED (ou dispositivo)
  dutyCycle[1] = 0;
  } */


/*int newLedBright( float* point_u, float* point_k0, float* point_tau, float* point_v_i)
  {
    x_modelo = Serial.parseInt();
    if(x_modelo <= 0 && x_modelo >= G*255){
      Serial.print("Valor Invalido.\n");
      x_modelo = 0;
    }
     point_v_i = sensorVoltage_modelo;

     point_u = x_modelo/G;                          //lux modelo, sinal x
     point_k0 = 0.406*exp(0.0009478*x_modelo) + -0.3953*exp(-0.01245*x_modelo) ; //sensorResist_modelo 0.4196*x^(0.1733) - 0.5875)
    //*point_k0 = 0.4174*exp(0.0008621*x_modelo) + -0.4083*exp(-0.01186*x_modelo) ;
     point_tau = 1800 + 2.204*pow(10,4)*exp(-0.01741*x_modelo) + 1.987*pow(10,4)*exp(-0.001265*x_modelo); // 2.204e4*exp(-0.01741*x) + 1.987e4*exp(-0.001265*x)

    u = x_modelo/G;
    Serial.println("time\t\tlux\t\tlux modelo\tvolt\tvolt modelo");

    tempo_write = micros();

    return;

  }*/
