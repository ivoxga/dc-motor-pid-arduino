#include <MegunoLink.h>
#include <Arduino.h>

//  MegunoLink
TimePlot rpmPlot;

//  Motor e sensor
int sensor = 2;             // Pino do opto-switch
int motor = 3;              // Pino PWM do motor
int dentesPorRotacao = 40;  // Numero de dentes da roda

//  Interrupção
volatile unsigned long tempoAnterior = 0;   // Ultimo pulso
volatile unsigned long tempoAtual = 0;      // Pulso atual
volatile unsigned long deltaTempo = 0;      // Tempo entre dois pulsos (us)
volatile bool novoPulso = false;            // Flag para indicar novo pulso
unsigned long intervalo = 0;                // Intervalo entre pulsos

//  Medição de tempo e envio de dados
static unsigned long tempoEnvio = 0;    // Tempo de atualizaçao dos dados no grafico
unsigned long tempoDelta = 0;           // Copia de deltaTempo

//  Variaveis para calcular RPM
float alpha = 0.3;      // Suavizaçao exponencial
float novaRPM = 0;      // Ultimo valor calculado de RPM - sem suavizaçao
float rpm = 0;          // Valor da RPM atual - suavizada 

//  Veriaveis para descobrir RPM MAX e o tempo que demorou
float rpmMax = 0;
unsigned long tempoArranque = 0;

//  Variaveis para calculo da media em X segundos
float media = 0;                    // Valor do RPM medio
unsigned long tempoInicio = 0;      // Marca o inicio da contagem dos X segundos
float somaRPM = 0;                  // Soma dos valores de RPM para fazer a media
int amostras = 0;                   // Contador de quantas amostras foram feitas
int flag = 0;                       // Flag para calcular a media apenas uma vez

//  Paragem depois de calcular a media
unsigned long tempoPausa = 0;
unsigned long pausa3s = 0;

//  PID
float setpointPercent = 0;      // Setpoint percentual para o PID
float setpoint = 0;             // Setpoit RPM
float erro = 0;                 // Erro
float ei = 0;                   // Erro para a integral
float ed = 0;                   // Erro para a derivada
float erroAnt = 0;
float Kp = 0;
float Ki = 0;
float Kd = 0;
float p = 0;
float i = 0;
float d = 0;
int pid = 0;

//  Reta de aceleraçao e travagem
float y = 0;
float m = 0;
float b = 0;
unsigned long intervaloReta = 0.0;

//  Comportamento do motor
unsigned long tempoPID = 0.0;


void setup() {
  Serial.begin(115200);

  pinMode(motor, OUTPUT);
  analogWrite(motor, 255);

  attachInterrupt(digitalPinToInterrupt(sensor), contarPulso, RISING);

  rpmPlot.SetTitle("RPM do motor");
  rpmPlot.SetXLabel("Tempo");
  rpmPlot.SetYLabel("RPM");

  tempoInicio = millis();       // Inicio a contagem dos 5 segundos
  //intervaloReta = millis();   // Tempo ate reta descer

  //setpointPercent = 0.25;      // Para calibrar PID
  setpointPercent = 1.0;    // Para arranque do Comportamento
}

void loop() {
  static unsigned long tempoUltimoPulso = 0;

  // Regista o pico de RPM e o tempo em que foi atingido
  if ((rpm > rpmMax) && flag == 0) {
    rpmMax = rpm;
    tempoArranque = (millis() - tempoInicio) / 1000.0;
  }
  
  // Processo de controlo
  if (micros() - tempoEnvio >= 10000) {   // A casa 10ms atualiza o output 
    tempoEnvio = micros();
    tempoDelta = deltaTempo;              // Guarda uma copia do delta

    // Se detetar novo pulso
    if (novoPulso) {
      // RPM
      novaRPM = (60000000.0) / (tempoDelta * dentesPorRotacao);   // Calcula a RPM "crua"
      rpm = alpha * novaRPM + (1 - alpha) * rpm;                  // Calcula media exponencial movel para filtrar
      
      // Para calcular a media
      somaRPM += rpm;
      amostras++;
      
      // Atualiza ultimo pulso
      tempoUltimoPulso = micros(); 
      
      // Desativa flag
      novoPulso = false;
    } 

    // 30ms sem pulsos
    else {
      if (micros() - tempoUltimoPulso > 30000) 
        rpm = 0;
    }

    // Calcula a media de RPMs durante 10 segundos
    if ((millis() - tempoInicio >= 10000) && flag == 0)
      mediaRPM();
      
    // Pausa de 3 segundos
    if (flag == 1)
      pausa3s = millis() - tempoPausa;
    
    // PID
    if (flag == 1 && pausa3s >= 3000)
      PID();

    // Mostra valores no gráfico
    mostraDados();
  }
}

// Mostra os dados
void mostraDados() {
  rpmPlot.SendData("RPM Max", rpmMax);
  rpmPlot.SendData("RPM", rpm);
  rpmPlot.SendData("Tempo de arranque (s)", tempoArranque);
  rpmPlot.SendData("Velocidade media em 10s", media);
  rpmPlot.SendData("SetPoint", setpoint);
  rpmPlot.SendData("PWM", pid);
  rpmPlot.SendData("Rampa", RampaAcelDecel());
}

// Calcula o intervalo desde o ultimo pulso
void contarPulso() {
  tempoAtual = micros();                    // Tempo do pulso atual
  intervalo = tempoAtual - tempoAnterior;   // Tempo entre pulsos
  
  // Elimina valores lidos em menos de 1ms
  if (intervalo < 1000) 
    return;              
  
  // Calcula o intervalo desde o ultimo pulso
  deltaTempo = tempoAtual - tempoAnterior;
  tempoAnterior = tempoAtual;
  novoPulso = true;
}

// Calcula a media de RPMs
void mediaRPM() {
  if (amostras > 0) {
    media = somaRPM / amostras;
    setpoint = media * setpointPercent;
  }
  
  // Desliga motor e inicia a pausa antes do arranque com controlo
  if (flag == 0) {
    analogWrite(motor, 0);
    flag = 1;
    tempoPausa = millis();
  }
}

//  PID
void PID() {
  // Erro
  //erro = setpoint - rpm;            // Sem reta de aceleraçao e abrandamento
  //erro = RampaAcelDecel() - rpm;    // Com reta de aceleraçao e abrandamento
  Comportamento();

  pid = P() + I() + D();
  pid = constrain(pid, 0, 255);
  analogWrite(motor, pid);
}


//  Parametro P
float P() {
  if (setpointPercent == 0.25)
    Kp = 2.13;

  else if (setpointPercent == 0.5)
    Kp = 3;

  else if (setpointPercent == 0.75)
    Kp = 8.3;

  else if (setpointPercent == 1)
    Kp = 10;

  p = Kp * erro;
  return p;
}

//  Parametro I
float I() {
  if (setpointPercent == 0.25)
    Ki = 0.08;

  else if (setpointPercent == 0.5)
    Ki = 0.03;

  else if(setpointPercent == 0.75)
    Ki = 0.035;

  else if (setpointPercent == 1)
    Ki = 0.1;
  
  // erro integral
  ei = ei + erro;

  i = Ki * ei;
  return i;
}

//  Parametro D
float D() {
  if (setpointPercent == 0.25)
    Kd = 0.4;

  else if (setpointPercent == 0.5)
    Kd = 15;

  else if(setpointPercent == 0.75)
    Kd = 30;

  else if(setpointPercent == 1)
    Kd = 50;
  
  //erro derivada
  ed = erro - erroAnt;
  erroAnt = erro;

  d = Kd * ed;
  return d;
}

//  Equação da reta
float RampaAcelDecel() {  
  // Subida 5 segundos - para setpoit 50%
  //m = (setpoint / 5000.0);
  //y = ((m * millis()) - 260.0);

  // Subida 5 segundos - para setpoit media rpm
  m = setpoint / 5000.0;
  y = (m * millis()) - (2.6 * setpoint);

  // Descida 5 segundos - para setpoit 50%
  //m = ((0.0 - setpoint) / 5000.0);
  //y = ((m * millis()) + 500);  

  // Subida 10 segundos - para setpoit 50%
  //m = (setpoint / 10000.0);
  //y = ((m * millis()) - 130.0);

  // Descida 5 segundos depois de 10s em reta - para setpoit 50%
  //if (millis() - intervaloReta >= 30000.0) {
  //  m = ((0.0 - setpoint) / (5000.0));
  //  y = ((m * millis()) + 700);  
  //}
  
  if(y < 0)
    y = 0;

  else if (y >= setpoint)
    y = setpoint;

  return y;
}


//  Comportamento do motor
void Comportamento() {
  tempoPID = millis() - 13000;  // 13s depois inicia PID

  if (tempoPID <= 5000 || (tempoPID > 5000 && tempoPID <= 8000))
    erro = RampaAcelDecel() - rpm;

  else if (tempoPID > 8000 && tempoPID <= 11000) {
    setpoint = media * 0.5;
    erro = setpoint - rpm;
  }

  else if (tempoPID > 11000 && tempoPID <= 14000) {
    setpoint = media * 0.75;
    erro = setpoint - rpm;
  }

  else if (tempoPID > 14000 && tempoPID <= 17000) {
    setpoint = media * 0.25;
    erro = setpoint - rpm;
  }

  else {
    setpoint = 0;
    analogWrite(motor, 0);
  }
}