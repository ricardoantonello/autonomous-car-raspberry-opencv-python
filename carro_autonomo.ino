//Autor : Antonello
/**
Este projeto recebe informações via comunicação serial com o Raspberry e executa os comandos com o Shield AF_Motor no Arduíno.
*/
 
#include <AFMotor.h>
 
AF_DCMotor m1(1); //Seleciona o motor 1
AF_DCMotor m2(2); //Seleciona o motor 1
AF_DCMotor m3(3); //Seleciona o motor 1
AF_DCMotor m4(4); //Seleciona o motor 1

//Variáveis globais
int comando_serial=0; // -1=re 0=parar 1=frente 2=direita 3=esquerda
int tempo_acionamento=10; // tempo de atuação do movimento antes de nova leitura

/** tipo_mov = 0/para 1/frente -1/ré */
void mov(int motor, int tipo_mov){
  mov(motor, tipo_mov, 150); //velocidade padrao = 150 
}
void mov(int motor, int tipo_mov, int vel){
  if(tipo_mov==1){
    if(motor==1) {
      m1.setSpeed(vel); //Define a velocidade maxima
      m1.run(FORWARD); //Gira o motor sentido horario
    } else if(motor==2) {
      m2.setSpeed(vel); //Define a velocidade maxima
      m2.run(FORWARD); //Gira o motor sentido horario
    } else if(motor==3) {
      m3.setSpeed(vel); //Define a velocidade maxima
      m3.run(FORWARD); //Gira o motor sentido horario
    } else if(motor==4) {
      m4.setSpeed(vel); //Define a velocidade maxima
      m4.run(FORWARD); //Gira o motor sentido horario
    }
  } else if(tipo_mov==-1) { 
    if(motor==1) {
      m1.setSpeed(vel); //Define a velocidade maxima
      m1.run(BACKWARD); //Gira o motor sentido horario
    } else if(motor==2) {
      m2.setSpeed(vel); //Define a velocidade maxima
      m2.run(BACKWARD); //Gira o motor sentido horario
    } else if(motor==3) {
      m3.setSpeed(vel); //Define a velocidade maxima
      m3.run(BACKWARD); //Gira o motor sentido horario
    } else if(motor==4) {
      m4.setSpeed(vel); //Define a velocidade maxima
      m4.run(BACKWARD); //Gira o motor sentido horario
    }
  } else { 
    if(motor==1) {
      m1.run(RELEASE); //Desliga o motor
    } else if(motor==2) {
      m2.run(RELEASE); //Desliga o motor
    } else if(motor==3) {
      m3.run(RELEASE); //Desliga o motor
    } else if(motor==4) {
      m4.run(RELEASE); //Desliga o motor
    }
  }
}

void parar(){
  mov(1,0); mov(2,0); mov(3,0);  mov(4,0);
}
void frente(){
  mov(1,1); mov(2,1); mov(3,1); mov(4,1);
}
void re(){
  mov(1,-1); mov(2,-1); mov(3,-1); mov(4,-1);
}
void direita(){
  mov(1,-1); mov(2,-1); mov(3,1);  mov(4,1);
}
void esquerda(){
  mov(1,1); mov(2,1); mov(3,-1);  mov(4,-1);
}

void setup(){
  Serial.begin(9600);
  Serial.setTimeout(1); // em milis, fundamental para nao atrasar o Serial.parseInt()
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop(){
  // Leitura da Serial
  if (Serial.available() > 0) {
    //digitalWrite(LED_BUILTIN, HIGH); //liga led
    comando_serial = Serial.parseInt();
    //Serial.println(i);
  }
  
  if(comando_serial==-1){
    //Serial.println("Parar"); 
    re();
  }else if(comando_serial==0){
    //Serial.println("Parar"); 
    parar(); 
  }else if(comando_serial==1){
    //Serial.println("Frente"); 
    frente(); 
  }else if(comando_serial==2){
    //Serial.println("Direita"); 
    direita(); 
  }else if(comando_serial==3){
    //Serial.println("Esquerda"); 
    esquerda(); 
  }else{
    Serial.print("ERRO: comando serial desconhecido! Recebido: "); 
    Serial.println(comando_serial); 
  }

}
