/* 
 *  Autor : Gianfranco Avona Covello
 *  Data  : 29/10/2021
 *  P.I.D = Proporcional, Integral, Derivativo
 * -----------------------------------------------  
 *  Proporcional -> Ganho no sistema
 *  
 *  Erro = (setPoint - valorMedido)
 *  
 *  P = kp x Erro
 *  I = Somatória dos erros infinitesimais visando aproximar ao máximo o erro
 *  D = Acelerar o processo para atingir o SetPoint
 *  
*/

// --------------------------------------------------
// --- MAPEAMENTO DE HARDWARE ---
#define   ldr       15
#define   led        4
//#define   ctrl_pwm  13  

// ==============================================================
// --- Constantes do Projeto --- //
#define   out1_time   500   //TEMPO PARA LEITURA DO SISTEMA
#define   dt          500   //atualização do PID

// ==============================================================
// --- Protótipo de Funçoes --- //
float pid_control(float meas, float set_point);

// ==============================================================
// --- Variáveis Globais --- //
int out1_time_save = 0, 
    pid_upt        = 0;

float kp = 1.5,
      ki = 0.6,
      kd = 0.01;

float LeituraLDR;

// --------------------------------------------------
// --- FUNÇÃO PRINCIPAL ---
void setup() {
  
  Serial.begin(115200);
  
  pinMode(ldr, INPUT);
  //pinMode(ctrl_pwm, INPUT);
  pinMode(led, OUTPUT); 

  ledcAttachPin(led, 0);            //(pino controlado, canal)
  ledcSetup(0, 2000, 12);           //(canal, frequência, bits) 12bits = 4095 + 1
  ledcWrite(0, 4095);               //(canal, amplitudeDesejadaEmBits)
  delay(3000);
} // end setup

void loop() {
  
  //unsigned duty = 0;
  //LeituraLDR = (analogRead(ldr));

  float pwm_val;
  if(millis() - pid_upt >= dt)
  {
      pwm_val = pid_control((analogRead(ldr)),2800);
      ledcWrite(0, pwm_val+2048);
      pid_upt = millis();
  }
  
  if(millis()-out1_time_save >= out1_time)
  {   
        
   /* ----- APENAS QUANDO O AJUSTE FOR FEITO PELO POTENCIÔMETRO ------
    *  duty = analogRead(ctrl_pwm);
    *         ledcWrite(0, duty);
    *  Serial.println(duty);
    *  //Serial.print("Potencia do PWM: ");
    *  //Serial.println(analogRead(ctrl_pwm));
    */ 
           
    Serial.print("Luminosidade: ");
    Serial.println(analogRead(ldr));

    out1_time_save = millis();
  } // end if
  
} //end loop


// --------------------------------------------------
// --- Desenvolvimento de funções ---

float pid_control(float meas, float set_point)
{
  static float last_meas; 
  
  float     error_meas,
            proportional,
            integral, 
            derivative;

  error_meas = set_point - meas;
  
  proportional = kp*error_meas;
  
  integral += (error_meas*ki) * (dt /1000);

  derivative = ((last_meas - meas)*kd) * (dt/1000);
  last_meas = meas;
  
  return (proportional + integral + derivative);
}
