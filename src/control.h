#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <ros/ros.h>

#include "data.h"
#include "futdata.h"


//################definicoes do Controle############
// Distância na qual se considera que o robô atingiu o alvo
#define EPSILON_L 0.01
// Distância na qual se considera que o robô não mais está no alvo
#define DELTA_L (2*EPSILON_L)


// Distância a partir da qual o controle de orientação começa a atuar
#define DIST_ORIENT 0.15
// Valor mínimo do PWM abaixo do qual o robô não anda...
//#define PWM_MINIMO (1.0/8.0)
#define PWM_MINIMO 0.07//CHECAR: fazer pwms diferenciados pra cada roda

// valor abaixo do qual o sinal enviado do pwm eh zero
#define PWM_ZERO (1.0/127.0)

class PID {
private:
  double K, Ti, Td, N;
  double e_ant, I_ant, I_ant2, D_ant;
public:
  // A relação com as constantes tradicionais do PID é a seguinte:
  // Kp=k, Ki=k/ti, Kd=k*td
  // Para eliminar a parte integral (Ki=0), ti=infinito
  // Para eliminar a parte derivativa (Kd=0), td=0.0
  // O número n é o limitador de altas freqüencias da parte derivativa.
  // Normalmente, 3<=n<=20. Para eliminar a filtragem, n=infinito
  inline PID(double k=85.0, double ti=0.44/*1E+6*/, double td=0.11/*0.0*/, double n=1E+6) {
    fixa_constantes(k,ti,td,n);
  }
  void fixa_constantes(double k, double ti, double td, double n=1E+6);
  void anti_windup();
  void reset();
  double controle(double erro, double h);
};

class Control : public FutData
{
private:
  // Dados que estavam em FutData
  /** \brief Armazena a configuração atual do jogo.
   * 
   * A configuração atual é definida pela rotina de
   * aquisição, baseando-se na imagem obtida pela câmera e
   * no processamento dessa imagem. A pose dos robôs e
   * posição da bola são armazenados em coordenadas de
   * mundo.
   *
   * Essa variável é o meio de comunicação entre a rotina
   * de aquisição e as demais rotinas do programa.
   */
  CONFIG pos; 

  /** \brief Variável utilizada para pular o cálculo dos sinais de
   * controle para um determinado robô.
   *
   * Em alguns momentos, vai ser desejado que seja aplicado um sinal
   * de PWM específico no robô, ao invés de deixar o algoritmo de
   * controle calcular os valores de PWM. A variável bypassControl
   * trata de informar ao controle que não deve calcular o valor de
   * PWM para determinados robôs.
   *
   * A função de controle deve, para cada robô i, verificar se o valor
   * no indice i desta variável é verdadeiro. Se for, ele não deve
   * calcular o valor de PWM para o robô i.
   *
   * Geralmente essa variável é setada durante o processamento da
   * estratégia, principalmente para fazer testes específicos com o
   * robô.
   */
  bool bypassControl[3]; 

  /** \brief Variável que informa se um robô está bloqueado.
   *
   * O valor no indice i desta variável deve ser setada se o robô i
   * estiver bloqueado, passando assim essa informação para as demais
   * etapas de processamento. Isso é importante, por exemplo, para
   * evitar que a parcela integrativa seja utilizada no controle
   * enquanto um robô estiver bloqueado.
   * 
   * Geralmente o algoritmo de detecção de bloqueio é implementado na
   * estratégia.
   */
  bool bloqueado[3];

  /** \brief Contém a referência para onde os robôs devem se deslocar.
   * 
   * As referẽncias dos robôs são definidas pela classe de
   * estratégia. Essa vável funciona como um meio de comunicação entre
   * a estratégia e o controle dos robôs.
   *
   * Os valores contidos nesta variável podem ser alterados pelo
   * algoritmo de desvio de obstáculos, caso o caminho até o ponto não
   * esteja livre. Geralmente o desvio de obstáculos é processado
   * entre as chamadas das rotinas de estratégia e controle.
   */
  REFERENCES ref; 
  /** \brief Contém o valor de PWM a ser transmitido para os robos.
   *
   * Os valores de PWM são normalmente definidos pela rotina de
   * controle, mas podem ser definidos em outra parte do programa,
   * utilizando o vetor #bypassControl. 
   * 
   * Esta variável funciona como um meio de comunicação entre a rotina
   * de controle e a de transmissão.
   */
  PWM_ROBOTS pwm;  

  double dt_amostr;

private:
  bool controle_orientacao;
  PID lin[3],ang[3];
  bool chegou[3];
  int sentidoGiro[3];
public:
  Control(ros::NodeHandle& n);
  ~Control();
  
  inline void set_pos_me(const TPOS_ROBO& new_pos_me){pos.me = new_pos_me;}
  inline void set_ref(const REFERENCES& new_ref){ref = new_ref;}
  void set_bypass_control(bool new_bypass_control[3], const PWM_ROBOTS& new_pwm);
  void set_bloqueado(bool new_bloqueado[3]);
  inline void get_pwm(PWM_ROBOTS& pwm_out){pwm_out = pwm;}

  bool control();
  bool stop_control();
};

#endif
