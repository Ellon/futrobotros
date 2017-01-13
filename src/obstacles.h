#ifndef _OBSTACLES_H_
#define _OBSTACLES_H_

#include <ros/ros.h>

#include "data.h"
#include "futdata.h"

#define ZONA_INFL_MIN 0.05
#define ZONA_INFL_MAX 0.10

// gameState pos (ball op me) ref

class Obstacles : public FutData
{
private:
  // Variaveis que estavam em FutData
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

private:
  //Obstacles deve alterar o valor de ref pra uma nova referencia
  //procurando desviar de possiveis obstaculos no caminho.
  void desvio_area(int i);

public:
  Obstacles(ros::NodeHandle& n);
  ~Obstacles();

  void set_pos_me(const TPOS_ROBO& new_pos_me){pos.me = new_pos_me;}
  void set_pos_op(const TPOS_ROBO& new_pos_op){pos.op = new_pos_op;}
  void set_ref(const REFERENCES& new_ref){ref = new_ref;}
  void set_pos_ball(const POS_BOLA& new_pos_ball){pos.ball = new_pos_ball;}
  void get_ref(REFERENCES& ref_out){ref_out = ref;}

  bool obstacles();
};

#endif
