/** \file futdata.h
 *
 *  \brief Contém a classe FutData e definições utilizadas pelos
 *  arquivos que a utilizam.
 */
#ifndef _FUTDATA_H_
#define _FUTDATA_H_

#include <ros/ros.h>

#include "data.h"

/** \brief Define o tempo de amostragem inicial. 
 *
 * Necessário na primeira iteração, onde o tempo de amostragem ainda
 * não foi calculado. Significa 1.0/frame_rate_da_camera .
 *
 * \todo Fazer o valor to tempo inicial ser calculado automaticamente
 * de acordo com o frame rate da camera utilizada.
 */
#define DT_AMOSTR_INICIAL 1.0/30.0 

/** \brief Classe de comunicação entre as subclasses do Futebol de Robôs
 * 
 * A classe FutData funciona como um meio de comunicação entre as
 * diversas classes do futebol de robôs. Todas as classes que
 * desempenham algum papel no processamento do futebol de robôs herdam
 * esta classe virtualmente, incluindo a própria classe
 * FutRobot. Desta forma, o programa final vai ter somente uma area de
 * memória relativa a herança desta classe, e essa área de memória vai
 * ser comum a todas as classes.
 * 
 * As variáveis do FutData funcionam como entradas e saídas para as
 * classes que a herdam. Por exemplo, a classe de acquisição recebe a
 * imagem da camera, processa-a e tem como saída a posição em metros
 * dos robôs e da bola. Essa posição é escrita na variável "pos" de
 * FutData, pois assim ela fica disponível para os métodos das outras
 * classes que são chamadas na sequência. Por exemplo, a classe de
 * estratégia pode ler a posição dos robôs e da bola através da
 * variável "pos" que foi escrita pela classe de acquisição, já que
 * ambas herdam virtualmente a classe FutData.
 * 
 * Existem outras variáveis privadas, que vão armazenar informações
 * relativas a partida atual. Elas não funcionam como "variáveis de
 * comunicação".
 *
 */
class FutData{
private:
  ros::NodeHandle& n;

  TEAM my_team; /**< \brief Armazena qual o seu time. */
  SIDE my_side; /**< \brief Armazena qual o lado o seu time está
		   jogando. */
  GAME_STATE game_state; /**< \brief Armazena qual o estado atual da
          partida.*/
public:
  FutData(ros::NodeHandle& n);
  /** \brief Retorna qual é o seu time.
   * \retval #my_team O seu time.
   */
  inline TEAM myTeam() const {return my_team;}
  /** \brief Retorna qual é o lado que seu time está jogando.
   * \retval #my_side O lado do seu time.
   */
  inline SIDE mySide() const {return my_side;}
  /** \brief Retorna qual é o estado atual do jogo.
   * \retval #game_state O estado do jogo.
   */
  inline GAME_STATE gameState() const {return game_state;} 

};

#endif
