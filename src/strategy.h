#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include <ros/ros.h>

#include "data.h"
#include "futdata.h"

#define TSIZE_ZAG 1024
#define TSIZE_ATA 512
#define TSIZE_GOL 2

class Strategy : public FutData
{
private:
  // Atributos que estavam em FutData

  /** \brief Armazena qual time está com a posse da bola no lance atual. */
  bool advantage;

  /**Identificador da posição atual, usando no modo simulado.*/
  IDQUADRO id_pos; 

  /** \brief Armazena a configuração do jogo na iteração
   *anterior.
   * 
   * Vai sempre conter o valor que estava em #pos na
   * iteração anterior. Variável importante pois algumas
   * rotinas comparam a configuração anterior do jogo com
   * a atual.
   */
  CONFIG ant; 

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

  /** \brief Armazena os papeis dos seus robos. */
  ROLES papeis; 

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

private:

  POS_ROBO ref_desc[3];

  double t;
  
  double yalin[3], ypos_fut[3], thetapos_fut[3], dang_desc[3], dlin_desc[3];

  //predicados para os adversarios
  bool adv_na_area[3], adv_mais_prox_bola[3];

  double dist_meu[3], dist_adv[3];

  //a principio os booleanos dos jogadores tem uso especifico, mas mesmo assim foram criados para os tres jogadores 

  //booleanos do predicados do goleiro
  bool meu_lado_area[3], bola_frente_x_do_meu[3];

  //booleanos para ambos 
  bool meu_na_area[3];

  //booleanos do predicado com bola
  bool meu_na_area_hist[3], meu_na_frente_bola[3], meu_na_frente_bola_hist[3], 
     meu_posicionado_descolar[3], meu_furou[3], meu_frente_area[3], 
     meu_alinhado_chutar[3], meu_bem_alinhado_chutar[3], bola_frente_y_do_meu[3],
     bola_frente_y_do_meu_hist[3], meu_posicionado_isolar[3], meu_posicionado_isolar_hist[3], 
     meu_alinhado_isolar[3], meu_bem_alinhado_isolar[3];

  // Analise da situacao da bola
  bool bola_parada, bola_no_ataque, bola_minha_area_lateral;

  // So um desses predicados pode ser true ao mesmo tempo
  bool bola_dentro_gol, bola_area_defesa, bola_frente_area, bola_lado_area,
    bola_parede_fundo, bola_quina_fundo, bola_fundo, bola_parede_lateral,
    bola_lateral, bola_quina_frente, bola_parede_frente, bola_frente,
    bola_regiao_central;

  // Para determinar jogadores bloqueados
  unsigned contador_parado[3];

  // Para determinar jogadores perdidos
  unsigned contador_perdido[3];
  bool perdido[3];

  // Lado do campo
  int sinal;

  //ACTION resp_com_bola[512];
  //ACTION resp_sem_bola[1024];

  //void preenchimento();

  int estado_penalty;

 private:
  // Métodos que estavam em FutData

  /** \brief Retorna o ID do robô goleiro.
   * \retval 0 se o robô 0 for o goleiro.
   * \retval 1 se o robô 1 for o goleiro.
   * \retval 2 se o robô 2 for o goleiro.
   */
  inline int goleiro() const { return (papeis.me[0].funcao==GOLEIRO ? 0 : (papeis.me[1].funcao==GOLEIRO?1:2) ); }
  /** \brief Retorna o ID do robô com bola (atacante).
   * \retval 0 se o robô 0 for o com bola.
   * \retval 1 se o robô 1 for o com bola.
   * \retval 2 se o robô 2 for o com bola.
   */
  inline int com_bola() const { return (papeis.me[0].funcao==COM_BOLA ? 0 : (papeis.me[1].funcao==COM_BOLA?1:2) ); }
  /** \brief Retorna o ID do robô sem bola ("zagueiro", ou também conhecido como "robô idiota").
   * \retval 0 se o robô 0 for o sem bola.
   * \retval 1 se o robô 1 for o sem bola.
   * \retval 2 se o robô 2 for o sem bola.
   */
  inline int sem_bola() const { return (papeis.me[0].funcao==SEM_BOLA ? 0 : (papeis.me[1].funcao==SEM_BOLA?1:2) ); }

  /** \brief Retorna se o seu time está com a posse da bola. 
   * \retval true se estiver com a posse da bola.
   * \retval false caso contrário.
   */ 
  inline bool getAdvantage() const {return advantage;}

 private: 
  //bool setint (FUNCTION f, const char * binario, ACTION resp, bool silence);
  //bool full_table (FUNCTION f);
  void analisa_jogadores();
  void analisa_adversarios();
  void analisa_bola();
  PWM_WHEEL descolar_parede(int id);
  POS_ROBO posicao_para_descolar_bola();
  POS_ROBO calcula_origem_parede();
  void detecta_bloqueados();
  void escolhe_funcoes();
  void acao_goleiro(int id);
  void acao_goleiro_play(int id);
  void acao_com_bola(int id); 
  void acao_com_bola_play(int id);
  void acao_sem_bola(int id);
  void calcula_referencias(int id);
public:
  Strategy(ros::NodeHandle& n);
  ~Strategy();
  inline void set_pos(const CONFIG& new_pos){pos = new_pos;}
  inline void get_ref(REFERENCES& ref_out){ref_out = ref;}
  void get_bypass_control(bool bypass_control_out[3], PWM_ROBOTS& pwm_out);
  void get_bloqueado(bool bloqueado_out[3]);
  bool strategy();

};

#endif
