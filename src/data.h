/** \file data.h
 * \brief Define os tipos básicos usados nos futebol de robôs
 *
 * O arquivo data.h define varios tipos básicos utilizados pelas
 * diversas classes que compõem o futebol de robôs.
 *
 */

#ifndef _DATA_H_
#define _DATA_H_

#include <math.h>
#include <iostream>

// #include <imagem.h>
#include "parameters.h"

#define NUM_POINTS 28 /**< \brief Numero de pontos notáveis no
			 campo.*/
#define POSITION_UNDEFINED -9999 /**< \brief Valor que indica uma
				    posição indefinida no campo*/

#define DT_AMOSTR_INICIAL 1.0/30.0 

//Se voce so quiser usar o simulador, pode descomentar a linha abaixo
//e nao sera necessario instalar as bibliotecas de acesso a camera
//firewire e de acesso ao dispositivo USB-Serial
//#define _SO_SIMULADO_

/** \todo Colocar o modo simulado para funcionar novamente. */

/** \todo Fazer a pagina inicial da documentação.*/

// ##################################################################
// Variaveis CoordX declaradas em <imagem.h>, copied here while Image lib is
// not ported to ROS version.

 // As classes para armazenar coordenadas de pontos

// Ponto 2D

struct Coord2{
  double X,Y;
  inline Coord2(double pX=0.0, double pY=0.0):X(pX),Y(pY) {}
  inline Coord2 operator+(const Coord2 &C) const {
    return Coord2(X+C.X, Y+C.Y);
  }
  inline Coord2 operator-(const Coord2 &C) const {
    return Coord2(X-C.X, Y-C.Y);
  }
  inline double x() const {return X;}
  inline double y() const {return Y;}
  inline double u() const {return X;}
  inline double v() const {return Y;}
  inline double &x() {return X;}
  inline double &y() {return Y;}
  inline double &u() {return X;}
  inline double &v() {return Y;}
};
inline Coord2 operator*(double K, const Coord2 &C) {
  return Coord2(K*C.X, K*C.Y);
}
inline double sum2(const Coord2 &C) {
  return C.X*C.X+C.Y*C.Y;
}
inline double euclid(const Coord2 &C) {
  return sqrt(sum2(C));
}
inline std::ostream& operator<<(std::ostream& OS, const Coord2 &C) {
  return OS << '(' << C.X << ',' << C.Y << ')';
}

// Conjunto de dois pontos 2D

struct DCoord2
{
  Coord2 m,i;
  inline DCoord2():
    m(),i() {}
  inline DCoord2(const Coord2 &pm, const Coord2 &pi):
    m(pm),i(pi) {}
};

// Um conjunto de três pontos 2D

struct TCoord2
{
  Coord2 p0,p1,p2;
  inline TCoord2():
    p0(),p1(),p2() {}
  inline TCoord2(const Coord2 &P0, const Coord2 &P1, const Coord2 &P2):
    p0(P0),p1(P1),p2(P2) {}
  inline const Coord2 operator[](unsigned i) const {
    return (i==0 ? p0 : (i==1 ? p1 : p2) );
  }
  inline Coord2 &operator[](unsigned i) {
    return (i==0 ? p0 : (i==1 ? p1 : p2));
  }
};

// Ponto 3D

struct Coord3{
  double X,Y,Z;
  inline Coord3(double pX=0.0, double pY=0.0, double pZ=0.0):
    X(pX),Y(pY),Z(pZ) {}
  inline double x() const {return X;}
  inline double y() const {return Y;}
  inline double z() const {return Z;}
  inline double u() const {return X;}
  inline double v() const {return Y;}
  inline double theta() const { return Z; }
  inline double &x() {return X;}
  inline double &y() {return Y;}
  inline double &z() {return Z;}
  inline double &u() {return X;}
  inline double &v() {return Y;}
  inline double &theta() { return Z; }
};

// Um conjunto de três pontos 3D

struct TCoord3
{
  Coord3 p0,p1,p2;
  inline TCoord3():
    p0(),p1(),p2() {}
  inline TCoord3(const Coord3 &P0, const Coord3 &P1, const Coord3 &P2):
    p0(P0),p1(P1),p2(P2) {}
  inline const Coord3 operator[](unsigned i) const {
    return (i==0 ? p0 : (i==1 ? p1 : p2) );
  }
  inline Coord3 &operator[](unsigned i) {
    return (i==0 ? p0 : (i==1 ? p1 : p2));
  }
};

// ##################################################################


/** \brief Lado do campo 
 */
enum SIDE {
  RIGHT_SIDE, /**< Lado direito */
  LEFT_SIDE   /**< Lado esquerdo */
}; 

/** \brief Cor da marca principal de um time
 */
enum TEAM {
  BLUE_TEAM, /**< Time azul */
  YELLOW_TEAM /**< Time amarelo */
};

/** \brief Modo de jogo 
 *
 * Pode ser real ou simulado
 */
enum GAME_MODE {
  REAL_MODE, /**< Jogo real.*/
  SIMULATED_MODE /**< Jogo simulado.*/
};

/** \brief Define um vetor de velocidade 
 *
 * O vetor de velocidade é definido em coordenadas polares, ou seja,
 * ele tem um módulo e um ángulo. Geralmente utilizado para armazenar
 * a velocidade da bola durante a partida.
 *
 */
struct VELOCITY {
  double mod; /**< Módulo do vetor da velocidade */
  double ang; /**< Angulo do vetor velocidade */
};

/** \brief Posição da bola
 *
 * Estrutura que define a posição da bola no jogo. Ela herda de
 * Coord2, definido na classe de imagem. 
 *
 * \todo Eliminar a herança de Coord2 e a dependencia do Coord3,
 * substituindo por Eigen::Vector2d e Eigen::Vector3d
 */
struct POS_BOLA: public Coord2 {
  /** Construtor padrão*/
  inline POS_BOLA(): Coord2() {} 
  /** Construtor a partir de um Coord3. Criado por conveniência. O
      terceiro valor do Coord3 ("z" ou "theta") é ignorado. */
  inline POS_BOLA(const Coord3 &C): Coord2(C.x(), C.y()) {}
  /** Metodo para testar se a posição da bola está indefinida.*/
  inline bool undef() const {
    return(x() == POSITION_UNDEFINED || y() == POSITION_UNDEFINED);
  }
};

/** \brief Pose de um robô
 *
 * Estrutura que define a pose de um robô. Herda de Coord3, definido
 * na classe de imagem.
 *
 * \todo Eliminar a herança de Coord3, substituindo por uma estrutura
 * equivalente da classe Eigen, por exemplo, Eigen::Vector3d
 */
struct POS_ROBO: public Coord3 {
  /** \brief Metodo para testar se a pose do robô está indefinida.*/
  inline bool undef() const {
    return(x() == POSITION_UNDEFINED || y() == POSITION_UNDEFINED);
  }
};

/** \brief Tripla de posições de robôs
 * 
 * A estrutura contém a posição de três robôs. Ela é utilizada para
 * armazenar as posições de um time em uma única variável. 
 */
struct TPOS_ROBO
{
  POS_ROBO p0; /**< \brief Pose do robô de ID 0. 
		*
		* Privado. Acesso através do #operator[].
		*/
  POS_ROBO p1; /**< \brief Pose do robô de ID 1. 
		*
		* Privado. Acesso através do #operator[].
		*/
  POS_ROBO p2; /**< \brief Pose do robô de ID 2. 
		*
		* Privado. Acesso através do #operator[].
		*/

  /** \brief Construtor padrão.*/
  inline TPOS_ROBO():
    p0(),p1(),p2() {}

  /** \brief Construtor a partir de três posições. 
   *
   * \param[in] P0 Posição do robô 0.
   * \param[in] P1 Posição do robô 1.
   * \param[in] P2 Posição do robô 2.
   */
  inline TPOS_ROBO(const POS_ROBO &P0, const POS_ROBO &P1, const POS_ROBO &P2):
    p0(P0),p1(P1),p2(P2) {}

  /** \brief Retorna a posição de um robô
   * \param[in] i Id do robô
   */
  inline const POS_ROBO operator[](unsigned i) const {
    return (i==0 ? p0 : (i==1 ? p1 : p2) );
  }

  /** \brief Retorna o endereço da posição de um robô
   *
   * Necessário para usar o operador [] a esquerda de uma atribuição,
   * permitindo alterar diretamente o valor do robô indexado pela
   * variável i.
   *
   * \param[in] i Id do robô
   */
  inline POS_ROBO &operator[](unsigned i) {
    return (i==0 ? p0 : (i==1 ? p1 : p2));
  }
};


/** \brief Estrutura que define um estado do jogo
 *
 * A estrutura CONFIG armazema o estádo completo do jogo: a posição
 * dos seus robôs, a posição dos robôs adversários, a posição da bola,
 * a velocidade da bola e a posição estimada da bola. 
 * 
 */
struct CONFIG {
  TPOS_ROBO me; /**< Posições dos seus robôs. */
  TPOS_ROBO op; /**< Posições dos robôs adversários. */
  POS_BOLA ball; /**< Posição atual da bola. */
  POS_BOLA future_ball; /**< Posição futura da bola. */
  VELOCITY vel_ball; /**< Velocidade atual da bola. */
};

/** \brief Referência para os robôs se movimentarem.
 *
 * A estrutura REFERENCES armazena as posições que os seus robôs devem
 * ser movimentar.
 * 
 * De fato ela não seria necessária, já que ela contém apenas uma
 * variável TPOS_ROBO. Porém desejou-se manter o padrão em que todas
 * as informações relativas ao seu time sejam acessadas através de
 * variáveis de nome "me", e portanto a criação desta estrutura.
 */
struct REFERENCES {
  TPOS_ROBO me; /**< Posições para onde os robôs devem se deslocar.*/
};

/** \brief Sinal de PWM aplicado em um robô.
 *
 * A estrutura representa o PWM a ser aplicado nos motores esquerdo e
 * direito de um robô. O PWM de cada motor pode assumir valores de
 * -1.0 a +1.0, onde o módulo vai indicar a porcentagem do ciclo de
 * trabalho (duty-cycle), sendo 1.0 o ciclo de trabalho máximo e 0.0 o
 * mínimo, e o sinal indica o sentido de giro: positivo manda o robô
 * para frente, negativo manda o robô para trás.
 * 
 * Note que o sinal é relativo a frente do robô, e não ao sentido de
 * giro da roda, já que a roda direita gira no sentido horário para
 * deslocar o robô para frente, enquanto a roda esquerda gira no
 * sentido anti-horário.
 *
 */
struct PWM_WHEEL {
  double left; /**< PWM da roda esquerda. */
  double right; /**< PWM da roda direita. */
};

/** \brief Tripla de PWMs de robôs 
 *
 * A estrutura armazena uma tripla de valores de PWM, sendo cada um
 * relativos a um dos robôs do time. Estrutura necessária para agrupar
 * todos os PWMs a serem aplicados nos robôs em uma única variável.
 *
 */
struct TPWM_WHEEL{
 private:
  PWM_WHEEL x0, x1, x2;
 public:
  /** Operador de acesso ao PWM do robô de ID i.
   * \param[in] i ID do robô
   */
  inline const PWM_WHEEL operator[] (unsigned i) const {
    return (i==0 ? x0 : (i==1 ? x1 : x2) );
  }
  /** \brief Operador de acesso ao endereço do PWM do robô de ID i.
   * 
   * Necessário para usar os colchetes como operador esquerdo de uma
   * atribuição.
   *
   * \param[in] i ID do robô
   */
  inline PWM_WHEEL &operator[] (unsigned i) {
    return (i==0 ? x0 : (i==1 ? x1 : x2));
  }
};

/** \brief PWMs tos robôs de um time
 *
 * Armazena os PWMs dos robôs de um time. 
 *
 * De fato esta estrutura não seria necessária, já que ela contém
 * apenas uma variável TPWM_WHEEL. Porém desejou-se manter o padrão em
 * que todas as informações relativas ao seu time sejam acessadas
 * através de variáveis de nome "me", e portanto a criação desta
 * estrutura.
 */
struct PWM_ROBOTS {
  TPWM_WHEEL me;
};

/** \brief Estado do jogo
 *
 * Possíveis estados que o sistema pode entrar durante o jogo. O
 * estado principal é PLAY_STATE, onde o time realmente vai jogar. O
 * estado FINISH_STATE informa que o sistema está sendo finalizado. O
 * PAUSE_STATE informa que o jogo foi pausado, provavelmente devido a
 * alguma ordem do juiz. Os demais estados são usados para definir o
 * posicionamento de acordo com os lances do jogo.
 */
enum GAME_STATE {
  FINISH_STATE,   /**< Indica o fim de jogo.*/
  PAUSE_STATE,    /**< Jogo pausado.*/
  PENALTY_STATE,  /**< Indica que um penalti vai ser cobrado*/
  FREEKICK_STATE, /**< Indica que um lance de tiro livre vai ser cobrado.*/
  GOALKICK_STATE, /**< Indica que um tiro de meta vai ser cobrado.*/
  FREEBALL_STATE, /**< Indica que um lance de "bola livre" vai ser cobrado.*/
  INICIALPOSITION_STATE, /**< Indica que a partida vai recomeçar do centro do campo.*/
  PLAY_STATE, /**< Jogo em andamento.*/
  CALIBRATION_CONTROL_STATE, /**< ?? */
  CALIBRATION_IMAGE_STATE /**< ?? */
};

//Variaveis necessarias para a simulacao

/** \brief Identificador de um quadro. 
 * 
 * Identificador de um quadros para sincronização entre o simulador e
 * o cliente.
 */
typedef long unsigned IDQUADRO; 

/** \brief Dado utilizado para comunicar as posições dos robos e bola
 * no jogo simulado.
 *
 * A estrutura POSICAO armazena as posições dos robôs e da bola. Ele é
 * semelhante a estrutura CONFIG, porém sem a velocidade e a posição
 * futura da bola.
 * 
 * Essa estrutura armazena as posições dos robôs em relação a cor do
 * time em questão, ao invés de definir as posições dos seus robôs e
 * dos robôs adversários, como ocorre na estrutura CONFIG.
 *
 */
struct POSICAO {
  TPOS_ROBO azul; /**< Posições dos robôs do time azul. */
  TPOS_ROBO amrl; /**< Posições dos robôs do time amarelo. */
  POS_BOLA bola; /**< Posição da bola. */
};

/** \brief Posição dos robôs e da bola, usado pela classe de exportação 
 *
 * A estrutura POSITION armazena as posições do robô e da bola, se
 * forma semelhante a estrutura POSICAO, porém as posições dos robôs
 * estão agrupadas em posições dos seus robôs e dos robôs adversários
 * (ou do oponente).
 *
 */
struct POSITION {
  TPOS_ROBO me; /**< Posições dos seus robôs. */
  TPOS_ROBO op; /**< Posições dos robôs adversários. */
  POS_BOLA ball; /**< Posição da bola. */
};

/** \brief Placar do jogo
 *
 * Estrutura criada para comunicar o placar do jogo entre os clientes
 * e o simulador do futebol de robôs.
 */
struct PLACAR {
  unsigned azul; /**< Numero de gols do time azul. */
  unsigned amrl; /**< Numero de gols do time amarelo. */
};


/** \brief Placar do jogo
 *
 * Estrutura criada para exportar o placar do jogo através da classe
 * de exportação. Diferencia da estrutura PLACAR por indicar numero de
 * gols do seu time e do time adversário, ao invés do numero de gols
 * do time azul e do time amarelo.
 */
struct SCORE {
  unsigned me; /**< Numero de gols do seu time. */
  unsigned op; /**< Numero de gols do time adversário. */
};

/** \brief Situação do jogo simulado
 *
 * Esta estrutura é utilizada simular o resultado do processamento de
 * imagem no simulador. Esse resultado é a posição atual dos robôs
 * azuis e amarelos, além da posição da bola. Ela ainda contém um ID
 * que deve ser incrementado a cada nova situação informada pelo
 * servidor. Assim os clientes podem saber quando perderam alguma
 * informação do servidor, ou seja, se id_atual != id_anterior + 1.
 *
 */
struct SITUACAO  {
  IDQUADRO id; /**< Identificador da situação atual. */
  POSICAO pos; /**< Posições dos robôs e da bola na situação atual.*/
};

/** \brief PWM para os robôs informados ao simulador pelos clientes
 *
 * Esta estrutura simula o sinal de rádio que seria enviado aos robôs
 * reais, sendo no entando enviado para o simulador. Ela contém um ID,
 * assim como a estrutura SITUACAO, usado para permitir que o
 * simulador saiba quando perdeu alguma transmissão dos clientes.
 * 
 */
struct SINAL_RADIO {
  IDQUADRO id; /**< Identificador do sinal de rádio atual. */
  PWM_ROBOTS c; /**< Valores de PWMs dos robôs. */
};

/** \brief Funções dos robôs.
 *
 * Os robôs podem ter 3 funções: ser goleiro, ser o robô que está com
 * a bola, e ser o robô sem bola. A função de cada robô pode ser
 * reatribuida durante o jogo, mas não devem existir dois robôs com
 * uma mesma função ao mesmo tempo.
 */
enum FUNCTION {
  GOLEIRO, /**< Função de ser goleiro.*/
  COM_BOLA, /**< Função de "atacante".*/
  SEM_BOLA /**< Função de ficar sem bola, ou seja, não atrapalhar o
	      goleiro ou o robô com bola.*/
};

/** \brief Ações dos robôs
 *
 * Aqui são listadas as ações que os robôs podem desempenhar. Algumas
 * ações são somente para determinadas funções, e outras podem ser
 * desempenhadas por robôs em qualquer função. 
 * 
 * \todo Pedir a Luiz Henrique que documente os valores que não estão
 * documentados.
 *
 * \todo Entender e documentar a lógica dos numeros neste enum.
 */
enum ACTION {
  // As acoes que servem para todos os estados
  ESTACIONAR=1, /**< Ficar parado na posição atual. */
  // As acoes que servem para sem_bola e com_bola
  IR_MEIO_CAMPO=11, /**< Ir para o meio do campo*/
  FORMAR_BARREIRA=12, 
  CIRCULO_CENTRAL=13, 
  COMEMORAR=14, /**< Fica comemorando o gol. Ação geralmente utilizada
		   para testes de controle, onde o robô fica rodando
		   no circulo central.*/
  // As acoes que servem para goleiro e com_bola
  ISOLAR_BOLA=21, 
  LADO_AREA=22, 
  IR_BOLA=23, 
  POS_PENALTY1=24, 
  POS_PENALTY2=25,
  // As acoes do goleiro
  G_DEFENDER=101, /**< Evitar que a bola entre no gol.*/
  G_CENTRO_GOL=102, /**< Esperar no centro do gol.*/
  // As acoes do jogador com bola
  A_IR_MARCA=201, 
  A_FREE_BALL=202, 
  A_BOLA_CENTRO=203,
  A_DESCOLAR=204, 
  A_POSICIONAR_PARA_DESCOLAR=205,
  A_POSICIONAR_FRENTE_AREA=206, 
  A_CONTORNAR=208, 
  A_CONTORNAR_POR_DENTRO=209,
  A_ALINHAR_GOL=210, 
  A_ALINHAR_SEM_ORIENTACAO=211, 
  A_CHUTAR_GOL=212,
  // As acoes do jogador sem bola
  D_NAO_ATRAPALHAR=301, /**< Ficar em posição que não atrapalhe os
                           outros jogadores do time*/
  // As acoes impossiveis
  IMPOSSIVEL=-1, 
  NAO_DEFINIDO=0, /**< Ação não definida. */
  CALIBRATION_CONTROL_LINEAR_POSITION_1=500,   CALIBRATION_CONTROL_LINEAR_POSITION_2=501,
  CALIBRATION_CONTROL_ANGULAR_POSITION_1=502,   CALIBRATION_CONTROL_ANGULAR_POSITION_2=503,
  CALIBRATION_CONTROL_CIRCLE=504
};

/** \brief Papel de um robô
 *
 * O papel de um jogador (ou de um robô) é composto de uma função e
 * uma ação, que são atribuidas e reatribuidas durante a partida e
 * dependem do estado atual do jogo.
 *
 */
struct ROLE_JOGADOR {
  FUNCTION funcao; /**< Função atual. */
  ACTION acao; /**< Ação atual. */
};

/** \brief Tripla de papeis dos robôs
 *
 * A estrutura armazena os papeis dos robôs de um time, e provê
 * operadores de acesso a estes papeis através do ID do robô.
 *
 */
struct TROLE_JOGADOR{
 private:
  ROLE_JOGADOR x0; /**< \brief Papel do robô de ID 0. 
		    *
		    * Privado. Acesso através do #operator[].
		    */
  ROLE_JOGADOR x1; /**< \brief Papel do robô de ID 1. 
		    *
		    * Privado. Acesso através do #operator[].
		    */
  ROLE_JOGADOR x2; /**< \brief Papel do robô de ID 2. 
		    *
		    * Privado. Acesso através do #operator[].
		    */
 public:
  /** Operador de acesso ao papel do robô de ID i.
   * 
   * \param[in] i ID do robô
   */
  inline const ROLE_JOGADOR operator[] (unsigned i) const {
    return (i==0 ? x0 : (i==1 ? x1 : x2) );
  }
  /** \brief Operador de acesso ao endereço do papel do robô de ID i.
   * 
   * Necessário para usar os colchetes como operador esquerdo de uma
   * atribuição.
   *
   * \param[in] i ID do robô
   */
  inline ROLE_JOGADOR &operator[] (unsigned i) {
    return (i==0 ? x0 : (i==1 ? x1 : x2));
  }
};

/** \brief Papeis dos robôs do seu time.
 *
 * Armazena os papeis dos robôs do seu time. 
 *
 * De fato esta estrutura não seria necessária, já que ela contém
 * apenas uma variável TROLE_JOGADOR. Porém desejou-se manter o padrão
 * em que todas as informações relativas ao seu time sejam acessadas
 * através de variáveis de nome "me", e portanto a criação desta
 * estrutura.
 */
struct ROLES {
  TROLE_JOGADOR me; /**< Papeis dos robôs. */ 
};

/** \brief Referências dos robôs do seu time.
 *
 * Armazena as referências dos robôs do seu time. 
 *
 * De fato esta estrutura não seria necessária, já que ela contém
 * apenas uma variável TPOS_ROBO. Porém desejou-se manter o padrão em
 * que todas as informações relativas ao seu time sejam acessadas
 * através de variáveis de nome "me", e portanto a criação desta
 * estrutura.
 */
struct REFS {
  TPOS_ROBO me; /**< Referencias dos robôs.*/
};


/** \brief Pacote de informações exportadas para a interface.
 *
 * Dados que são compartilhados pelo programa para a interface do
 * futebol de robôs. Esta estrutura vai ser definida em uma memória
 * compartilhada que vai ser lida e escrita tanto pelo programa
 * principal, quanto pela interface.
 *
 * \todo Pedir a Luiz Henrique verifique e corrija a documentação
 * desta estrutura.
 */
struct PACKAGEDAT {
  /* sentido FutRobot -> interface */
  SIDE side; 
  TEAM team; 
  IDQUADRO id;//FALTA Este
  POSITION pos;
  VELOCITY vel_ball;
  ROLES roles;
  REFS ref;  
  PWM_ROBOTS pwm; 
  bool block0, block1, block2;

  //Imagem
  // PxRGB imagem[IMAGE_WIDTH][IMAGE_HEIGHT];
  
  GAME_STATE game;
  //  SIDE ret_side;

  /*sentido interface -> FutRobot*/
  //Vantagem
  bool adv;
  GAME_STATE ret_game;
};

#endif
