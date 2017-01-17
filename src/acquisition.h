/** \file acquisition.h 
 *
 * \brief Arquivo que contem a definição da classe Acquisition e
 * algumas outras estruturas utilizadas pela classe.
 *
 * O arquivo foi criado de forma a ser compilado tanto para o jogo
 * real quanto para o simulado. Porém, existe também a opção de
 * compilá-lo somente para o simulado, caso você queira testar o
 * programa simulado em um computador sem acesso a câmera. Para tal,
 * todos os trechos do código que exigem as bibliotecas da câmera ou
 * acessam o dispositivo só são compilados se _SO_SIMULADO_ estiver
 * definido. Tal definição é feita no Makefile.
 *
 * Esse modo de compilação _SO_SIMULADO_ foi criado para os
 * alunos da disciplina robótica probabilistica poderem testar suas
 * estratégias no simulador nos seus próprios computadores, e depois
 * recompilarem o código e testarem com o sistema real.
 *
 * Neste arquivo são definidos também o enum REG_COLOR e a estrutura
 * REGION.
 *
 * \todo Recolocar as linhas que fazem o modo _SO_SIMULADO_ funcionar
 * no Makefile do main.
 *
 */
#ifndef _ACQUISITION_H_
#define _ACQUISITION_H_

#include <ros/ros.h>

#include "futdata.h"
#include "dados_calibracao.h"
#include "parameters.h"
#include <sys/time.h>

#ifndef _SO_SIMULADO_
// #include <camera.h>
#include <imagem.h>
#include "dados_calibracao.h"
#endif

/** \brief Enumera as regiões de cor possíveis de se encontrar no
 * processamento de imagem.
 *
 */
enum REG_COLOR {
  REG_COLOR_BLUE   =0, /**< Região com cor azul (marca do time azul).*/
  REG_COLOR_YELLOW =1, /**< Região com cor amarela (marca do time amarelo).*/
  REG_COLOR_ORANGE =2, /**< Região com cor laranja (bola).*/
  REG_COLOR_CIAN   =3, /**< Região com cor ciano (marca auxiliar do robô 0).*/
  REG_COLOR_PINK   =4, /**< Região com cor rosa (marca auxiliar do robô 1).*/
  REG_COLOR_GREEN  =5, /**< Região com cor verde (marca auxiliar do robô 2).*/
  REG_COLOR_BLACK  =6, /**< Região com cor preta (fundo do campo).*/
  REG_COLOR_WHITE  =7, /**< Região com cor (linhas do campo).*/
  REG_COLOR_OTHER1 =8, /**< Região com cor auxiliar 1 do adversário.*/
  REG_COLOR_OTHER2 =9, /**< Região com cor auxiliar 2 do adversário.*/
  REG_COLOR_OTHER3 =10, /**< Região com cor auxiliar 3 do adversário.*/
  REG_COLOR_UNDEFINED = -1 /**< Região com cor indefinida. Pode ser
			      qualquer cor que não esteja definida
			      pelos limites definidos na calibração.*/
};

/** \brief Estrutura que define uma região de pixels de uma mesma cor.
 *
 * Esta estrutura é utilizada pelo algoritmo de processamento de
 * imagem para armazenar as informações relativas a uma região de
 * imagem encontrada. As informações importantes são o centro da
 * região; a orientação da região; se ela é simétrica ou não; o ID que
 * define a sua cor; e o número de pixels que a região contém. 
 *
 * O centro da região é definido como sendo o centro de massa de todos
 * os pixels da região. Ou seja, sendo C o centro formado pelas
 * coordenadas Xc e Yc, o Xc é calculado pela média de todos as
 * coordenadas X dos pixels dessa região e o Y também é calculado de
 * forma similar, mas utilizando as coordenadas Y dos pixels.
 *
 * A orientação de uma marca representa o ângulo da reta formado pelo
 * segundo momento de inércia da região. Essa reta vai divide a região
 * em duas partes simétricas, e que tenha um menor "momento", ou seja,
 * a sessão mais longa da marca. Esse ângulo é de extrema importancia
 * para as marcas principais dos seus robôs. Como sabemos o formato da
 * marca e o posicionamento dela no topo do robô segue sempre o mesmo
 * padrão, podemos utilizar esse angulo e a posição da marca auxiliar
 * para sabermos qual a orientação da frente do robô.
 *
 * As marcas simétricas porém não tem segundo momento, logo o valor de
 * orientação não deve ser usado. A variável booleana #symetric
 * informa se a região é simétrica ou não. A cor da região é informada
 * na variável #colorID, enquanto o numero de pixels da região fica
 * salvo na variável #nPixel. Essa ultima informação é importante caso
 * sejam encontrados um numero de regiões de uma cor maior do que o
 * esperado. Pode-se por exemplo escolher as N maiores regiões que
 * tenham um maior numero de pixels.
 *
 * \todo Verificar a teoria de segundo momento e corrigir/clarificar a
 * documentação se necessário.
 */
struct REGION{
  Coord2 center; /**< \brief Centro da região. 
		  *
		  * \todo Mudar o tipo da variável de Coord2 para
		  * algum outro, por exemplo Eigen::Vector2d.
		  */
  double orientation; /**< \brief Orientação da região.*/
  bool symetric; /**< \brief Diz se a região é simétrica.*/
  REG_COLOR colorID; /**< \brief Indica a cor da região.*/
  int nPixel;  /**< \brief Indica o numero de pixels da região.*/
};

/* struct WIDEREGION{ */
/*   Coord3 pose;  */
/*   REG_COLOR colorID; */
/*   int nPixel; */
/*   bool symetric; */
/* }; */

class Acquisition : public FutData
{
private:
  // Atributos que estavam em Camera
  unsigned int width, height;
  ImagemRGB ImBruta; // Tem que ser um ponteiro porque ImagemRGB não tem construtor default

  // Atributos que estavam em FutData
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

private:
#ifndef _SO_SIMULADO_
  // ImagemRGB image;
  Homografia Homography;
  DistRadial *RDistortion;
  unsigned int MinU, MaxU, MinV, MaxV;
  PARAMETROS_CALIBRACAO calibracaoParam;

#endif
  IDQUADRO id_ant;
  bool saveNextImage;
  //FUNCTIONS
#ifndef _SO_SIMULADO_
  REGION seedFill( REG_COLOR colorID, unsigned int u, unsigned int v);
  bool processGameState();  
#endif
  // bool readGameState();
  bool calculaMinhaPose(REGION regTeam, double angBusca,
			double angCorrecao,int &index, POS_ROBO &teamPose);
  bool calculaMinhaPoseAproximada(REGION regTeam,double angCorrecao,
				  int &index, POS_ROBO &teamPose);
  bool calculaPoseAdv(REGION regTeam, int &index,POS_ROBO &teamPose,
			     double corrX, double corrY, double corrTheta);

private:
  // Metodos que estavam em Camera
 inline unsigned int Width() {return width;};
 inline unsigned int Height() {return height;};
public:
  Acquisition(ros::NodeHandle& n, ros::NodeHandle& pn);
  ~Acquisition();
  bool configAcquisition(const char *str);
  // bool acquisitionWait();
  // bool acquisitionCapture();
  bool acquisition();
  void save_image();
  PxRGB* getImgPointer () {return (ImBruta.getRawData());};
  inline void get_pos(CONFIG& pos_out){pos_out = pos;}
};

#endif

