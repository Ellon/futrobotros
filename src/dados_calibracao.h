/** \file dados_calibracao.h
 *
 *  \brief Arquivo que define os dados comuns entre o programa
 *  principal main e o calibrador.
 *
 * 
 */
#ifndef _DADOS_CALIBRACAO_
#define _DADOS_CALIBRACAO_

#include <cstdio>
#include <matrix.h>
#include <imagem.h>

#define FATOR_CONVERSAO 100000.0 /**< Fator de conversão para escrever
				    os pontos de mundo em
				    arquivo. Necessário para não
				    salvar esses pontos em formato
				    decimal. Usado na escrita e
				    leitura dos pontos em arquivo.*/

/** \brief Estrutura que define uma reta entre dois pontos notáveis.
 *
 * A reta sempre liga dois pontos notáveis. O indice dos pontos
 * notáveis é armazenado nas variáveis p1 e p2.
 */
struct RETA {
  unsigned p1; /**< Indice do ponto notável de inicio da reta.*/
  unsigned p2; /**< Indice do ponto notável do fim da reta.*/
};

/** \brief Limites de um componente de cor.
 *
 * A estrutura define os limites máximos e mínimos para uma componente
 * de cor. Usado na estrutura limitesHPG.
 */
struct limites {
  int max; /**< Valor máximo.*/
  int min; /**< Valor mínimo.*/
};

/** \brief Limites de uma cor no formato HPG.
 *
 * Estrutua que define uma cor. A estrutura armazena os limites
 * máximos e mínimos para cada uma das componentes do pixel: H, P e
 * G. Um pixel é dito como sendo desta cor se cada uma das suas
 * componentes tiver um valor entre o mínimo e o máximo.
 *
 */
struct limitesHPG {
  limites H; /**< Limites da componente H. */
  limites P; /**< Limites da componente P. */
  limites G; /**< Limites da componente G. */    
};

/** \brief Parâmetros de calibração do futebol de robôs.
 *
 * Estrutura que define a informação salva pelo calibrador, e que é
 * lida pelo programa principal durante a partida. Define metodos de
 * leitura e escrita, além de metodos para dizer se um pixel pertence
 * a uma determinada cor.
 * 
 */
struct PARAMETROS_CALIBRACAO {
  /** \brief Numero de pontos notáveis da aplicação.
   *
   *  No futebol de robôs são 28 pontos, mas alguma outra aplicação
   *  pode ter um numero diferente de pontos notáveis.
   */
  unsigned nPontosNotaveis; 
  /** \brief Numero de cores utilizadas pela da aplicação.
   *
   *  No futebol de robôs são 11 cores: 1 da bola (laranja), 2 do time
   *  (azul e amarelo), 3 auxiliares (ciano, rosa e verde), 1 cor do
   *  campo (preto), 1 cor da linha (branco), e 3 cores dos
   *  adversários. Mas alguma outra aplicação pode ter um numero
   *  diferente de cores.
   */
  unsigned nCores;
  /** \brief Vetor dos pontos na imagem, definidos através de calibração.
   *
   * Cada um dos pontos na imagem vai corresponder a um dos pontos de
   * mesmo indice no vetor pontosReais.
   *
   * \todo Mudar o tipo do vetor, de Coord2 para algum tipo da
   * biblioteca Eigen, por exemplo Eigen::Vector2d.
   */
  Coord2 *pontosImagem;
  /** \brief Vetor com a posição dos pontos reais.
   *
   * O valor dos pontos reais dependem da aplicação. Os valores destes
   * pontos são definidos em um arquivo lido pelo calibrador,
   * arquivo esse que depende da aplicação. 
   * 
   * Cada ponto real corresponde ao ponto de mesmo indice no vetor
   * pontosImagem.
   *
   * \todo Mudar o tipo do vetor, de Coord2 para algum tipo da
   * biblioteca Eigen, por exemplo Eigen::Vector2d.
   */ 
  Coord2 *pontosReais; 
  /** \brief Limiar inferior de Pureza de um pixel. 
   *
   * Qualquer pixel com valor P menor que limiarPInf vai ser
   * considerado um tom de cinza (P=0).
   */
  int limiarPInf; 
  /** \brief Limiar superior de Pureza de um pixel.
   * 
   * Qualquer pixel com valor P maior que limiarPSup vai ser
   * considerado uma cor pura (P=100).
   */
  int limiarPSup; 
  /** \brief Vetor com os limites que definem cada uma das cores
   * utilizadas pela aplicação.
   */
  limitesHPG *limHPG; 
  PARAMETROS_CALIBRACAO();
  ~PARAMETROS_CALIBRACAO();
  bool read(const char* arquivo);
  bool write(const char* arquivo) const;
  int getSoftColor(const PxRGB &p) const;
  int getHardColor(const PxRGB &p) const;
};

#endif
