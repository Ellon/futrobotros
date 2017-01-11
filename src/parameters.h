/** \file parameters.h
 *  \brief Define os parâmetros globais aos programas do futrobot
 *
 * No arquivo são definidos as dimensões do campo, dimensões do robô,
 * dimensões da imagem, altura da camera e alguns outras definições
 * uteis.
 *
 * No código, todos os defines com "WIDTH" são relativos a "largura"
 * do campo, ou seja, o eixo X, enquanto "HEIGHT" se refere a altura,
 * ou seja, a dimensão em relação ao eixo Y.
 * 
 * \author Ellon Paiva (ellonpaiva@gmail.com)
 */

#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

// Dimensões do campo (m)
#define FIELD_WIDTH 1.50 /**< Largura do campo em metros*/
#define FIELD_HEIGHT 1.30 /**< Altura do campo em metros*/

// Dimensões da trave
#define GOAL_WIDTH 0.10 /**< Profundidade do gol em metros*/
#define GOAL_HEIGHT 0.40 /**< Altura do gol em metros*/

// Largura/altura da quina
#define CORNER_DIMENSION 0.07 /**< Dimensão da lateral do pequeno
				 triangulo que forma a quina do campo,
				 em metros*/

// Raio do círculo central
#define CIRCLE_RADIUS 0.20 /**< Raio do circulo central, em metros.*/

// Dimensões da grande área
#define GOAL_FIELD_HEIGHT 0.70 /**< Altura da area do goleiro, em
				  metros*/
#define GOAL_FIELD_WIDTH 0.15 /**< Largura da area do goleiro, em
				 metros*/
#define ARC_HEIGHT 0.20 /**< Altura do arco na frente da area, em
			   metros*/
#define ARC_WIDTH 0.05 /**< Largura do arco na frente da area, em
			  metros*/
#define ARC_APOTHEM (ARC_HEIGHT*ARC_HEIGHT-4*(ARC_WIDTH*ARC_WIDTH))/(8*ARC_WIDTH) /**< Distancia entre o centro do circulo que forma o arco da área e a borda da área*/
#define ARC_RADIUS (ARC_WIDTH + ARC_APOTHEM) /**< Raio do circulo que forma o arco da área*/

//Bolas paradas
#define PK_X 0.375 /**< Coordenada X da posição de penalti, em metros*/
#define PK_Y 0.0 /**< Coordanada Y da posição de penalti, em metros*/
#define FB_X 0.375 /**< Coordenada X da posição de bola livre*/
#define FB_Y 0.4 /**< Coordenada Y da posição de bola livre */
#define DIST_FB 0.2 /**< Distancia mínima que os robôs tem que ficar
		       da posição de bola livre, durante um lance de
		       bola livre*/

//Câmera
#define IMAGE_WIDTH 640 /**< Largura da imagem da camera, em pixels*/
#define IMAGE_HEIGHT 480 /**< Altura da imagem da camera, em pixels*/
#define CAMERA_HEIGHT 2.21 /**< Altura da camera em relação ao campo,
			      em metros*/

// Dimensões dos objetos (m)

// Robô
#define ROBOT_EDGE 0.075 /**< Largura do robô, em metros */
#define ROBOT_RADIUS (ROBOT_EDGE/sqrt(2.0)) /**< Diagonal do robô, em metros */
#define ROBOT_MARK_RADIUS 0.01875 /**< Raio da marca do robô, em metros*/
#define ROBOT_HEIGHT 0.083 /**< Altura do robô, em metros. Deveria ser
			      igual ROBOT_EDGE, mas devido a falta de
			      espaço e marca do robô, a altura do robô
			      é um pouco maior que a largura.*/

#define BALL_RADIUS 0.02135 /**< Raio da bola, em metros.*/
#define BALL_HEIGHT 0.03 /**< Altura da bola. Me pergunto porquê ela
			    seria diferente do seu raio.*/

#define VEL_BOLA_PARADA 0.025 /**< Velocidade máxima para o qual
				 considera-se que a bola está
				 parada. */

#endif
