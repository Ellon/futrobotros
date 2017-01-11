/** \file functions.h 
 *
 * \brief Define algumas funções utilizadas com frequência no futebol
 *  de robôs.
 */
#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <math.h>
#include <stdlib.h>
#include <time.h>

/** \brief Função sinal de um double
 *
 * Retorna o +1.0 se x for positivo, -1.0 se x for negativo e 0.0 se x
 * for igual a zero.
 *
 * \param[in] x variável a ter o sinal verificado
 * \retval +1.0 se x > 0.0
 * \retval -1.0 se x < 0.0
 * \retval 0.0 se x == 0.0
  */
static inline double sgn(double x) {
  return ( x>0.0 ? +1.0 : (x<0.0 ? -1.0 : 0.0) );
}

/** \brief Um arco tangente que funciona para ângulos de +- 90 graus
 *
 * Esta função simplesmente trata a singularidade do atan2 para o caso
 * de x ser igual a zero. 
 *
 * \param[in] y Cateto oposto ao ângulo.
 * \param[in] x Cateto adjacente ao ângulo.
 * \retval atan2(y,x) se x != 0.0
 * \retval pi/2 se x == 0.0 e y >= 0.0
 * \retval -pi/2 se x == 0.0 e y < 0.0
 */
static inline double arc_tang(double y, double x)
{
  return (x != 0.0 ? atan2(y,x) : (y>=0 ? M_PI/2.0 : -M_PI/2.0));
}

/** \brief Retorna um ângulo equivalente no intervalo entre -180 graus
 * e +180 graus
 *
 * Essa função deve ser utilizada para manter o valor de um angulo
 * dentro do intervalo de -pi e + pi. Recomenda-se utilizar a função
 * sempre que se realizar operações com valores que envolvam ângulos,
 * como subtração ou diferenças entre ângulos.
 *
 * \param[in] theta ângulo a ser corrigido.
 * \retval angulo_corrigido angulo equivalente entre -pi e +pi 
 */
static inline double ang_equiv(double theta)
{
  return (arc_tang(sin(theta), cos(theta)));
}

/** \brief Retorna um ângulo rebatido para o 1o ou o 4o quadrantes
 * 
 * \param[in] theta angulo a ser rebatido
 * \retval angulo_rebatido angulo rebatido no 1º ou 4º quadrante.
 */
static inline double ang_equiv2(double theta)
{
  return asin(sin(theta));
}

/** \brief Um tan que não dá erro quanto theta=90graus 
 *
 * \todo Verificar porquê a linha que usa a função fabs está
 * comentada. Do jeito que está, essa função não faz nada de mais.
 * 
 * \param[in] x ângulo 
 * \retval tan(x)
 */
static inline double mytan(double x) {
  return tan(x);
  //return fabs(ang_equiv2(x))<1.5608 ? tan(x) : 100.0;
}

/** \brief Eleva ao quadrado
 *
 * \param[in] x valor a ser elevado.
 * \retval x^2
 */
static inline double pow2(double x)
{
  return (x*x);
}

/** \brief Retorna o maior de dois valores.
 *
 * \param x,y 
 * \retval max(x,y) 
 */
static inline double mymax(double x, double y) {
  return (x>y ? x : y);
}
/** \brief Retorna o maior de três valores.
 *
 * \param x,y,z
 * \retval max(x,y,z) 
 */
static inline double mymax(double x, double y, double z) {
  return (x>y ? (x>z ? x : z) : (y>z ? y : z) );
}
/** \brief Retorna o menor de dois valores.
 *
 * \param x,y 
 * \retval min(x,y) 
 */
static inline double mymin(double x, double y) {
  return (x<y ? x : y);
}
/** \brief Retorna o menor de três valores.
 *
 * \param x,y,z
 * \retval min(x,y,z) 
 */
static inline double mymin(double x, double y, double z) {
  return (x<y ? (x<z ? x : z) : (y<z ? y : z) );
}

/** \brief Função degrau
 *
 * \param[in] x entrada
 * \retval 1.0 se x >= 0.0
 * \retval 0.0 caso contrário.
 */
static inline double d(double x){return (x>=0?1.0:0.0);};

/** \brief Função que satura um valor entre um mínimo e um máximo.
 *
 * \param[in] value valor a ser saturado
 * \param[in] min valor mínimo
 * \param[in] max valor máximo
 * \retval value se min <= value <= max
 * \retval min se value < min
 * \retval max se value > max
 */
static inline double sature(double value, double min, double max)
{return ((value < min)?(min):((value > max)?(max):(value)));};

double rnd_gauss(const double &med, const double &desv);

#endif
