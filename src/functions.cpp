#include "functions.h"

/** \brief Função que retorna um valor aleatório baseado em uma
 * distribuição gaussiana
 *
 * Função baseada em uma equação encontrada na wikipedia. Cada chamada
 * usa um par de valores aleatórios U e V, gerados pela função
 * drand48(), em uma equação mágica para gerar os valores aleatórios
 * baseados em uma distribuição gaussiana. Cada nova chamada alterna a
 * equação utilizada: uma vez a equação com o cosseno, outra com o
 * seno. A cada duas chamadas, os valores das variáveis U e V são
 * gerados novamente, utilizando a função drand48().
 *
 * \todo Colocar na documentação a formula utilizada para gerar o
 * valor aleatório.
 *
 * \param[in] med valor médio
 * \param[in] desv desvio padrão
 */
double rnd_gauss(const double &med, const double &desv)
{
  //Variáveis internas declaradas como static para o seu valor se
  //manter entre as diversas chamadas da desta função.

  //Switch para chavear entre a equação com o cosseno ou a equação com
  //o seno
  static double swt = false; 
  
  //Variáveis aleatórias U e V
  static double U=drand48(); 
  static double V=drand48();

  //Valor normal
  static double normal;

  if(swt){
    //Se swt for true, recalcule U e V...
    U=drand48();
    V=drand48();
    //... e utilize a equação com o cosseno para calcular o valor
    //normal.
    normal = sqrt(-2.0*log(U))*cos(2.0*M_PI*V);
    // Por fim, faça swt=false, assim na proxima chamada a função vai
    // executar o calculo do normal utilizando o a função seno.
    swt = false;
  }else{
    // Calcule a normal utilizando U e V calculados na execução
    // anterior da função e a equação que usa o seno ...
    normal = sqrt(-2.0*log(U))*sin(2.0*M_PI*V);
    //... e faça swt = true pra na proxima execução recalcular U e V,
    //e executar o calculo da normal utilizando a função cosseno.
    swt = true;
  }

  // Calcula o valor gaussiano aleatório usando o valor normal, o
  // desvio padrão e a média.
  return normal*desv + med;
  
}
