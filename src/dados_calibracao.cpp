#include "dados_calibracao.h"
#include "functions.h"

using namespace std;

/** \brief Construtor padrão.
 *
 */
PARAMETROS_CALIBRACAO::PARAMETROS_CALIBRACAO() :
  nPontosNotaveis(0),
  nCores(0),
  pontosImagem(NULL),
  pontosReais(NULL),
  limiarPInf(0),
  limiarPSup(100),
  limHPG(NULL)
{
  
}

/** \brief Destrutor.
 *
 * Deleta as variáveis criadas dinamicamente. Verifica se a variável é
 * nula antes de deletar já que os vetores são criados na função
 * PARAMETROS_CALIBRACAO::read(const char*).
 * 
 */
PARAMETROS_CALIBRACAO::~PARAMETROS_CALIBRACAO(){
  if(pontosImagem != NULL)
    delete[] pontosImagem;

  if(pontosReais != NULL)
    delete[] pontosReais;

  if(limHPG != NULL)
    delete[] limHPG;
}

/** \brief Lê os parâmetros de calibração de um arquivo.
 *
 * O método lê numero de pontos notáveis e o numero de cores, usando
 * essa informação para criar o vetor de pontos e o vetor dos limites
 * que definem as cores usadas na aplicação. Esses vetores são
 * preenchidos com valores também lidos deste arquivo.
 *
 * Todo o procedimento de leitura é feito inicialmente em variáveis
 * auxiliares, e somente passadas para as variáveis internas da
 * estrutura quando a leitura do arquivo termina sem erros. Isso evita
 * inconsistencia nos dados, caso alguma variável interna fosse
 * alterada antes de um erro na escrita do arquivo ser detectada.
 *
 * O arquivo deve estar no formato definido pelo método
 * PARAMETROS_CALIBRACAO::write(const char*). 
 *
 * \param[in] arquivo Caminho do arquivo que contem os parametros de
 * calibração a serem lidos.
 * 
 * \retval true em caso de erro.
 * \retval false caso contrário.
 */
bool PARAMETROS_CALIBRACAO::read(const char* arquivo)
{
  //Declara as variáveis auxiliares.
  unsigned nPontosNotaveis_aux;
  unsigned nCores_aux;
  Coord2 *pontos_aux;
  Coord2 *pontosReais_aux;
  int limiarPInf_aux;
  int limiarPSup_aux;
  limitesHPG *limHPG_aux;

  bool OK = true; // Variável que indica se a ocorreu algum erro de
		  // leitura no arquivo.
  FILE *arq = fopen(arquivo,"r"); //Abre o arquivo.
  if(arq == NULL){ // Testa se ocorreu conseguiu abrir o arquivo.
    return true; // Erro
  }
  // Lê o numero de pontos
  if(fscanf(arq,"Numero de Pontos: %d\n",&nPontosNotaveis_aux) != 1 ) {
    OK = false;
  }

  if(OK){
    // Cria os vetores de pontos com o tamanho lido.
    pontos_aux = new Coord2[nPontosNotaveis_aux];
    pontosReais_aux = new Coord2[nPontosNotaveis_aux];
  }
  
  // Lê uma sequencia de caracteres pre-determinado, que escrito pelo
  // metodo write para facilitar a leitura do arquivo por humanos.
  if(fscanf(arq,"Pontos Notaveis (MundoX, MundoY, ImagemX, ImagemY):\n") != 0 ) {
    OK = false;
  }
  
  // Lê os valores dos pontos notáveis no mundo e na imagem.
  for(unsigned i = 0; i < nPontosNotaveis_aux; i++){
    if(OK && fscanf(arq,"%lf %lf %lf %lf\n",
		    &pontosReais_aux[i].X,
		    &pontosReais_aux[i].Y,
		    &pontos_aux[i].X,
		    &pontos_aux[i].Y) != 4){ 
      OK = false;
    }else{
      // Se leu um novo ponto, converte cada coordenada de ponto real
      // lido pelo fator de conversão.
      pontosReais_aux[i].X = pontosReais_aux[i].x()/FATOR_CONVERSAO;
      pontosReais_aux[i].Y = pontosReais_aux[i].y()/FATOR_CONVERSAO;
    }
  }
  

  // Lê uma sequencia de caracteres pre-determinado, que escrito pelo
  // metodo write para facilitar a leitura do arquivo por humanos.
  if(OK && fscanf(arq,"Limites de P Inferior e Superior:\n") != 0 ){ OK = false; }
  //Lê os valores dos limiares superiores e inferiores de P
  if(OK && fscanf(arq,"%d %d\n",&limiarPInf_aux,&limiarPSup_aux) != 2 ){ OK = false; }
  //Lê o numero de cores da aplicação.
  if(fscanf(arq,"Numero de Cores: %d\n",&nCores_aux) != 1 ) {
    OK = false;
  }
  if(OK){
    //Cria o vetor que vai conter as definições das cores.
    limHPG_aux = new limitesHPG[nCores_aux];
  }
  
  // Lê uma sequencia de caracteres pre-determinado, que escrito pelo
  // metodo write para facilitar a leitura do arquivo por humanos.
  if(OK && fscanf(arq,"Limites HPG das Cores\n") != 0){ OK = false; }
  // Lẽ os limites maximos e mínimos que definem cada cor utilizada
  // pela aplicação.
  for(unsigned i = 0; i < nCores_aux; i++){
    if(OK && fscanf(arq,"%d %d %d %d %d %d\n",
		    &(limHPG_aux[i].H.min),
		    &(limHPG_aux[i].H.max),
		    &(limHPG_aux[i].P.min),
		    &(limHPG_aux[i].P.max),
		    &(limHPG_aux[i].G.min),
		    &(limHPG_aux[i].G.max)) != 6) { OK = false; }
  }
  fclose(arq); // Fecha o arquivo.

  if(!OK) return true; // Se o arquivo não foi lido com sucesso,
		       // retorna erro.

  // Se chegou até aqui, então o arquivo foi lido com sucesso. Agora
  // passe todos os valores das variáveis auxiliáres para as variáveis
  // da estrutura.
  nPontosNotaveis = nPontosNotaveis_aux;
  nCores = nCores_aux;
  pontosImagem = new Coord2[nPontosNotaveis];
  pontosReais =  new Coord2[nPontosNotaveis];
  for(unsigned i = 0; i < nPontosNotaveis_aux; i++){
    pontosImagem[i] = pontos_aux[i];
    pontosReais[i] = pontosReais_aux[i];
  }
  limiarPInf = limiarPInf_aux;
  limiarPSup = limiarPSup_aux;
  limHPG = new limitesHPG[nCores];
  for(unsigned i = 0; i < nCores_aux; i++){
    limHPG[i] = limHPG_aux[i];
  }
  return false; //Sem erros.
}

/** \brief Escreve os parâmetros de calibração em um arquivo.
 *
 * O arquivo gerado vai conter os parâmetros de calibração baseados
 * nas variáveis da estrutura e pode ser lido com o método
 * PARAMETROS_CALIBRACAO::read(const char*).
 *
 * O arquivo de saída vai conter algumas strings pre-definidas, para
 * facilitar a leitura do arquivo por humanos.
 *
 * \param[in] arquivo Caminho do arquivo onde serão escritos os
 * parâmetros de calibração.
 * 
 * \retval true em caso de erro
 * \retval false caso contrário
 */
bool PARAMETROS_CALIBRACAO::write(const char* arquivo) const{
  FILE *arq=fopen(arquivo,"w"); //Abre o arquivo
  if(arq == NULL) return true; //Retorna erro caso não consiga abrir o
			       //arquivo
  
  //Escreve o numero de pontos notáveis
  fprintf(arq,"Numero de Pontos: %d\n",nPontosNotaveis);
  //Escreve os valores X e Y dos pontos notáveis reais e na imagem.
  fprintf(arq,"Pontos Notaveis (MundoX, MundoY, ImagemX, ImagemY):\n");
  for(unsigned i = 0; i < nPontosNotaveis; i++){
    fprintf(arq,"%d %d %d %d\n",
	    (int)(pontosReais[i].x()*FATOR_CONVERSAO),
	    (int)(pontosReais[i].y()*FATOR_CONVERSAO),
	    (int)pontosImagem[i].u(),
	    (int)pontosImagem[i].v());
  }
  //Escreve os limites superior e inferiore de P
  fprintf(arq,"Limites de P Inferior e Superior:\n");
  fprintf(arq,"%d %d\n",limiarPInf,limiarPSup);
  //Escreve o numero de cores utilizadas pela aplicação.
  fprintf(arq,"Numero de Cores: %d\n",nCores);
  //Escreve os limites das componentes que definem uma cor, para cada
  //cor utilizada pela aplicação.
  fprintf(arq,"Limites HPG das Cores\n");
  for(unsigned i = 0; i < nCores; i++){
    fprintf(arq,"%d %d %d %d %d %d\n",
	    limHPG[i].H.min,		    
	    limHPG[i].H.max,		    
	    limHPG[i].P.min,		    
	    limHPG[i].P.max,		    
	    limHPG[i].G.min,		    
	    limHPG[i].G.max);
  }
  fclose(arq); //Fecha o arquivo.
  return false; //Sem erros.
}

/** \brief Método para verificar se um pixel pertence a uma
 * determinada cor.
 *
 * O método extrai os valores HPG do pixel passado no argumento (que
 * são doubles) e converte-os para inteiros, escalando o H para um
 * valor entre -180 e +180, e P e G para valores entre 0 e 100. Em
 * seguida o método testa se cada cada uma destas componentes se
 * encontra dentro dos limites minimos e máximos das componentes que
 * definem uma cor. Se as três componentes estiverem dentro destes
 * limites, retorna o indice desta cor. Caso o pixel não pertença a
 * nenhuma das cores definidas no vetor limHPG, retorna -1.
 *
 * Note que o teste se o pixel pertence a uma cor é feito
 * sequêncialmente, da cor de indice 0 para a de indice N. Ou seja, se
 * for verificado que um pixel pertence a uma cor, o método retornará
 * o indice desta cor, mesmo que haja outras cores de indice maior
 * cujo teste também seria bem sucedido.
 * 
 * \par Informação sobre a verificação da crominância:
 * \n O teste se um pixel tem o valor de crominância (H) dentro de um
 * certo limite é feito de uma forma um pouco diferente do teste
 * trivial utiizado nas componentes P e G. Como a crominância é uma
 * componente circular, é necessário formas de detectar cores com H
 * próximo da descontinuidade em -180 e 180 (ou seja, a cor
 * "ciano"). Para isso, é feito o seguinte teste: Se o limite H.min de
 * uma cor for menor do que o limite H.max, o teste é realizado
 * normalmente (ou seja, de o H do pixel está dentro desse
 * limite). Mas já se o limite H.min for MAIOR que o limite H.max, o
 * teste vai ser se o H do pixel é maior que o valor mínimo OU menor
 * que o valor máximo. Assim, cores onde a faixa de crominância
 * engloba a descontinuidade também poderão ser identificadas.
 * 
 * \param[in] p pixel a ser verificado.
 *
 * \retval indice_da_cor Indice da primeira cor cujas componentes HPG
 * do pixel estejam dentro dos limites minimos e máximos que definem
 * aquela cor.  
 *
 * \retval -1 Caso pixel não pertença a nenhuma cor definida no vetor
 * limHPG.
 */
//retorna a cor da qual o pixel do parametro percente. Usa somente os
//limiares dos componentes HPG para definir a cor do pixel.
int PARAMETROS_CALIBRACAO::getHardColor(const PxRGB &p) const
{
  //Declara variaveis auxiliares do metodo.
  bool H_OK;
  float H,P,G;
  p.getHPG(H,P,G); // Pega os valores HPG do pixel.

  //Converte o valor HPG de double para int
  int iH = (int)round((H/M_PI)*180.0),
    iP = (int)round(P*100.0),
    iG = (int)round(G*100.0);

  //Faz um looping em todas as cores definidas na estrutura, testando
  //se suas componentes estão dentro do limite minimo e máximo.
  for( unsigned k = 0; k < nCores; k++){
    //Verifica se a crominancia está dentro dos limites. Existem dois
    //testes para tratar a descontinuidade em -180 e 180. Somente um
    //dos dois testes será realizado, dependendo dos valores dos
    //limites H.min e H.max de cada cor.
    H_OK = false;	
    if( limHPG[k].H.min <= limHPG[k].H.max ){
      if(iH >= limHPG[k].H.min && iH <= limHPG[k].H.max){
	H_OK = true;
      }
    }else{
      if(iH >= limHPG[k].H.min || iH <= limHPG[k].H.max){
	H_OK = true;
      }
    }
    
    //Verifica se o teste do H foi bem sucedido e também se os valores
    //de P e G estão dentro dos limites.
    if(H_OK &&
       iP >= limHPG[k].P.min && iP <= limHPG[k].P.max &&
       iG >= limHPG[k].G.min && iG <= limHPG[k].G.max){
      //Se estiverem, então retorna o indice da cor.
      return k;
    }
  }

  //Se chegar até aqui, então o pixel não foi reconhecido como
  //pertencendo a nenhuma cor.
  return -1;

}

/** \brief Método para verificar se um pixel pertence a uma
 * determinada cor, retornando uma "cor próxima" caso o pixel não
 * pertença a nenhuma das cores conhecidas.
 *
 * Este método chama o método
 * PARAMETROS_CALIBRACAO::getHardColor(const PxRGB &) const para
 * verificar se o pixel pertence a alguma das cores conhecidas. Caso
 * não pertença, o método calcula a distancia de Mahalanobis para cada
 * componente HPG do pixel. Então o método calcula a distancia
 * euclidiana utilizando a distancia de Mahalanobis de cada componente
 * para achar uma "distancia do pixel até a cor". Por fim, ele retorna
 * o indice da cor cujo "pixel está mais próximo", se essa distância
 * for menor que o threshold de 1 desvio padrão. Caso contrário,
 * retorna -1, que indica uma cor indefinida.
 * 
 * \todo Colocar as formulas que explicam o calculo da distancia de
 * Mahalanobis na documentação.
 * 
 * \warning O método não foi testado exaustivamente para justificar
 * seu uso. A ideia foi implementada e os testes realizados não deram
 * muito certo. Isso pode indicar que existe algum erro no código da
 * função ou a ideia de usar a distancia de Mahalanobis para definir a
 * cor não é, de fato, uma boa ideia.
 *
 * \param[in] p pixel a ser verificado.
 *
 * \retval indice_da_cor Indice da primeira cor cujas componentes HPG
 * do pixel estejam dentro dos limites minimos e máximos que definem
 * aquela cor, ou então o indice da cor mais próxima e com distância
 * menor do que um desvio padrão.
 * 
 * \retval -1 Caso não exista uma cor próxima o suficiente.
 */
int PARAMETROS_CALIBRACAO::getSoftColor(const PxRGB &p) const
{
  //Verifica se existe uma cor cujo pixel se se encontra dentro dos seus
  //limiares.
  int min_ID = getHardColor(p); 
  // Se existir, retorne o seu ID.
  if(min_ID != -1)
    return min_ID;
  
  //Se chegou até aqui, o pixel não foi considerado como pertencendo a
  //nenhuma das cores conhecidas. Vamos então calcular a menor
  //distância.

  bool H_OK;
  float H,P,G;
  p.getHPG(H,P,G); // Pega os valores HPG do pixel.
  float distH, fHmin, fHmax, fHmean, fHstd;
  float meanP, stdP, distP;
  float meanG, stdG, distG;

  float dist;
  // Declara a distancia mínima inicial, que tecnicamente é um valor
  // maior do que a maior distância possivel de se obter através dos
  // calculos, mas é necessário verificar.
  float min_dist = 400.0; // min_dist inicial > sqrt(360*360 + 100*100
			  // + 100*100);

  //Calcula o valor inteiro das componentes HPG, convertendo o H para
  //uma faixa de -180 a 180, e o P e G para uma faixa de 0 a 100.
  int iH = (int)round((H/M_PI)*180.0),
    iP=(int)round(P*100.0),
    iG=(int)round(G*100.0);

  //Faz um loop pelas cores, calculando a distancia do pixel para cada
  //uma delas e alterando o indice da cor de menor distância caso a
  //distancia calculada numa iteração seja menor do que a calculada na
  //iteração anterior.
  for( unsigned k = 0; k < nCores; k++){
    H_OK = false;
    // Testa se o H esta dentro dos limiares, se estiver, atribui uma
    // "distancia H" de zero.
    if( limHPG[k].H.min <= limHPG[k].H.max ){
      if(iH >= limHPG[k].H.min && iH <= limHPG[k].H.max){
	H_OK = true;
	distH = 0;
      }
    }else{
      if(iH >= limHPG[k].H.min || iH <= limHPG[k].H.max){
	H_OK = true;
	distH = 0;
      }
    }
    
    // Se o H estiver fora dos limiares, calcula a distancia
    if(!H_OK){
      //Converte os limites da componente H para os valores double
      //equivalentes.
      fHmin = (((float)limHPG[k].H.min)/180.0)*M_PI;
      fHmax = (((float)limHPG[k].H.max)/180.0)*M_PI;

      //Calcula o valor médio de H
      fHmean = (ang_equiv(fHmax + fHmin))/2;
      //Calcula o desvio padrão de H, ou seja, a distancia da média
      //até o limite..
      fHstd = ang_equiv(fHmax - fHmean);
      //Calcula a distancia de Mahalanobis para a componente H
      distH = min(ang_equiv(H-fHmin),
		  ang_equiv(H-fHmax))/fHstd;
    }

    //Verifica se a componente P está dentro dos limites.
    if(iP >= limHPG[k].P.min && iP <= limHPG[k].P.max){
      distP = 0; //Atribui a distancia 0 se estiver.
    }else{
      //Se não estiver, calcula a "distancia P" se os valores dos
      //limites P forem válidos. O valor dos limites P serão inválidos
      //se P.min for maior que P.max.
      if(limHPG[k].P.min < limHPG[k].P.max){
	//Calcula o ponto médio na faixa de P.
	meanP = (limHPG[k].P.min + limHPG[k].P.max)/2;
	//Calcula o desvio padrão na faixa de P, ou seja, a distancia
	//do ponto médio até os limites.
	stdP = limHPG[k].P.max - meanP;
	//Calcula a distancia de Mahalanobis. Ambos os casos em que o
	//valor de P é maior que o valor máximo ou menor que o valor
	//mínimo são tratados abaixo.
	if(iP > limHPG[k].P.max){
	  distP = (iP - limHPG[k].P.max)/stdP;
	}else if(iP < limHPG[k].P.min){
	  distP = (limHPG[k].P.min - iP)/stdP;
	}
      }else{
	//Se os limites de P não forem válidos, atribui a distância
	//máxima para a distancia P.
	distP = 100;
      }
    }

    //Verifica se a componente G está dentro dos limites.
    if(iG >= limHPG[k].G.min && iG <= limHPG[k].G.max){
      distG = 0; //Atribui a distancia 0 se estiver.
    }else{
      //Se não estiver, calcula a "distancia G" se os valores dos
      //limites G forem válidos. O valor dos limites G serão inválidos
      //se G.min for maior que G.max.
      if(limHPG[k].G.min < limHPG[k].G.max){
	//Calcula o ponto médio na faixa de G.
	meanG = (limHPG[k].G.min + limHPG[k].G.max)/2;
	//Calcula o desvio padrão na faixa de G, ou seja, a distancia
	//do ponto médio até os limites.
	stdG = limHPG[k].G.max - meanG;
	//Calcula a distancia de Mahalanobis. Ambos os casos em que o
	//valor de G é maior que o valor máximo ou menor que o valor
	//mínimo são tratados abaixo.
	if(iG > limHPG[k].G.max){
	  distG = (iG - limHPG[k].G.max)/stdG;
	}else if(iG < limHPG[k].G.min){
	  distG = (limHPG[k].G.min - iG)/stdG;
	}
      }else{
	//Se os limites de G não forem válidos, atribui a distância
	//máxima para a distancia G.
	distG = 100;
      }
    }
    // Calcula a distancia geral usando a distancia euclidiana.
    dist = sqrt(distH*distH + distP*distP + distG*distG);
    
    //Verifica se a distancia está abaixo do threshold de 1 desvio
    //padrao e se a distancia para a cor é menor do que a menor
    //distancia encontrada
    if(dist <= 1.0 && dist < min_dist){
      //Se for, atualiza a menor distancia e o id da cor mais próxima
      //para o ID da cor atualmente sendo verificada.
      min_dist = dist;
      min_ID = k;
    }
  }
  //Retorna a cor de menor distancia. Retorna -1 (cor indefinida)
  //quando nao encontrou nenhuma cor próxima o suficiente.
  return min_ID;
}
