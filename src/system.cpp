#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <sys/sem.h>
#include <sys/time.h>
#include "system.h"

static struct termios stored; /**< Variável utilizada para salvar o
				 estado do termios pela função
				 set_keypress(), para que possa ser
				 restaurado pela função
				 reset_keypress(). */

/** \brief Lê o tempo atual em segundos
 *
 * \retval tempo_em_segundos
 */
double relogio(void)
{
  static struct timeval tbase={-1,-1};
  struct timeval t;

  gettimeofday(&t,NULL);
  if (tbase.tv_sec==-1 && tbase.tv_usec==-1)
    {
      tbase = t;
    }
  return( t.tv_sec-tbase.tv_sec + (t.tv_usec-tbase.tv_usec)/1000000.0 );
}

/** \brief Função para entrar em uma região crítica
 * 
 * Decrementa um semáforo para entrar na região critica.
 *
 * \todo Entender bem como essa função funciona para documentar direito.
 *
 * \param sem ID do semáforo (???)
 * \retval -1 em caso de erro
 * \retval 0 caso contrário
 */ 
int entrar_regiao_critica(int sem)
{
  struct sembuf x;
  x.sem_num = 0;
  x.sem_op = -1;
  x.sem_flg = 0;
  if (semop(sem,&x,1)==-1)
    {
      return(-1);
    }
  return(0);
}

/** \brief Função para sair de uma região crítica
 * 
 * Incrementa um semáforo ao sair da região critica.
 *
 * \todo Entender bem como essa função funciona para documentar direito.
 *
 * \param sem ID do semáforo (???)
 * \retval -1 em caso de erro
 * \retval 0 caso contrário
 */ 
int sair_regiao_critica(int sem)
{
  struct sembuf x;
  x.sem_num = 0;
  x.sem_op = +1;
  x.sem_flg = 0;
  if (semop(sem,&x,1)==-1)
    {
      return(-1);
    }
  return(0);
}

/** \brief Coloca o terminal no modo de ler teclas sem ENTER depois.
 *
 * Salva o estado atual do terminal antes de modificá-lo. O terminal
 * pode ser restaurado ao seu estado normal chamando a função
 * reset_keypress().
 * 
 */
void set_keypress(void)
{
  struct termios newterm;
  
  tcgetattr(0,&stored);
  
  memcpy(&newterm,&stored,sizeof(struct termios));
  
  /* Disable canonical mode, and set buffer size to 1 byte */
  newterm.c_lflag &= (~ICANON);
  newterm.c_cc[VTIME] = 10;
  newterm.c_cc[VMIN] = 0;
  
  tcsetattr(0,TCSANOW,&newterm);
  return;
}

/** \brief Restaurar o terminal ao seu modo normal
 *
 * Restaura o terminal ao estado salvo pela função set_keypress().
 * 
 */
// Restaurar o terminal ao seu modo normal
void reset_keypress(void)
{
  tcsetattr(0,TCSANOW,&stored);
  return;
}

/** \brief Captura um catactere do teclado.
 *
 * \todo Verificar o pq de usar essa função e não o "getchar"
 * direto. E verificar se essa função ainda é chamada no código ou se
 * pode ser deletada.
 */
int pega_caracter(void) {
  return getchar();
};

