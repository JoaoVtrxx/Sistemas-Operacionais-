// so.c
// sistema operacional
// simulador de computador
// so24b

// INCLUDES {{{1
#include "so.h"
#include "dispositivos.h"
#include "irq.h"
#include "programa.h"
#include "instrucao.h"
#include <time.h>

#include <stdlib.h>
#include <stdbool.h>

// CONSTANTES E TIPOS {{{1
// intervalo entre interrupções do relogio
#define INTERVALO_INTERRUPCAO 50   // em instruções executadas
#define QUANTUM 20

// ESTADOS PROCESSOS
#define PRONTO 0
#define EXECUTANDO 1
#define BLOQUEADO 2
#define MORTO 3

// MOTIVOS BLOQUEIO PROCESSO
#define BLOQUEIO_LEITURA 0
#define BLOQUEIO_ESCRITA 1
#define BLOQUEIO_ESPERA_PROC 2




typedef struct  {
    int num_processos_criados;
    double tempo_total_execucao;
    double tempo_ocioso;
    int interrupcoes[5]; 
    int num_preempcoes;
} metricas_so_t;

typedef struct {
    double tempo_criacao;
    double tempo_termino;
    int num_preempcoes;
    int transicoes_estado[4]; // PRONTO, EXECUTANDO, BLOQUEADO, MORTO
    double tempo_estado[4];   // Tempo total em cada estado
    double tempo_resposta_medio; // Calculado após o término
} metricas_processo_t;

typedef struct processo_t {
    int pid;
    int PC;
    int A;
    int X;
    int estado;
    int dispositivo;
    int motivo_bloqueio;
    int dado_escrever;
    
   metricas_processo_t *metricas;
} processo_t;

typedef struct nodo_fila_t {
    processo_t *processo;
    struct nodo_fila_t *proximo;
} nodo_fila_t;

typedef struct {
    nodo_fila_t *inicio;
    nodo_fila_t *fim;
    int tamanho;
} fila_t;

typedef struct {
    processo_t **tabela_processos; // Array de processos
    int num_processos;            // Número de processos na tabela
    int contador_pid;             // contador para o numero de serie PID
} tabela_processos_t;

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  es_t *es;
  console_t *console;
  bool erro_interno;
  tabela_processos_t *tabela;
  processo_t *processo_em_execucao; 
  // t1: tabela de processos, processo corrente, pendências, etc

  metricas_so_t *metricas;
  int quantum_restante;   
  fila_t *fila_prontos;
};

// DECLARACOES
static processo_t *processo_cria(so_t *self, int pid, int pc); 
static int processo_get_pid(processo_t *processo); 
static int processo_get_pc(processo_t *processo); 
static int processo_get_a(processo_t *processo);
static int processo_get_x(processo_t *processo);
static int processo_get_estado(processo_t *processo); 
static int processo_get_dispositivo(processo_t *processo);
static int processo_get_motivo_bloqueio(processo_t *processo);
static int processo_get_dado_escrever(processo_t *processo);
static void processo_set_pid(processo_t *processo, int pid);
static void processo_set_pc(processo_t *processo, int pc);
static void processo_set_a(processo_t *processo, int a);
static void processo_set_x(processo_t *processo, int x);
static void processo_set_estado(processo_t *processo, int estado);
static void processo_set_dispositivo(processo_t *processo, int dispositivo);
static void processo_set_motivo_bloqueio(processo_t *processo, int motivo_bloqueio);
static void processo_set_dado_escrever(processo_t *processo, int motivo_bloqueio);

static tabela_processos_t *tabela_cria();
static int tabela_adicionar_processo(tabela_processos_t *tabela, processo_t *processo);
static processo_t *tabela_buscar_processo(tabela_processos_t *tabela, int pid);
static void tabela_destroi(tabela_processos_t *tabela);
static void tabela_imprimir(tabela_processos_t *tabela);

static metricas_processo_t *metricas_processo_cria(double tempo_criacao);
static void metricas_processo_destroi(metricas_processo_t *metricas);
static void metricas_processo_atualiza_tempo_estado(metricas_processo_t *metricas, int estado, double tempo_decorrido);
static void metricas_processo_incrementa_transicao(metricas_processo_t *metricas, int estado);
static void metricas_processo_incrementa_preempcao(metricas_processo_t *metricas);
static void metricas_processo_calcula_tempo_resposta(metricas_processo_t *metricas);
static void metricas_processo_para_tempo_estado(metricas_processo_t *metricas, so_t *self, int estado);
static void metricas_processo_imprimir(processo_t *processo);
static void metricas_imprimir_todos_processos(tabela_processos_t *tabela);

static metricas_so_t *metricas_so_cria(so_t *self);
static void metricas_so_destroi(metricas_so_t *metricas);
static void metricas_so_incrementa_processos(metricas_so_t *metricas);
static void metricas_so_adiciona_tempo_ocioso(metricas_so_t *metricas, double tempo);
static void metricas_so_incrementa_interrupcoes(metricas_so_t *metricas, int tipo);
static void metricas_so_incrementa_preempcao(metricas_so_t *metricas);
static void metricas_so_adiciona_tempo_execucao(metricas_so_t *metricas, double tempo) ;
static void metricas_so_imprimir(metricas_so_t *metricas_so);

double tempo_atual(so_t *self);
fila_t *fila_cria();
void fila_inserir(fila_t *fila, processo_t *processo);
processo_t *fila_remover(fila_t *fila);
processo_t *fila_remover_por_pid(fila_t *fila, int pid);
bool fila_vazia(fila_t *fila);

// FIM DECLARACOES

// funcao de tratamento de interrupcao (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// carrega o programa contido no arquivo na memória do processador; retorna end. inicial
static int so_carrega_programa(so_t *self, char *nome_do_executavel);

// copia para str da memória do processador, até copiar um 0 (retorna true) ou tam bytes 

static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender);

// CRIAcaO {{{1

so_t *so_cria(cpu_t *cpu, mem_t *mem, es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  if (self == NULL) return NULL;

  self->cpu = cpu;
  self->mem = mem;
  self->es = es;
  self->console = console;
  self->erro_interno = false;
  self->processo_em_execucao = NULL;
  self->tabela = tabela_cria();
  self->metricas = metricas_so_cria(self);
  self->quantum_restante = QUANTUM;
  self->fila_prontos = fila_cria();

  // quando a CPU executar uma instrucao CHAMAC, deve chamar a funcao
  //   so_trata_interrupcao, com primeiro argumento um ptr para o SO
  cpu_define_chamaC(self->cpu, so_trata_interrupcao, self);

  // coloca o tratador de interrupcao na memória
  // quando a CPU aceita uma interrupcao, passa para modo supervisor, 
  //   salva seu estado à partir do endereço 0, e desvia para o endereço
  //   IRQ_END_TRATADOR
  // colocamos no endereço IRQ_END_TRATADOR o programa de tratamento
  //   de interrupcao (escrito em asm). esse programa deve conter a 
  //   instrucao CHAMAC, que vai chamar so_trata_interrupcao (como
  //   foi definido acima)
  int ender = so_carrega_programa(self, "trata_int.maq");
  if (ender != IRQ_END_TRATADOR) {
    console_printf("SO: problema na carga do programa de tratamento de interrupcao");
    self->erro_interno = true;
  }

  // programa o relogio para gerar uma interrupcao após INTERVALO_INTERRUPCAO
  if (es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO) != ERR_OK) {
    console_printf("SO: problema na programacao do timer");
    self->erro_interno = true;
  }

  return self;
}

void so_destroi(so_t *self)
{
  cpu_define_chamaC(self->cpu, NULL, NULL);
  tabela_destroi(self->tabela);
  metricas_so_destroi(self->metricas);
  free(self);
}

static int so_para(so_t *self)
{
  err_t e1, e2;
  e1 = es_escreve(self->es, D_RELOGIO_TIMER, 0);
  e2 = es_escreve(self->es, D_RELOGIO_INTERRUPCAO, 0);
  if (e1 != ERR_OK || e2 != ERR_OK)
  {
    console_printf("SO: nao foi possivel desligar o relogio");
    self->erro_interno = true;
  }

  metricas_so_adiciona_tempo_ocioso(self->metricas, tempo_atual(self));
  metricas_so_adiciona_tempo_execucao(self->metricas, tempo_atual(self));

  for(int i = 0; i < self->tabela->num_processos; i++){
    metricas_processo_calcula_tempo_resposta(self->tabela->tabela_processos[i]->metricas);
    metricas_processo_atualiza_tempo_estado(self->tabela->tabela_processos[i]->metricas, MORTO, tempo_atual(self));
  }
  metricas_imprimir_todos_processos(self->tabela);

  metricas_so_imprimir(self->metricas);

  return 1;
}

// TRATAMENTO DE INTERRUPcaO {{{1

// funções auxiliares para o tratamento de interrupcao
static void so_salva_estado_da_cpu(so_t *self);
static void so_trata_irq(so_t *self, int irq);
static void so_trata_pendencias(so_t *self);
static void so_escalona(so_t *self);
static int so_despacha(so_t *self);

// funcao a ser chamada pela CPU quando executa a instrucao CHAMAC, no tratador de
//   interrupcao em assembly
// essa é a única forma de entrada no SO depois da inicializacao
// na inicializacao do SO, a CPU foi programada para chamar esta funcao para executar
//   a instrucao CHAMAC
// a instrucao CHAMAC só deve ser executada pelo tratador de interrupcao
//
// o primeiro argumento é um ponteiro para o SO, o segundo é a identificacao
//   da interrupcao
// o valor retornado por esta funcao é colocado no registrador A, e pode ser
//   testado pelo código que está após o CHAMAC. No tratador de interrupcao em
//   assembly esse valor é usado para decidir se a CPU deve retornar da interrupcao
//   (e executar o código de usuário) ou executar PARA e ficar suspensa até receber
//   outra interrupcao
static bool todo_mundo_morto(so_t *self)
{
  for (int i = 0; i < self->tabela->num_processos; i++)
  {
    if (self->tabela->tabela_processos[i]->estado != MORTO)
    {
      return false;
    }
  }
  return true;
}

static int so_trata_interrupcao(void *argC, int reg_A)
{
  
  so_t *self = argC;
  irq_t irq = reg_A;
  // esse print polui bastante, recomendo tirar quando estiver com mais confiança
  // console_printf("SO: recebi IRQ %d (%s)", irq, irq_nome(irq));
  // salva o estado da cpu no descritor do processo que foi interrompido
  so_salva_estado_da_cpu(self);
  // faz o atendimento da interrupcao

  so_trata_irq(self, irq);
  // faz o processamento independente da interrupcao
  so_trata_pendencias(self);
  // escolhe o próximo processo a executar
  
  so_escalona(self);
  
  tabela_imprimir(self->tabela);
 
  if(todo_mundo_morto(self)){
    console_printf("SO: Todos os processos estao mortos...");
    return so_para(self); 
  }else{
     // recupera o estado do processo escolhido
    return so_despacha(self);
  }

  
}

static void so_salva_estado_da_cpu(so_t *self)
{
  // t1: salva os registradores que compõem o estado da cpu no descritor do
  //   processo corrente. os valores dos registradores foram colocados pela
  //   CPU na memória, nos endereços IRQ_END_*
  // se não houver processo corrente, não faz nada

  if(self->processo_em_execucao != NULL){
    

    int regA, regX, regPC;

    mem_le(self->mem, IRQ_END_A, &regA);
    mem_le(self->mem, IRQ_END_X, &regX);
    mem_le(self->mem, IRQ_END_PC, &regPC);

    processo_set_a(self->processo_em_execucao, regA);
    processo_set_x(self->processo_em_execucao, regX);
    processo_set_pc(self->processo_em_execucao, regPC);

    console_printf("SO: Salvando valores da CPU A: %d X: %d PC: %d no processo %d", regA, regX, regPC, processo_get_pid(self->processo_em_execucao));
  }
  
  return;
}

static void so_trata_pendencias(so_t *self)
{
  // t1: realiza ações que não são diretamente ligadas com a interrupcao que
  //   está sendo atendida:
  // - E/S pendente
  // - desbloqueio de processos
  // - contabilidades
  // ITERANDO SOBRE CADA PROCESSO DA TABELA VENDO SE ESTA BLOQUEADO
   for (int i = 0; i < self->tabela->num_processos; i++)
  { 
    
    processo_t *processo = self->tabela->tabela_processos[i];

    int estado_processo = processo_get_estado(processo);
    if (estado_processo == BLOQUEADO)
    {
      
      int dispositivo = processo_get_dispositivo(processo);
      int teclado_ok = dispositivo +1;
      int tela_ok = dispositivo+3;
      int tela = dispositivo +2;
      int estado_teclado, estado_tela;

      int motivo_boqueio = processo_get_motivo_bloqueio(processo);
      //print para ver os processos que estao bloquados e porque
      //console_printf("PROCESSO %d | ESTADO %d | MOTIVO %d | REG_A %d", processo->pid, estado_processo, motivo_boqueio,  processo_get_a(processo));

      switch (motivo_boqueio)
      {
      case BLOQUEIO_LEITURA:
        // verifica o estado do dispositivo de teclado
        if (es_le(self->es, teclado_ok, &estado_teclado) == ERR_OK)
        {
          if(estado_teclado != 0){
            processo_set_estado(processo, PRONTO);
            metricas_processo_incrementa_transicao(processo->metricas, PRONTO);
            metricas_processo_para_tempo_estado(processo->metricas, self, PRONTO);
            console_printf("SO: Processo %d desbloqueado para ler teclado.", processo_get_pid(processo));
          } 
        }
        break;
      case BLOQUEIO_ESCRITA:
        // verifica o estado do dispositivo de tela 
        if (es_le(self->es, tela_ok, &estado_tela) == ERR_OK)
        { 
          if(estado_tela != 0){
            int dado_escrever = processo_get_dado_escrever(processo);
            if (es_escreve(self->es, tela, dado_escrever) == ERR_OK)
            {
              processo_set_estado(processo, PRONTO);
              metricas_processo_incrementa_transicao(processo->metricas, PRONTO);
              metricas_processo_para_tempo_estado(processo->metricas, self, PRONTO);
              console_printf("SO: Processo %d desbloqueado para escrever na tela.", processo_get_pid(processo));
            }  
          }

         
        }
        break;
      case BLOQUEIO_ESPERA_PROC:
        // verifica se o processo esperado morreu
        for (int j = 0;  j < self->tabela->num_processos; j++)
        { 

          if(self->tabela->tabela_processos[j] != NULL){
            int processo_tabela_pid = processo_get_pid(self->tabela->tabela_processos[j]);
            int processo_tabela_estado = processo_get_estado(self->tabela->tabela_processos[j]);
            int processo_A = processo_get_a(processo);
            if(processo_A == 0){
              console_printf("SO: Nao existe processo 0.");
              self->erro_interno = true;
              return;
             
            }

            if (processo_tabela_pid == processo_A && processo_tabela_estado == MORTO)
            {
              processo_set_estado(processo, PRONTO);
              metricas_processo_incrementa_transicao(processo->metricas, PRONTO);
              metricas_processo_para_tempo_estado(processo->metricas, self, PRONTO);
              console_printf("SO: Processo %d desbloqueado depois de esperar processo morrer.", processo_get_pid(processo));
              break;
            }
          }

          
        }
        break;
      default:
        break;
      }

      if(processo_get_estado(processo) == PRONTO){
        fila_inserir(self->fila_prontos, processo);
      }
     
    }
  }


}

static void so_escalona(so_t *self)
{
  // escolhe o próximo processo a executar, que passa a ser o processo
  //   corrente; pode continuar sendo o mesmo de antes ou não
  // t1: na primeira versão, escolhe um processo caso o processo corrente não possa continuar
  //   executando. depois, implementar escalonador melhor

  if(self->processo_em_execucao != NULL){
    int estado = processo_get_estado(self->processo_em_execucao);
    if(estado ==  EXECUTANDO && self->quantum_restante > 0){
      return;
    }
  }

  if (self->processo_em_execucao != NULL && self->processo_em_execucao->estado == EXECUTANDO)
  {
    processo_set_estado(self->processo_em_execucao, PRONTO);
    metricas_processo_incrementa_transicao(self->processo_em_execucao->metricas, PRONTO);
    metricas_processo_para_tempo_estado(self->processo_em_execucao->metricas, self, PRONTO);

    fila_inserir(self->fila_prontos, self->processo_em_execucao);
    metricas_so_incrementa_preempcao(self->metricas);
    metricas_processo_incrementa_preempcao(self->processo_em_execucao->metricas);
  }

  processo_t *primeiro_da_fila = fila_remover(self->fila_prontos);
  if(primeiro_da_fila){
    if(self->processo_em_execucao != NULL){
      console_printf("SO: Escalonando processo... Sai processo %d, entra processo %d.", processo_get_pid(self->processo_em_execucao), processo_get_pid(primeiro_da_fila));
        
    }else{
      console_printf("SO: Escalonando processo... entra processo %d.", processo_get_pid(primeiro_da_fila));
      metricas_so_adiciona_tempo_ocioso(self->metricas, tempo_atual(self));
    }
    processo_set_estado(primeiro_da_fila, EXECUTANDO);
    metricas_processo_incrementa_transicao(primeiro_da_fila->metricas, EXECUTANDO);
    metricas_processo_para_tempo_estado(primeiro_da_fila->metricas, self, EXECUTANDO);
    self->processo_em_execucao = primeiro_da_fila;
    self->quantum_restante = QUANTUM;
    return;
  }

  console_printf("SO: Escalonando processo... Nenhum processo esta pronto para executar.");
  if(self->processo_em_execucao != NULL){
    metricas_so_adiciona_tempo_ocioso(self->metricas, tempo_atual(self)*-1);
  }
  
  self->processo_em_execucao = NULL;
    
  return;

}

static int so_despacha(so_t *self)
{
  // t1: se houver processo corrente, coloca o estado desse processo onde ele
  //   será recuperado pela CPU (em IRQ_END_*) e retorna 0, senão retorna 1
  // o valor retornado será o valor de retorno de CHAMAC
  if ( self->processo_em_execucao == NULL) return 1;

  int regA = processo_get_a(self->processo_em_execucao);
  int regX =  processo_get_x(self->processo_em_execucao);
  int regPC =  processo_get_pc(self->processo_em_execucao);

  mem_escreve(self->mem, IRQ_END_A, regA);
  mem_escreve(self->mem, IRQ_END_X, regX);
  mem_escreve(self->mem, IRQ_END_PC, regPC);
  console_printf("SO: Despachando processo %d A: %d X: %d PC: %d na CPU ", processo_get_pid(self->processo_em_execucao), regA, regX, regPC );

  return 0;
}

// TRATAMENTO DE UMA IRQ {{{1

// funções auxiliares para tratar cada tipo de interrupcao
static void so_trata_irq_reset(so_t *self);
static void so_trata_irq_chamada_sistema(so_t *self);
static void so_trata_irq_err_cpu(so_t *self);
static void so_trata_irq_relogio(so_t *self);
static void so_trata_irq_desconhecida(so_t *self, int irq);

static void so_trata_irq(so_t *self, int irq)
{
  // verifica o tipo de interrupcao que está acontecendo, e atende de acordo

  switch (irq) {
    case IRQ_RESET:
      metricas_so_incrementa_interrupcoes(self->metricas, IRQ_RESET);
      so_trata_irq_reset(self);
      break;
    case IRQ_SISTEMA:
      metricas_so_incrementa_interrupcoes(self->metricas, IRQ_SISTEMA);
      so_trata_irq_chamada_sistema(self);
      break;
    case IRQ_ERR_CPU:
      metricas_so_incrementa_interrupcoes(self->metricas, IRQ_ERR_CPU);
      so_trata_irq_err_cpu(self);
      break;
    case IRQ_RELOGIO:
      metricas_so_incrementa_interrupcoes(self->metricas, IRQ_RELOGIO);
      so_trata_irq_relogio(self);
      break;
    default:
      metricas_so_incrementa_interrupcoes(self->metricas, 4);
      so_trata_irq_desconhecida(self, irq);
  }
}

// interrupcao gerada uma única vez, quando a CPU inicializa
static void so_trata_irq_reset(so_t *self)
{
  // t1: deveria criar um processo para o init, e inicializar o estado do
  //   processador para esse processo com os registradores zerados, exceto
  //   o PC e o modo.
  // como não tem suporte a processos, está carregando os valores dos
  //   registradores diretamente para a memória, de onde a CPU vai carregar
  //   para os seus registradores quando executar a instrucao RETI  

  

  // coloca o programa init na memória
  int ender = so_carrega_programa(self, "init.maq");
  if (ender != 100) {
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }

  processo_t *processo_init = processo_cria(self, 1, ender);
  if(tabela_adicionar_processo(self->tabela, processo_init) != 0){
    console_printf("SO: Erro ao adicionar init na tabela");
    self->erro_interno = true;
    return;
  }
  fila_inserir(self->fila_prontos, processo_init);

  // altera o PC para o endereço de carga
  mem_escreve(self->mem, IRQ_END_PC, ender);
  // passa o processador para modo usuário
  mem_escreve(self->mem, IRQ_END_modo, usuario);
}

// interrupcao gerada quando a CPU identifica um erro
static void so_trata_irq_err_cpu(so_t *self)
{
  // Ocorreu um erro interno na CPU
  // O erro está codificado em IRQ_END_erro
  // Em geral, causa a morte do processo que causou o erro
  // Ainda não temos processos, causa a parada da CPU
  int err_int;
  // t1: com suporte a processos, deveria pegar o valor do registrador erro
  //   no descritor do processo corrente, e reagir de acordo com esse erro
  //   (em geral, matando o processo)
  mem_le(self->mem, IRQ_END_erro, &err_int);
  err_t err = err_int;
  console_printf("SO: IRQ nao tratada -- erro na CPU: %s", err_nome(err));

  if (self->processo_em_execucao != NULL)
  {
    console_printf("SO: matando processo %d -- erro na CPU", processo_get_pid(self->processo_em_execucao));
    processo_set_estado(self->processo_em_execucao, MORTO);
    metricas_processo_incrementa_transicao(self->processo_em_execucao->metricas, MORTO);
    metricas_processo_para_tempo_estado(self->processo_em_execucao->metricas, self, PRONTO);
    for (int i = 0; i < self->tabela->num_processos; i++)
    {   
      processo_t *processo_tabela = self->tabela->tabela_processos[i];
      int estado = processo_get_estado(processo_tabela);
      int regA = processo_get_a(processo_tabela);
      if (estado == BLOQUEADO && regA == processo_get_pid(self->processo_em_execucao))
      { 
        processo_set_estado(processo_tabela, PRONTO);
        metricas_processo_incrementa_transicao(processo_tabela->metricas, PRONTO);
        metricas_processo_para_tempo_estado(processo_tabela->metricas, self, PRONTO);
        fila_inserir(self->fila_prontos, processo_tabela);
        console_printf("SO: processo %d desbloqueado após a morte do processo %d", processo_get_pid(processo_tabela), processo_get_pid(self->processo_em_execucao));
      }
    }
  }
  else
  {
    console_printf("SO: erro na CPU sem processo em execucao");
  }

  self->erro_interno = true;
  
}

// interrupcao gerada quando o timer expira
static void so_trata_irq_relogio(so_t *self)
{
  // rearma o interruptor do relogio e reinicializa o timer para a próxima interrupcao
  err_t e1, e2;
  e1 = es_escreve(self->es, D_RELOGIO_INTERRUPCAO, 0); // desliga o sinalizador de interrupcao
  e2 = es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO);
  if (e1 != ERR_OK || e2 != ERR_OK) {
    console_printf("SO: problema da reinicializacao do timer");
    self->erro_interno = true;
  }
  // t1: deveria tratar a interrupcao
  //   por exemplo, decrementa o quantum do processo corrente, quando se tem
  //   um escalonador com quantum
  console_printf("SO: interrupcao do relogio decrementando QUANTUM atual %d", self->quantum_restante);
  self->quantum_restante--;
  
}

// foi gerada uma interrupcao para a qual o SO não está preparado
static void so_trata_irq_desconhecida(so_t *self, int irq)
{
  console_printf("SO: não sei tratar IRQ %d (%s)", irq, irq_nome(irq));
  self->erro_interno = true;
}

// CHAMADAS DE SISTEMA {{{1

// funções auxiliares para cada chamada de sistema
static void so_chamada_le(so_t *self);
static void so_chamada_escr(so_t *self);
static void so_chamada_cria_proc(so_t *self);
static void so_chamada_mata_proc(so_t *self);
static void so_chamada_espera_proc(so_t *self);

static void so_trata_irq_chamada_sistema(so_t *self)
{
  // a identificacao da chamada está no registrador A
  // t1: com processos, o reg A tá no descritor do processo corrente

  if(self->processo_em_execucao == NULL){
    console_printf("SO: erro no acesso ao id da chamada de sistema");
    self->erro_interno = true;
    return;
  }

  int id_chamada = processo_get_a(self->processo_em_execucao);

  switch (id_chamada) {
    case SO_LE:
      so_chamada_le(self);
      break;
    case SO_ESCR:
      so_chamada_escr(self);
      break;
    case SO_CRIA_PROC:
      so_chamada_cria_proc(self);
      break;
    case SO_MATA_PROC:
      so_chamada_mata_proc(self);
      break;
    case SO_ESPERA_PROC:
      so_chamada_espera_proc(self);
      break;
    default:
      console_printf("SO: chamada de sistema desconhecida (%d)", id_chamada);
      // t1: deveria matar o processo
      console_printf("SO: Matando processo PID %d...", processo_get_pid(self->processo_em_execucao));
      processo_set_estado(self->processo_em_execucao, MORTO);
      metricas_processo_incrementa_transicao(self->processo_em_execucao->metricas, MORTO);
      metricas_processo_para_tempo_estado(self->processo_em_execucao->metricas, self, MORTO);
      self->erro_interno = true;
  }
}

// implementacao da chamada se sistema SO_LE
// faz a leitura de um dado da entrada corrente do processo, coloca o dado no reg A
static void so_chamada_le(so_t *self)
{
  // implementacao com espera ocupada
  //   T1: deveria realizar a leitura somente se a entrada estiver disponível,
  //     senão, deveria bloquear o processo.
  //   no caso de bloqueio do processo, a leitura (e desbloqueio) deverá
  //     ser feita mais tarde, em tratamentos pendentes em outra interrupcao,
  //     ou diretamente em uma interrupcao específica do dispositivo, se for
  //     o caso
  // implementacao lendo direto do terminal A
  //   T1: deveria usar dispositivo de entrada corrente do processo
  int dispositivo_processo = processo_get_dispositivo(self->processo_em_execucao);
  int teclado = dispositivo_processo;
  int teclado_ok = dispositivo_processo +1;

  int estado;
  if (es_le(self->es, teclado_ok, &estado) != ERR_OK)
  {
    console_printf("SO: problema no acesso ao estado do teclado do terminal %d", (dispositivo_processo/4));
    self->erro_interno = true;
    return;
  }

  if (estado == 0)
  {
    processo_set_estado(self->processo_em_execucao, BLOQUEADO);
    metricas_processo_incrementa_transicao(self->processo_em_execucao->metricas, BLOQUEADO);
    metricas_processo_para_tempo_estado(self->processo_em_execucao->metricas, self, BLOQUEADO);
    processo_set_motivo_bloqueio(self->processo_em_execucao, BLOQUEIO_LEITURA);
    console_printf("SO: Processo %d foi ler teclado, mas nao podia... BLOQUEADO!", processo_get_pid(self->processo_em_execucao));
    return;
  }

  int dado;
  if (es_le(self->es, teclado, &dado) != ERR_OK)
  {
    console_printf("SO: problema no acesso ao teclado do terminal %d", (dispositivo_processo/4));
    self->erro_interno = true;
    return;
  }

  // escreve no reg A do processador
  // (na verdade, na posicao onde o processador vai pegar o A quando retornar da int)
  // T1: se houvesse processo, deveria escrever no reg A do processo
  // T1: o acesso só deve ser feito nesse momento se for possível; se não, o processo
  //   é bloqueado, e o acesso só deve ser feito mais tarde (e o processo desbloqueado)

  //mem_escreve(self->mem, IRQ_END_A, dado);
  processo_set_a(self->processo_em_execucao, dado);
}

// implementacao da chamada se sistema SO_ESCR
// escreve o valor do reg X na saída corrente do processo
static void so_chamada_escr(so_t *self)
{
  // implementacao com espera ocupada
  //   T1: deveria bloquear o processo se dispositivo ocupado
  // implementacao escrevendo direto do terminal A
  //   T1: deveria usar o dispositivo de saída corrente do processo
 
  int dispositivo_processo = processo_get_dispositivo(self->processo_em_execucao);
  int tela = dispositivo_processo +2;
  int tela_ok = dispositivo_processo +3;

  int estado;
  if (es_le(self->es, tela_ok, &estado) != ERR_OK)
  {
    console_printf("SO: problema no acesso ao estado da tela do terminal %d", (dispositivo_processo/4));
    self->erro_interno = true;
    return;
  }

  int dado = processo_get_x(self->processo_em_execucao);
  if (estado == 0)
  { 
    processo_set_dado_escrever(self->processo_em_execucao, dado);
    processo_set_estado(self->processo_em_execucao, BLOQUEADO);
    metricas_processo_incrementa_transicao(self->processo_em_execucao->metricas, BLOQUEADO);
    metricas_processo_para_tempo_estado(self->processo_em_execucao->metricas, self, BLOQUEADO);
    processo_set_motivo_bloqueio(self->processo_em_execucao, BLOQUEIO_ESCRITA);
    return;
  }
  
  // está lendo o valor de X e escrevendo o de A direto onde o processador colocou/vai pegar
  // T1: deveria usar os registradores do processo que está realizando a E/S
  // T1: caso o processo tenha sido bloqueado, esse acesso deve ser realizado em outra execucao
  //   do SO, quando ele verificar que esse acesso já pode ser feito.
  
  if (es_escreve(self->es, tela, dado) != ERR_OK)
  {
    console_printf("SO: problema no acesso à tela do terminal %d", (dispositivo_processo/4));
    self->erro_interno = true;
    return;
  }

  processo_set_a(self->processo_em_execucao, 0);
  //mem_escreve(self->mem, IRQ_END_A, 0);
}

// implementacao da chamada se sistema SO_CRIA_PROC
// cria um processo
static void so_chamada_cria_proc(so_t *self)
{
  // ainda sem suporte a processos, carrega programa e passa a executar ele
  // quem chamou o sistema não vai mais ser executado, coitado!
  // T1: deveria criar um novo processo

  // em X está o endereço onde está o nome do arquivo
  int ender_proc;

  // t1: deveria ler o X do descritor do processo criador
  if (mem_le(self->mem, IRQ_END_X, &ender_proc) != ERR_OK)
  {
    console_printf("SO: erro ao acessar o endereço do nome do arquivo");
    self->erro_interno = true;
    processo_set_a(self->processo_em_execucao, -1);
    return;
  }

  char nome[100];
  if (!copia_str_da_mem(100, nome, self->mem, ender_proc))
  {
    console_printf("SO: erro ao copiar o nome do arquivo da memória");
    processo_set_a(self->processo_em_execucao, -1);
    return;
  }

  int ender_carga = so_carrega_programa(self, nome);
  processo_t *novo_processo = processo_cria(self, self->tabela->contador_pid, ender_carga);

  if (novo_processo == NULL)
  {
    console_printf("SO: erro ao criar o novo processo");
    processo_set_a(self->processo_em_execucao, -1);
    return;
  }

  if(tabela_adicionar_processo(self->tabela, novo_processo) != 0){
    console_printf("SO: Erro ao adicionar processo na tabela");
    processo_set_a(self->processo_em_execucao, -1);
    return;
  }

  fila_inserir(self->fila_prontos, novo_processo);
  tabela_imprimir(self->tabela);
  // deveria escrever -1 (se erro) ou o PID do processo criado (se OK) no reg A
  //   do processo que pediu a criacao
  
  processo_set_a(self->processo_em_execucao, processo_get_pid(novo_processo));
}

// implementacao da chamada se sistema SO_MATA_PROC
// mata o processo com pid X (ou o processo corrente se X é 0)
static void so_chamada_mata_proc(so_t *self)
{
  // T1: deveria matar um processo
  // ainda sem suporte a processos, retorna erro -1
  if(self->processo_em_execucao == NULL){
    console_printf("SO: nenhum processo em execucao");
    mem_escreve(self->mem, IRQ_END_A, -1);
    return;
  }

  int pid = processo_get_x(self->processo_em_execucao);
  

  if (pid == 0)
  {
    console_printf("SO: Matando processo PID %d...", processo_get_pid(self->processo_em_execucao));
    processo_set_estado(self->processo_em_execucao, MORTO);
    metricas_processo_incrementa_transicao(self->processo_em_execucao->metricas, MORTO);
    metricas_processo_para_tempo_estado(self->processo_em_execucao->metricas, self, MORTO);
    self->processo_em_execucao->metricas->tempo_termino = tempo_atual(self);
    mem_escreve(self->mem, IRQ_END_A, 0);
    return;
  }
  console_printf("SO: Matando processo PID %d...", pid);
   processo_t *processo_buscado = fila_remover_por_pid(self->fila_prontos, pid);

    if (!processo_buscado) {
        processo_buscado = tabela_buscar_processo(self->tabela, pid);
    }


  if(processo_buscado){
    processo_set_estado(processo_buscado, MORTO);
    metricas_processo_incrementa_transicao(processo_buscado->metricas, MORTO);
    metricas_processo_para_tempo_estado(processo_buscado->metricas, self, MORTO);
    processo_buscado->metricas->tempo_termino = tempo_atual(self);
    mem_escreve(self->mem, IRQ_END_A, 0);
    return;
  }
      
  console_printf("SO: Processo com PID %d não encontrado", pid);
  mem_escreve(self->mem, IRQ_END_A, -1);
} 

// implementacao da chamada se sistema SO_ESPERA_PROC
// espera o fim do processo com pid X
static void so_chamada_espera_proc(so_t *self)
{
  // T1: deveria bloquear o processo se for o caso (e desbloquear na morte do esperado)
  // ainda sem suporte a processos, retorna erro -1
  
  int pid = processo_get_x(self->processo_em_execucao);

  if (pid == processo_get_pid(self->processo_em_execucao))
  {
    console_printf("SO: processo não pode esperar por si mesmo");
    self->erro_interno = true;
    mem_escreve(self->mem, IRQ_END_A, -1);
    return;
  }

  processo_t *processo_esperado = tabela_buscar_processo(self->tabela, pid);

  if (processo_esperado == NULL)
  {
    console_printf("SO espera: processo com PID %d não encontrado", pid);
    mem_escreve(self->mem, IRQ_END_A, -1);
    self->erro_interno = true;
    return;
  }

  if (processo_get_estado(processo_esperado) == MORTO)
  {
    console_printf("SO: processo esperado com PID %d ja esta morto", pid);
    mem_escreve(self->mem, IRQ_END_A, 0);
    return;
  }
  processo_set_estado(self->processo_em_execucao, BLOQUEADO);
  metricas_processo_incrementa_transicao(self->processo_em_execucao->metricas, BLOQUEADO);
  metricas_processo_para_tempo_estado(self->processo_em_execucao->metricas, self, BLOQUEADO);

  
  processo_set_motivo_bloqueio(self->processo_em_execucao, BLOQUEIO_ESPERA_PROC);
  processo_set_a(self->processo_em_execucao, pid); 

    

  mem_escreve(self->mem, IRQ_END_A, 0);
}

// CARGA DE PROGRAMA {{{1

// carrega o programa na memória
// retorna o endereço de carga ou -1
static int so_carrega_programa(so_t *self, char *nome_do_executavel)
{
  // programa para executar na nossa CPU
  programa_t *prog = prog_cria(nome_do_executavel);
  if (prog == NULL) {
    console_printf("Erro na leitura do programa '%s'\n", nome_do_executavel);
    return -1;
  }

  int end_ini = prog_end_carga(prog);
  int end_fim = end_ini + prog_tamanho(prog);

  for (int end = end_ini; end < end_fim; end++) {
    if (mem_escreve(self->mem, end, prog_dado(prog, end)) != ERR_OK) {
      console_printf("Erro na carga da memoria, endereco %d\n", end);
      return -1;
    }
  }

  prog_destroi(prog);
  console_printf("SO: carga de '%s' em %d-%d", nome_do_executavel, end_ini, end_fim);
  return end_ini;
}

// ACESSO À MEMÓRIA DOS PROCESSOS {{{1

// copia uma string da memória do simulador para o vetor str.
// retorna false se erro (string maior que vetor, valor não char na memória,
//   erro de acesso à memória)
// T1: deveria verificar se a memória pertence ao processo
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender)
{
  for (int indice_str = 0; indice_str < tam; indice_str++) {
    int caractere;
    if (mem_le(mem, ender + indice_str, &caractere) != ERR_OK) {
      return false;
    }
    if (caractere < 0 || caractere > 255) {
      return false;
    }
    str[indice_str] = caractere;
    if (caractere == 0) {
      return true;
    }
  }
  // estourou o tamanho de str
  return false;
}

// vim: foldmethod=marker

//  MINHAS FUNCOES ---------------------------------------------------
//  PROCESSO ---------------------------------------------------------

static processo_t *processo_cria(so_t *self, int pid, int pc) {
    processo_t *novo_processo = malloc(sizeof(processo_t));
    if (novo_processo == NULL) {
        console_printf("SO: não foi possível alocar memória para o processo.\n");
        return NULL;
    }
    processo_set_pid(novo_processo, pid);
    processo_set_pc(novo_processo, pc);
    processo_set_a(novo_processo, 0);
    processo_set_x(novo_processo, 0);
    processo_set_estado(novo_processo, PRONTO);
    processo_set_dispositivo(novo_processo, ((pid % 4) * 4));
    
    novo_processo->metricas = metricas_processo_cria(tempo_atual(self));
    metricas_processo_incrementa_transicao(novo_processo->metricas, PRONTO);
    metricas_processo_atualiza_tempo_estado(novo_processo->metricas, PRONTO, tempo_atual(self) *-1);
    metricas_so_incrementa_processos(self->metricas);
   
    

    return novo_processo;
}

static int processo_get_pid(processo_t *processo) {
  
    return processo->pid;
}

static int processo_get_pc(processo_t *processo) {
    return processo->PC;
}

static int processo_get_a(processo_t *processo) {
    return processo->A;
}

static int processo_get_x(processo_t *processo) {
    return processo->X;
}

static int processo_get_estado(processo_t *processo) {
    return processo->estado;
}

static int processo_get_dispositivo(processo_t *processo) {
    return processo->dispositivo;
}

static int processo_get_motivo_bloqueio(processo_t *processo) {
    return processo->motivo_bloqueio;
}

static int processo_get_dado_escrever(processo_t *processo) {
    return processo->dado_escrever;
}

static void processo_set_pid(processo_t *processo, int pid) {
    processo->pid = pid;
}

static void processo_set_pc(processo_t *processo, int pc) {
    processo->PC = pc;
}

static void processo_set_a(processo_t *processo, int a) {
    processo->A = a;
}

static void processo_set_x(processo_t *processo, int x) {
    processo->X = x;
}

static void processo_set_estado(processo_t *processo, int estado) {
  if (estado < 0 || estado > 3) {
        console_printf("SO: estado inválido.\n");
        return;
  }
  processo->estado = estado;
}

static void processo_set_dispositivo(processo_t *processo, int dispositivo) {
    if (dispositivo < 0 || dispositivo > 12) {
        console_printf("SO: dispositivo inválido.\n");
        return;
    }

    processo->dispositivo = dispositivo;
}

static void processo_set_motivo_bloqueio(processo_t *processo, int motivo_bloqueio) {
    if (motivo_bloqueio < 0 || motivo_bloqueio > 3) {
        console_printf("SO: motivo de bloqueio inválido.\n");
        return;
    }

    processo->motivo_bloqueio = motivo_bloqueio;
}

static void processo_set_dado_escrever(processo_t *processo, int dado_escrever) {
    processo->dado_escrever = dado_escrever;
}



// TABELA PROCESSOS --------------------

static tabela_processos_t *tabela_cria() {
    tabela_processos_t *tabela = malloc(sizeof(tabela_processos_t));
    if (tabela == NULL) {
        console_printf("SO: falha ao alocar memória para a tabela de processos.\n");
        return NULL;
    }
    tabela->tabela_processos = NULL; 
    tabela->num_processos = 0;      
    tabela->contador_pid = 1; 
    return tabela;
}

static int tabela_adicionar_processo(tabela_processos_t *tabela, processo_t *processo) {
  // SUCESSO -> 0 || FALHA -> -1
  if (tabela == NULL || processo == NULL) return -1;

  processo_t **nova_tabela = realloc(tabela->tabela_processos, (tabela->num_processos + 1) * sizeof(processo_t *));
  if (nova_tabela == NULL) {
      console_printf("SO: falha ao realocar a tabela de processos.\n");
      return -1;
  }

  
  tabela->tabela_processos = nova_tabela;
  tabela->tabela_processos[tabela->num_processos] = processo;
  tabela->num_processos++;
  tabela->contador_pid++;
  console_printf("Processo %d adicionado a tabela.", processo_get_pid(processo));
  return 0; 
}

static processo_t *tabela_buscar_processo(tabela_processos_t *tabela, int pid) {
  // SE ACHAR O PROCESSO RETORNA ELE, SE NAO RETORNA NULL
  if (tabela == NULL || tabela->tabela_processos == NULL) return NULL;

  for (int i = 0; i < tabela->num_processos; i++) {
      if (tabela->tabela_processos[i]->pid == pid) {
          return tabela->tabela_processos[i];
      }
  }

  return NULL; 
}

static void tabela_destroi(tabela_processos_t *tabela) {
  if (tabela == NULL) return;

  for (int i = 0; i < tabela->num_processos; i++) {
    metricas_processo_destroi(tabela->tabela_processos[i]->metricas);
    free(tabela->tabela_processos[i]); 
  }

  free(tabela->tabela_processos); 
  free(tabela); 
}

static void tabela_imprimir(tabela_processos_t *tabela) {
    if (tabela == NULL || tabela->tabela_processos == NULL) {
        console_printf("Tabela de processos não inicializada.\n");
        return;
    }
    console_printf("|--------------------|\n");
    console_printf("|Tabela de processos:|\n");
    console_printf("|--------------------|\n");
    for (int i = 0; i < tabela->num_processos; i++) {
        processo_t *proc = tabela->tabela_processos[i];
        if (proc != NULL) {
            console_printf("|PID: %d  |  estado: %d|\n", proc->pid, proc->estado);
        }
    }
    console_printf("|--------------------|\n");
}

// METRICAS PROCESSOS --------------------

static metricas_processo_t *metricas_processo_cria(double tempo_criacao) {
    metricas_processo_t *metricas = malloc(sizeof(metricas_processo_t));
    metricas->tempo_criacao = tempo_criacao;
    metricas->tempo_termino = 0.0;
    metricas->num_preempcoes = 0;
    for (int i = 0; i < 4; i++) {
        metricas->transicoes_estado[i] = 0;
        metricas->tempo_estado[i] = 0.0;
    }
    metricas->tempo_resposta_medio = 0.0;
    return metricas;
}

static void metricas_processo_destroi(metricas_processo_t *metricas) {
    free(metricas);
}

static void metricas_processo_atualiza_tempo_estado(metricas_processo_t *metricas, int estado, double tempo_decorrido) {
    metricas->tempo_estado[estado] += tempo_decorrido;
}

static void metricas_processo_incrementa_transicao(metricas_processo_t *metricas, int estado) {
    metricas->transicoes_estado[estado]++;
}

static void metricas_processo_incrementa_preempcao(metricas_processo_t *metricas) {
    metricas->num_preempcoes++;
}

static void metricas_processo_calcula_tempo_resposta(metricas_processo_t *metricas) { 
    metricas->tempo_resposta_medio = metricas->tempo_estado[PRONTO] / metricas->transicoes_estado[PRONTO];
}

static void metricas_processo_para_tempo_estado(metricas_processo_t *metricas, so_t *self, int estado){
  for(int i = 0; i < 4; i++){
    if(metricas->tempo_estado[i] < 0){
      metricas_processo_atualiza_tempo_estado(metricas, i, tempo_atual(self));
    }
  }

  metricas_processo_atualiza_tempo_estado(metricas, estado, tempo_atual(self) *-1);
}

static void metricas_processo_imprimir(processo_t *processo) {
    if (processo == NULL) {
        console_printf("Processo invalido.\n");
        return;
    }

    metricas_processo_t *metricas = processo->metricas;

    console_printf("Metricas do Processo %d:\n", processo->pid);
    console_printf("  - Tempo de criacao: %.2f s\n", metricas->tempo_criacao);
    console_printf("  - Tempo de termino: %.2f s\n", metricas->tempo_termino);
    console_printf("  - Tempo total nos estados:\n");
    console_printf("      * PRONTO: %.2f s\n", metricas->tempo_estado[PRONTO]);
    console_printf("      * EXECUTANDO: %.2f s\n", metricas->tempo_estado[EXECUTANDO]);
    console_printf("      * BLOQUEADO: %.2f s\n", metricas->tempo_estado[BLOQUEADO]);
    console_printf("      * MORTO: %.2f s\n", metricas->tempo_estado[MORTO]);
    console_printf("  - Transicoes de estado:\n");
    console_printf("      * PRONTO: %d vezes\n", metricas->transicoes_estado[PRONTO]);
    console_printf("      * EXECUTANDO: %d vezes\n", metricas->transicoes_estado[EXECUTANDO]);
    console_printf("      * BLOQUEADO: %d vezes\n", metricas->transicoes_estado[BLOQUEADO]);
    console_printf("      * MORTO: %d vezes\n", metricas->transicoes_estado[MORTO]);
    console_printf("  - Numero de preempções: %d\n", metricas->num_preempcoes);
    console_printf("  - Tempo medio de resposta: %.2f s\n", metricas->tempo_resposta_medio);
    console_printf("\n");
}

static void metricas_imprimir_todos_processos(tabela_processos_t *tabela) {
    if (tabela == NULL || tabela->tabela_processos == NULL) {
        console_printf("Tabela de processos inválida.\n");
        return;
    }

    console_printf("=== Métricas de Todos os Processos ===\n");
    for (int i = 0; i < tabela->num_processos; i++) {
        metricas_processo_imprimir(tabela->tabela_processos[i]);
    }
}

// METRICAS SO --------------------

static metricas_so_t *metricas_so_cria(so_t *self) {
    metricas_so_t *metricas = malloc(sizeof(metricas_so_t));
    metricas->num_processos_criados = 0;
    metricas->tempo_total_execucao = tempo_atual(self) * - 1;
    metricas->tempo_ocioso = 0.0;
    metricas->num_preempcoes = 0;
    for (int i = 0; i < 5; i++) {
        metricas->interrupcoes[i] = 0;
    }
    return metricas;
}

static void metricas_so_destroi(metricas_so_t *metricas) {
    free(metricas);
}

static void metricas_so_incrementa_processos(metricas_so_t *metricas) {
    metricas->num_processos_criados++;
}

static void metricas_so_adiciona_tempo_ocioso(metricas_so_t *metricas, double tempo) {
    metricas->tempo_ocioso += tempo;
}

static void metricas_so_incrementa_interrupcoes(metricas_so_t *metricas, int tipo) {
    if (tipo >= 0 && tipo < 5) {
        metricas->interrupcoes[tipo]++;
    }
}

static void metricas_so_incrementa_preempcao(metricas_so_t *metricas) {
    metricas->num_preempcoes++;
}

static void metricas_so_adiciona_tempo_execucao(metricas_so_t *metricas, double tempo) {
    metricas->tempo_total_execucao += tempo;
}

static void metricas_so_imprimir(metricas_so_t *metricas_so) {
    if (metricas_so == NULL) {
        console_printf("Metricas do sistema operacional invalidas.\n");
        return;
    }

    console_printf("=== Metricas do Sistema Operacional ===\n");
    console_printf("  - Numero de processos criados: %d\n", metricas_so->num_processos_criados);
    console_printf("  - Tempo total de execucao: %.2f s\n", metricas_so->tempo_total_execucao);
    console_printf("  - Tempo total ocioso: %.2f s\n", metricas_so->tempo_ocioso);
    console_printf("  - Numero de preempcoes: %d\n", metricas_so->num_preempcoes);
    console_printf("  - Interrupcoes recebidas por tipo:\n");

    for (int i = 0; i < 5; i++) {
        console_printf("      * Tipo %d: %d vezes\n", i, metricas_so->interrupcoes[i]);
    }

    console_printf("\n");
}

// TEMPO --------------------------

double tempo_atual(so_t *self) {
  int hora_atual = 0;

  if (es_le(self->es, D_RELOGIO_REAL, &hora_atual) != ERR_OK)
  {
    console_printf("SO: erro na leitura do relógio");
    self->erro_interno = true;
    return 0;
  }



  return (double)hora_atual/1000;
}

// FILA ----------------------------

fila_t *fila_cria() {
    fila_t *fila = (fila_t *)malloc(sizeof(fila_t));
    fila->inicio = fila->fim = NULL;
    fila->tamanho = 0;
    return fila;
}

void fila_inserir(fila_t *fila, processo_t *processo) {
    nodo_fila_t *novo = (nodo_fila_t *)malloc(sizeof(nodo_fila_t));
    novo->processo = processo;
    novo->proximo = NULL;
    if (fila->fim) {
        fila->fim->proximo = novo;
    } else {
        fila->inicio = novo;
    }
    fila->fim = novo;
    fila->tamanho++;
}

processo_t *fila_remover(fila_t *fila) {
    if (!fila->inicio) return NULL;
    nodo_fila_t *removido = fila->inicio;
    processo_t *processo = removido->processo;
    fila->inicio = removido->proximo;
    if (!fila->inicio) fila->fim = NULL;
    free(removido);
    fila->tamanho--;
    return processo;
}

processo_t *fila_remover_por_pid(fila_t *fila, int pid) {
    if (!fila || !fila->inicio) {
        return NULL; 
    }

    nodo_fila_t *anterior = NULL;
    nodo_fila_t *atual = fila->inicio;

    while (atual) {
        if (processo_get_pid(atual->processo) == pid) {
            if (anterior) {
                anterior->proximo = atual->proximo;
            } else {
                fila->inicio = atual->proximo; 
            }

            if (atual == fila->fim) {
                fila->fim = anterior; 
            }

            processo_t *processo = atual->processo;
            free(atual); 
            fila->tamanho--;
            return processo; 
        }
        anterior = atual;
        atual = atual->proximo;
    }

    return NULL; 
}

bool fila_vazia(fila_t *fila) {
    return fila->tamanho == 0;
}
