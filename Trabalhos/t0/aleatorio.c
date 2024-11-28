// aleatorio.c
// dispositivo de E/S para gerar numero aleatorio
// simulador de computador
// so24b

#include "aleatorio.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

struct aleatorio_t {
  int numero_aleatorio;
};

aleatorio_t *aleatorio_cria(void)
{
  aleatorio_t *self;
  self = malloc(sizeof(aleatorio_t));
  assert(self != NULL);

  srand(time(NULL));

  self->numero_aleatorio = 1 + rand() % 101;

  return self;
}

void aleatorio_destroi(aleatorio_t *self)
{
  free(self);
}


err_t aleatorio_leitura(void *disp, int id, int *pvalor)
{
  aleatorio_t *self = disp;
  err_t err = ERR_OK;
  switch (id) {
    case 0:
      *pvalor = self->numero_aleatorio;
      break;
    default: 
      err = ERR_OP_INV;
  }
  return err;
}
