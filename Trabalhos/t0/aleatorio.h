// aleatorio.h
// dispositivo de E/S para gerar numero aleatorio
// simulador de computador
// so24b

#ifndef ALEATORIO_H
#define ALEATORIO_H


#include "err.h"

typedef struct aleatorio_t aleatorio_t;

// cria e inicializa dispositivo aleatorio
aleatorio_t *aleatorio_cria(void);

// destrói o dispositivo aleatorio
// nenhuma outra operação pode ser realizada no dispositivo aleatorio após esta chamada
void aleatorio_destroi(aleatorio_t *self);

err_t aleatorio_leitura(void *disp, int id, int *pvalor);

#endif // ALEATORIO_H
