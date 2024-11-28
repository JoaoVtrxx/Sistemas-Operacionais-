# configuração para o make, para a compilação do simulador e do montador
# se alterar este arquivo, cuidado para manter os caracteres "tab" no início das linhas de continuação

# opções de compilação
CC = gcc
CFLAGS = -Wall -Wno-unused-function -g
LDLIBS = -lcurses

# arquivos objeto compilados (.o) que compõem o simulador (main) e o montador
OBJS_MAIN = cpu.o es.o memoria.o relogio.o console.o terminal.o tela_curses.o \
		instrucao.o err.o programa.o controle.o main.o \
		so.o irq.o
OBJS_MONTADOR = instrucao.o err.o montador.o
OBJS = ${OBJS_MAIN} ${OBJS_MONTADOR}
# arquivos .maq a gerar, com seus endereços
MAQS = trata_int.maq init.maq ex1.maq ex2.maq ex3.maq ex4.maq ex5.maq ex6.maq p1.maq p2.maq p3.maq
ENDS = 10            100      1000    2000    3000    4000    5000    6000    7000   8000   9000
TARGETS = main montador ${MAQS}

# arquivos que devem ser feitos, se não for especificado no comando do make
all: ${TARGETS}

# para gerar o montador, precisa de todos os .o do montador
montador: ${OBJS_MONTADOR}

# para gerar o programa principal, precisa de todos os .o do main
main: ${OBJS_MAIN}

# para transformar um .asm em .maq, precisamos do montador
# monta os programas de usuário nos endereços equivalentes em ENDS
# se alguém souber de uma forma menos escrota de casar o endereço com
# o nome, por favor fala
%.maq: %.asm montador
	@case "$@" in \
		trata_int.maq) end="10";; \
		init.maq) end="100";; \
		ex1.maq) end="1000";; \
		ex2.maq) end="2000";; \
		ex3.maq) end="3000";; \
		ex4.maq) end="4000";; \
		ex5.maq) end="5000";; \
		ex6.maq) end="6000";; \
		p1.maq) end="7000";; \
		p2.maq) end="8000";; \
		p3.maq) end="9000";; \
		*) echo "Arquivo não encontrado"; exit 1;; \
	esac; \
	./montador -e $$end `basename $@ .maq`.asm > $@


# apaga os arquivos gerados
clean:
	rm -f ${OBJS} ${TARGETS} ${MAQS} ${OBJS:.o=.d}

# para calcular as dependências de cada arquivo .c (e colocar no .d)
%.d: %.c
	@set -e; rm -f $@; \
	 $(CC) -MM $(CPPFLAGS) $< > /tmp/$@.$$$$; \
	 sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < /tmp/$@.$$$$ > $@; \
	 rm -f /tmp/$@.$$$$

# inclui as dependências
include $(OBJS:.o=.d)
