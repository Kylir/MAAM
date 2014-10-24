INCLUDE_REP=/home/forest/include
LIBS_REP=/home/forest/lib
LIBS=-ldrawstuff -lode -lm -lstdc++ -lGL -lGLU
#DEBUG=-g
#DEBUG=

all : main.c simulation.o agent.o gestion_liste_agents.o init_agent.o init_env.o init_attractant.o dessin_agent.o dessin_env.o dessin_attractant.o geometrie.o boule.h
	gcc $(DEBUG) -Wall main.c -o robots simulation.o agent.o gestion_liste_agents.o init_agent.o init_env.o init_attractant.o dessin_agent.o dessin_env.o dessin_attractant.o geometrie.o -I $(INCLUDE_REP) -L $(LIBS_REP) $(LIBS)

simulation.o : simulation.c boule.h
	gcc $(DEBUG) -Wall simulation.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

agent.o : agent.c boule.h
	gcc $(DEBUG) -Wall agent.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

gestion_liste_agents.o : gestion_liste_agents.c boule.h
	gcc $(DEBUG) -Wall gestion_liste_agents.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

init_agent.o : init_agent.c boule.h
	gcc $(DEBUG) -Wall init_agent.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

init_env.o : init_env.c boule.h
	gcc $(DEBUG) -Wall init_env.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

init_attractant.o : init_attractant.c boule.h
	gcc $(DEBUG) -Wall init_attractant.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

dessin_agent.o : dessin_agent.c boule.h
	gcc $(DEBUG) -Wall dessin_agent.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

dessin_env.o : dessin_agent.c boule.h
	gcc $(DEBUG) -Wall dessin_env.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

dessin_attractant.o : dessin_attractant.c boule.h
	gcc $(DEBUG) -Wall dessin_attractant.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

geometrie : geometrie.c boule.h
	gcc $(DEBUG) -Wall geometrie.c -o geometrie -I $(INCLUDE_REP) -L $(LIBS_REP)

geometrie.o : geometrie.c boule.h
	gcc $(DEBUG) -Wall geometrie.c -c -I $(INCLUDE_REP) -L $(LIBS_REP)

clean :
	rm *.o
