/* Fichier de test des fonctions de init_agent.c et gestion_liste_agents.c */
//gcc test_listes.c -o test gestion_liste_agents.o init_agent.o -I/home/forest/include -L/home/forest/lib -I/home/forest/include -L/home/forest/lib -ldrawstuff -lode -lm -lstdc++ -lGL -lGLU

#include "boule.h"


int main (int argc, char **argv){
	Agent a0,a1,a2,a3,a4,a5,a6,a7,a8,a9;
	Agent *pa0,*pa1,*pa2,*pa3,*pa4,*pa5,*pa6,*pa7,*pa8,*pa9;
	listeAgents *lia = NULL;
	dJointID jo;
	int boo1,boo2;

	//Création du monde
	world = dWorldCreate();
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	accrogroup = dJointGroupCreate (0);
	dWorldSetGravity (world,0,0,-0.5);
	//le sol :
	dCreatePlane (space,0,0,1,0);

	//on crée 10 agents
	pa0 = initAgent (&a0 , 0.0, 0.0, 0.0, world, space);
	pa1 = initAgent (&a1 , 0.0, 0.0, 0.0, world, space);
	pa2 = initAgent (&a2 , 0.0, 0.0, 0.0, world, space);
	pa3 = initAgent (&a3 , 0.0, 0.0, 0.0, world, space);
	pa4 = initAgent (&a4 , 0.0, 0.0, 0.0, world, space);
	pa5 = initAgent (&a5 , 0.0, 0.0, 0.0, world, space);
	pa6 = initAgent (&a6 , 0.0, 0.0, 0.0, world, space);
	pa7 = initAgent (&a7 , 0.0, 0.0, 0.0, world, space);
	pa8 = initAgent (&a8 , 0.0, 0.0, 0.0, world, space);
	pa9 = initAgent (&a9 , 0.0, 0.0, 0.0, world, space);

	printf("Liste vide :\n");
	affiche( lia );
	lia = ajout_agent( lia, pa0, jo, 0 );
	lia = ajout_agent( lia, pa1, jo, 1 );
	lia = ajout_agent( lia, pa2, jo, 2 );
	lia = ajout_agent( lia, pa3, jo, 3 );
	lia = ajout_agent( lia, pa4, jo, 4 );
	lia = ajout_agent( lia, pa5, jo, 5 );
	lia = ajout_agent( lia, pa6, jo, 6 );
	lia = ajout_agent( lia, pa7, jo, 7 );
	lia = ajout_agent( lia, pa8, jo, 8 );
	lia = ajout_agent( lia, pa9, jo, 9 );

	printf("Liste 10 elmts :\n");
	affiche( lia );

	printf("suppression massive !!!\n");
	lia = supprime( lia, pa0->bodyID );
	lia = supprime( lia, pa1->bodyID );
	lia = supprime( lia, pa2->bodyID );
	lia = supprime( lia, pa3->bodyID );
	//lia = supprime( lia, pa4->bodyID );
	lia = supprime( lia, pa5->bodyID );
	lia = supprime( lia, pa6->bodyID );
	lia = supprime( lia, pa7->bodyID );
	lia = supprime( lia, pa8->bodyID );
	lia = supprime( lia, pa9->bodyID );
	affiche( lia );

	return(0);
}

