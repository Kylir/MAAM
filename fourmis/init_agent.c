/* création et initialisation d'un agent */
//gcc init_agent.c -c -I/home/forest/include -L/home/forest/lib

#include "boule.h"

//DILEMME !!! on crée dynamiquement ou pas ???
//REPONSE : On fait les deux ! si le agy en parametre est NULL alors on crée l'objet en dynamique

Agent *initAgent ( Agent *agy, dReal x, dReal y, dReal z, dWorldID world, dSpaceID space ) {
	int i;

	//on crée la structure si on nous passe un truc à NULL
	if(agy == NULL){ agy = (Agent *) malloc( sizeof( Agent ) ); }

	//on crée le corps
	agy->bodyID = dBodyCreate (world);

	//on lui donne une forme de masse et une masse
	if( FORME_AGENT == SPHERE ) {
		dMassSetSphere ( &(agy->masse), 1, RAYON);
	}
	if( FORME_AGENT == BOX ) {
		dMassSetBox ( &(agy->masse), 1, RAYON, RAYON, RAYON );
	}

	dMassAdjust (&(agy->masse), MASSE);

	//on créer une forme et on la lie au body
	if( FORME_AGENT == SPHERE ) {
		agy->geomID = dCreateSphere (space, RAYON);
	}
	if( FORME_AGENT == BOX ) {
		agy->geomID = dCreateBox (space, RAYON, RAYON, RAYON);
	}
	dGeomSetBody (agy->geomID, agy->bodyID);

	//On place l'agent
	dBodySetPosition (agy->bodyID,x,y,z);

	 //le joint de patte
	agy->patte_move = dJointCreateHinge(world, 0);

	//le joint de patte
	agy->patte = dJointCreateUniversal(world, 0);

	//le joint d'aggripage avant
	agy->mandy = dJointCreateHinge(world, 0);

        //le joint d'aggripage arriere
	agy->arriere = dJointCreateHinge(world, 0);


	//le booléen en_marche
	agy->etat = -1; //etat de départ de simulation

	for(i=0; i < MAX_INFOS_COLLISION ; i++) {
                agy->capteurs_collision[i].objet = (dBodyID) 0;
                agy->capteurs_collision[i].type_objet = 0;
		agy->capteurs_collision[i].etat = 0;
	}

	//combien de collision détéctées
	agy->nombre_collision = 0;

        //la position du but...
	agy->pos_but.x = x; agy->pos_but.y = y; agy->pos_but.z = z;

	//le bodyID du but (pour l'instant l'agent lui meme)
	agy->bodyID_but = agy->bodyID;

	//on lui donne une jolie couleur verte !
	agy->couleur = 0;

	//
	agy->au_dessus = 0;

	//
	agy->au_dessous = 0;

	//
	agy->au_devant = 0;

	//sa position par defaut est tres loin, pour le test de pos plus proche
	//agy->old_pos.x = 1000.0;agy->old_pos.y = 1000.0;agy->old_pos.z = 1000.0;

	//le nombre d'iterations
	agy->cycle = 0;

	return(agy);
}


//initialisation d'une collection d'agent placés au hasard sur le bloc de départ
void init_groupe_agents( Agent tab_agents[] , int nb_agents, dWorldID monde, dSpaceID espace ){
	//on utilise une astuce pour les placer, on a une matrice des places vides qu'on remplie peu à peu.
	int places[(int)(LARGEUR*4)][(int)(LONGUEUR*4)];
	int i,j,randx,randy;
	float abs,ord,ajustx,ajusty,ajustz;
	Agent *pa;

	//
	abs = (LARGEUR*4);
	ord = (LONGUEUR*4);

	ajustx = (FOSSE/2);
	ajusty = -(LONGUEUR/4);
	ajustz = ( EPAISSEUR + HAUSSE + HAUSSE_AGENT + RAYON);

	//init des places libres
	for( i=0 ; i < abs ; i++ ) { for( j=0 ; j < ord ; j++ ){ places[i][j] = 0; } }

	//pour tous les agent
	for( i=0 ; i<nb_agents ; i++ ){
		srand( time(NULL) );
		//on choisit au hasard une case vide
		do {	randx = abs * ( (double) rand() / (double) ( RAND_MAX - 1) );
			randy = ord * ( (double) rand() / (double) ( RAND_MAX - 1) );
		} while ( places[randx][randy] != 0 );
		//on initialise un agent dedans.
		places[randx][randy] = 1;
		pa = initAgent ( &(tab_agents[i]), ajustx+((double)randx/4), ajusty+((double)randy/8)+0.0001, ajustz , monde, espace );//
		tab_agents[i] = *pa;
	}
}
/*
//initialisation d'une collection d'agent placés au hasard sur le bloc de départ
void init_ligne_agents( Agent tab_agents[] , int nb_agents, dWorldID monde, dSpaceID espace ){
	//on place les agents en ligne au millieu du bloc, espacés par une taille d'agent
	int i;
	dReal x_esp,x_agent,y_agent,z_agent;
	Agent *pa;

	x_agent = (FOSSE/2)+(0.05);//+LARGEUR);
	y_agent = 0.0001;
	z_agent = ( EPAISSEUR + HAUSSE + HAUSSE_AGENT + RAYON);

	x_esp = (RAYON*2);

	//pour tous les agent
	for( i=0 ; i<nb_agents ; i++ ){
		pa = initAgent ( &(tab_agents[i]), x_agent , y_agent , z_agent , monde, espace );
		tab_agents[i] = *pa;
		x_agent += x_esp;
	}
}

*/
//initialisation d'un groupe de 10 agents à des positions, couleurs et états bien spécifiques
void init_ligne_agents( Agent tab_agents[] , int nb_agents, dWorldID monde, dSpaceID espace ){
	Agent *pa;
        int i;
	dReal x_esp,x_agent,y_agent,z_agent, tage;

	x_esp = (RAYON*2);
	tage = (RAYON*2);

        i = 0;
	x_agent = (FOSSE/2)+(0.05);
	y_agent = 0.0001;
	z_agent = ( EPAISSEUR + HAUSSE + HAUSSE_AGENT + RAYON);


        //une ligne de 10

        //1
	pa = initAgent ( &(tab_agents[i]), x_agent + ( 0* tage) , y_agent + (0 * tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

	//2
	pa = initAgent ( &(tab_agents[i]), x_agent + ( 1* tage) , y_agent + ( 0* tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

	//3
	pa = initAgent ( &(tab_agents[i]),x_agent + ( 2* tage) , y_agent + ( 0* tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

	//4
	pa = initAgent ( &(tab_agents[i]), x_agent + ( 3* tage) , y_agent + ( 0* tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

	//5
	pa = initAgent ( &(tab_agents[i]), x_agent + ( 0* tage) , y_agent + ( 1* tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

	//6
	pa = initAgent ( &(tab_agents[i]), x_agent + ( 1* tage) , y_agent + ( 1* tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

	//7
	pa = initAgent ( &(tab_agents[i]), x_agent + ( 0* tage) , y_agent + ( -1* tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

	//8
	pa = initAgent ( &(tab_agents[i]), x_agent + ( 1 * tage) , y_agent + ( -1 * tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

	//9
	pa = initAgent ( &(tab_agents[i]), x_agent + ( 2* tage) , y_agent + ( -1* tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

	//10
	pa = initAgent ( &(tab_agents[i]), x_agent + ( 0* tage) , y_agent + ( -2* tage), z_agent , monde, espace );
	//pa->etat = ;
	pa->couleur = 2;
	tab_agents[i] = *pa;
	i++;
	//x_agent += x_esp;

}

