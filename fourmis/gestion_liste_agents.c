//gcc gestion_liste_agents.c -c -I/home/forest/include -L/home/forest/lib

#include "boule.h"

/*************** gestion des listes chainées de joints ********************/

//ajout d'un element en fin de liste - pas de retour !!!
listeAgents *ajout_agent( listeAgents *p_liste, Agent *agy, dJointID joint, int nb_avant ) {
	listeAgents *newLA;
	listeAgents *p = p_liste;
	int i;

	//création de l'element
	newLA = ( listeAgents * ) malloc( sizeof( listeAgents ) );

	//initialisation de l'element
	newLA->agent = agy;
	newLA->id = joint;
	newLA->p_suivant = NULL;

	//si la liste est vide (insertion en premier)
	if( p_liste == NULL ){
		return( newLA );
	} else {
		//insertion dans la liste : je parcours jusqu'au dernier non NULL
		for( i = 0 ; i < (nb_avant-1) ; i++ ){
			p = p->p_suivant;
		}
		//et je m'insere
		p->p_suivant = newLA;
		return(p_liste);
	}
}

//test de la presence d'un element en passant son dBodyID : 0->faux, 1->vrai
int present( listeAgents *p_liste, dBodyID body ){
	listeAgents *p = p_liste;

	//tant que p ne pointe pas sur celui que je cherche et que p n'est pas nul
	while( (p != NULL) && ((p->agent)->bodyID != body) ) {
		p = p->p_suivant;
	}

	if( p == NULL) {
		return(0);
	} else {
		return(1);
	}
}

//recherche d'un element en passant son dBodyID : 0->faux, 1->vrai
Agent *recherche( listeAgents *p_liste, dBodyID body ){
	listeAgents *p = p_liste;

	//tant que p ne pointe pas sur celui que je cherche et que p n'est pas nul
	while( (p != NULL) && ((p->agent)->bodyID != body) ) {
		p = p->p_suivant;
	}

	if( p == NULL) {
		return(NULL);
	} else {
		return(p->agent);
	}
}

//suppression d'un element
listeAgents *supprime( listeAgents *p_liste, dBodyID body ){
	listeAgents *p = p_liste;
	listeAgents *p_avant = p;

	//si c'est le premier element
	if( (p_liste->agent)->bodyID == body ){
		p = p->p_suivant;
		free( p_liste );
		return( p );
	} else {
		//sinon, tant que p ne pointe pas sur celui que je cherche et que p n'est pas nul
		while( (p != NULL) && ((p->agent)->bodyID != body) ) {
			p_avant = p;
			p = p->p_suivant;
		}

		if( p != NULL) {
			//quand p pointe sur l'element a enlever et p avant sur le precedent, on
			p_avant->p_suivant = p->p_suivant;
			free(p);
		}
		return(p_liste);
	}
}

//affichage de la liste - pour DEBUG
void affiche( listeAgents *p_liste ){
	int body;
	listeAgents *p = p_liste;

	while( p != NULL ) {
		body = (int) (p->agent)->bodyID;
		printf("%i -> ", body);
		p = p->p_suivant;
	}
	printf("NULL\n");
}
