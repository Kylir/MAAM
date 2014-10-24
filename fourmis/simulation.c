/* les fonctions qui gerent la simulation proprement dite */

#include "boule.h"

/**********************************************************************************/
//Fonction de demarrage de la simulation
void start(){
	nb_pas_simulation = 0;
	nb_noyades = 0;

	//vue des deux blocs (0.1801,-4.8489,4.3100,92.0000,-10.0000,0.0000)
	//vue au dessus du premier bloc (1.7989,0.0229,6.9900,90.0000,-90.5000,0.0000)
	//vue de biais (0.1425,-0.9999,3.4700,68.5000,-23.0000,0.0000)
	//vue du haut (-0.4781,-0.0360,5.0800,0.0000,-51.0000,0.0000)
 	//pour la vidéo -0.7500,0.6331,3.5700,-38.5000,-26.5000,0.0000
	static float xyz[3] = {-0.7500,0.6331,3.5700};
	static float hpr[3] = {-38.5000,-26.5000,0.0000};
	dsSetViewpoint (xyz,hpr);
	//printf("Début de la simulation...\n");
}

/**********************************************************************************/
//Fonction de boucle de simulation
void simLoop (int pause){
	int i;

	if (!pause) {

		//Affichage du pas de simulation
		//printf("%d\n",nb_pas_simulation);
		nb_pas_simulation++;


		//pas de simulation pour les agents
		for( i=0 ; i<NB_AGENTS ; i++ ){
			//on tue les agents qui sont à terre !!!
			//noyade( &(agents[i]) );

			//si un agent arrive au bord il s'arrete
			//arret_au_bord( &(agents[i]) );

			//Chaque agent accompli ses actions
			step( &(agents[i]) );
		}

		//collisions eventuelles
		dSpaceCollide (space, 0, &nearCallback );

		//un pas de simulation
		dWorldStep (world, PAS_SIMU );
		//dWorldStepFast1 ((world, PAS_SIMU, 1 );

		//nettoyage apres collision
		dJointGroupEmpty (contactgroup);
	}

	//dessin de la scène
	//if ( nb_pas_simulation % 1 == 0 ) {
	dessin_env( &bloc_depart, &bloc_arrive );
	for( i=0 ; i < NB_AGENTS ; i++ ){ dessinAgent( &(agents[i]) ); }
	dessin_attractant( &attract );
	//}
}

/**********************************************************************************/
//Fonction de pre-collision ;)

void nearCallback (void *data, dGeomID o1, dGeomID o2){

	Agent *page1,*page2;
	dContact contact;
	dBodyID b1,b2;
        int n;

	//on retrouve les corps en fonction des geometries
	b1 = dGeomGetBody(o1);
	b2 = dGeomGetBody(o2);

	//Si il y a collision
	if (dCollide (o1,o2,0,&contact.geom,sizeof(dContactGeom))) {

        	//b1 et b2 sont-ils des agents ?
		page1 = (Agent *)cherche_agent(b1);
		page2 = (Agent *)cherche_agent(b2);

		//Si le contact est avec un autre agent les frottements changent !!!
		if( page1 != NULL && page2 != NULL ) {
			//ici on a un contact entre deux agents : on met les frottements a COEFF_FROTTEMENTS_AGENTS
			contact.surface.mode = dContactSoftERP;
			contact.surface.mu = COEFF_FROTTEMENTS_AGENTS;
			contact.surface.soft_erp = 0.01;
		} else {
                	//autres types de contact
			contact.surface.mode = dContactSoftERP;
			contact.surface.mu = COEFF_FROTTEMENTS;
			contact.surface.soft_erp = 0.01;
		}

		//on crée le joint de contact
		dJointID c = dJointCreateContact(world,contactgroup,&contact);
		dJointAttach (c,b1,b2);
		//printf("********* Nouveau tour ! *************\n");


                //Maintenant on remplie les informations de collision

		//Pour l'agent 1 : s'il existe...
		if( page1 != NULL ) {
		//Pour l'instant aucun mecanisme de verification de redondance est mis en oeuvre...

                        //pour etre plus lisible, on ramene le nombre de collision deja remplies
			n = page1->nombre_collision;

			//on s'assure qu'il y a encore une place dans le tableau de collision...
			if( n < MAX_INFOS_COLLISION ) {
				//on rempli l'objet
				page1->capteurs_collision[n].objet = b2;

				//on s'occupe du type_objet
				if( page2 != NULL) {
					page1->capteurs_collision[n].type_objet = 4;
					page1->capteurs_collision[n].etat = page2->etat;
				} else {
                                        page1->capteurs_collision[n].type_objet = quel_objet(b2);
				}

                                //on incremente le nombre de case remplies
                                page1->nombre_collision++;

			} else {
				printf("Attention : tentative de depassement de MAX_INFOS_COLLISION.\n");
			}
		}

		//Pour l'agent 2 : s'il existe...
		if( page2 != NULL ) {
		//Pour l'instant aucun mecanisme de verification de redondance est mis en oeuvre...

                        //pour etre plus lisible, on ramene le nombre de collision deja remplies
			n = page2->nombre_collision;

			//on s'assure qu'il y a encore une place dans le tableau de collision...
			if( n < MAX_INFOS_COLLISION ) {
				//on rempli l'objet
				page2->capteurs_collision[n].objet = b1;

				//on s'occupe du type_objet
				if( page1 != NULL) {
					page2->capteurs_collision[n].type_objet = 4;
					page2->capteurs_collision[n].etat = page1->etat;
				} else {
                                        page2->capteurs_collision[n].type_objet = quel_objet(b1);
				}

                                //on incremente le nombre de case remplies
                                page2->nombre_collision++;

			} else {
				printf("Attention : tentative de depassement de MAX_INFOS_COLLISION.\n");
			}
		}



	}
}


/**********************************************************************************/
//Fonction d'interaction clavier
void command (int cmd){
  	switch (cmd) {
  	case 't':
		printf("Pas de simulation numero %ld.\n",nb_pas_simulation);
		break;
	case 'g':
		printf("Go !!!\n");
		break;
	case 'n':
		printf("Il y a %ld agents au sol.\n",nb_noyades);
		break;
	case 'e':
		printf("L'agent 0 est dans l'etat %d.\n",agents[0].etat);
		break;

  	}
}

/**********************************************************************************/
//Fonction qui me dit avec quoi l'agent cogne : 1-> collision bloc 1 ; 2-> collision bloc 2 ; 3-> collision avec l'attractant
int quel_objet(dBodyID b) {
	int ret=5;
        if ( b == bloc_depart.bodyID ) { ret = 1; }
	if ( b == bloc_arrive.bodyID ) { ret = 2; }
	if ( b == attract.bodyID ) { ret = 3; }
	return(ret);
}

/**********************************************************************************/
//Fonction qui
