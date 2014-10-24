/* Fichier de gestion du comportement des ROBOTS
   - Deplacements
   - Comportements suivant les collisions
*/

#include "boule.h"


/****************************************************************************/
//fonction de recherche d'un agent dans le tableau d'agent
int is_agent( dBodyID bod ){
	int i=0;
	//on parcours le tableau en comparant
	while( (i < NB_AGENTS ) && ( agents[i].bodyID != bod ) ) {i++;}
	if( i < NB_AGENTS ){return(1);} else {return(0);}
}

/****************************************************************************/
//fonction de recherche avec renvoie de l'agent
Agent *cherche_agent( dBodyID bod ){
	int i=0;
	//on parcours le tableau en comparant
	while( (i < NB_AGENTS ) && ( agents[i].bodyID != bod ) ) {i++;}
	if( i < NB_AGENTS ){return( &(agents[i]) );} else { return( NULL ); }
}

/****************************************************************************/
//fonction de distance en 2D entre deux points par projection sur un plan (pas de z)
dReal distance2D( Vecteur *pv1 , Vecteur *pv2 ){
	dReal rez;
	rez = sqrt( ((pv1->x)-(pv2->x))*((pv1->x)-(pv2->x)) + ((pv1->y)-(pv2->y))*((pv1->y)-(pv2->y)) );
	return(rez);
}

/****************************************************************************/
//fonction de distance en 3D entre deux points
dReal distance3D( Vecteur *pv1 , Vecteur *pv2 ){
	dReal rez,d2d,diff_z;

	d2d = distance2D( pv1 , pv2 ); //la distance 2d
	diff_z = (pv1->z)-(pv2->z); //la difference entre les z
	rez = sqrt( d2d*d2d + diff_z*diff_z );
	return(rez);
}

/****************************************************************************/
//fonction qui regarde si un agent est entre moi et le but
int devant_moi( Agent *moi, Agent *agy, Attractant *but, dReal *pos_moi, dReal *pos_lui, dReal *pos_but ) {
	//dReal *pos_moi,*pos_lui,*pos_but;
	Vecteur v1,v2,v3;

	//je remplis les vecteurs
	v1.x = pos_moi[0]; v1.y = pos_moi[1]; v1.z = pos_moi[2];
	v2.x = pos_but[0]; v2.y = pos_but[1]; v2.z = pos_but[2];
	v3.x = pos_lui[0]; v3.y = pos_lui[1]; v3.z = pos_lui[2];

	if( dans_pyramide( &v1, &v2, &v3 ) ) { return(1); } else { return(0); }
}

/****************************************************************************/
//fonction qui tue un agent s'il tombe
void noyade( Agent *agy ){
	dReal *pos;
	pos = (dReal*) dBodyGetPosition( agy->bodyID );
	if ( pos[2] <= (5.0 * RAYON) ) {
		if ( dBodyIsEnabled( agy->bodyID ) == 1 ) {
			nb_noyades++;
			dBodyDisable( agy->bodyID );
			//printf("Noyade !\n");
		}
	}

}
/****************************************************************************/
//fonction qui me dit si je suis sur la bande gluante (bord du fossé)
int dans_bande_gluante1( Agent *agy ){
	dReal *pos;

	//ma position
	pos = (dReal*) dBodyGetPosition( agy->bodyID );

	//l'agent est au bord
	if ( pos[0] <= ( (FOSSE/2) + TAILLE_BANDE_GLUANTE) &&
	     pos[0] >= (FOSSE/2) &&
	     pos[1] <= (LONGUEUR/2) &&
	     pos[1] >= (-LONGUEUR/2) &&
	     pos[2] <= (EPAISSEUR + HAUSSE + RAYON ) &&
	     pos[2] >= (EPAISSEUR ) ) {
	     	//printf("Cas 1.\n");
		return(1);
	}
	//l'agent est sur le cote en haut
	if ( pos[0] <= ((FOSSE/2) + LONGUEUR) &&
	     pos[0] >= (FOSSE/2) &&
	     pos[1] <= (LONGUEUR/2) &&
	     pos[1] >= ((LONGUEUR/2) - TAILLE_BANDE_GLUANTE) &&
	     pos[2] <= (EPAISSEUR + HAUSSE + RAYON ) &&
	     pos[2] >= (EPAISSEUR ) ) {
		//printf("Cas 2.\n");
		return(2);
	}
	//l'agent est sur le cote en bas
	if ( pos[0] <= ((FOSSE/2) + LONGUEUR) &&
	     pos[0] >= (FOSSE/2) &&
	     pos[1] <= ((-LONGUEUR/2) - TAILLE_BANDE_GLUANTE) &&
	     pos[1] >= (-LONGUEUR/2) &&
	     pos[2] <= (EPAISSEUR + HAUSSE + RAYON ) &&
	     pos[2] >= (EPAISSEUR ) ) {
		//printf("Cas 3.\n");
		return(3);
	}
	return(0);
}

/****************************************************************************/
//cette fonction verifie que la vitesse de l'agent ne dépasse pas un certain seuil
//l'agent a une vitesse dans l'intervalle [SEUIL_VITESSE, -SEUIL_VITESSE ]
int vitesse_ok( dReal *v_vitesse ) {
	if( v_vitesse[0] < SEUIL_MAX_VITESSE && v_vitesse[0] > -SEUIL_MAX_VITESSE ) {
		return(1);
	} else {
		return(0);
	}
}


/****************************************************************************/
//fonction qui calcule le point de contact entre les deux agents
void calc_point_accro( dReal *pos_agent1, dReal *pos_agent2, Vecteur *rez ) {
	rez->x = (pos_agent1[0] + pos_agent2[0])/2;
	rez->y = (pos_agent1[1] + pos_agent2[1])/2;
	rez->z = (pos_agent1[2] + pos_agent2[2])/2;
}

/****************************************************************************/
//fonction de calcul des coordonnées du point d'ancrage
void calcul_pt_ancrage( Agent *moi, dBodyID but, Vecteur *vrez ){

	dReal D,d;
	dReal *p1,*p2;
	Vecteur v1,v2;

	//la distance d
	d = RAYON;

	//je ramene ma position
	p1 = (dReal*) dBodyGetPosition( moi->bodyID );

	//je ramene la position de l'attractant
	p2 = (dReal*) dBodyGetPosition( but );

	//je trouve D la distance entre moi et le but
	v1.x = *(p1+0); v1.y = *(p1+1); v1.z = *(p1+2);
	v2.x = *(p2+0); v2.y = *(p2+1); v2.z = *(p2+2);
	D = distance2D( &v1 , &v2 );

	//je remplis le vecteur resultat avec un bon vieux thm de Thales ;)
	vrez->x = v1.x + ( ( ( v2.x - v1.x )*d )/D );
	vrez->y = v1.y + ( ( ( v2.y - v1.y )*d )/D );
	vrez->z = v1.z-d;// + d;


}

/****************************************************************************/
//Calcul de la normale pour l'axe du joint hinge
void calcul_normale( Agent *agy, Attractant *att, Vecteur *pt_ancrage, Vecteur *vnorm){
	dReal denom,x1,x2,x3,y1,y2,y3,z1,z2,z3;
	dReal *p1,*p2;

	//je ramene la position de l'agent
	p1 = (dReal*) dBodyGetPosition( agy->bodyID );
	x1 = *(p1+0); y1 = *(p1+1); z1 = *(p1+2);

	//je ramene la position de l'attractant
	p2 = (dReal*) dBodyGetPosition( att->bodyID );
	x2 = *(p2+0); y2 = *(p2+1); z2 = *(p2+2);

	//et enfin la position du point d'attache
	x3 = pt_ancrage->x ; y3 = pt_ancrage->y ; z3 = pt_ancrage->z ;

	//le denominateur
	denom = x1*y2*z3 - x1*y3*z2 + x2*y3*z1 - x2*z3*y1 + x3*y1*z2 - x3*y2*z1;
	if (denom != 0) {
		vnorm->x = (y1*z2-y2*z1)/denom;
		vnorm->y = (x2*z1-x1*z2)/denom;
		vnorm->z = (x1*y2-x2*y1)/denom;
	} else {
		vnorm->x = 0;
		vnorm->y = 0;
		vnorm->z = 0;
	}
	//printf("\tVecteur du Hinge : ( %f ; %f ; %f )\n",vnorm->x,vnorm->y,vnorm->z);
}

/****************************************************************************/
//fonction qui lance la simulation
void marche( Agent *agy, dBodyID att ){
	dReal *pos_att;
	agy->etat = 0;
	agy->couleur = 0;
	agy->cycle = 0;

	pos_att = (dReal*) dBodyGetPosition( att );

	agy->pos_but.x = pos_att[0];
	agy->pos_but.y = pos_att[1];
	agy->pos_but.z = pos_att[2] - RAYON_ATTRACTANT;

}



/****************************************************************************/
//fonction qui fait un pas de calcul de l'agent
void step(Agent *agy){
	dReal *pos_agent,*pos_agent2, *pos_att;
	int glu;
	Agent *a;//,*b;
	dBodyID bobo;
	//Vecteur v;

        //on regarde l'etat de l'agent
	switch( agy->etat ) {

	//etat de début de simulation
        case 0 :
		//Description : au moment ou je rencontre le sol je passe à l'état 1. Avant je ne fais rien.

		//je cherche dans la liste des collisions une avec le bloc 1 (type 1)
		bobo = cherche_collision_type( agy, 1 );

		//si elle s'y trouve alors je passe à l'etat 0
		if( bobo != (dBodyID) -1 ) {
			printf("Attractant trouvé !\n");
			agy->etat = 1;
                 	agy->couleur = 1;
			agy->cycle = 0;
			//on assigne un nouveau but à l'agent
			pos_att = (dReal*) dBodyGetPosition( attract.bodyID );
			agy->pos_but.x = pos_att[0]; agy->pos_but.y = pos_att[1]; agy->pos_but.z = 0;//pos_att[2] - RAYON_ATTRACTANT;

			agy->bodyID_but = attract.bodyID;
		}
		break;

        case 1 :
		//Description : dans cet etat l'agent se dirige vers l'attractant en avancant sur le sol
		// Quand il rencontre un autre agent dans l'etat 2 ou le fossé alors il passe à l'etat 2

                //si je rencontre le fossé ou un autre à l'état 2 je passe à l'état 2.
		glu = dans_bande_gluante1(agy);

		a = cherche_collision_agent_etat( agy, 2 );

		if( glu == 1 || a != NULL ) {

			//le bloc 1
			bobo = cherche_collision_type( agy, 1 );

			if( bobo != (dBodyID) -1 ) {

				//on passe a l'etat de reconfiguration.
				agy->etat = 2;
				agy->couleur = 2;
				agy->cycle = 0;

				//la position de l'agent
				pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );

				//puis on s'accroche au bloc
				dJointAttach( agy->patte, agy->bodyID, bobo );
				dJointSetUniversalAnchor( agy->patte, pos_agent[0], pos_agent[1], pos_agent[2]-(RAYON/2) );
				dJointSetUniversalAxis1( agy->patte, 0.0, 1.0, 0.0);
				dJointSetUniversalAxis2( agy->patte, 1.0, 0.0, 0.0);

				//Et on triche un peu en mettant le but sur le bloc de départ
				agy->pos_but.x = ((FOSSE/2) + (RAYON) );
				agy->pos_but.z = (EPAISSEUR + HAUSSE + RAYON);

			}
		} else {
                        	pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
				regle_vitesse( agy, pos_agent );
		}
		break;


	case 2 :
		//Description : Etat de reconfiguration.
		if( ordre_arrivee == 9 ){
                        printf("Yo !\n");
                	agy->couleur = 3;
			agy->etat = 3;
			agy->cycle = 0;
		}

		if( agy->cycle > 15 && agy->stop == 0 ){
                	//si je rencontre un agent à l'état 1 je me cramponne
			a = cherche_collision_agent_etat( agy, 1 );

			if( a != NULL ) {
				//printf("Pour pas tomber je me cramponne !!!\n");

				bobo = cherche_collision_type( agy, 1 );

				if( bobo != (dBodyID) -1 ) {
					agy->cycle = 0;
					pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
					dJointAttach( agy->patte, agy->bodyID, bobo );
					dJointSetUniversalAnchor( agy->patte, pos_agent[0], pos_agent[1], pos_agent[2]-(RAYON/2) );
					dJointSetUniversalAxis1( agy->patte, 0.0, 1.0, 0.0);
					dJointSetUniversalAxis2( agy->patte, 1.0, 0.0, 0.0);
				}
			//sinon si je rencontre un agent à l'état 2 qui n'est pas encore accroché à moi : partage de ll'info, accrochage et on se place
			} else {
                                //si je rencontre un agent à l'état 2
				a = cherche_collision_agent_etat( agy, 2 );

				if( a != NULL ) {
					//printf("Un autre ! on s'accroche !!!\n");

					//On compare nos x le plus grand perd et se place derriere
					pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
					pos_agent2 = (dReal*) dBodyGetPosition( a->bodyID );

					if( (pos_agent[0] > pos_agent2[0]) || a->stop == 1 ) {
                                        	agy->en_reconf = 1;
						agy->bodyID_but = a->bodyID;
					}
				}
				if( agy->en_reconf == 0 ) {
                                        //je longe le bord

					//si je suis a mon but
					if( au_but( agy ) ) {
                                        	//printf("Je suis arrivé !!!\n");
                                                //Je suis arrivé je m'immobilise
						bobo = cherche_collision_type( agy, 1 );
						if( bobo != (dBodyID) -1 ) {
							//indicateur de stop
							agy->stop = 1;
							//je me cramponne
							pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
							dJointAttach( agy->patte, agy->bodyID, bobo );
							dJointSetUniversalAnchor( agy->patte, pos_agent[0], pos_agent[1], pos_agent[2]-(RAYON/2) );
							dJointSetUniversalAxis1( agy->patte, 0.0, 1.0, 0.0);
							dJointSetUniversalAxis2( agy->patte, 1.0, 0.0, 0.0);
						}
					} else {
						//je me détache
						dJointAttach( agy->patte, 0, 0 );
						//et j'avance
						pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
						reglage_fin_vitesse( agy, pos_agent, 1, 1, 1 );
					}
				} else {
                                        pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
					pos_agent2 = (dReal*) dBodyGetPosition( agy->bodyID_but );

					if( derriere_agent_ok( pos_agent, pos_agent2 ) ) {
                                                //printf("Je suis derrière.\n");
						a = cherche_agent( agy->bodyID_but );
						if( a->stop == 1 ) {
							//printf("Je suis arrivé !!!\n");
							//Je suis arrivé je m'immobilise
							bobo = cherche_collision_type( agy, 1 );
							if( bobo != (dBodyID) -1 ) {
								//indicateur de stop
								agy->stop = 1;
								//je me cramponne
								pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
								dJointAttach( agy->patte, agy->bodyID, bobo );
								dJointSetUniversalAnchor( agy->patte, pos_agent[0], pos_agent[1], pos_agent[2]-(RAYON/2) );
								dJointSetUniversalAxis1( agy->patte, 0.0, 1.0, 0.0);
								dJointSetUniversalAxis2( agy->patte, 1.0, 0.0, 0.0);

								//et je m'accroche à celui de devant - j'ai son bodyID !!! niarf niarf niarf !
								dJointAttach( agy->avant, agy->bodyID, a->bodyID );
								a->arriere = agy->avant;
								dJointSetHingeAnchor( agy->avant, pos_agent[0],  pos_agent[1], pos_agent[2] );
								dJointSetHingeAxis( agy->avant, 1, 0, 0);

								//j'augmente le compteur
								ordre_arrivee++;
								//printf("ordre_arrivee = %d\n",ordre_arrivee);

							}
						}
					} else {
						//je "suis" l'agent qui est mon but
						//je me détache
						dJointAttach( agy->patte, 0, 0 );
						//et j'avance
						pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
						pos_agent2 = (dReal*) dBodyGetPosition( agy->bodyID_but );
						suivi_agent( agy, pos_agent, pos_agent2, 0.25, 0.25, 0.25);
					}
				}
			}
		}
		break;

	case 3 :
		//Description : dans cet etat, l'agent avance devant lui

		//on décroche les agents du sol pour pouvoir avancer
                dJointAttach( agy->patte, 0, 0 );

		//si l'agent est le dernier on lui applique une force
		if( agy->cycle < 25 ){
			//pas d'accrochant derriere ?
			bobo = dJointGetBody ( agy->arriere, 0);
			if( bobo == (dBodyID ) 0 ) {
				//c'est vraiment sur ?
				bobo = dJointGetBody ( agy->arriere, 1);
				if( bobo == (dBodyID ) 0 ) {
                                        //est ce que je touche le sol ?
					//bobo = cherche_collision_type( agy, 1 );
					//if( bobo !=  (dBodyID) -1 ) {
						dBodySetForce( agy->bodyID, -15,0,0 ); //ICI C'EST LA FORCE !!!!
					//}
				}
			}
		}
		if( agy->cycle > 50 ) {
			agy->cycle = 0;
		}

		break;

	/*case X :
		//Description : dans cet etat, l'agent
		break;*/


	} //fin du switch

        //Quel que soit l'etat de l'agent, il y a des choses à faire
	//on incremente le cycle
	(agy->cycle) += 1;
	//On remet à 0 le nombre de collision
	agy->nombre_collision = 0;

}

/****************************************************************************/
//fonction qui met regle ma vitesse
void regle_vitesse( Agent *agy, dReal *pos_agent ) {
	Vecteur va;

	//Je trouve un vecteur vers mon but (ma_position - but_position)
	va.x = ( agy->pos_but.x - pos_agent[0] );
	va.y = ( agy->pos_but.y - pos_agent[1] );
	va.z = ( agy->pos_but.z - pos_agent[2] );

	//Je le norme et le pondere pour avoir un vecteur de norme == VITESSE_AGENT
	va.x =  0.2 * (va.x / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));
	va.y =  0.2 * (va.y / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));
	va.z = (va.z / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));//*0.1;

	//J'applique cette vitesse à l'agent
	dBodySetLinearVel( agy->bodyID, va.x, va.y, va.z);
}

/****************************************************************************/
//fonction qui met regle finement ma vitesse
void reglage_fin_vitesse( Agent *agy, dReal *pos_agent, float param_x, float param_y, float param_z ) {
	Vecteur va;

	//Je trouve un vecteur vers mon but (ma_position - but_position)
	va.x = ( agy->pos_but.x - pos_agent[0] );
	va.y = ( agy->pos_but.y - pos_agent[1] );
	va.z = ( agy->pos_but.z - pos_agent[2] );

	//Je le norme et le pondere pour avoir un vecteur de norme == VITESSE_AGENT
	va.x =  0.2 * (va.x / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));
	va.y =  0.2 * (va.y / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));
	va.z = (va.z / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));//*0.1;

	//les params :
	va.x = param_x * va.x;
	va.y = param_y * va.y;
	va.z = param_z * va.z;

	//J'applique cette vitesse à l'agent
	dBodySetLinearVel( agy->bodyID, va.x, va.y, va.z);
}

/****************************************************************************/
//fonction qui suis un agent legerement en retrait
void suivi_agent( Agent *agy, dReal *pos_agent, dReal *pos_cible, float param_x, float param_y, float param_z ) {
	Vecteur va;

	//Je trouve un vecteur vers mon but (ma_position - but_position)
	va.x = ( (pos_cible[0] + 2*RAYON + DIST_SECUR) - pos_agent[0] );
	va.y = ( pos_cible[1] - pos_agent[1] );
	va.z = ( pos_cible[2] - pos_agent[2] );

	//Je le norme et le pondere pour avoir un vecteur de norme == VITESSE_AGENT
	va.x =  0.2 * (va.x / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));
	va.y =  0.2 * (va.y / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));
	va.z = (va.z / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));//*0.1;

	//les params :
	va.x = param_x * va.x;
	va.y = param_y * va.y;
	va.z = param_z * va.z;

	//J'applique cette vitesse à l'agent
	dBodySetLinearVel( agy->bodyID, va.x, va.y, va.z);
}


/****************************************************************************/
//fonction qui met regle ma vitesse mais juste sur le coté : pour me balader
void regle_vitesse_balade( Agent *agy, dReal *pos_agent ) {
	Vecteur va;

	//Je trouve un vecteur vers mon but (ma_position - but_position)
	va.x = ( agy->pos_but.x - pos_agent[0] );
	va.y = ( agy->pos_but.y - pos_agent[1] );
	va.z = ( agy->pos_but.z - pos_agent[2] );

	//Je le norme et le pondere pour avoir un vecteur de norme == VITESSE_AGENT
	va.x = 0;// 0.2 * (va.x / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));
	va.y =  0.2 * (va.y / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));
	va.z = (va.z / ( sqrt( va.x*va.x + va.y*va.y + va.z*va.z ) ));//*0.1;

	//J'applique cette vitesse à l'agent
	dBodySetLinearVel( agy->bodyID, va.x, va.y, va.z);
}

/****************************************************************************/
//fonction qui cherche dans la liste des collisions un objet de type_collision == n ; renvoie le 1er trouvé.
//si aucun, renvoie -1
dBodyID cherche_collision_type( Agent *agy, int ty ) {
	int n = agy->nombre_collision;
        int i;

	for( i = 0 ; i < n ; i++ ) {
		//si a la case i j'ai un objet de type n je le renvoie
        	if( agy->capteurs_collision[i].type_objet == ty ) {
			return(agy->capteurs_collision[i].objet);
		}
	}
	return( (dBodyID) -1 );
}

/****************************************************************************/
//fonction qui cherche dans la liste des collisions un agent ; renvoie un pointeur sur le 1er trouvé.
//si aucun, renvoie NULL
Agent *cherche_collision_agent( Agent *agy ) {
	int n = agy->nombre_collision;
        int i;
	Agent *a;

	for( i = 0 ; i < n ; i++ ) {
        	if( agy->capteurs_collision[i].type_objet == 4 ) {
			//si a la case i j'ai un agent je le renvoie
			a = cherche_agent( agy->capteurs_collision[i].objet );
			return(a);
		}
	}
	return( NULL );
}

/****************************************************************************/
//fonction qui cherche dans la liste des collisions un agent ; renvoie un pointeur sur le 1er trouvé.
//si aucun, renvoie NULL
Agent *cherche_collision_agent_etat( Agent *agy, int etat ) {
	int n = agy->nombre_collision;
        int i;
	Agent *a;

	for( i = 0 ; i < n ; i++ ) {
        	if( (agy->capteurs_collision[i].type_objet == 4) && (agy->capteurs_collision[i].etat == etat) ) {
			//si a la case i j'ai un agent d'etat "etat" je le renvoie
			a = cherche_agent( agy->capteurs_collision[i].objet );
			return(a);
		}
	}
	return( NULL );
}

/****************************************************************************/
//fonction qui assigne un nouveau but à l'agent
void nouveau_but( Agent *agy, Vecteur pos_but, dBodyID id_but ) {
	agy->pos_but.x = pos_but.x;
	agy->pos_but.y = pos_but.y;
	agy->pos_but.z = pos_but.z;

	agy->bodyID_but = id_but;
	//DEBUG
	//printf("New Goal !!!\n");
}

/****************************************************************************/
//fonction qui fais que l'agent va à sa place = lui assigne un but en fonction de sa place.
void a_ta_place( Agent *agy, int place ) {
	//lla première place est celle située : X = (1/2 * FOSSE) + RAYON   Y = 0.00001  Z =
	//Vecteur v;
	agy->pos_but.x = ( (1/2 * FOSSE) + (4*RAYON) ) + (place * ( 2*RAYON ));
	agy->pos_but.y = 0;
	agy->pos_but.z = 0;


}



/****************************************************************************/
//fonction permettant a un agent de grimper sur un autre
void grimpe_dessus( Agent *passif, Agent *actif ) {
	Vecteur new_but;
	dReal *pos_passif;

	//on dit au passif qu'on lui grimpe dessus
	passif->au_dessus = 1;

	//on ramene la position du passif
	pos_passif = (dReal*) dBodyGetPosition( passif->bodyID );

	//on calcul la position du nouveau but
	new_but.x = pos_passif[0];
	new_but.y = pos_passif[1];
	new_but.z = pos_passif[2] + (2.0*RAYON);

	//on l'assigne à l'agent
	nouveau_but( actif, new_but, passif->bodyID );
}

/****************************************************************************/
//fonction permettant a un agent de passer au suivat dans la chaine
void grimpe_suivant( Agent *passif, Agent *actif ) {
	Vecteur new_but;
	dReal *pos_passif;

        if( passif->au_dessus == 0 ) {
		//on lui dit au passif que lui grimpe dessus
		passif->au_dessus = 1;

		//on ramene la position du passif
		pos_passif = (dReal*) dBodyGetPosition( passif->bodyID );

		//on calcul la position du nouveau but
		new_but.x = pos_passif[0];
		new_but.y = pos_passif[1];
		new_but.z = pos_passif[2] + (2.0*RAYON);

		//on l'assigne à l'agent
		nouveau_but( actif, new_but, passif->bodyID );
	}
}


/****************************************************************************/
//fonction qui me dit quand je suis arrivé à mon but
int au_but( Agent *agy ) {
	dReal *pos;
	Vecteur diff;

        //je trouve ma position
	pos = (dReal*) dBodyGetPosition( agy->bodyID );

	//je fait la difference entre ma position et celle du but
	diff.x = agy->pos_but.x - pos[0];
	diff.y = agy->pos_but.y - pos[1];
	diff.z = agy->pos_but.z - pos[2];

	//si la difference est inférieure à un certain seuil je répond oui
	if( diff.x < SEUIL_BUT && diff.x > -SEUIL_BUT &&
	    diff.y < SEUIL_BUT && diff.y > -SEUIL_BUT &&
	    diff.z < SEUIL_BUT && diff.z > -SEUIL_BUT ) {
		return(1);
	} else {
		//sinon je dis non
		return(0);
	}
}

/****************************************************************************/
//fonction qui me dit si il y a un agent de plus à la chaine ou si celui sur lequel je suis est le dernier
//renvoi cet agent ou NULL
Agent *suivant_de_la_chaine( Agent *agy ) {
	dBodyID suivant;
	dBodyID autre;
	Agent *a;

	//je regarde quel agent est relié par le joint mandy
        suivant = dJointGetBody(agy->mandy, 1);
	autre = dJointGetBody(agy->mandy, 0);

	if( suivant == 0 && autre == 0 ){
		return(NULL);
	} else {
		a = cherche_agent( suivant );
        	return(a);
	}
}

/****************************************************************************/
//fonction qui me dit si il y a déjà un agent au sommet de l'agent
int deja_qqun_dessus( Agent *agy ) {
	return( agy->au_dessus );
}

/****************************************************************************/
//fonction qui me fait parcourir la chaine des agents
void parcours_chaine( Agent *passif, Agent *actif) {
        Agent *suivant;

	suivant = suivant_de_la_chaine( passif );

	if( suivant == NULL ) {
		//accroche_en fin( passif );
		actif->au_dessus = 0;
	} else {
		grimpe_suivant( suivant, actif );
		actif->au_dessus = 0;
	}

}


/****************************************************************************/
//fonction qui deplace à la barbare un agent
//on met l'agent 1 au dessus de l'agent 2.
void deplace_barbare( Agent *agent1 , Agent *agent2 ) {
        dReal *pos_agent2;

	//on trouve la position de l'agent 2
	pos_agent2 = (dReal*) dBodyGetPosition( agent2->bodyID );

	//on deplace l'agent 1 au dessus de l'agent 2
	dBodySetPosition(agent1->bodyID, pos_agent2[0], pos_agent2[1], pos_agent2[2]+ (2.0*RAYON) + HAUSSE_AGENT );

}

/****************************************************************************/
//fonction qui dis quel agent est le plus proche de l'attractant
Agent *plus_proche( Agent *agy1 , Agent *agy2 ) {
        dReal *p1, *p2, *pa;
	dReal d1,d2;

	//on trouve la position de l'agent 1
	p1 = (dReal*) dBodyGetPosition( agy1->bodyID );

	//on trouve la position de l'agent 2
	p2 = (dReal*) dBodyGetPosition( agy2->bodyID );

	//on trouve la position de l'attractant
	pa = (dReal*) dBodyGetPosition( attract.bodyID );

	//distance entre agy1 et attract
	d1 = sqrt( ( pa[0] - p1[0] )*( pa[0] - p1[0] ) + ( pa[1] - p1[1] )*( pa[1] - p1[1] ) );

	//distance entre agy2 et attract
	d2 = sqrt( ( pa[0] - p2[0] )*( pa[0] - p2[0] ) + ( pa[1] - p2[1] )*( pa[1] - p2[1] ) );

        //comparaison
	if( d1 < d2 ) { return( agy1 ); } else { 	return( agy2 ); }
}


/****************************************************************************/
//fonction qui regarde les places de libres les plus proches de moi et qui m'assigne l'une d'elle en but
int je_m_accroche_ou( Agent *cible, Agent *actif ) {
        Vecteur pos_plus_proche;
	Vecteur pos_courante;
	Vecteur ma_pos;
	dReal d_min, d_courante;
	dReal *pos_cible,*pos_actif;
	int lequel = 0; //0 : aucun ; 1 derriere ; 2 droite ; 3 devant ; 4 gauche

        pos_cible = (dReal*) dBodyGetPosition( cible->bodyID );
	pos_actif = (dReal*) dBodyGetPosition( actif->bodyID );

	//initialisation à tres loin
	d_min = 10000;
	pos_plus_proche.x = 1000;
	pos_plus_proche.y = 1000;
	pos_plus_proche.z = 1000;

	//ma position
	ma_pos.x = pos_actif[0];
	ma_pos.y = pos_actif[1];
	ma_pos.z = pos_actif[2];

	//je commence par l'arriere
	if( cible->au_derriere == 0 ) {
                //la position à l'arrière est :
		pos_courante.x = pos_cible[0] + ( 2 * RAYON );
		pos_courante.y = pos_cible[1];
		pos_courante.z = pos_cible[2];

		//je calcule la distance entre moi et cette position
		d_courante = distance2D( &ma_pos , &pos_courante );

		//comparaison
		if( d_courante < d_min ) {
			//si c'est mieux alors on la prend comme nouvelle d_min
			d_min = d_courante;

			pos_plus_proche.x = pos_courante.x;
			pos_plus_proche.y = pos_courante.y;
			pos_plus_proche.z = pos_courante.z;

			lequel = 1;
		}
	}

	//puis la droite
	if( cible->au_droite == 0 ) {
                //la position à l'arrière est :
		pos_courante.x = pos_cible[0];
		pos_courante.y = pos_cible[1]  + ( 2 * RAYON );
		pos_courante.z = pos_cible[2];

		//je calcule la distance entre moi et cette position
		d_courante = distance2D( &ma_pos , &pos_courante );

		//comparaison
		if( d_courante < d_min ) {
			//si c'est mieux alors on la prend comme nouvelle d_min
			d_min = d_courante;

			pos_plus_proche.x = pos_courante.x;
			pos_plus_proche.y = pos_courante.y;
			pos_plus_proche.z = pos_courante.z;

			lequel = 2;
		}
	}

	if( cible->au_devant == 0 ) {
                //la position à l'arrière est :
		pos_courante.x = pos_cible[0] - ( 2 * RAYON );
		pos_courante.y = pos_cible[1];
		pos_courante.z = pos_cible[2];

		//je calcule la distance entre moi et cette position
		d_courante = distance2D( &ma_pos , &pos_courante );

		//comparaison
		if( d_courante < d_min ) {
			//si c'est mieux alors on la prend comme nouvelle d_min
			d_min = d_courante;

			pos_plus_proche.x = pos_courante.x;
			pos_plus_proche.y = pos_courante.y;
			pos_plus_proche.z = pos_courante.z;

			lequel = 3;
		}
	}

	if( cible->au_gauche == 0 ) {
                //la position à l'arrière est :
		pos_courante.x = pos_cible[0];
		pos_courante.y = pos_cible[1] - ( 2 * RAYON );
		pos_courante.z = pos_cible[2];

		//je calcule la distance entre moi et cette position
		d_courante = distance2D( &ma_pos , &pos_courante );

		//comparaison
		if( d_courante < d_min ) {
			//si c'est mieux alors on la prend comme nouvelle d_min
			d_min = d_courante;

			pos_plus_proche.x = pos_courante.x;
			pos_plus_proche.y = pos_courante.y;
			pos_plus_proche.z = pos_courante.z;

			lequel = 4;
		}
	}

	//ouf !!! on sait quelle est la position libre la plus proche
	//on fait un switch dessus :
	switch( lequel ) {
        case 1 :
		nouveau_but( actif, pos_plus_proche, cible->bodyID );
		cible->au_derriere = 1;
		actif->au_devant = 1;
		break;
	case 2 :
		nouveau_but( actif, pos_plus_proche, cible->bodyID );
		cible->au_droite = 1;
		actif->au_gauche = 1;
		break;
	case 3 :
		nouveau_but( actif, pos_plus_proche, cible->bodyID );
		cible->au_devant = 1;
		actif->au_derriere = 1;
		break;
	case 4 :
		nouveau_but( actif, pos_plus_proche, cible->bodyID );
		cible->au_gauche = 1;
		actif->au_droite = 1;
		break;
	}

	return( lequel );
}

/****************************************************************************/
//fonction qui regarde les places de libres avec une hiérarchie sans chercher la plus proche
int je_m_accroche_derriere( Agent *cible, Agent *actif ) {
        Vecteur pos_plus_proche;
	Vecteur pos_courante;
	Vecteur ma_pos;
	dReal d_min, d_courante;
	dReal pos_cible[3],*pos_actif;
	int lequel = 0; //0 : aucun ; 1 derriere ; 2 droite ; 3 devant ; 4 gauche

        //pos_cible = (dReal*) dBodyGetPosition( cible->bodyID );
	pos_cible[0] = cible->pos_but.x;
	pos_cible[1] = cible->pos_but.y;
	pos_cible[2] = cible->pos_but.z;

	pos_actif = (dReal*) dBodyGetPosition( actif->bodyID );

	//initialisation à tres loin
	d_min = 10000;
	pos_plus_proche.x = 1000;
	pos_plus_proche.y = 1000;
	pos_plus_proche.z = 1000;

	//ma position
	ma_pos.x = pos_actif[0];
	ma_pos.y = pos_actif[1];
	ma_pos.z = pos_actif[2];

	//je commence par l'arriere
	if( cible->au_derriere == 0 ) {
		pos_courante.x = pos_cible[0] + ( 2 * RAYON );
		pos_courante.y = pos_cible[1];
		pos_courante.z = pos_cible[2];

		pos_plus_proche.x = pos_courante.x;
		pos_plus_proche.y = pos_courante.y;
		pos_plus_proche.z = pos_courante.z;

		lequel = 1;
	} else {

		if( cible->au_droite == 0 ) {
			//la position à l'arrière est :
			pos_courante.x = pos_cible[0];
			pos_courante.y = pos_cible[1]  + ( 2 * RAYON );
			pos_courante.z = pos_cible[2];

			//je calcule la distance entre moi et cette position
			d_courante = distance2D( &ma_pos , &pos_courante );

			//comparaison
			if( d_courante < d_min ) {
				//si c'est mieux alors on la prend comme nouvelle d_min
				d_min = d_courante;

				pos_plus_proche.x = pos_courante.x;
				pos_plus_proche.y = pos_courante.y;
				pos_plus_proche.z = pos_courante.z;

				lequel = 2;
			}
		}

		if( cible->au_devant == 0 ) {
			//la position à l'arrière est :
			pos_courante.x = pos_cible[0] - ( 2 * RAYON );
			pos_courante.y = pos_cible[1];
			pos_courante.z = pos_cible[2];

			//je calcule la distance entre moi et cette position
			d_courante = distance2D( &ma_pos , &pos_courante );

			//comparaison
			if( d_courante < d_min ) {
				//si c'est mieux alors on la prend comme nouvelle d_min
				d_min = d_courante;

				pos_plus_proche.x = pos_courante.x;
				pos_plus_proche.y = pos_courante.y;
				pos_plus_proche.z = pos_courante.z;

				lequel = 3;
			}
		}

		if( cible->au_gauche == 0 ) {
			//la position à l'arrière est :
			pos_courante.x = pos_cible[0];
			pos_courante.y = pos_cible[1] - ( 2 * RAYON );
			pos_courante.z = pos_cible[2];

			//je calcule la distance entre moi et cette position
			d_courante = distance2D( &ma_pos , &pos_courante );

			//comparaison
			if( d_courante < d_min ) {
				//si c'est mieux alors on la prend comme nouvelle d_min
				d_min = d_courante;

				pos_plus_proche.x = pos_courante.x;
				pos_plus_proche.y = pos_courante.y;
				pos_plus_proche.z = pos_courante.z;

				lequel = 4;
			}
		}
	}

	//ouf !!! on sait quelle est la position libre la plus proche
	//on fait un switch dessus :
	switch( lequel ) {
        case 1 :
		nouveau_but( actif, pos_plus_proche, cible->bodyID );
		cible->au_derriere = 1;
		actif->au_devant = 1;
		break;
	case 2 :
		nouveau_but( actif, pos_plus_proche, cible->bodyID );
		cible->au_droite = 1;
		actif->au_gauche = 1;
		break;
	case 3 :
		nouveau_but( actif, pos_plus_proche, cible->bodyID );
		cible->au_devant = 1;
		actif->au_derriere = 1;
		break;
	case 4 :
		nouveau_but( actif, pos_plus_proche, cible->bodyID );
		cible->au_gauche = 1;
		actif->au_droite = 1;
		break;
	}

	return( lequel );
}
/****************************************************************************/
//fonction qui regarde si des coordonnées sont à peu près égales
int environ_egales( dReal x1 , dReal y1 , dReal z1 , dReal x2 , dReal y2 , dReal z2 ){

	if( abs( x1 - x2 ) < SEUIL_BUT && abs( y1 - y2 ) < SEUIL_BUT && abs( z1 - z2 ) < SEUIL_BUT ) {
        	return 1;
	} else {
		return 0;
	}

}

/****************************************************************************/
//fonction qui regarde si un agent est au bonnes coordonnées
int a_sa_place( Agent *moi, Agent *lui, int ou ) {
	dReal *pos_moi, *pos_lui;
	dReal diff_x, diff_y, diff_z;

        pos_moi = (dReal*) dBodyGetPosition( moi->bodyID );
	pos_lui = (dReal*) dBodyGetPosition( lui->bodyID );

	diff_x = 0;
	diff_y = 0;
	diff_z = 0;

	//je regarde si l'agent est immobile là ou il devrait etre
	switch( ou ) {
		case 1 : diff_x = -(2*RAYON); break;
		case 2 : diff_y = -(2*RAYON); break;
		case 3 : diff_x = (2*RAYON); break;
		case 4 : diff_y = (2*RAYON); break;
	}

	if( environ_egales( pos_moi[0]  + diff_x , pos_moi[1] + diff_y , pos_moi[2] + diff_z , pos_lui[0] , pos_lui[1] , pos_lui[2] ) ){
		return 1;
	} else {
		return 0;
	}

}

/****************************************************************************/
//fonction qui regarde si un agent est déja accroché à moi.
int deja_accro_a_moi( Agent *moi, Agent *lui ) {
	dBodyID mon_ID, son_ID;

	if( moi->au_devant == 1 && lui->au_derriere == 1 ) {
		mon_ID = dJointGetBody ( moi->avant, 0);
		son_ID = dJointGetBody ( moi->avant, 1);
		if( ( mon_ID == moi->bodyID && son_ID == lui->bodyID ) || ( son_ID == moi->bodyID && mon_ID == lui->bodyID ) ) {
			return(1);
		}

	}

	if( moi->au_derriere == 1 && lui->au_devant == 1 ) {
                mon_ID = dJointGetBody ( moi->arriere, 0);
		son_ID = dJointGetBody ( moi->arriere, 1);
		if( ( mon_ID == moi->bodyID && son_ID == lui->bodyID ) || ( son_ID == moi->bodyID && mon_ID == lui->bodyID ) ) {
			return(1);
		}
	}

	if( moi->au_gauche == 1 && lui->au_droite == 1 ) {
		mon_ID = dJointGetBody ( moi->gauche, 0);
		son_ID = dJointGetBody ( moi->gauche, 1);
		if( ( mon_ID == moi->bodyID && son_ID == lui->bodyID ) || ( son_ID == moi->bodyID && mon_ID == lui->bodyID ) ) {
			return(1);
		}
	}

	if( moi->au_droite == 1 && lui->au_gauche == 1 ) {
		mon_ID = dJointGetBody ( moi->droite, 0);
		son_ID = dJointGetBody ( moi->droite, 1);
		if( ( mon_ID == moi->bodyID && son_ID == lui->bodyID ) || ( son_ID == moi->bodyID && mon_ID == lui->bodyID ) ) {
			return(1);
		}
	}
	return(0);
}


/****************************************************************************/
//fonction qui me dit si je suis à la bonne position dérriere l'agent que je suis.
int derriere_agent_ok( dReal *pos_moi, dReal *pos_lui ) {
	//dReal *pos;
	Vecteur diff;

	//je fait la difference entre ma position et celle du but
	diff.x = (pos_lui[0]+ 2*RAYON + DIST_SECUR) - pos_moi[0];
	diff.y = pos_lui[1] - pos_moi[1];
	diff.z = pos_lui[2] - pos_moi[2];

	//si la difference est inférieure à un certain seuil je répond oui
	if( diff.x < SEUIL_BUT && diff.x > -SEUIL_BUT &&
	    diff.y < SEUIL_BUT && diff.y > -SEUIL_BUT &&
	    diff.z < SEUIL_BUT && diff.z > -SEUIL_BUT ) {
		return(1);
	} else {
		//sinon je dis non
		return(0);
	}
}


