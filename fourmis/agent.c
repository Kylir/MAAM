/* Fichier de gestion du comportement de l'agent
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
	Agent *a,*b;
	dBodyID bobo;

        //on regarde l'etat de l'agent
	switch( agy->etat ) {

	//etat de début de simulation
        case -1 :
		//Description : au moment ou je rencontre le sol je passe à l'état 0. Avant je ne fais rien.

		//je cherche dans la liste des collisions une avec le bloc 1 (type 1)
		bobo = cherche_collision_type( agy, 1 );

                //HACKING !!!
		//bobo = -1;

		//si elle s'y trouve alors je passe à l'etat 0
		if( bobo != (dBodyID) -1 ) {
			agy->etat = 0;
                 	agy->couleur = 2;//1;
			agy->cycle = 0;
			//on assigne un nouveau but à l'agent
			pos_att = (dReal*) dBodyGetPosition( attract.bodyID );
			agy->pos_but.x = pos_att[0]; agy->pos_but.y = pos_att[1]; agy->pos_but.z = 0;//pos_att[2] - RAYON_ATTRACTANT;

			agy->bodyID_but = attract.bodyID;
		}
		break;

        case 0 :
		//Description : dans cet etat l'agent se dirige vers l'attractant en avancant sur le sol
		// Quand il rencontre un autre agent dans l'etat 1 alors il grimpe dessus et passe à l'etat 2
                // Si par contre l'agent est au bord du fosse alors il s'arrete et passe à l'etat 1

		//on regarde si l'agent est dans une bande gluante
		glu = dans_bande_gluante1(agy);


		//si l'agent est dedans on l'accroche et on passe à l'état 1
		if( glu == 1) {
			//on passe a l'etat stoppé.
			agy->etat = 1;
			agy->couleur = 2;
			agy->cycle = 0;

			//le bloc 1
			bobo = cherche_collision_type( agy, 1 );

			if( bobo != (dBodyID) -1 ) {
				//la position de l'agent
				pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );

				//puis on s'accroche au bloc
				dJointAttach( agy->patte, agy->bodyID, bobo );
				dJointSetUniversalAnchor( agy->patte, pos_agent[0], pos_agent[1], pos_agent[2]-(RAYON/2) );
				dJointSetUniversalAxis1( agy->patte, 0.0, 1.0, 0.0);
				dJointSetUniversalAxis2( agy->patte, 1.0, 0.0, 0.0);
			}

                } else {
			//sinon on regarde si l'agent cogne un autre dans l'état 1
			a = cherche_collision_agent_etat( agy, 1 );

			//Si personne alors je recherche les agents dans l'etat 5
			if( a == NULL ) {
                        	a = cherche_collision_agent_etat( agy, 5 );
			}

                        //HACKING : on s'assure que a == null !!! C'est pour empecher le passage à l'etat 2
			//a = NULL;

			//si c'est le cas on va lui grimper dessus mais il faut etre sur qu'il n'y a personne sur lui
			if( a != NULL && a->au_dessus == 0 ) {
				//on passe à l'état 2
				agy->etat = 2;
				//on change de couleur
				agy->couleur = 3;

                                agy->cycle = 0;

				//on grimpe dessus
				grimpe_dessus( a, agy );

			//sinon, on ne rencontre pas d'agent à l'état 1 alors on poursuit notre route
			} else {
                        	pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
				regle_vitesse( agy, pos_agent );
			}
		}
		break;

	case 1 :
		//Description : L'agent ne fait rien dans cet état.
		break;

	case 2 :
		//Description : dans cet etat, l'agent parcours une chaine jusqu'au bout et s'accroche au dernier
		//tant qu'il n'a pas atteint son but il y va.
		//Si il y arrive alors on lui affecte un nouveau but (le suivant dans la chaine).
		//Ou alors il s'accroche si il n'y a pas de suivant.

		//Si l'agent atteint son but on trouve le suivant et on y va sinon on s'accroche
                if( au_but( agy ) ) {
			a = cherche_agent( agy->bodyID_but );
			b = (Agent *) suivant_de_la_chaine( a );

			//test pour voir si il n'y a pas de sivant
			if( b == NULL ) {
                                if( a != NULL && a->au_devant == 0 ) {
					//on repere la position du dernier de la chaine
					pos_agent = (dReal*) dBodyGetPosition( a->bodyID );

					//on assigne le but
					agy->pos_but.x = pos_agent[0]-(2.0*RAYON );//- 0.001);
					agy->pos_but.y = pos_agent[1];
					agy->pos_but.z = pos_agent[2];

					//passage à l'état 3
					agy->etat = 3;

					//on change de couleur
					agy->couleur = 4;

					agy->cycle = 0;

					//et on libère la place du dessus
					a->au_dessus = 0;

					//on signale qu'on s'accroche
					a->au_devant = 1;
				}
			//ici on traite le cas ou un suivant existe
			} else {

                                if( b->au_dessus == 0 ) {
					//printf("J'ai un suivant ! Je m'y dirige !\n");

					//on repere la position du suivant de la chaine
					pos_agent = (dReal*) dBodyGetPosition( b->bodyID );

					//on assigne le but
					agy->pos_but.x = pos_agent[0];
					agy->pos_but.y = pos_agent[1];
					agy->pos_but.z = pos_agent[2]+(2.0*RAYON );//- 0.001);

					agy->bodyID_but = b->bodyID;

					//et on libère la place du dessus
					if( a != NULL ) {
						a->au_dessus = 0;
					}

					//mais on prend celle de b !
					b->au_dessus = 1;
				}


			}

		} else {
			pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
			regle_vitesse( agy, pos_agent );
		}

		break;


	case 3 :
		//Description : dans cet etat, l'agent se place à la fin de la chaine et s'accroche (ou se fait accrocher ???).
		//Tant qu'il n'est pas à son but il s'y dirige - Au moment où il atteint l'objectif il se fait accrocher par l'agent

		//Si l'agent est au but
		if( au_but( agy ) ) {
                        //on s'assure d'une collision avec un agent
			a = cherche_agent( agy->bodyID_but );

			if( a != NULL ) {
                        	//ma position
                        	pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );

				//la sienne
                                pos_agent2 = (dReal*) dBodyGetPosition( a->bodyID );

				//on se fait accrocher par le joint mandy de l'agent qui nous precede
                         	dJointAttach( a->mandy, a->bodyID, agy->bodyID );

                                //ma vitesse mise à zéro pour éviter les bugs...;)
				dBodySetLinearVel( agy->bodyID, 0, 0, 0);

				//reglage des paramètres
                                dJointSetHingeAnchor( a->mandy, pos_agent[0],  pos_agent[1], pos_agent[2] );
				/*dJointSetHingeAnchor( a->mandy, ( pos_agent[0] + pos_agent2[0] ) / 2.0,
										   ( pos_agent[1] + pos_agent2[1] ) / 2.0,
										   ( pos_agent[2] + pos_agent2[2] ) / 2.0 );*/

				dJointSetHingeAxis( a->mandy, 1, 0, 0);

				//ce joint mandy est aussi mon joint arriere
				agy->arriere = a->mandy;

				//On passe à l'état 4
				agy->etat = 4;

				//on change de couleur
				agy->couleur = 5;

				agy->cycle = 0;
			}

		//Si on est pas encore au but on avance vers lui
		} else {
			pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
			regle_vitesse( agy, pos_agent );
		}
		break;


	case 4 :
		//Description : dans cet etat, l'agent est dans la chaine et il regarde s'il touche le bloc 2
		//Si c'est le cas il s'aggripe et passe à l'état 5
		bobo = cherche_collision_type( agy, 2 ) ;

		if( bobo != (dBodyID) -1 ) {
			//printf("Je touche le bloc 2 !!!!\n");

			//la position de l'agent
			pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );

			//puis on s'accroche au bloc
			dJointAttach( agy->patte, agy->bodyID, bobo );
			dJointSetUniversalAnchor( agy->patte, pos_agent[0], pos_agent[1], pos_agent[2]-(RAYON/2) );
			dJointSetUniversalAxis1( agy->patte, 0.0, 1.0, 0.0);
			dJointSetUniversalAxis2( agy->patte, 1.0, 0.0, 0.0);

			agy->etat = 5;
			agy->couleur = 6;
			agy->cycle = 0;
		}
		break;

	case 5 :
		//Description : dans cette état je touche le bloc 2... Donc je propage l'info a celui qui me tient.
		//Si personne ne me tient alors c'est que je suis celui qui dois commencer la resorbption

		//Je regarde si je suis accroché par quelqu'un
                //je regarde quel agent est relié par le joint mandy
        	bobo = dJointGetBody(agy->arriere, 0);

		//si il y a quelqu'un de connecté
		if( bobo != (dBodyID) 0 ){

			//a quel agent appartient ce bodyID
			a = cherche_agent( bobo );

                        //si je trouve un agent alors il
			if( a != NULL && a->etat != 5 ) {
				a->etat = 5;
				a->couleur = 6;
				a->cycle = 0;
			}
		} else {
		//Si il n'y a personne qui s'attache à moi

			//Y a-t-il encore des gens qui veulent passer
			a = cherche_collision_agent_etat( agy, 0 );

			//si personne
			if( a == NULL) {

                        	//personne qui me grimpe dessus ?
                        	if( agy->au_dessus == 0 ){

					//Je me détache du sol
					dJointAttach( agy->patte, 0, 0 );

                                        //quel objet tiens-je
					bobo = dJointGetBody(agy->mandy, 1);

					//quel agent est-ce
					a = cherche_agent( bobo );

					//Si personne sur celui que je tiens
					if( a != NULL && a->au_dessus == 0 ) {

						//Je me décroche
						dJointAttach( agy->mandy, 0, 0);

						//Je lui grimpe dessus
						grimpe_dessus( a, agy );

						//Je passe en état de parcours de chaine
						agy->etat = 2;
						agy->couleur = 3;
						agy->cycle = 0;
					}
				}
			}
		}
		break;

	/*case X :
		//Description : dans cet etat, l'agent
		break;*/


	}

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













