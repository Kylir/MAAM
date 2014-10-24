a = cherche_collision_agent_etat( agy, 2 );

		//si je rencontre un autre agent
		if( a != NULL ) {

                	//je m'accroche au sol

			//mon cycle = 0

			//je détermine qui est le plus proche

			//Si je suis le plus éloigné cet agent devient mon but

			//et je passe en mode de suivie

		//si mon cycle est supérieure à 10 je me dirige vers mon but




                //Si je rencontre un autre agent je me stoppe
		a = cherche_collision_agent_etat( agy, 2 );

		if( a!= NULL ) {
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

			agy->cycle = 0;
		}


		//au bout de 100 pas de temps je me détache et me déplace le long de la bande gluante
		if( agy->cycle > 5 ) {
			//je me dettache
                        dJointAttach( agy->patte, 0, 0 );
                        //je me deplace le long du fossé
			pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
			reglage_fin_vitesse( agy, pos_agent, 0, 1, 1 );
		}

		break;

	case 3 :
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


	case 4 :
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


	case 5 :
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

	case 6 :
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
