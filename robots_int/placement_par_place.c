//on regarde si l'agent est dans une bande gluante
		glu = dans_bande_gluante1(agy);

		//ou si il touche un autre dans l'état de reconfiguration
		a = cherche_collision_agent_etat( agy, 2 );
		//a = NULL;

		//si l'agent est dedans on l'accroche et on passe à l'état 1
		if( glu == 1 || a != NULL ) {
			//on passe a l'etat de reconfiguration.
			agy->etat = 2;
			agy->couleur = 2;
			agy->cycle = 0;

                        //Je me dirige vers ma place = mon nouveau but c'est ma place
			printf("Je me dirige vers la place %d.\n", ordre_arrivee);
			a_ta_place( agy, ordre_arrivee++ );

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
                        	pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
				regle_vitesse( agy, pos_agent );
		}*/
                break;

	case 2 :
		//Description : Etat de reconfiguration.


                if( ordre_arrivee >= 10 ) {
                        //je me dettache
                        dJointAttach( agy->patte, 0, 0 );
                        //je me deplace le long du fossé
			pos_agent = (dReal*) dBodyGetPosition( agy->bodyID );
			reglage_fin_vitesse( agy, pos_agent, 5, 5, 1 );
		}