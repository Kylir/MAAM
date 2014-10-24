#include "boule.h"

//Fonction de dessin d'un atome dans la boucle de simulation avec Drawstuff
void dessinAgent( Agent *agy ){
	dVector3 sides;

	switch ( agy->couleur ) {
  		case 0:dsSetColor (1,0,0);break;
		case 1:dsSetColor (0,1,0);break;
		case 2:dsSetColor (0,0,1);break;
		case 3:dsSetColor (1,1,0);break;
		case 4:dsSetColor (1,0,1);break;
		case 5:dsSetColor (0,1,1);break;
		case 6:dsSetColor (1,1,1);break;
		case 7:dsSetColor (0,0,0);break;
	}
	//dessin de la sphere
	dsSetTexture (DS_WOOD);

	if( FORME_AGENT == SPHERE ) {
		dsDrawSphere (dBodyGetPosition(agy->bodyID), dBodyGetRotation(agy->bodyID), RAYON);
	}
	if( FORME_AGENT == BOX ) {
    		dGeomBoxGetLengths (agy->geomID, sides);
    		dsDrawBox (dBodyGetPosition(agy->bodyID), dBodyGetRotation(agy->bodyID), sides);
	}

}
