#include "boule.h"

//Fonction de dessin d'un atome dans la boucle de simulation avec Drawstuff
void dessin_attractant( Attractant *aty ){
	//dessin de la sphere
	dsSetColor (COULEUR_ATTRACTANT);
	dsSetTexture (DS_WOOD);
	dsDrawSphere (dBodyGetPosition(aty->bodyID), dBodyGetRotation(aty->bodyID), RAYON_ATTRACTANT);
}
