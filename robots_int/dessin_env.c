/* Pour dessiner les deux blocs d'environnement */

#include "boule.h"
//Fonction de dessin avec drawstuff
void dessin_env( Bloc *bloc1, Bloc *bloc2 ){
	dReal sides[3] = {LARGEUR, LONGUEUR, EPAISSEUR };
	dsSetColor (COULEUR_BLOC);
	dsSetTexture (DS_WOOD);

	dsDrawBox ( dBodyGetPosition( bloc1->bodyID ) , dBodyGetRotation( bloc1->bodyID ), sides);
	dsDrawBox ( dBodyGetPosition( bloc2->bodyID ) , dBodyGetRotation( bloc2->bodyID ), sides);
}
