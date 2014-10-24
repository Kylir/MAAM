/* init de l'attractant */

#include "boule.h"

void init_attractant ( Attractant *aty, dWorldID world, dSpaceID space ) {
	float x,y,z;
	//on crée la structure si on nous passe un truc à NULL
	if(aty == NULL){ aty = (Attractant *) malloc( sizeof( Attractant ) ); }

	//on crée le corps
	aty->bodyID = dBodyCreate (world);

	//on lui donne une forme de masse et une masse
	dMassSetSphere ( &(aty->masse), 1, RAYON_ATTRACTANT);
	dMassAdjust (&(aty->masse), MASSE_ATTRACTANT);

	//on créer une forme et on la lie au body
	aty->geomID = dCreateSphere (space, RAYON_ATTRACTANT);
	dGeomSetBody (aty->geomID, aty->bodyID);

	//On place l'agent
	x = -( ( 0.5 * FOSSE ) + ( 0.9*LARGEUR) ) ;
	y = 0;
	z = (EPAISSEUR + HAUSSE + RAYON_ATTRACTANT + HAUSSE_AGENT );
	dBodySetPosition (aty->bodyID, x, y, z);
	printf("Attractant a la position ( %f , %f , %f ).\n",x,y,z);
}
