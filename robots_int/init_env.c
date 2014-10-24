/* Initialisation des deux blocs séparés par un fossé de taile FOSSE */

#include "boule.h"

Bloc *init_bloc( Bloc *bob, dReal x, dReal y, dReal z, dWorldID monde, dSpaceID espace ){


	//si le pointeur sur bloc est null on crée dynamiquement le bloc
	if (bob == NULL) {
		bob = (Bloc *) malloc( sizeof( Bloc ) );
	}

	//initialisation du bloc
	bob->bodyID = dBodyCreate (monde);
	dBodySetPosition ( bob->bodyID, x, y, z);
	dMassSetBox ( &(bob->masse) , 1, LARGEUR, LONGUEUR, EPAISSEUR);
	dMassAdjust (&(bob->masse), MASSE_BLOC);
	dBodySetMass ( bob->bodyID, &(bob->masse) );
	(bob->geomID) = dCreateBox ( espace, LARGEUR, LONGUEUR, EPAISSEUR);
	dGeomSetBody ( bob->geomID ,bob->bodyID );
	return( bob );
}

void init_env( Bloc *bloc_depart,Bloc *bloc_arrive, dWorldID monde, dSpaceID espace ){
	dReal x1,x2,y1,y2,z1,z2;
	dJointID acc1,acc2,acc3,acc4,acc5,acc6;

	x1 = (FOSSE / 2) + (LARGEUR / 2);
	y1 = 0.0;
	z1 = (EPAISSEUR / 2)+HAUSSE;

	x2 = -x1;
	y2 = 0.0;
	z2 = z1;
	//le bloc 1 bien placé :
	bloc_depart = init_bloc( bloc_depart, x1, y1, z1, monde, espace );

	//le bloc 2 bien placé :
	bloc_arrive = init_bloc( bloc_arrive, x2, y2, z2, monde, espace );

	//on accroche les blocs:
	acc1 = dJointCreateHinge (world,accrogroup);
	acc2 = dJointCreateHinge (world,accrogroup);
	acc3 = dJointCreateHinge (world,accrogroup);
	acc4 = dJointCreateHinge (world,accrogroup);
	acc5 = dJointCreateHinge (world,accrogroup);
	acc6 = dJointCreateHinge (world,accrogroup);

	dJointAttach (acc1, bloc_depart->bodyID, 0);
	dJointAttach (acc2, bloc_depart->bodyID, 0);
	dJointAttach (acc3, bloc_depart->bodyID, 0);
	dJointAttach (acc4, bloc_arrive->bodyID, 0);
	dJointAttach (acc5, bloc_arrive->bodyID, 0);
	dJointAttach (acc6, bloc_arrive->bodyID, 0);

	dJointSetHingeAnchor (acc1, 0.0, 0.0, 0.0);
	dJointSetHingeAxis (acc1, 1.0, 0.0, 0.0);

	dJointSetHingeAnchor (acc2, 0.0, 0.0, 0.0);
	dJointSetHingeAxis (acc2, 0.0, 1.0, 0.0);

	dJointSetHingeAnchor (acc3, 0.0, 0.0, 0.0);
	dJointSetHingeAxis (acc3, 0.0, 0.0, 1.0);

	dJointSetHingeAnchor (acc4, 0.0, 0.0, 0.0);
	dJointSetHingeAxis (acc4, 1.0, 0.0, 0.0);

	dJointSetHingeAnchor (acc5, 0.0, 0.0, 0.0);
	dJointSetHingeAxis (acc5, 0.0, 1.0, 0.0);

	dJointSetHingeAnchor (acc6, 0.0, 0.0, 0.0);
	dJointSetHingeAxis (acc6, 0.0, 0.0, 1.0);





}
