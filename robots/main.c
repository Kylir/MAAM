//fichier main

#include "boule.h"
int main (int argc, char **argv){
	//pour test
	//Vecteur v;

	//Pour utiliser Drawstuff
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.command = &command;
	fn.step = &simLoop;
	fn.stop = 0;
	fn.path_to_textures = "/home/forest/textures";

	//Création du monde
	world = dWorldCreate();
	space = dSimpleSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	accrogroup = dJointGroupCreate (0);
	dWorldSetGravity (world,GRAVITE);

	//le sol :
	ground = dCreatePlane (space,0,0,1,0);

	//les deux "ilots" séparés par le fossé
	init_env( &bloc_depart, &bloc_arrive, world, space);

	//on met en place un attractant
	init_attractant( &attract, world, space);

	if( EN_LIGNE ) {
		//init en ligne des agents
		init_ligne_agents( agents , NB_AGENTS, world, space );
	} else {
		//init au hasard des agents
		init_groupe_agents( agents , NB_AGENTS, world, space );
	}

	//Mise en route de la simulation
	dsSimulationLoop (argc,argv,640,480,&fn);

	//destruction en mémoire des objets
	dJointGroupDestroy (accrogroup);
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);

	return 0;
}
