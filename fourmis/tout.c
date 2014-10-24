//gcc tout.c -o tout -I/home/forest/include -L/home/forest/lib -ldrawstuff -lode -lm -lstdc++ -lGL -lGLU

/**************************************************************************************/
//boule.h

//les includes
#include <stdio.h>
#include "ode/ode.h"
#include "drawstuff/drawstuff.h"

/**********************************************************************************/
//Au cas où l'option double soit activée
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

/**********************************************************************************/
//Les defines
//pour un agent
#define NB_AGENTS 1	//le nombre de boules dans ma simulation
#define RAYON 0.1	//le rayon d'une boule
#define MASSE 0.1	//la masse d'une boule

//pour l'environnement
#define LONGUEUR 5.0
#define LARGEUR 2.5
#define EPAISSEUR 3.0
#define FOSSE 1.0
#define MASSE_BLOC 40.0
#define COULEUR_BLOC 1,1,1

//le pas de simulation
#define PAS_SIMU 0.05

//un decalage vers le haut pour le début de la simulation
#define HAUSSE 0.01

/**********************************************************************************/
//l'element de la liste des agents
typedef struct listeAgents{
	struct Agent *agent;		//l'agent avec qui je suis lié
	dJointID id;			//l'ID du joint qui nous lie
	struct listeAgents *p_suivant;  //le pointeur sur l'element suivant
} listeAgents;

/**********************************************************************************/
//voici la structure pour gerer un agent boule
typedef struct Agent{
	dBodyID bodyID;			//le num d'ident d'un agent
	dGeomID geomID;			//le num de geometrie de l'agent
	int couleur;			//la couleur de l'agent
	long identification;		//son numero d'ident unique
	struct listeAgents *p_tenus;	//la liste chainée de ses joints d'accrochages
	int nb_tenus;			//a combien je m'accroche
	struct listeAgents *p_teneurs;	//la liste de ceux qui me tiennent
	int nb_teneurs;			//combien me tiennent
} Agent;

/**********************************************************************************/
//structure pour les blocs du terrain
typedef struct Bloc{
	dBodyID bodyID;			//le num d'ident d'un agent
	dGeomID geomID;			//le num de geometrie de l'agent
	dMass masse;			//la masse du bloc
	int couleur;			//la couleur de l'agent
	long identification;		//son numero d'ident unique
} Bloc;

/**********************************************************************************/
//Les objets
static dWorldID world;				//le monde
static dSpaceID space;				//l'espace physique
static Bloc bloc_depart,bloc_arrive;		//les deux blocs formants les "rives"
static dJointGroupID contactgroup;		//pour gerer les contacts
static dJointGroupID accrogroup;		//la liste des accrochages
static unsigned long pas;			//le numero d'iteration de la simu
static struct Agent agents[NB_AGENTS];		//la liste des agents.
static dGeomID ground;				//le sol

/**********************************************************************************/
//init_env.c
//void init_bloc( dBodyID *body, dGeomID *geom, dReal x, dReal y, dReal z, dWorldID *monde, dSpaceID *espace ){
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
	//dSpaceAdd ( espace, bob->geomID );
	return( bob );
}

void init_env( Bloc *bloc_depart,Bloc *bloc_arrive, dWorldID monde, dSpaceID espace ){
	dReal x1,x2,y1,y2,z1,z2;

	x1 = (FOSSE / 2) + (LARGEUR / 2);
	y1 = 0.0;
	z1 = (EPAISSEUR / 2)+HAUSSE;

	x2 = -x1;
	y2 = 0.0;
	z2 = z1;
	printf("creation.\n");
	//le bloc 1 bien placé :
	bloc_depart = init_bloc( bloc_depart, x1, y1, z1, monde, espace );

	//le bloc 2 bien placé :
	bloc_arrive = init_bloc( bloc_arrive, x2, y2, z2, monde, espace );
}

/***********************************************************************************************/
//dessin_env.c

void dessin_env( Bloc *bloc1, Bloc *bloc2 ){
	dReal sides[3] = {LARGEUR, LONGUEUR, EPAISSEUR };
	dsSetColor (COULEUR_BLOC);
	dsSetTexture (DS_WOOD);

	dsDrawBox ( dBodyGetPosition( bloc1->bodyID ) , dBodyGetRotation( bloc1->bodyID ), sides);
	dsDrawBox ( dBodyGetPosition( bloc2->bodyID ) , dBodyGetRotation( bloc2->bodyID ), sides);
}

/***********************************************************************************************/
//simulation.c

/**********************************************************************************/
//Fonction de demarrage de la simulation
void start(){
	static float xyz[3] = {5.5244,-0.2056,4.3100};
	static float hpr[3] = {158.5000,-29.5000,0.0000};
	dsSetViewpoint (xyz,hpr);
	printf("Début de la simulation...\n");
	//verif:
	//printf("space = %i\n",space);
}


/**********************************************************************************/
//Fonction de collision
/*static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i,n;
  //printf("nearCallback !\n");

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}*/

static void nearCallback (void *data, dGeomID o1, dGeomID o2){
	dContact contact;
	dBodyID b1,b2;

	printf("nearCallback !\n");
	//on retrouve les corps en fonction des geometries

	b1 = dGeomGetBody(o1);
	b2 = dGeomGetBody(o2);

	//si les objets sont connéctés on fait rien
	if (dAreConnected (b1,b2)){
		return;
	}

	//reglage du type de surface
	contact.surface.mode = 0;
	contact.surface.mu = 0.5;//dInfinity;
	//contact.surface.mu2 = dInfinity;//0;

	//Si il y a collision
	if (dCollide (o1,o2,0,&contact.geom,sizeof(dContactGeom))) {
		dJointID c = dJointCreateContact (world,contactgroup,&contact);
		dJointAttach (c,b1,b2);
	}


}

/**********************************************************************************/
//Fonction de boucle de simulation
static void simLoop (int pause){
	int i;
	//dReal sides[3] = {LARGEUR, LONGUEUR, EPAISSEUR };

	if (!pause) {
		//collisions eventuelles
		dSpaceCollide (space, 0, &nearCallback );
		dWorldStep (world, PAS_SIMU );

		//nettoyage apres collision
		dJointGroupEmpty (contactgroup);
	}

	//dessin de mon objet de test :
	dsSetColor (COULEUR_BLOC);
	dsSetTexture (DS_WOOD);


	//pas de simulation pour les agents
	for( i=0 ; i<NB_AGENTS ; i++ ){
		//step( agents[i] );
	}

	//dessin de la scène
	dessin_env( &bloc_depart, &bloc_arrive );
	//for( i=0 ; i < NB_AGENTS ; i++ ){ dessinAgent( &(agents[i]) ); }


}



/**********************************************************************************/
//Fonction d'interaction clavier
static void command (int cmd){
  	switch (cmd) {
  	case 'X':
		break;
  	}
}

/**********************************************************************************/
//main.c
int main (int argc, char **argv){
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
	//printf("Etape 1 : Création du monde %i.\n",world);
	space = dSimpleSpaceCreate (0);
	//printf("Etape 2 : Création de l'espace %i.\n",space);

	contactgroup = dJointGroupCreate (0);
	accrogroup = dJointGroupCreate (0);
	dWorldSetGravity (world,0,0,-0.5);

	//le sol :
	ground = dCreatePlane (space,0,0,1,0);

	//les deux "ilots" séparés par le fossé
	init_env( &bloc_depart, &bloc_arrive, world, space);

	//init des agents

	//Mise en route de la simulation
	dsSimulationLoop (argc,argv,352,288,&fn);

	//destruction en mémoire des objets
	dJointGroupDestroy (accrogroup);
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);

	return 0;
}
