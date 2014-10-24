/* atome.c
	création : 05/05/2003
	auteur : Jason Forest
	Description : C'est ma première modélisation d'un atome.
	ligne de compilation : "gcc atome.c -o atome -I/home/forest/include -L/home/forest/lib -ldrawstuff -lode -lm -lstdc++ -lGL -lGLU"
*/


#include <stdio.h>
#include "ode/ode.h"
#include "drawstuff/drawstuff.h"

//Au cas où l'option double soit activée
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif


//Les defines
#define NB_AGENTS 1//4
#define RAYON 0.3
#define MASSE_C 0.1
#define MASSE_B 0.1

/**********************************************************************************/
//Un atome
typedef struct Atome{
 dJointID hinge1[6];		//ces 2 joints permettront les mouvements haut/bas et gauche/droite
	dJointID hinge2[6];
	dJointID motors1[6];		//idem pour pouvoir bouger de facon controlée.
	dJointID motors2[6];
	dJointID joints_accro[6];	//joints pour accrocher la boule à d'autres boules.
	int bool_accro[6]; 		//me permet de savoir quelles boules sont deja accrochées 0->pas accrochée, 1->accrochée.
	dBodyID atome[7];		//Les différentes parties de l'atome : 1cube et 6 boules.
	dGeomID cube;			//la forme de cube.
	dGeomID boules[6];		//les formes des boules.
	int couleurs[6];		//les couleurs des 6 boules : 0->rouge, 1->vert, 2->bleue.
	int identification;		//un identifiant unique pour chaque atome.
} Atome;

//pour pouvoir retrouver plus rapidement les boules qui se touchent on a une liste
//chainée de stucture info_boule.
//on a l'atome pere et le numero de la boule dans la hierarchie de l'atome, donc on peut retrouver sa couleur, son lien, etc...
typedef struct InfoBoule {
	dBodyID *boule;			//un pointeur vers l'identifiant de la boule (pointe vers Atome.atome[X])
	Atome *atome_pere;		//Quel atome contient cette boule (pointe vers un Atome)
	int num_boule;			//Cette boule est la boule atome_pere->atome[num_boule + 1],
	struct InfoBoule *p_suivant;    //Le suivant de la liste
}InfoBoule;

/**********************************************************************************/
//Les objets
static dWorldID world;
static dSpaceID space;
static Atome *agents[NB_AGENTS];
static Atome atomic1;//,atomic2,atomic3,atomic4,atomic5,atomic6,atomic7,atomic8;
static dJointGroupID contactgroup;
static dJointGroupID accrogroup;
InfoBoule *p_liste_boules = NULL;
static long pas = 0;
static int direction = 0;


/**********************************************************************************/
//gestion de la liste des boules

//ajout dans la liste
InfoBoule *ajout_boule( dBodyID *id_boule, Atome *at_pere, int n_b, InfoBoule *p_liste ) {

	struct InfoBoule *newIB;

	newIB = (struct InfoBoule *) malloc( sizeof(struct InfoBoule) );
	newIB->boule = id_boule;
	newIB->atome_pere = at_pere;
	newIB->num_boule = n_b;
	newIB->p_suivant = p_liste;
	return(newIB);
}



//fonction de recherche d'une boule dans la liste
//	Renvoie un pointeur sur la structure qui contient la boule et NULL si la boule n'est pas dans la liste
InfoBoule *recherche( dBodyID b , InfoBoule *p_liste ) {
	InfoBoule *p = p_liste;

	while( (p != NULL) ){
		if( *(p->boule) == b ) { return(p); }
		else { p = p->p_suivant; }
	}
	return(NULL);
}

/**********************************************************************************/
//NE SERT PAS ENCORE
//cette fonction crée dynamiquement NB_AGENTS agents et les reference par le tableau passé en parametre
void initAgents( Atome *tab_atome[] ) {
	int i;

	for( i = 0 ; i < NB_AGENTS ; i++ ) {
		tab_atome[i] = (Atome *) malloc( sizeof(Atome) );
	}
}


//Fonction d'initialisation d'un atome passé en parametre
//	aty pointe sur l'atome DEJA crée en mémoire
//	x,y,z sont les coordonnées du centre de gravité de l'atome
//	world et space sont representent l'univers ou on place notre atome
//	p_liste est un pointeur sur liste de boules -- pour referencer rapidement les boules
InfoBoule *initAtome ( Atome *aty, int ident, dReal x, dReal y, dReal z, dWorldID world, dSpaceID space, InfoBoule *p_liste) {
	dMass m;
	int i;

	aty->identification = ident;

	//le cube central
	aty->atome[0] = dBodyCreate (world);
	dBodySetPosition (aty->atome[0],x,y,z);
	dMassSetBox (&m, 1, COTE, COTE, COTE);
	dMassAdjust (&m, MASSE_C);
	dBodySetMass (aty->atome[0], &m);
	aty->cube = dCreateBox (0, COTE, COTE, COTE);
	dGeomSetBody (aty->cube, aty->atome[0]);

	//les boules ; ATTENTION : demarrage a 1 !
	for (i=1; i<=NB_BOULES; i++) {
		aty->atome[i] = dBodyCreate (world);
		//dBodySetPosition (body[i],k,k,k+0.4);
		dMassSetBox (&m, 1, RAYON, RAYON, RAYON);
		dMassAdjust (&m,MASSE_B);
		dBodySetMass (aty->atome[i],&m);
		aty->boules[i-1] = dCreateSphere (space, RAYON);
		dGeomSetBody (aty->boules[i-1],aty->atome[i]);
	}
	//placement des boules autour du cube
	//comme sur mon dés rouge !
	dBodySetPosition (aty->atome[1], x+0.0, y+0.5, z+0.0);
	dBodySetPosition (aty->atome[2], x+0.0, y+0.0, z-0.5);
	dBodySetPosition (aty->atome[3], x+0.5, y+0.0, z+0.0);
	dBodySetPosition (aty->atome[4], x-0.5, y+0.0, z+0.0);
	dBodySetPosition (aty->atome[5], x+0.0, y+0.0, z+0.5);
	dBodySetPosition (aty->atome[6], x+0.0, y-0.5, z+0.0);



	//création des joints boules et moteurs
	for (i=0; i<NB_BOULES; i++) {
		//on va faire deux joints hinge et un joints moteur
		aty->hinge1[i] = dJointCreateHinge (world, 0); //dJointCreateBall (world,0);
		dJointAttach ( aty->hinge1[i], aty->atome[0] , aty->atome[i+1] );

		aty->hinge2[i] = dJointCreateHinge (world, 0); //dJointCreateBall (world,0);
		dJointAttach ( aty->hinge2[i], aty->atome[0] , aty->atome[i+1] );

		aty->motors1[i] = dJointCreateAMotor (world,0);
		dJointAttach ( aty->motors1[i], aty->atome[0] , aty->atome[i+1] );

		aty->motors2[i] = dJointCreateAMotor (world,0);
		dJointAttach ( aty->motors2[i], aty->atome[0] , aty->atome[i+1] );

		//type des moteurs
		dJointSetAMotorMode (aty->motors1[i], dAMotorUser); //on def. soit meme les angles
		dJointSetAMotorNumAxes (aty->motors1[i], 1); //on a 3 degrés de liberté

		dJointSetAMotorMode (aty->motors2[i], dAMotorUser); //on def. soit meme les angles
		dJointSetAMotorNumAxes (aty->motors2[i], 1); //on a 3 degrés de liberté

		//couleur des boules
		aty->couleurs[i] = 0;//bleue par défaut

		//par defaut personne n'est accroché
		aty->bool_accro[i] = 0;

		//ajout de la boules dans la liste
		p_liste = ajout_boule( &(aty->atome[i+1]) , aty, i, p_liste );
	}

	//void dJointSetHingeAnchor (dJointID, dReal x, dReal y, dReal z);
	//void dJointSetHingeAxis (dJointID, dReal x, dReal y, dReal z);

	//placement des joint type hinge
	dJointSetHingeAnchor (aty->hinge1[0], x+0.0, y+0.2, z+0.0);
	dJointSetHingeAnchor (aty->hinge2[0], x+0.0, y+0.2, z+0.0);
	dJointSetHingeAxis (aty->hinge1[1], 0.0, 0.0, 1.0);
	dJointSetHingeAxis (aty->hinge2[1], 1.0, 0.0, 0.0);

	dJointSetHingeAnchor (aty->hinge1[1], x+0.0, y+0.0, z-0.2);
	dJointSetHingeAnchor (aty->hinge2[1], x+0.0, y+0.0, z-0.2);
	dJointSetHingeAxis (aty->hinge1[1], 1.0, 0.0, 0.0);
	dJointSetHingeAxis (aty->hinge2[1], 0.0, 1.0, 0.0);

	dJointSetHingeAnchor (aty->hinge1[2], x+0.2, y+0.0, z+0.0);
	dJointSetHingeAnchor (aty->hinge2[2], x+0.2, y+0.0, z+0.0);
	dJointSetHingeAxis (aty->hinge1[2], 0.0, 0.0, 1.0);
	dJointSetHingeAxis (aty->hinge2[2], 0.0, 1.0, 0.0);

	dJointSetHingeAnchor (aty->hinge1[3], x-0.2, y+0.0, z+0.0);
	dJointSetHingeAnchor (aty->hinge2[3], x-0.2, y+0.0, z+0.0);
	dJointSetHingeAxis (aty->hinge1[3], 0.0, 0.0, 1.0);
	dJointSetHingeAxis (aty->hinge2[3], 0.0, 1.0, 0.0);

	dJointSetHingeAnchor (aty->hinge1[4], x+0.0, y+0.0, z+0.2);
	dJointSetHingeAnchor (aty->hinge2[4], x+0.0, y+0.0, z+0.2);
	dJointSetHingeAxis (aty->hinge1[4], 1.0, 0.0, 0.0);
	dJointSetHingeAxis (aty->hinge2[4], 0.0, 1.0, 0.0);

	dJointSetHingeAnchor (aty->hinge1[5], x+0.0, y-0.2, z+0.0);
	dJointSetHingeAnchor (aty->hinge2[5], x+0.0, y-0.2, z+0.0);
	dJointSetHingeAxis (aty->hinge1[5], 0.0, 0.0, 1.0);
	dJointSetHingeAxis (aty->hinge2[5], 1.0, 0.0, 0.0);

	/* Récapitulation des axes
	boules[i]| Axe1 | Axe2
	0        |  z   |  x
	1        |  x   |  y
	2        |  z   |  y
	3        |  z   |  y
	4        |  x   |  y
	5        |  z   |  x
	*/

	return(p_liste);
}


//Fonction de dessin d'un atome dans la boucle de simulation avec Drawstuff
void dessinAtome( Atome *aty ){
	int i;

	//dessin de la scène
	//le corps
	dsSetColor (1,1,0);
	dsSetTexture (DS_WOOD);
	dReal sides[3] = {COTE, COTE, COTE };
	dsDrawBox (dBodyGetPosition(aty->atome[0]),dBodyGetRotation(aty->atome[0]),sides);

	//les boules
	dsSetTexture (DS_WOOD);
	for (i=1; i<=NB_BOULES; i++){
		switch ( aty->couleurs[i-1] ) {
  			case 0:dsSetColor (1,0,0);break;
			case 1:dsSetColor (0,1,0);break;
			case 2:dsSetColor (0,0,1);break;
		}
		dsDrawSphere (dBodyGetPosition(aty->atome[i]), dBodyGetRotation(aty->atome[i]), 0.3);
	}
}

/**********************************************************************************/
//Tentative de marche
//un pas de simulation pour un agent
//cette methode est reprise par tous les agents
void step( Atome *aty ){
	if (pas > 550) {
		//printf("*");
		if(pas == 558){
			//printf("\n");
			pas = 0;
		}
		if ( direction == 0 ) {
			dBodySetForce  (aty->atome[0], 0.0, 1.5, 0.1);
			direction = 1;
		} else {
			dBodySetForce  (aty->atome[0], 1.5, 0.0, 0.1);
			direction = 0;
		}
	}
	pas++;
	/*dReal *pointe,x1,y1,z1;

	pointe = (dReal *) dBodyGetForce( aty->atome[0] );
	x1 = *(pointe+0); y1 = *(pointe+1); z1 = *(pointe+2);

	//je regarde ou je suis
	pointe = (dReal *) dBodyGetPosition( aty->atome[0] );
	x1 = *(pointe+0); y1 = *(pointe+1); z1 = *(pointe+2);

	//si je suis "proche" de là ou on m'a dit d'aller alors je m'arrete (force mise à zero)
	if (x1 < 5 && z1 < 0.8) {
		dBodySetForce  (aty->atome[0], 0.1, 0, 0);
	}

	//sinon "j'avance" (force mise à F dans la direction de mon) */
}


/**********************************************************************************/
//Fonction de collision
static void nearCallback (void *data, dGeomID o1, dGeomID o2){

	dBodyID b1,b2;
	InfoBoule *pib1,*pib2;
	dContact contact;
	int num_boule_1,num_boule_2;
	Atome *atome_1,*atome_2;
	dReal *pos_boule_1,x1,y1,z1;
	dReal *pos_boule_2,x2,y2,z2;
	dReal xf,yf,zf;

	//si les objets sont connéctés on fait rien
	b1 = dGeomGetBody(o1);
	b2 = dGeomGetBody(o2);
	if (dAreConnected (b1,b2)){
		return;
	}

	contact.surface.mode = 0;
	contact.surface.mu = 0.5;//dInfinity;
	//contact.surface.mu2 = dInfinity;//0;

	//Si il y a collision
	if (dCollide (o1,o2,0,&contact.geom,sizeof(dContactGeom))) {
		/* PAS D'ACCROCHAGE !!!
		//on recherche si les deux en collisio
		pib1 = recherche( b1 , p_liste_boules );
		pib2 = recherche( b2 , p_liste_boules );
		//si la recherche nous apprend que deux boules se rencontrent...
		if ( (pib1 != NULL) && (pib2 != NULL) && 0){
			atome_1 = pib1->atome_pere;
			atome_2 = pib2->atome_pere;
			num_boule_1 = pib1->num_boule;
			num_boule_2 = pib2->num_boule;
			//si ce ne sont pas deux boules d'un meme atome...
			if( (atome_1->identification) != (atome_2->identification) ) {
				//si la couleur de 1 est rouge et que la boule 1 n'est pas déjà accrochées...
				if( (atome_1->couleurs[num_boule_1] == 0) && //boule couleur rouge
				(atome_1->bool_accro[num_boule_1] == 0) && //boule pas accrochée
				(atome_2->couleurs[num_boule_2] == 0) && //autre boule rouge
				(atome_2->bool_accro[num_boule_2] == 0) ) { //autre boule pas accrochée

					//printf("Accrochage possible !!!\n");
					atome_1->couleurs[num_boule_1] = 1;
					atome_2->couleurs[num_boule_2] = 2;

					//calcul de l'intersection des deux sphères (emplacement du lien).
					pos_boule_1 = (dReal *) dBodyGetPosition( *(pib1->boule) );
					pos_boule_2 = (dReal *) dBodyGetPosition( *(pib2->boule) );
					x1 = *(pos_boule_1+0); y1 = *(pos_boule_1+1); z1 = *(pos_boule_1+2);
					x2 = *(pos_boule_2+0); y2 = *(pos_boule_2+1); z2 = *(pos_boule_2+2);
					xf = (x1+x2)/2; yf = (y1+y2)/2; zf = (z1+z2)/2;

					//on crée un joint et on fixe a TRUE le fait d'etre accroché
					atome_1->joints_accro[num_boule_1] = dJointCreateBall(world,accrogroup);
					dJointAttach( atome_1->joints_accro[num_boule_1], *(pib1->boule), *(pib2->boule) );
					dJointSetBallAnchor( atome_1->joints_accro[num_boule_1] , xf, yf, zf );
					atome_1->bool_accro[num_boule_1] = 1;
					atome_2->bool_accro[num_boule_2] = 1;


				//si la boule 2 est rouge et qu'elle n'est pas déjà attachée
				} else if( (atome_2->couleurs[num_boule_2] == 0) && //boule couleur rouge
					(atome_2->bool_accro[num_boule_2] == 0) && //boule pas accrochée
					(atome_1->couleurs[num_boule_1] == 0) && //autre boule rouge
					(atome_1->bool_accro[num_boule_1] == 0) ) { //autre boule pas accrochée

					//printf("Accrochage possible !!!\n");
					atome_1->couleurs[num_boule_1] = 2;
					atome_2->couleurs[num_boule_2] = 1;

					//position des objets et emplacement du lien
					pos_boule_1 = (dReal *) dBodyGetPosition( *(pib1->boule) );
					pos_boule_2 = (dReal *) dBodyGetPosition( *(pib2->boule) );
					x1 = *(pos_boule_1+0); y1 = *(pos_boule_1+1); z1 = *(pos_boule_1+2);
					x2 = *(pos_boule_2+0); y2 = *(pos_boule_2+1); z2 = *(pos_boule_2+2);
					xf = (x1+x2)/2; yf = (y1+y2)/2; zf = (z1+z2)/2;

					//on crée un joint et on fixe a TRUE le fait d'etre accroché
					atome_2->joints_accro[num_boule_2] = dJointCreateBall(world,accrogroup);
					dJointAttach( atome_2->joints_accro[num_boule_2], *(pib2->boule), *(pib1->boule) );
					dJointSetBallAnchor( atome_2->joints_accro[num_boule_2] , xf, yf, zf );

					atome_1->bool_accro[num_boule_1] = 1;
					atome_2->bool_accro[num_boule_2] = 1;
				}
			}
		}*/
		dJointID c = dJointCreateContact (world,contactgroup,&contact);
		dJointAttach (c,b1,b2);

	}
}

//debut de simulation
static void start(){
	//Pour 4 Viewpoint = (5.0786,-3.8783,1.7600,134.0000,15.5000,0.0000)
	//pour 8 Viewpoint = (5.5244,-0.2056,4.3100,158.5000,-29.5000,0.0000)
	static float xyz[3] = {5.5244,-0.2056,4.3100};
	static float hpr[3] = {158.5000,-29.5000,0.0000};
	dsSetViewpoint (xyz,hpr);
}


//boucle de simulation
static void simLoop (int pause){
	int i;

	if (!pause) {
		//collision
		dSpaceCollide (space,0,&nearCallback);
		dWorldStep (world,0.05);

		//nettoyage apres collision
		dJointGroupEmpty (contactgroup);
	}

	//pas de simulation pour les agents
	for( i=0 ; i<NB_AGENTS ; i++ ){
		step( agents[i] );
	}

	//dessin de la scène
	for( i=0 ; i<NB_AGENTS ; i++ ){
		dessinAtome( agents[i] );
	}
}

//Interaction clavier
static void command (int cmd){
  	switch (cmd) {
  	case 'x':
		applique_forceX();
  		break;
  	case 'y':
		applique_forceY();
  		break;
  	}
}
/**********************************************************************************/
//MAIN
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
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	accrogroup = dJointGroupCreate (0);

	dWorldSetGravity (world,0,0,-0.5);
	//le sol :
	dCreatePlane (space,0,0,1,0);

	//création des atomes
	p_liste_boules = initAtome(&atomic1,1, 0.0, 0.0, 1.0, world, space, p_liste_boules);
	/*p_liste_boules = initAtome(&atomic2,2, 1.5, 0.0, 1.5, world, space, p_liste_boules);
	p_liste_boules = initAtome(&atomic3,3, -1.5, 0.0, 1.5, world, space, p_liste_boules);
	p_liste_boules = initAtome(&atomic4,4, 3.0, 0.0, 2.0, world, space, p_liste_boules);
	p_liste_boules = initAtome(&atomic5,5, 0.0, 1.5, 2.0, world, space, p_liste_boules);
	p_liste_boules = initAtome(&atomic6,6, 1.5, 1.5, 2.5, world, space, p_liste_boules);
	p_liste_boules = initAtome(&atomic7,7, -1.5, 1.5, 2.5, world, space, p_liste_boules);
	p_liste_boules = initAtome(&atomic8,8, 3.0, 1.5, 1.0, world, space, p_liste_boules);*/

	//ajout des objets pour les collisions
	dSpaceAdd (space, atomic1.cube);
	/*dSpaceAdd (space, atomic2.cube);
	dSpaceAdd (space, atomic3.cube);
	dSpaceAdd (space, atomic4.cube);
	dSpaceAdd (space, atomic5.cube);
	dSpaceAdd (space, atomic6.cube);
	dSpaceAdd (space, atomic7.cube);
	dSpaceAdd (space, atomic8.cube);*/

	//on ajoute les agents dans le tableau
	agents[0] = &atomic1;
	/*agents[1] = &atomic2;
	agents[2] = &atomic3;
	agents[3] = &atomic4;
	agents[4] = &atomic5;
	agents[5] = &atomic6;
	agents[6] = &atomic7;
	agents[7] = &atomic8;*/

	//Mise en route de la simulation
	dsSimulationLoop (argc,argv,352,288,&fn);

	//destruction en mémoire des objets
	dJointGroupDestroy (accrogroup);
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);

	return 0;
}
