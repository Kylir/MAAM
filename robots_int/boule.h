/* boules.h des ROBOTS !!! */

//les includes
#include <stdio.h>
#include <time.h>
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
//pour les agents
#define NB_AGENTS 10			//le nombre de boules dans ma simulation
#define RAYON 0.05				//le rayon d'une boule
#define MASSE 100.0				//la masse d'une boule
#define COEFF_FORCE 1//0.5		//defini le coeff mult. de force que chaque agent a pour pousser
#define SEUIL_MAX_VITESSE 0.09	//la vitesse seuil des agents
#define SEUIL_MIN_VITESSE 0.02	//La vitesse que l'agent doit atteindre pour qu'on arrete de le pousser selon Z
#define INC_Z 0.5//5				//la valeur de l'incrementation des Z (pour calc_force() )
#define SEUIL_INC_Z 25.0			//La valeur seuil de la force en Z
#define HAUTEUR_PYRAMIDE 1.0	//vision sur le plan vertical des agents
#define LARGEUR_PYRAMIDE 5000.0	//vision sur le plan horizontal des agents
#define SEUIL_BUT 0.0025			//dans cette fourchette l'agent estime qu'il a atteint son but
#define DIST_SECUR 0.005		//la distance derrière laquelle je suis un agent (du verbe suivre)


//on peut choisir la forme des agents
#define SPHERE 0
#define BOX 1
#define FORME_AGENT 0

//on peut choisir si les agents sont en ligne ou au hasard
#define EN_LIGNE 0//1

//on peut choisir si un agent aggrippe une seule fois ou plusieurs
#define ATTACHES_MULTIPLES 0//1

//pour l'environnement
#define LONGUEUR 3.0
#define LARGEUR 10.0
#define EPAISSEUR 3.0
#define FOSSE 0.25
#define MASSE_BLOC 10000.0
#define COULEUR_BLOC 0.3,0.7,1
#define TAILLE_BANDE_GLUANTE 0.05

//le pas de simulation
#define PAS_SIMU 0.01

//un decalage vers le haut pour le début de la simulation et pour les agent
#define HAUSSE 0.01
#define HAUSSE_AGENT 0.01

//La gravitation de la simulation
#define GRAVITE	 0,0,-9.1

//Les differents type de frottements
#define COEFF_FROTTEMENTS 1//dInfinity
#define COEFF_FROTTEMENTS_AGENTS 0//dInfinity

//Les informations concernant l'attractant
#define RAYON_ATTRACTANT 0.2
#define MASSE_ATTRACTANT 1.0
#define COULEUR_ATTRACTANT 0,1,1

//Pour l'initialisation du nb max de capteurs de collisions
#define MAX_INFOS_COLLISION 10

/**********************************************************************************/
//un vecteur a 3 composantes
typedef struct Vecteur{
	dReal x;
	dReal y;
	dReal z;
} Vecteur;

/**********************************************************************************/
//l'element de la liste des agents pour savoir qui me tient et qui je tiens
typedef struct listeAgents{
	struct Agent *agent;		//l'agent avec qui je suis lié
	dJointID id;			//l'ID du joint qui nous lie
	struct listeAgents *p_suivant;  //le pointeur sur l'element suivant
} listeAgents;

/**********************************************************************************/
// Voici la structure qui implemente un capteur de collision
typedef struct infos_collision {
	dBodyID objet;	//l'objet rencontré
	int type_objet;	//1-> bloc 1 ; 2-> bloc 2 ; 3-> attractant ; 4-> agent ; 5-> autre
	int etat;		//Si type_collision == 4 alors etat est l'etat de l'agent rencontré
} infos_collision;

/**********************************************************************************/
//voici la structure pour gerer un agent boule
typedef struct Agent{
	dBodyID bodyID;		//le num d'ident d'un agent
	dGeomID geomID;		//le num de geometrie de l'agent
	dMass masse;			//La masse de l'agent
	int couleur;				//la couleur de l'agent

	int place;				//sa place dans la configuration
	int stop;				//boolean qui me dit si on souhaite que je sois stoppé
	int en_reconf;			//boolean qui dit si je suis en reconfiguration

	int au_dessus;			//pour savoir si il y a quelqu'un au dessus
	int au_dessous;			//pour savoir si il y a quelqu'un au dessous
	int au_devant;                    //pour savoir si il y a quelqu'un devant
	int au_derriere;                   //pour savoir si il y a quelqu'un derriere
	int au_gauche;			// - - -
	int au_droite;			// - - -

	dJointID patte_move;		//le joint pour avancer (Hinge)
	dJointID patte;			//le joint qui me sert pour stopper la marche (Universal)
	dJointID mandy;			//le joint pour accrocher un autre agent (Hinge)
        dJointID avant;
	dJointID arriere;			//ce joint est celui qui m'accrocheavec le joint mandy d'un autre (Hinge)
	dJointID gauche;
	dJointID droite;

	int etat;				//etat de l'agent : 0-> en marche ; 1-> bande gluante; 2-> parcours de chaine;

	infos_collision capteurs_collision[ MAX_INFOS_COLLISION ];	//les capteur de collisions
	int nombre_collision;		//combien de cases de infos_collisiosns sont remplies

	Vecteur pos_but;		//l'agent retient où se trouve son but
	dBodyID bodyID_but;		//le bodyID du but

	long cycle;				//me donne le num d'iterations dans le cycle de l'agent
} Agent;

/**********************************************************************************/
//structure pour les blocs du terrain
typedef struct Bloc{
	dBodyID bodyID;		//le num d'ident d'un agent
	dGeomID geomID;		//le num de geometrie de l'agent
	dMass masse;			//la masse du bloc
	int couleur;				//la couleur de l'agent
	long identification;		//son numero d'ident unique
} Bloc;

/**********************************************************************************/
//gestion de l'attractant
typedef struct Attractant{
	dBodyID bodyID;		//le num d'ident
	dGeomID geomID;		//le num de geometrie
	dMass masse;			//La masse
	int couleur;				//la couleur
} Attractant;


/**********************************************************************************/
//Les objets
dWorldID world;				//le monde
dSpaceID space;				//l'espace physique
struct Bloc bloc_depart,bloc_arrive;	//les deux blocs formants les "rives"
dJointGroupID contactgroup;		//pour gerer les contacts
dJointGroupID accrogroup;		//la liste des accrochages
unsigned long nb_pas_simulation;	//le nombre d'iterations de la simu
unsigned long nb_noyades;		//le nombre d'agents qui tombent
struct Agent agents[NB_AGENTS];		//la liste des agents.
dGeomID ground;				//le sol
struct Attractant attract;		//ce qui va attirer les agents
int ordre_arrivee;



/**********************************************************************************/
//prototypes des fonctions du fichier gestion_liste_joints.c
//ajout dans la liste
listeAgents *ajout_agent( listeAgents *p_liste, Agent *agy, dJointID joint, int nb_avant );
//test de presence
int present( listeAgents *p_liste, dBodyID body );
//renvoie l'agent ou NULL si il n'est pas dans cette liste
Agent *recherche( listeAgents *p_liste, dBodyID body );
//supprime de la liste
listeAgents *supprime( listeAgents *p_liste, dBodyID body );
//affiche la liste
void affiche( listeAgents *p_liste );

/**********************************************************************************/
//prototypes des fonctions du fichier init_agent.c
//initialisation de l'agent - si agy est NULL l'agent est crée dynamiquement
Agent *initAgent ( Agent *agy, int place, dReal x, dReal y, dReal z, dWorldID world, dSpaceID space );
//un groupe d'agent au hasard sur le bloc de départ
void init_groupe_agents( Agent tab_agents[] , int nb_agents, dWorldID monde, dSpaceID espace );
//les agents place en ligne
void init_ligne_agents( Agent tab_agents[] , int nb_agents, dWorldID monde, dSpaceID espace );

/**********************************************************************************/
//prototypes des fonctions du fichier dessin_agent.c
//dessin d'un agent
void dessinAgent( Agent *agy );

/**********************************************************************************/
//prototypes des fonctions du fichier init_env.c
//init d'un bloc
Bloc *init_bloc( Bloc *bob, dReal x, dReal y, dReal z, dWorldID monde, dSpaceID espace );
//init des deux blocs en calculant leur emplacement
void init_env( Bloc *bloc_depart,Bloc *bloc_arrive, dWorldID monde, dSpaceID espace );

/**********************************************************************************/
//prototypes des fonctions du fichier dessin_env.c
//dessin des deux blocs
void dessin_env( Bloc *bloc1, Bloc *bloc2 );

/**********************************************************************************/
//prototypes des fonctions du fichier init_attractant.c
//
void init_attractant ( Attractant *aty, dWorldID world, dSpaceID space );

/**********************************************************************************/
//prototypes des fonctions du fichier dessin_attractant.c
//dessin
void dessin_attractant( Attractant *aty );

/**********************************************************************************/
//prototypes des fonctions du fichier simulation.c
//au demarrage de la simulation - ex: reglage du point de vue
void start();
//quand deux objets sont proches cette fonctions se lance
void nearCallback (void *data, dGeomID o1, dGeomID o2);
//represente le coeur de la simulation
void simLoop (int pause);
//interaction clavier
void command (int cmd);
//Pour savoir ce que cogne un agent : 1-> collision bloc 1 ; 2-> collision bloc 2 ; 3-> collision avec l'attractant
int quel_objet(dBodyID b);

/**********************************************************************************/
//prototypes des fonctions du fichier geometrie.c
//Calcul de l'equation d'une droite avec deux points
void eqn_droite2D( Vecteur *point1 , Vecteur *point2 , Vecteur *coeff_eqn );
//Dit si le point3 est au dessous de la droite passant par p1 et p2
int au_dessous_dt( Vecteur *point1 , Vecteur *point2 , Vecteur *point3 );
//Dit si le point3 est au dessus de la droite passant par p1 et p2
int au_dessus_dt( Vecteur *point1 , Vecteur *point2 , Vecteur *point3 );
//Renvoie 1 si le point candidat est dans le triangle formé par p1, p2 et p3
int dans_triangle( Vecteur *point1 , Vecteur *point2 , Vecteur *point3 , Vecteur *candidat );
//Renvoie 1 si le point candidat est dans la pyramide de vision de l'agent
int dans_pyramide( Vecteur *pos_agent, Vecteur *pos_but, Vecteur *candidat );

/**********************************************************************************/
//prototypes desfonctions du fichier agent.c
//calcul de lu vecteur direction entre un agent et un attractant
//void calc_force( Agent *agy, Attractant *att, Vecteur *vret, dReal *vitesse );
//test booleen pour savoir si un corps est un agent
int is_agent( dBodyID bod );
//retrouve un agent par le bodyID
Agent *cherche_agent( dBodyID bod );
//calcul de la distance entre deux points dans un plan (pas de z)
dReal distance2D( Vecteur *pv1 , Vecteur *pv2 );
//calcul de la distance entre deux points dans l'espace
dReal distance3D( Vecteur *pv1 , Vecteur *pv2 );
//regarde si un agent est devant moi
int devant_moi( Agent *moi, Agent *agy, Attractant *but, dReal *pos_moi, dReal *pos_lui, dReal *pos_but );
//supprime les agents qui sont tombés
void noyade( Agent *agy );
//test si l'agent est dans la bande gluante
int dans_bande_gluante1( Agent *agy );
//test si la vitesse est entre -SEUIL_VITESSE et SEUIL_VITESSE
int vitesse_ok( dReal *v_vitesse );
//met a jour la position de l'agent
//void maj_pos(Agent *agy);
//calcul le point de gravité entre deux agents
void calc_point_accro( dReal *pos_agent1, dReal *pos_agent2, Vecteur *rez );
//
void calcul_pt_ancrage( Agent *moi, dBodyID but, Vecteur *vrez );
//
void calcul_normale( Agent *agy, Attractant *att, Vecteur *pt_ancrage, Vecteur *vnorm);
//la procedure de deplacement de l'agent
void marche( Agent *agy, dBodyID att );
//un pas de simulation pour l'agent
void step(Agent *agy);
//reglages fins de la vitesse
void reglage_fin_vitesse( Agent *agy, dReal *pos_agent, float param_x, float param_y, float param_z );
//pour suivre un agent
void suivi_agent( Agent *agy, dReal *pos_agent, dReal *pos_cible, float param_x, float param_y, float param_z );
//fonction qui regle la vitesse pour atteindre le but
void regle_vitesse( Agent *agy, dReal *pos_agent );
//
void regle_vitesse_balade( Agent *agy, dReal *pos_agent );
//cherche un objet de type n
dBodyID cherche_collision_type( Agent *agy, int ty );
//cherche un agent
Agent *cherche_collision_agent( Agent *agy );
//cherche un agent à l'etat "etat"
Agent *cherche_collision_agent_etat( Agent *agy, int etat ) ;
//assigner un nouveau sous-but à l'agent
void nouveau_but( Agent *agy, Vecteur pos_but, dBodyID id_but );
//assigne sa place comme but à l'agent
void a_ta_place( Agent *agy, int place );
//fait en sorte que actif monte sur passif
void grimpe_dessus( Agent *passif, Agent *actif );
//me dit quand je suis arrivé à mon but
int au_but( Agent *agy );
//Trouve le suivant de la chaine
Agent *suivant_de_la_chaine( Agent *agy );
//teleportation d'agent
void deplace_barbare( Agent *agent1 , Agent *agent2 );
//determine le plus proche de l'attractant entre deux agents.
Agent *plus_proche( Agent *agy1, Agent *agy2 );
//fonction qui assigne une place de libre comme but
int je_m_accroche_ou( Agent *cible, Agent *actif );
//fonction qui assigne une place avec  comme priorité l'arrière
int je_m_accroche_derriere( Agent *cible, Agent *actif );
//cette fonction regarde si deux agents sont bien à leur place selon le type de liaison (avant, arriere, etc.)
int a_sa_place( Agent *moi, Agent *lui, int ou );
//
int derriere_agent_ok( dReal *pos_moi, dReal *pos_lui );






