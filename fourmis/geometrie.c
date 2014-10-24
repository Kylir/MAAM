/* Fichier de gestion des opérations géometriques
   - equation de droite
   - appartenance au cone de detection
*/

#include "boule.h"


//fonction qui prend deux points et donne l'equation de la droite
//pas besoin de malloc le retour est en parametre
void eqn_droite2D( Vecteur *point1 , Vecteur *point2 , Vecteur *coeff_eqn ){
	dReal x1,x2,y1,y2,a,b;

	x1 = point1->x; y1 = point1->y;
	x2 = point2->x; y2 = point2->y;

	if( x1 == x2 ) {
		printf("Erreur : Division par zéro dans la fonction eqn_droite2D (x1 == x2).\n");
		exit(-1);
	}
	a = (y1 - y2) / ( x1 - x2 );
	b = y1 - (x1*a);

	coeff_eqn->x = a;
	coeff_eqn->y = b;
}

/****************************************************************************/
//fonction qui me dit si un point est en dessous de (ou sur) la droite
int au_dessous_dt( Vecteur *point1 , Vecteur *point2 , Vecteur *point3 ){
	Vecteur eqn;
	dReal a,b,rez;

	//je trouve l'equation de la droite qui passe par point1 et point2
	eqn_droite2D( point1 , point2 , &eqn );

	a = eqn.x;
	b = eqn.y;

	//je clacule rez = y3 - a*x1 -b ; si rez > 0 je dis oui
	rez = (a * point3->x) + b - point3->y;
	if( rez >= 0 ) { return(1); } else { return(0); }
}
/****************************************************************************/
//fonction qui me dit si un point est au dessus de (ou sur) la droite
int au_dessus_dt( Vecteur *point1 , Vecteur *point2 , Vecteur *point3 ){
	Vecteur eqn;
	dReal a,b,rez;

	//je trouve l'equation de la droite qui passe par point1 et point2
	eqn_droite2D( point1 , point2 , &eqn );

	a = eqn.x;
	b = eqn.y;

	//je clacule rez = y3 - a*x1 -b ; si rez > 0 je dis oui
	rez = (a * point3->x) + b - point3->y;
	if( rez <= 0 ) { return(1); } else { return(0); }
}

/****************************************************************************/
//fonction qui me dit si un point est dans un triangle délimité par trois points
int dans_triangle( Vecteur *point1 , Vecteur *point2 , Vecteur *point3 , Vecteur *candidat ) {
	//Pour traiter tous les cas il faut d'abord savoir si le point 1 est au dessus de la droite (p2,p3)
	if( au_dessus_dt( point2 , point3 , point1 ) ){
		if( au_dessous_dt( point1 , point2 , candidat ) ) {
			if( au_dessus_dt( point2 , point3 , candidat ) ) {
				if( au_dessous_dt( point1 , point3 , candidat ) ) {
					return(1);
				}
			}
		}
		return(0);
	} else {
		if( au_dessus_dt( point1 , point2 , candidat ) ) {
			if( au_dessous_dt( point2 , point3 , candidat ) ) {
				if( au_dessus_dt( point1 , point3 , candidat ) ) {
					return(1);
				}
			}
		}
		return(0);
	}
}

/****************************************************************************/
//fonction qui prend trois points de l'espace (agent ,attractant et candidat)
// et regarde si le candidat est dans la pyramide
int dans_pyramide( Vecteur *pos_agent, Vecteur *pos_but, Vecteur *candidat ) {
	Vecteur p1,p2,p3,c;

	//On verifie d'abord que le candidat est dans le triangle du plan vertical.
	//Pour cela on doit faire un changement de repère et trouver les coord des points 2 et 3
	p1.x = pos_agent->z;
	p1.y = pos_agent->x;
	p1.z = 0;

	p2.x = pos_but->z + (HAUTEUR_PYRAMIDE/2);
	p2.y = pos_but->x;
	p2.z = 0;

	p3.x = pos_but->z - (HAUTEUR_PYRAMIDE/2);
	p3.y = pos_but->x;
	p3.z = 0;

	c.x = candidat->z;
	c.y = candidat->y;
	c.z = 0;

	//on regarde si il est dans le premier triangle
	if( !dans_triangle( &p1 , &p2 , &p3 , &c ) ) { return(0); }

	//maintenant on s'occupe du triangle du plan horizontale
	//on fait comme au-dessus :
	p1.x = pos_agent->y;
	p1.y = pos_agent->x;
	p1.z = 0;

	p2.x = pos_but->y + (LARGEUR_PYRAMIDE/2);
	p2.y = pos_but->x;
	p2.z = 0;

	p3.x = pos_but->y - (LARGEUR_PYRAMIDE/2);
	p3.y = pos_but->x;
	p3.z = 0;

	c.x = candidat->y;
	c.y = candidat->x;
	c.z = 0;

	//si il n'est pas dans le triangle on renvoie faux
	if( !dans_triangle( &p1 , &p2 , &p3 , &c ) ) { return(0); }

	//si on arrive ici c'est bon !!!
	return(1);

}

/****************************************************************************/
//Main de test
/*int main() {
	Vecteur p1,p2,c;
	int rez;
	p1.x = 5; p1.y = 0; p1.z = 0;
	p2.x = -4; p2.y = 0; p2.z = 0;
	c.x = -4;  c.y = 0.0;  c.z = 0.0;
	//rez = dans_triangle( &p1, &p2, &p3, &c );
	rez = dans_pyramide( &p1, &p2, &c );
	if( rez == 0 ){printf("Pas dedans !\n");} else {printf("Dedans !\n");}
	return(0);
}*/





