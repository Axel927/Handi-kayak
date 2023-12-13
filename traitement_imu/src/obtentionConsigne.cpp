#include <iostream> 
#include <cmath>
#include <stdlib.h>

using namespace std;

float obtentionConsigne(float angle, float position,float seuil_angle_haut,float seuil_distance_haut,float seuil_distance_bas);
bool haut =true;

int main(void)
{
    for (int i = 0 ; i < 18 ; i ++){
        float test = obtentionConsigne(i * 0.01f,1,5,20,10);
        cout<<test<<endl;
    }
    
    return 0;
}

//obtentionConsigne renvoie l'intensite avec laquelle le kayakiste doit tourner 
//float angle est l'angle du kayak en radian
//float position est la position horizontal du kayak en m
//float seuil_angle_haut est la rotation maximale autorisé du kayak
//float seuil_distance_haut est la distance en avant maximale visée par le kayak
//float seuil_distance_bas est la distance en avant minimale visée par le kayak
float obtentionConsigne(float angle, float position,float seuil_angle_haut,float seuil_distance_haut,float seuil_distance_bas){
    float bip = 0.0; //if bip - turn right if bip + turn left
    float abs_angle = abs(angle); //valeur absolue de l'angle
    float signe_angle = copysign( 1.0, angle); //Signe de l'angle détermine si tu tourne à droite ou à gauche
    const float one =1.0f; //Constante 1 en flottant
    float cinq_degree_rad = 0.087266; //Valeur de 5° en rad
   
    float abs_position = abs(position); //Valeur absolue de la position
    float signe_position = copysign( 1.0,position); //Signe de la position dit si le kayak est à droite ou à gouche de la ligne

    float cote_opp; //Longueur du coté oppose forme par le kayak

    if(abs_position < 0.25f){ //Si on est environ au centre
        if( (signe_angle * signe_position < 0 && abs_angle !=0) || abs_angle > seuil_angle_haut){ //Si on pointe vers l'exterieur du couloir ou qu'on tourne trop fort
            bip = signe_angle * min( one , abs_angle / cinq_degree_rad ); //Calcule l'intensité du bip
        }
    }
    else if( abs_angle != 0 ){ //Si on ne va pas tout droit

        cote_opp = tan( M_PI / 2 - abs_angle ) * abs_position; //Calcule de la longueur du coté opposé
        cout<<""<<endl;
        cout<<"cote_opp : "<< cote_opp<<endl;
	    cout<<"haut : "<< haut<<endl;

        if(!haut){
            haut = cote_opp > (seuil_distance_haut + seuil_distance_bas) / 2;
            bip = signe_position * max( -one, min( one, cote_opp / (( seuil_distance_haut + seuil_distance_bas) / 2) - 1));
        }
        
        else if(signe_angle * signe_position < 0 && !abs_angle == 0){ //Si le kayak pointe vers l'exterieur du couloir

            bip = signe_position;
        }
    
        else if(cote_opp > seuil_distance_haut){ //Si le kayak ne vise pas entre les deux seuille

            bip = -signe_position*max(-one, min( one, cote_opp / seuil_distance_haut - 1)); //
        
        }
        else if( cote_opp < seuil_distance_bas || !haut){ //Si le kayak arrete de viser entre les deux seuil

                bip = signe_position * max( -one, min( one, cote_opp / (( seuil_distance_haut + seuil_distance_bas) / 2) - 1)); //Tourne afin d'arriver entre les deux seuil
                haut = false;
                cout<<"Test"<<endl;
        }
    }
    return bip;
}
