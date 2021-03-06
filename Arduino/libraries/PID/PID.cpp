#include "PID.h"
using namespace std;


Correcteur::Correcteur(float *m_Kv, float *m_Kp, float *m_Tp, long unsigned (*m_millis)(), float *m_consigne, float *m_sortie, float* m_capteur, float *m_Gcapt) : Kv(m_Kv), Kp(m_Kp), Tp(m_Tp), millisec(m_millis), p_consigne(m_consigne), p_sortie(m_sortie), p_capteur(m_capteur), Gcapt(m_Gcapt)
{
    millisec_old = (*millisec)();
    consigne_old = *p_consigne;
    somme_old=0;
    capteur_d = false;
}

Correcteur::Correcteur(float *m_Kv, float *m_Kp, float *m_Tp, long unsigned (*m_millis)(), float *m_consigne, float *m_sortie, float* m_capteur, float *m_Gcapt, float *m_Gcapt_d, float *m_capteur_d) : Kv(m_Kv), Kp(m_Kp), Tp(m_Tp), millisec(m_millis), p_consigne(m_consigne), p_sortie(m_sortie), p_capteur(m_capteur), Gcapt(m_Gcapt), Gcapt_d(m_Gcapt_d), p_capteur_d(m_capteur_d)
{
    millisec_old = (*millisec)();
    consigne_old = *p_consigne;
    somme_old=0;
    capteur_d = true;
}



void Correcteur::iteration() // fourni la valeur du signal de sortie en fonction de la consigne et du capteur
{
    float dt = (millisec_old - (*millisec)())/1000.;

    if (dt < 0.002) return;    // empeche la fonction de s'executer trop souvent pour des raisons de stabilite

    if(capteur_d)
    {
        float ecart = (*p_consigne) - *Gcapt * (*p_capteur);
        float dif = (ecart - ecart_old) / dt;
        somme_old = (somme_old * (*Tp-dt) + dt *(ecart+*Kv * dif- *Gcapt_d * (*p_capteur_d))) / (*Tp);
        *p_sortie = *Kp * (ecart + dif) + somme_old;
        ecart_old=ecart;
    }
    else
    {
        float dif = *p_consigne - consigne_old;
        float ecart = dif + *Kv * (*p_consigne - *Gcapt * (*p_capteur));


        somme_old = (somme_old * (*Tp-dt) + dt *(ecart)) / (*Tp);

        *p_sortie = *Kp * ecart + somme_old ;
    }

}
