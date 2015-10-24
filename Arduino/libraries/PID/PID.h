#ifndef PID
#define PID

class Correcteur
{
    public:

    Correcteur(float *m_Kv, float *m_Kp, float *m_Tp, long unsigned (*m_millis)(), float *m_consigne, float *m_sortie, float *m_capteur, float *m_Gcapt);

    Correcteur(float *m_Kv, float *m_Kp, float *m_Tp, long unsigned (*m_millis)(), float *m_consigne, float *m_sortie, float *m_capteur, float *m_Gcapt, float *m_Gcapt_d, float *m_capteur_d);

    void iteration();

    protected:

    float *Kv, *Kp, *Tp;
    long unsigned millisec_old;
    long unsigned (*millisec)();
    float *p_consigne, *p_sortie, *p_capteur, *Gcapt, *Gcapt_d, *p_capteur_d;
    float consigne_old;
    float ecart_old;
    float somme_old;
    bool capteur_d;

};

#endif // PID
