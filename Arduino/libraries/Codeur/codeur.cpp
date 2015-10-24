#include "codeur.h"

Codeur::Codeur() {}
Codeur::Codeur(uint8_t pinInt, uint8_t pinNoInt)
{
    initialiser(pinInt, pinNoInt);
    pinMode(pinInt, INPUT);
    pinMode(pinNoInt, INPUT);
}
Codeur::Codeur(uint8_t pinInt, uint8_t pinNoInt, bool v)
{
    initialiser(pinInt, pinNoInt, v);
}
Codeur::Codeur(uint8_t pinInt, uint8_t pinNoInt, bool v, bool a)
{
    initialiser(pinInt, pinNoInt, v, a);
}
Codeur::~Codeur()
{
    delete myEnc;
}
void Codeur::initialiser(uint8_t pinInt, uint8_t pinNoInt)
{
    myEnc = new Encoder(pinInt, pinNoInt);
    mv = false;
    ma = false;

    for (int i=0; i<5; i++)
    {
        memoire[i][0] =0;
        memoire[i][1] =0;
    }
    position=0;
    vitesse=0.;
    acc=0.;
}
void Codeur::initialiser(uint8_t pinInt, uint8_t pinNoInt, bool v)
{
    Codeur::initialiser(pinInt, pinNoInt);
    mv=v;
}
void Codeur::initialiser(uint8_t pinInt, uint8_t pinNoInt, bool v, bool a)
{
    Codeur::initialiser(pinInt, pinNoInt);
    mv=v;
    ma=a;
}
long* Codeur::get_p_position()
{
    return &position;
}
long Codeur::get_deplacement()
{
    return (position - memoire[1][1]);
}
double* Codeur::get_p_vitesse()
{
    return &vitesse;
}
double* Codeur::get_p_acc()
{
    return &acc;
}
void Codeur::update()
{
    if(ma || mv)
    {
        for (int i=0; i< 4; i++)
        {
            memoire[i+1][0] = memoire[i][0];
            memoire[i+1][1] = memoire[i][1];
        }
        memoire[0][0] = millis();
        memoire[0][1] = myEnc->read();
        position = memoire[0][1];
        vitesse = 0.;
        acc = 0.;
        double v0=0;
        for(int i=1; i<5; i++)
        {
            double vi = (memoire[0][1] - memoire[i][1]) *1. / (memoire[0][0] - memoire[i][0]);
            vitesse += vi / i;
            if (i==1)
            {
                v0=vi;
            }
            else if (ma)
            {
                double ai = (v0 - vi) * 2. / (memoire[0][0] - memoire[i][0]);
                acc += ai / i;
            }
        }
        vitesse /= 2.08333 / 1000.;
        acc /= 1.08333 / 1000.;

    }
    else
    {
        position = myEnc->read();
    }
}
