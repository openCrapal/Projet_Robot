#ifndef CODEUR
#define CODEUR
#include "Arduino.h"
#include "Encoder.h"


class Codeur
{
public:
    Codeur();
    Codeur(uint8_t pinInt, uint8_t pinNoInt);
    Codeur(uint8_t pinInt, uint8_t pinNoInt, bool v);
    Codeur(uint8_t pinInt, uint8_t pinNoInt, bool v, bool a);
    ~Codeur();
    void initialiser(uint8_t pinInt, uint8_t pinNoInt);
    void initialiser(uint8_t pinInt, uint8_t pinNoInt, bool v);
    void initialiser(uint8_t pinInt, uint8_t pinNoInt, bool v, bool a);
    long* get_p_position();
    long  get_deplacement();
    double* get_p_vitesse();
    double* get_p_acc();
    void update();

protected:
    Encoder *myEnc;
    long memoire[5][2];
    long position;
    double acc, vitesse;
    bool ma, mv;


};


#endif //CODEUR
