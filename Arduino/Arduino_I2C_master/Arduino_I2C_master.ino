/*
  Programme Arduino commande_moteur
 Element du projet robot auto-balance
 Ulysse Delplanque, juillet 2015
 */
#include "Trajectoire.h"
#include "Z_Variable.h"
#include <Encoder.h>
#include "codeur.h"

// Capteur MPU6050
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 gyro;

/* Selection de la plage de mesure du capteur gyroscopique. Full Scale Range :
0 : ± 250 °/s
1 : ± 500 °/s
2 : ± 1000 °/s
3 : ± 2000 °/s
*/
int gyro_range = 1000;
uint8_t Gyro_range = 2;

/* Selection de la plage de mesure de l'accelerometre. Full Scale Range :
0 : ± 20m/s/s cad 2g
1 : ± 40
2 : ± 80
3 : ± 160
*/
int accel_range = 40;
uint8_t Accel_range = 1;

bool gyro_connecte = false;

// pins moteurs, carte ampli Cytron. Pin 9 et 10 PWM frequence modifiee pour reduire bruit
#define DIR_GAUCHE 11
#define PWM_GAUCHE 10
#define PWM_DROIT 9
#define DIR_DROIT 8

int PWM_gauche = 0;
int PWM_droit =  0;
// pins codeurs. Sur la carte Arduino UNO, les pins 2 et 3 commandent les interruptions exterieures
#define AV_DROIT 2
#define AR_DROIT 4
#define AV_GAUCHE 3
#define AR_GAUCHE 5

// Si le mode debug est actif DEBUG, la consigne est définie à la fin du Setup()
//#define DEBUG


// Initialisation des codeurs avec mesure de vitesse, sans acceleration
Codeur encodeur_gauche = Codeur(AV_GAUCHE,AR_GAUCHE, true, false);
Codeur encodeur_droit  = Codeur(AV_DROIT, AR_DROIT , true, false);

// Accelerometre/gyroscope... mis en place 2015/09/30
int16_t ax, ay, az;
int16_t gx, gy, gz;
double gain_a(accel_range / 32000.), gain_g(-gyro_range * PI /180 / 32000.);
double offset_gx(0.), offset_gy(0.), offset_gz(0.0);
double d_inclinaison(0.);

// Parametre mesure de position/vitesse (m/impulsion, rad/impuls,  envergure env.= 0.27m)
// DX = PI * Diam roue / impulsions par tour / 2.
// Dteta = DX / ecart roues
double DX_gauche(0.00147), DX_droit(-0.00147);
double Dteta_gauche(-0.01091), Dteta_droit(-0.01091);

// Variables localisation absolue, cad par rapport à la position initiale du robot, coords cartésiennes
double X_abs(0.), Y_abs(0.), Teta_abs(0.);
double X_debut(0.), Y_debut(0.);
double eq_orientation(0.);
double X_obj(0.), Y_obj(0.);

double distance_parcourue(0.), distance_obj(0.);
double d_old(0.);
double consigne_deplacement(0.);
double chemin_abs(0.), chemin_int(0.);

boolean rotation = true;

int const n= 2;
int indice_actuel = 0;
double tableau_obj[n][2];


long compteur =0;
//double a(0), b(0);

// Coordonnees locales de l'objectif, par rapport au robot instant t, polaires
double r_obj(0.), teta_obj(0.);

// Tolerances : distance point (m), orientation vers point suivant (rad)
double tolerance_distance(0.05), tolerance_orientation(0.2);

// Z_Variables de consigne. 2 boucles indépendantes, teta (direction, rad) et X_Axe_Soll (m)
Z_Capteur X_A_Soll(&r_obj), Teta_Soll(&teta_obj);

// Parametre moteur imp/sec/volt  340tr/min / 6V / 60s = 0.9444
// 0.9444 * 96 imp/tour = 90.666
//double constante_vitesse_moteur = 50.;

// Parametre asservissement
/*
  kp_trans_krit=1300
  tp_trans_krit=0.006
  kp_rot_krit = 100
  tp_rot_krit = 0.01
*/
double kv_trans(2.), kp_trans(600.), tp_trans(.05), kd_trans(0.1), satt_trans(64.);
double kv_rot(1.)  , kp_rot  (50)  , tp_rot(0.05) , kd_rot(.1)  , satt_rot(64.);
double kv_in (0.)  , kp_in   (50.)  , tp_in (10000.) , kd_in (0.)  , satt_in (60.);
double t_coupure_vitesse(.5);

// Parametres générateur de consigne axes
double r_max(.5), a_max(0.3), v_max(.5);
double teta_ppp_max(5.), teta_pp_max(1.), teta_p_max(5.);


// Initialisation des grandeurs physiques du shema bloc """"transformee en Z"""""
double zero(0.);
Z_Variable* Zero = new Z_Variable(zero);

Z_Capteur* D_Inclinaison = new Z_Capteur(&d_inclinaison);

Z_Capteur* Distance_parcourue = new Z_Capteur(&chemin_abs, -1.);
Z_Capteur* Teta_absolut = new Z_Capteur(&Teta_abs);

Z_Capteur* VitesseGauche = new Z_Capteur(encodeur_gauche.get_p_vitesse());
Z_Capteur* VitesseDroite = new Z_Capteur(encodeur_droit.get_p_vitesse());

Z_Somme* VitesseTranslationAxe = new Z_Somme(VitesseGauche, VitesseDroite, DX_gauche, DX_droit);    // en m/s
Z_Somme* VitesseRotationAxe    = new Z_Somme(VitesseGauche, VitesseDroite, Dteta_gauche, Dteta_droit); //en rad/s

Z_Integrale* FiltreVitesseTranslationAxe = new Z_Integrale(VitesseTranslationAxe, &t_coupure_vitesse);
Z_Integrale* FiltreVitesseRotationAxe    = new Z_Integrale(VitesseRotationAxe   , &t_coupure_vitesse);

// Commande continue des axes :
Axe AxeC   = Axe(v_max,      a_max,       r_max);
Axe AxeTeta= Axe(teta_p_max, teta_pp_max, teta_ppp_max);

Z_Capteur* ConsigneV = new Z_Capteur(AxeC.p_v());
Z_Capteur* ConsigneTp = new Z_Capteur(AxeTeta.p_v());

Z_Capteur* ConsignePositionAxe = new Z_Capteur(AxeC.p_x());
Z_Capteur* ConsigneOrientation = new Z_Capteur(AxeTeta.p_x());

Z_Somme* Delta_X = new Z_Somme(ConsignePositionAxe, Distance_parcourue, 1., -1.);
Z_Somme* Delta_T = new Z_Somme(ConsigneOrientation, Teta_absolut,       1., -1.);

Z_Somme* Delta_V = new Z_Somme(ConsigneV , FiltreVitesseTranslationAxe, 1.,  1.);
Z_Somme* Delta_Tp = new Z_Somme(ConsigneTp, FiltreVitesseRotationAxe  , 1., -1.);

Z_Somme* Ecart_V = new Z_Somme(Delta_X, Delta_V, kv_trans, 1.);
Z_Somme* Ecart_T = new Z_Somme(Delta_T, Delta_Tp, kv_rot  , 1.);

Z_PI_D RegInclinaison = Z_PI_D(D_Inclinaison, Zero, &kp_in, &tp_in, &kd_in, satt_in);
Z_PI_D RegPosition    = Z_PI_D(Ecart_V, Zero, &kp_trans, &tp_trans, &kd_trans, satt_trans);
Z_PI_D RegOrientation = Z_PI_D(Ecart_T, Zero, &kp_rot,   &tp_rot,   &kd_rot,   satt_rot);
/*
Z_PI RegPosition    = Z_PI(Ecart_V, Zero, &kp_trans, &tp_trans, satt_trans);
Z_PI RegOrientation = Z_PI(Ecart_T, Zero, &kp_rot,   &tp_rot,  satt_rot);
*/

// Variables communication Serie
String inputString = "";
boolean stringComplete = false;

bool led = false;

long temps_debut(0);
long temps_cycle_mini(2);

// modifier la frequence du PWM sur les pin 9 et 10 (pas de conséquences sur millis() et delay()
// TCCR1B = TCCR1B & 0b11111000 | 0x01;  => 31250 Hz
// TCCR1B = TCCR1B & 0b11111000 | 0x04;  => 122 Hz
// TCCR1B = TCCR1B & 0b11111000 | 0x02;  => 3906 Hz

/* ======================================== !!! Setup !!! ================================================*/
void setup()
{
  pinMode(13, OUTPUT);
  TCCR1B = TCCR1B & 0b11111000 | 0x01; //31250Hz, 64*plus que 490 par defaut (bruit moteur reduit)
  pinMode(DIR_GAUCHE,  OUTPUT);
  pinMode(PWM_GAUCHE,  OUTPUT);
  pinMode(PWM_DROIT , OUTPUT);
  pinMode(DIR_DROIT , OUTPUT);

  // L'utilisation du port serie empeche les interruptions codeur, à reduire drastiquement
  Serial.begin(115200);
  inputString.reserve(16);
  
  Wire.begin();
  Serial.println("Initializing I2C devices...");
  gyro.initialize();
  gyro.setFullScaleGyroRange(Gyro_range);
  gyro.setFullScaleAccelRange(Accel_range);
  gyro_connecte = gyro.testConnection();
  Serial.println(gyro_connecte ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Moteurs à l'arret au debut du programme
  analogWrite(10, 0);
  digitalWrite(11, HIGH);
  analogWrite(10, 0);
  digitalWrite(11, HIGH);

  // Juste un rappel, ça ne coute rien
  Z_Variable::indiceDeReference = 1;
  

  // Tableau d'objectifs, càd des points à atteindre successivement
  // Ici un polygone régulier à n sommets
  for (int i=0; i<n; i++)
  {
    tableau_obj[i][0] = 0.5 - 0.5 * cos ( 2.*i*PI/n );
    tableau_obj[i][1] = 0.5 * sin ( 2.*i*PI/n ) + 0.5;
  }
  indice_actuel =0;
  X_obj = tableau_obj[indice_actuel][0];
  Y_obj = tableau_obj[indice_actuel][1];
  distance_obj = sqrt(sq(X_obj - X_debut)+sq(Y_obj - Y_debut));
  
  /*
  // Entete com Serie, pratique pour un graphe excel
  Serial.print("t"); Serial.print("\t");
  Serial.print("X_soll"); Serial.print("\t");
  Serial.print("Y_soll"); Serial.print("\t");
  Serial.print("X_ist"); Serial.print("\t");
  Serial.print("Y_ist"); Serial.print("\t");
  Serial.print("obj_teta"); Serial.print("\t");
  Serial.print("cons_teta"); Serial.print("\t");
  Serial.print("teta_ist"); Serial.print("\t");
  Serial.print("obj_C"); Serial.print("\t");
  Serial.print("cons_C"); Serial.print("\t");
  Serial.println("C_ist");
  */
  // Pour le reglage des correcteurs
#ifdef DEBUG
  AxeC.Nouveau_mouvement(MODE_VITESSE, 0., 0.1);
  AxeTeta.Nouveau_mouvement(MODE_POSITION, 0., 0.0);
#endif
}

/* =================================== !!! LOOOOOOOOOOOOOOP !!! ===========================================*/
void loop() 
{
  temps_debut = millis();
  
  digitalWrite(13, led);
  led = !led;
  // test connection gyro i2c
  if(!gyro_connecte) return;
  // Reception Serie. selection du mode de pilotage, à coder

  // reception gyro
  digitalWrite(PWM_GAUCHE, LOW);
  digitalWrite(PWM_DROIT , LOW);
  //gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //gyro.getAcceleration(&ax, &ay, &az);
  //gyro.getRotation(&gx, &gy, &gz);
  gz = gyro.getRotationZ();
  //gyro.getTemperature()
  
  analogWrite(PWM_DROIT, PWM_droit);
  analogWrite(PWM_GAUCHE, PWM_gauche);
  
  
  d_inclinaison =  gz * gain_g - offset_gz;

  // lecture codeurs
  encodeur_gauche.update();
  encodeur_droit.update();

  // Mesure deplacement absolut. 
  //Remarquez la demi rotation, translation puis deuxieme demi rotation. Correction minime je pense
  double demi_rot = 0.5 * (Dteta_gauche * encodeur_droit.get_deplacement() + Dteta_droit * encodeur_gauche.get_deplacement());
  double deplacement = DX_gauche * encodeur_droit.get_deplacement() + DX_droit * encodeur_gauche.get_deplacement();
  Teta_abs   += demi_rot;
  X_abs      += deplacement * cos(Teta_abs);
  Y_abs      += deplacement * sin(Teta_abs);
  Teta_abs   += demi_rot;
  chemin_abs += deplacement;
  
  
#ifndef DEBUG

  int mode_deplacement(MODE_POSITION), mode_rotation(MODE_POSITION); // MODE_POSITION , MODE_VITESSE, MODE_POSITION_VITESSE
  // Coordonnées de l'objectif dans le repère local attaché au robot
  // Distance robot / objectif
  double dist = sqrt(sq(X_obj - X_abs)+sq(Y_obj - Y_abs));
 
  // Si objectif atteint, nouvel objectif
  if ( abs ( chemin_abs - consigne_deplacement)  < tolerance_distance)
  {
    chemin_int = d_old;
    d_old += distance_obj;
    indice_actuel ++;
    indice_actuel = ( indice_actuel==n)? 0 : indice_actuel;
    X_obj = tableau_obj[indice_actuel][0];
    Y_obj = tableau_obj[indice_actuel][1];
    dist = sqrt(sq(X_obj - X_abs)+sq(Y_obj - Y_abs));

    chemin_int = chemin_abs;
    
    rotation = true;
  }
  //consigne_deplacement = d_old + distance_obj;
  
  // Orientation de l'objectif par rapport à l'axe longitudinal du robot
  // X vers l'avant, Y vers la gauche, téta positif de X vers Y
  
  // Un peu de trigonometrie :
  double Dx = X_obj - X_abs;
  double Dy = Y_obj - Y_abs;
  if (Dx == 0)
  {
    eq_orientation = HALF_PI;
    if (Dy < 0) eq_orientation -= PI;
  }
  else
  {
    eq_orientation = atan2 ( Dy, Dx);
    if (Dx<0)
    {
      //eq_orientation = PI + eq_orientation;
      eq_orientation = (eq_orientation > PI)? eq_orientation - TWO_PI : eq_orientation;
      eq_orientation = (eq_orientation < -PI)? eq_orientation + TWO_PI : eq_orientation;
    }
  }

  // Calcul de la direction objectif (angle en rad) dans le ref local est normée entre -Pi et Pi
  teta_obj = eq_orientation - Teta_abs;
  while (teta_obj <= -PI) teta_obj += TWO_PI;
  while (teta_obj > PI) teta_obj -= TWO_PI;
  
  // Stratégie de positionnement : tourner puis avancer, quand l'ojectif est "en ligne de mire" (orientation inclue dans + ou - tolerance orientation)
  if (rotation)
  {
    if ( abs(teta_obj) < tolerance_orientation) // fin de la rotation, initialisation deplacement
    {
      rotation = false;
      chemin_int = chemin_abs;
      distance_obj = sqrt(sq(X_obj - X_abs)+sq(Y_obj - Y_abs));
      consigne_deplacement = chemin_int + distance_obj;
      mode_deplacement = MODE_POSITION;
      mode_rotation = MODE_POSITION;
      
    }
    else
    {
      mode_deplacement = MODE_VITESSE;       //ici vitesse nulle
      mode_rotation = MODE_POSITION;
    }
  }
  else
  {
    mode_deplacement = MODE_POSITION;
    mode_rotation = MODE_POSITION;
  }
  
  
  // J'utilise provisoirement deux instance de la classe Axe pour transformer les coordonnées objectif en une consigne variant continuement, pour la rotation et pour la translation
  AxeC.Nouveau_mouvement(mode_deplacement, consigne_deplacement, 0.);
  AxeTeta.Nouveau_mouvement(mode_rotation, eq_orientation, 0.);

#endif //NO DEBUG

#ifdef DEBUG
  delay(2);
#endif 
  
  AxeC.update();
  AxeTeta.update();
  
  
  if (compteur == 0)
  {
    /*
    Serial.print("t");         Serial.print("\t");
    Serial.print("obj_teta");  Serial.print("\t");
    Serial.print("cons_teta"); Serial.print("\t");
    Serial.print("teta_ist");  Serial.print("\t");
    Serial.print("obj_C");     Serial.print("\t");
    Serial.print("cons_C");    Serial.print("\t");
    Serial.println("C_ist");   Serial.print("\t");
    
    Serial.print(millis()/1000.);
    Serial.print("\t");
    Serial.print(X_obj);
    Serial.print("\t");
    Serial.print(Y_obj);
    Serial.print("\t");
    Serial.print(X_abs);
    Serial.print("\t");
    Serial.print(Y_abs);
    Serial.print("\t");
    Serial.print(eq_orientation);
    Serial.print("\t");
    Serial.print(AxeTeta.get_consigne_vit());
    Serial.print("\t");
    Serial.print(Teta_abs);
    Serial.print("\t");
    Serial.print(teta_obj);
    Serial.print("\t");
    Serial.print(consigne_deplacement);
    Serial.print("\t");
    Serial.print(AxeC.get_consigne_pos());
    Serial.print("\t");
    Serial.println(distance_parcourue);
    */
    /*Serial.print("a/g:\t");
    Serial.print(ax * gain_a); Serial.print("\t");
    Serial.print(ay * gain_a); Serial.print("\t");
    Serial.print(az * gain_a); Serial.print("\t");
    Serial.print(gx * gain_g - offset_gx); Serial.print("\t");
    Serial.print(gy * gain_g - offset_gy); Serial.print("\t");*/
    Serial.println(gz * gain_g - offset_gz);
    //Serial.println(gyro.getTemperature()/340.+36.53);
  }


  // indiceDeReference permet l'iteration càd la mise à jour des variables Z
  Z_Variable::indiceDeReference++;
/*
  PWM_gauche = round( RegPosition.get_valeur() - RegOrientation.get_valeur());
  PWM_droit =  round( RegPosition.get_valeur() + RegOrientation.get_valeur());
*/
  int truc = RegPosition.get_valeur() - RegOrientation.get_valeur();
  PWM_gauche = round( RegInclinaison.get_valeur());
  PWM_droit = PWM_gauche;
  
  if (PWM_gauche >=0)
  {
    digitalWrite(DIR_GAUCHE, HIGH);
  }
  else
  {
    PWM_gauche = -PWM_gauche;
    digitalWrite(DIR_GAUCHE, LOW);
  }
  if (PWM_droit >=0)
  {
    digitalWrite(DIR_DROIT, HIGH);
  }
  else
  {
    PWM_droit = -PWM_droit;
    digitalWrite(DIR_DROIT,LOW);
  }
  analogWrite(PWM_DROIT, PWM_droit);
  analogWrite(PWM_GAUCHE, PWM_gauche);
  
  while(millis()-temps_debut <= temps_cycle_mini)
  {
    
  }

  //compteur = (compteur > 100)? 0 : compteur + 1;

}


/*void serialEvent()
 {
 while (Serial.available())
 {
 // get the new byte:
 char inChar = (char)Serial.read(); 
 // add it to the inputString:
 inputString += inChar;
 // if the incoming character is a newline, set a flag
 // so the main loop can do something about it:
 if (inChar == '\n')
 {
 stringComplete = true;
 //vSoll = inputString.toInt();
 }
 }
 }
 */




