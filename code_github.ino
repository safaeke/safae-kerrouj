#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

const char* ModeOptions[2] = {"   Mode Four ", " Mode Air Fry "};
const char* Type[6] = {"Bake", "ROAST", "GRILL", "Air Fry"};
int consigne[21] = {40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240};

int MaxT = 21;   // Indice max du tableau des températures
int MaxC = 6;    // Indice max du tableau de type de cuisson
int MaxR = 4;    // Indice max du tableau de Relais
int Consigne;
int Relais[4] = {1, 2, 3, 4};
int Rel = 0;
int GO = 0;                         // Variable pour OK
int VALID = 0;                      // Variable validation
int OK = 4;                         // Entrée validation digitale 4
int START = 0;                      // Variable démarrage du programme
int sens;                           // Sens de rotation du bouton
int incrementB = 2;                 // Entrée selection B digitale 2
int incrementA = 3;                 // Entrée selection A digitale 3
int codeurB;                        // Variable entrée B
int codeurA;                        // Variable entrée A
int codeurAt0 = 1;                  // Entrée A-1


float erreur1, erreur2, erreur3, erreur; // erreur de l'asservissement en °C de CTN1 et CTN2

long int debut0;                // Temps de boucle à t=0
long int debut;                 // Temps de boucle
int cons = 3;                   // Consigne de température par défaut

int Com_CTN1 = A0;               // commande lecture CTN1 Gauche sur A0
int Com_CTN2 = A1;               // commande lecture CTN2 Droite sur A1
int CTN = A3;                    // entrée analogique 3 pour les 3 CTNs
int RelaisG = 7;                     // EC Gauche
int RelaisD = 6;                     // EC Droite
int RelaisB = 9;                     // EC bas
int Relais4 = 8;
int Target;

// Paramètres PID sans ventilateur
float KP = 0.3;
float KI = 0.5;
float KD = 0.1;

// Paramètres PID ventilo
float KPV = 1.5;
float KIV = 1;
float KDV = 0;

float Sp;                       // Temps proportionnel
float Si = 0;                   // Temps intégral
float Sd;                       // Temps dérivée
int Pas = 1000;                                                                              // Pas de l'asservissement en ms (1s)
float error[30] = {0};          // Tableau erreur (30 données)

float error1[30] = {0};          // Tableau erreur (30 données)
float error2[30] = {0};          // Tableau erreur (30 données)
float error3[30] = {0};          // Tableau erreur (30 données)
int ST;                         // Nombre de pas de conduction
int Tc = 60 ;                   // Temps de conduction

float Tempr1, Tempr2;           // température en °C de CTN1 et CTN2
float TemprG;

float TemprI ;

float UCTN;                     // Valeur CTN en volt 1024=5 volt
float UAlim;                // Valeur tension d'alimentation
float UCTNm;                // valeur de UCTN moyennée
int m, n;
int tempo, temps;
float RCTN1, RCTN2;

float Rctn[151] = {158112,144080,130049,117721,107097,96473,88365,80256,73084,66849,60613,55783,50953,46654,42887,39119,36160,33201,30553,28215,25877,24017,22158,20484,18997,17510,16313,15116,14034,13066,12098,11311,10524,9809,9166,8523,7995,7466,6984,6548,6112,5751,5390,5059,4758,4457,4205,3954,3722,3511,3300,3122,2944,2780,2629,2479,2351,2223,2105,1996,1887,1794,1701,1614,1535,1455,1386,1317,1253,1194,1135,1083,1032,984,939,895,856,817,780,746,713,683,653,625,599,573,550,527,505,485,465,447,429,412,396,381,366,352,339,327,314,303,292,281,271,261,252,243,235,227,219,211,204,197,191,184,178,172,167,161,156,151,147,142,138,133};
int Temp[151] = {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,126,128,130,132,134,136,138,140,142,144,146,148,150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,188,190,192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,232,234,236,238,240,242,244,246,248,250};

// Variables to store selected mode, type, and temperature
int mode = 0;  // 0 for Mode Four, 1 for Mode Air Fry
int type = 0;
int Temperature1 = 0;
int Temperature2 = 0;
int Selection = 0;  // 0 pour  mode, 1 pour type/temperature1, 2 pour temperature2


void setup() {
  lcd.init();                         // initialize the lcd
  lcd.backlight();                    // Activation du backlight
  pinMode(OK, INPUT);                 // Definition OK en input
  pinMode(incrementA, INPUT);         // Definition incrementA en input
  pinMode(incrementB, INPUT);         // Definition incrementB en input
  pinMode(Com_CTN1, OUTPUT);          // Definition Com_CTN1 en output
  pinMode(Com_CTN2, OUTPUT);          // Definition Com_CTN2 en output
  pinMode(RelaisG , OUTPUT);          // Definition Relais en output
  pinMode(RelaisD , OUTPUT);          // Definition Relais en output
  pinMode(RelaisB , OUTPUT);          // Definition Relais en output
  pinMode(Relais4 , OUTPUT);          // Definition Relais en output
  
  Serial.begin(9600);                 // Vitesse liaison série.
  debut0 = millis(); 
  lcd.setCursor(1, 0);
  lcd.setCursor(0, 0);
  lcd.print("Mode Principal:");
  lcd.setCursor(0, 1);
  lcd.print(ModeOptions[selectedMode]);
}

void Start()                //détection d'un appui sur Start.
{
  GO = digitalRead(OK);
  //Serial.println(GO);
  if (GO==1)
  {  
    delay (100);
    GO = digitalRead(OK);      //antirebond de 100ms
    if (GO==1)
    {
        if (VALID==0)
        {
          VALID=1;
        }
        else
        {
          VALID=0;          
        }         
    delay (900);  
    GO = digitalRead(OK);      //détection appui long
    if (GO == 1)
    {
        if (VALID==0)
        {
          VALID=1;
        }
        else
        {
          VALID=0;          
        }

        if (START==0)
        {
          START=1;
        }
        else
        {
          START=0;          
        }  
    }
  }
}
}

void Temperature()                          //Sous  programme de détection rotation sur le bouton selection
{
  codeurA = digitalRead(incrementA);
  codeurB = digitalRead(incrementB);
  if (codeurA < codeurAt0)
  {
    delay (10);
    codeurA = digitalRead(incrementA);      //antirebond de 10 ms
    if (codeurA == 0)
    {
      if (codeurB == 1)
      {
        if (cons >= MaxT)                   // Valeur maximale des consignes
        {
          cons = MaxT;
        }
        else
        {
          cons = cons + 1;
        }
      }
      else
      {
        if (cons <= 0)
        {
          cons = 0;
        }
        else
        {
          cons = cons - 1;
        }
      }  
    }
  }
  codeurAt0 =codeurA;
 lcd.setCursor(11, 0);
 lcd.print(Type[type]);
  lcd.setCursor(11, 1);
  if((millis()-debut0)<300) {lcd.print("   ");}
  else if((millis()-debut0)<1000){lcd.print(consigne[cons]); }//                 Clignotement de la donnée.
        else(debut0=millis());
}


void TypeCuisson()                          //Sous  programme de détection rotation sur le bouton selection
{
  codeurA = digitalRead(incrementA);
  codeurB = digitalRead(incrementB);
  if (codeurA < codeurAt0)
  {
    delay (10);
    codeurA = digitalRead(incrementA);      //antirebond de 10 ms
    if (codeurA == 0)
    {
      if (codeurB == 1)
      {
        if (type >= MaxC)             
        {
          type = MaxC;
        }
        else
        {
          type = type + 1;
        }
      }
      else
      {
        if (type <= 0)
        {
          type = 0;
        }
        else
        {
          type = type - 1;
        }
      }       
    }
  }
  codeurAt0 = codeurA;
    lcd.setCursor(11, 1);
  lcd.print(consigne[cons]); 
  if((millis()-debut0)<300) {lcd.setCursor(11,0);lcd.print("      ");}
  else if((millis()-debut0)<1000){lcd.setCursor(11,0);lcd.print(Type[type]); } //                 Clignotement de la donnée.
       else(debut0=millis());
}
void Conversion(float RCTN, int capteur) {
  

  int i = 1;
  
  while (!(RCTN < Rctn[i] && RCTN >= Rctn[i + 1])) {
    i++;
  }

  
  if (RCTN < Rctn[i] && RCTN >= Rctn[i + 1]) {          // la méthode d'interpolation 
    
    TemprI = Temp[i] + ((Temp[i + 1] - Temp[i]) / (Rctn[i + 1] - Rctn[i])) * (RCTN - Rctn[i]);
    
  } else if (RCTN == Rctn[i]) {
    
    TemprI = Temp[i];
  }

  if (capteur == 1) {
    
    Tempr1 = TemprI;
    
  }
  else if (capteur == 2) {
    Tempr2 = TemprI;
  }
}

void mesure(int capteur) {
  
  int i = 1;
  digitalWrite(Com_CTN1, LOW);
  digitalWrite(Com_CTN2,LOW); 
  delay(10); 
  Acquisition();
  UAlim = UCTNm ;//Alim = UCTNm ;

  if (capteur == 1) {       // Choix du capteur 1 pour CTN1 ou Capteur 2 pour CTN2
  
  digitalWrite(Com_CTN1, HIGH);
  digitalWrite(Com_CTN2, LOW);
  delay(10); 
  Acquisition();
  UCTN = UCTNm;
  if(UAlim>UCTN)
  { 
  RCTN1 = (4750 * UCTN) / (UAlim - UCTN);
 
  }
  else
  {
  RCTN1=9999;
  }
  Conversion(RCTN1, capteur);
  }

  else if (capteur == 2) {
    
    digitalWrite(Com_CTN1, LOW);
    digitalWrite(Com_CTN2, HIGH);
    Acquisition();
    delay(10); 
  Acquisition();
  UCTN = UCTNm; 
  if(UAlim>UCTN)
  { 
  RCTN2 = (4750 * UCTN) / (UAlim - UCTN);
 
  }
  else
  {
  RCTN2=9999;
  }
  Conversion(RCTN2, capteur);
  }
}

void Acquisition() //mesure de la tempèrature et moyennage sur 10 valeurs
{
  m = 0;
  UCTNm = 0;
  while (m < 1) 
  {
    UCTN = analogRead(CTN);
    UCTNm = UCTNm + UCTN;
    m++;
  }  
}

void PID (float *error) //Calcul des corrections PID soit ST
{
  // PID Parallèle
  if ( type == 1 || type == 3 || type == 4)
  {
  Sp= KP * error[0];                                           //Calcul du temps proportionnel
  Sd= KD *(error[0]-error[29]);  
  Si = Si + (KI / 1000 * ((error[0] + error[1]) / 2));           //Calcul du temps intégral  
                                                 

  ST = Sp+Si+Sd;       // calcul du temps total
  ST = round(ST);     //Arrondi le plus proche
 
  }
  else if ( type == 0 || type == 2 || type == 5) {
  Sp= KPV * error[0];                                           //Calcul du temps proportionnel
  Sd= KDV *(error[0]-error[29]);  
  Si = Si + (KIV / 1000 * ((error[0] + error[1]) / 2));           //Calcul du temps intégral  
                                                 

  ST = Sp+Si+Sd;       // calcul du temps total
  ST = round(ST);     //Arrondi le plus proche
 
    
  }
  if (ST > (Tc - 2))                                              //Bornage du temps max
      {
        ST = Tc ;
      }
  if (ST < 2)                                                          //0 si ST<2 soit 2secondes pour eviter les une commutation du relais trop courte.
  {
    ST = 0;
  }
   }
   void remplirErreur( float erreur){
  
   int d = 29;
    while (d > 0) {
        error[d] = error[d - 1];
        d--;
    }
    error[0] = erreur;
}

void Display()                    //Affichage des valeurs
{
  Serial.print(F("  "));
 Serial.print(temps);
 Serial.print(F("  "));
 Serial.print(n);
 Serial.print(F("   CTNG: "));
 Serial.print(Tempr1);
 Serial.print(F("   CTND: "));
 Serial.print(Tempr2);
 Serial.print(F("   CTNB: "));
 Serial.print(TemprG);
 Serial.print(F("   Target "));
 Serial.print(Consigne);
 Serial.print(F("  Pas: "));
 Serial.print(ST);
 Serial.print(F("  Relais:  "));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("CTNG CTND MODE "));

  lcd.setCursor(0, 1);
  lcd.print(int(Tempr1));

  lcd.setCursor(2, 1);
  lcd.printByte(223);
  lcd.setCursor(3, 1);
  lcd.print("C ");
  lcd.setCursor(5, 1);
  lcd.print(int(Tempr2));
  lcd.setCursor(7, 1);
  lcd.printByte(223);
  lcd.setCursor(8, 1);
  lcd.print("C ");
  lcd.setCursor(10, 1);
  lcd.print(Type[type]);
  }
    void Boucle (int Relais1 , int Relais2, float erreur){

       while (n < ST)                        // partie relais On
    {
      debut0= millis();                   // mise en memoire de t0
      digitalWrite(Relais1, HIGH);
      digitalWrite(Relais2, HIGH);
      
      mesure(1);                           // mesure de la tempèrature. 
      mesure(2);  
      remplirErreur(erreur);
      PID(error);                              // calcul du temps de conduction ST
      Display();
      n++;
      Serial.println(F("   Relais B H: 10 "));
   
      temps = temps + (Pas/1000); 
      debut=millis();      
       while((debut-debut0)<Pas)          //Attente d'un seconde
          {
              debut=millis();
          }
    }
 
 while (n < 60)                    // partie relais Off
    { 
      debut0= millis();      
      digitalWrite(Relais1, LOW);
      digitalWrite(Relais2, LOW);
 
      mesure(1);// mesure de le tempèrature.
      mesure(2);      
      remplirErreur(erreur);
      PID(error);          
      Display();
      n++;
     
      Serial.println(F("  Relais B H : 0"));
      //delay(1000);
      temps = temps + (Pas/1000);
      debut=millis();   
      while((debut-debut0)<Pas)          //Attente d'un seconde
          {
               debut=millis();
          }
    }
}
void loop() // Boucle principale
{

   Start();
    if (START == 0)                      // Mesure bouton Start
  {          
      if (VALID==0)
      {
    
      TypeCuisson();  // Scrutation rotacteur réglage puissance

      }
      else 
      {       
      
      Temperature();
      
      }   

     digitalWrite(RelaisG, LOW);
     digitalWrite(RelaisD, LOW);
     digitalWrite(RelaisB, LOW);
     digitalWrite(Relais4, LOW);
  }
  else  {
    
   if ( type == 0) {  
    TemprG = ( Tempr1 + Tempr2) / 2;
    Consigne = consigne[cons];
    erreur = Consigne - TemprG;
    mesure(1);
    remplirErreur(erreur);
    mesure(2);
    delay(1000);
    PID(error);
    Boucle(RelaisB,Relais4,erreur);
  }

else if ( type == 1 ){
  TemprG = ( Tempr1 + Tempr2) / 2;
  Consigne = 0.88*consigne[cons]+5.36;
  erreur = Consigne - TemprG;
   mesure(1);
   remplirErreur(erreur);
  mesure(2);
  delay(1000);
  PID(error);
  Boucle(Relais4 , RelaisB , erreur);
}

else if ( type == 2 ){
   TemprG = ( Tempr1 + Tempr2) / 2;
  Consigne = consigne[cons];
  erreur = Consigne - TemprG;
   mesure(1);
  remplirErreur(erreur);
  mesure(2);
  delay(1000);
  PID(error);
  Boucle(RelaisG , RelaisD, erreur);
}

else if ( type == 3 ){
   TemprG = ( Tempr1 + Tempr2) / 2;
  Consigne = consigne[cons];
  erreur = Consigne - TemprG;
   mesure(1);
  remplirErreur(erreur);
  mesure(2);
  delay(1000);
  PID(error);
  Boucle(RelaisG , RelaisD, erreur);
}
  }
