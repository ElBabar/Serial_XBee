//Ce code est utilisé pour recevoir le Serial du Lidar et l'envoyer en XBee. 
//Cependant il est également capable de décoder des trames XBee reçues.

#define DEMOMODE 0 //le mode démo envoie une même trame en boucle. Pratique pour les tests.
#include <SoftwareSerial.h>
SoftwareSerial XBeeSerial(11,12); //RX, TX

bool isBusy = true; //activé si le module XBee est occupé. Ceci empêche le spam ed son port série, ce qui introduit une latence (buffer).
int error_watch = 0;  //compteur d'erreurs du module XBee. On reset si cette variable dépasse une certaine valeur
int busyCnt = 0;  //Compte le nombre de fois que le module est Busy d'affilée. Permet de détecter si lemodule a crashé (ça peut arriver...)
byte frame_number = 1;  //Le numéro de trame est indicatif, il permet de valider l'émission réussie (mode API du XBee...)
byte frame_header[17] = {0x7E, 0x00, 0x2E, 0x10, frame_number, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00}; //Broadcast
//byte frame_header[17] = {0x7E, 0x00, 0x18, 0x10, frame_number, 0x00, 0x13, 0xA2, 0x00, 0x41, 0x77, 0xE1, 0x89, 0xFF, 0xFE, 0x00, 0x00}; //XCTU
#if DEMOMODE
  byte frame_payload[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88}; //Demo mode
#else
  byte frame_payload[30];
#endif
byte frame_ID[2] = {'I', 'D'};  //ID par défaut, redéfinit dans le setup par les switchs
byte api_frame[100];  //déclaration de la frame reçue par le XBee. 100 car on ignore sa taille (on prend large). A améliorer.
byte received_ID[2];  //ID reçu par la trame XBee
byte received_payload[30]; //données reçues par la trame XBee

void checkSerial (void);  //lit le port série
void XBeeStatusCheck (void);  //vérifie que le module XBee est toujours up
void XBee_rst(void);  //Soft reset le module XBee
void receiveXBee (void);  //recoit une trame du module XBee
void decode_frame(void);  //décode une trame du module XBee
void sendXBee (void); //envoie une trame au module XBee
int scan_address_switch(void);  //scan des switchs d'adresse
int scan_channel_switch(void);  //scan des switchs de channel
void setChannel(int mode);  //redéfinit le channel radio du module XBee
void openATmode(void);  //Ouvre le mode de commande du mosule XBee
void closeATmode(void); //Ferme le mode de commaned du module XBee

void setup() {
  Serial.begin(57600); 
  XBeeSerial.begin(57600);
  delay(50);
  pinMode(2, INPUT_PULLUP); //pullups integrées associées aux switchs
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  int my_address = scan_address_switch(); //on récupère l'adresse en fonctions des switchs
  frame_ID[1] = my_address;
  frame_ID[0] = my_address>>8;
  int my_channel = scan_channel_switch(); //on récupère le channel en fonction des switchs
  setChannel(my_channel+48);//set channel prend un char en paramètre. +48 (correspondance tabme table ASCII)

}

void loop() {
  checkSerial();
  sendXBee();
  receiveXBee();
  XBeeStatusCheck();
}

void checkSerial (void){
  if(Serial.available()){
    int i = 0;
    while(Serial.available()){
      frame_payload[i]=Serial.read();
      //Serial.print(frame_payload[i]);
      delay(1);//nécessaire sinon on lit plus vite qu'on ne reçoit
      i++;
      if(i>=30){break;}//On ne travaille qu'avec des trames de 8 octets
    }
    //Serial.println(i);
    Serial.println(frame_payload[0]);
  }
}

void XBeeStatusCheck (void){
  if(error_watch>10){ //si on a accumulé plus de 10 erreurs, reset.
    //Un crash peut être dû au buffer XBee qui sature (côté ZigBee, pas Serial)
    //Le module peut ne pas répondre si il n'est pas correctement sorti du mode de commande.
    
    //Serial.println("XBEE RESET");
    XBee_rst();
    delay(1000);
    error_watch = 0;  //réinitialisation des erreurs
    while(XBeeSerial.available()){XBeeSerial.read();} //On vide les buffers Serial qui ont pu se remplir pendant le reset
    while(Serial.available()){Serial.read();}
    isBusy = false; //on libère le flag busy, au cas où il ait été mis à 0 avant.
  }
}

void XBee_rst(void){
  openATmode();
  while(XBeeSerial.available()){XBeeSerial.read();} //Le module renvoie "OK" à l'ouvertue
  while(!XBeeSerial.available()){//Commande déclenchant le reset
    XBeeSerial.write('A');
    XBeeSerial.write('T');
    XBeeSerial.write('F');
    XBeeSerial.write('R');
    XBeeSerial.write('\r');
    delay(200);
  }
}

void receiveXBee (void){
  if(XBeeSerial.available()){ //Si le XBee a quelque chose à nous dire
    int i = 0;
    while(XBeeSerial.available()){  //On récupère la trame API que nous donne le module XBee. On stocke dans api_frame. Chaque octet correspond à une valeur précise.
      api_frame[i]=XBeeSerial.read();
      i++;  //i contient par conséquent la longueur de la trame API
    }
    if(api_frame[3]==0x8B){ //frame type 0x8B : transmit status - équivalent ACK. Permet de ne pas spammer le port série du XBee.
      if(api_frame[8]==0x00){ //transmit status 0x00 : success
        //Serial.print("Frame ");
        //Serial.print(" sent successfully after ");
        //Serial.print(api_frame[7]); //nombre d'essais
        //Serial.println(" retries");
        for(int j=0; j<=i; j++){
          api_frame[j]=0; //on efface la trame API une fois qu'on a fini
        }
        #if !DEMOMODE
          for(int j=0; j<=8; j++){
            frame_payload[j]=0; //On efface les données reçues (si on est pas en mode DEMO, auquel cas on garde toujours la même payload).
          }
        #endif
      }
      else{ //Si le transmit status n'est pas un succès
        error_watch++;
        //Serial.print("Transmission failed, error : ");
        //Serial.println(api_frame[8]); //On renvoie l'erreur (ex. 2 = Collision)
        for(int j=0; j<=i; j++){
          api_frame[j]=0; //On nettoie la trame API.
        }
      }
    }
    else if (api_frame[3]==0x90){ //0x90 signale une réception de données par le XBee
      decode_frame(); //on décoe la trame reçue
    }
    isBusy = false; //On est sûr que le module est libre maintenant
  }
}

void decode_frame(void){
  //Serial.print("ID : ");
  for(int i=0; i<2; i++){
    received_ID[i] = api_frame[i+15];
    //Serial.print(char(received_ID[i]));
  }
  //Serial.print(" DATA : ");
  for(int i=0; i<30; i++){
    received_payload[i] = api_frame[i+17];
    Serial.print(char(received_payload[i]));
  }
  //Serial.println();
  for(int j=0; j<=api_frame[2]+4; j++){
      api_frame[j]=0;
  }
}

void sendXBee (void){ //création de la trame
  if(((!isBusy) || (busyCnt==0xFFFF)) && (frame_payload[0]!=0) && !XBeeSerial.available()){ //Si le module est dispo et qu'on a une payload à envoyer
    //Note : la valeur de BusyCount mise ici permet de forcer une communication avec le module une fois de temps en temps même si il est censé être Busy.
    //Cela permet depasser outre une éventuelle erreur de flag Busy, sans avoir à reset.

    byte checksum = 0;  //Initialisation du checksum
    if(frame_number == 0) {frame_number++;} //Pour ne pas avoir de nombre négatif en frame_number (rappel : valeur à titre indicatif)
    for (int i =0; i<3; i++){
      XBeeSerial.write(frame_header[i]);  //On envoie les trois premiers octets de la trame API, le checksum ne s'applique pas à ceux-ci
    }
    for (int i =3; i<17; i++){
      checksum += frame_header[i];
      XBeeSerial.write(frame_header[i]);  //On envoie le reste du header (qui est toujours le même car on envoie toujours 8 bits de la même manière) tout en gérant le checksum
    }
    for (int i=0; i<2; i++){
      checksum += frame_ID[i];
      XBeeSerial.write(frame_ID[i]);  //On envoie l'ID tout en gérant le checksum
    }
    for (int i=0; i<30; i++){
      checksum += frame_payload[i];
      XBeeSerial.write(frame_payload[i]); //On envoie la donnée tout en gérant le checksum
    }
    checksum = 0xFF - checksum; //Finalisation du checksum (c'est la formule donnée dans la datasheet)
    XBeeSerial.write(checksum); //On envoie le checksum
  isBusy = true;  //Le module est désormais occupé jusqu'à ce qu'il déclare qu'il a finit d'envoyer la trame (transmit status packet).
  busyCnt = 0;  //Le busycnt est remis à 0
  frame_number++; //Incrémentation du frame_number
    delay(50);
    while(Serial.available()){Serial.read();} //On vide le buffer Serial, pour ne pas avoir accumulé de latence pendant cette phase d'envoi.
  }
  else if (isBusy==true){ //Si le module n'est pas prêt à envoyer de trame
    busyCnt ++;
    if(busyCnt==0xFFFE){
      error_watch++;  //Si le module est Busy pendant trop longtemps, on ajoute une erreur au compteur
      //Serial.println("XBee busy...");
      isBusy = false; //On libère isBusy pour tenter de renvoyer de la donnée, au cas où le module ait planté où une mauvaise manipulation du flag Busy.
    }
  }
}

int scan_address_switch(void){  //Lecture des switchs d'adresse
  int value = 0;
  value = value + !digitalRead(16);
  value = value + !digitalRead(15) * 2;
  value = value + !digitalRead(14) * 4;
  value = value + !digitalRead(9) * 8;
  value = value + !digitalRead(8) * 16;
  value = value + !digitalRead(7) * 32;
  value = value + !digitalRead(6) * 64;
  value = value + !digitalRead(5) * 128;
  value = value + !digitalRead(4) * 256;
  value = value + !digitalRead(3) * 512;
  value = value + !digitalRead(2) * 1024;
  return value;
}

int scan_channel_switch(void){  //Lecture des switchs de channel radio
  int value = 0;
  value = value + !digitalRead(19);
  value = value + !digitalRead(18) * 2;
  value = value + !digitalRead(17) * 4;
  return value;
}

void setChannel(int mode){  //Paramétrage du channel radio
  openATmode(); //Ouverture du mode commande
  XBeeSerial.write('A');  //Commande permettant de modifier le channel d'émission
  XBeeSerial.write('T');
  XBeeSerial.write('C');
  XBeeSerial.write('M');
    
  switch (mode){  //Sélection des différents channels (3 channels de 200kHz adjacents) en fonction du mode
    case '0':
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('7');
    break;
    
    case '1':
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('3');
    XBeeSerial.write('8');

    break;
    
    case '2':
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('5');
    XBeeSerial.write('8');
    XBeeSerial.write('0');
    break;
    
    case '3':
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('3');
    XBeeSerial.write('8');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    break;
    
    case '4':
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('1');
    XBeeSerial.write('C');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    break;
    
    case '5':
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('E');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    break;
    
    case '6':
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('7');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    break;
    
    case '7':
    XBeeSerial.write('0');
    XBeeSerial.write('6');
    XBeeSerial.write('8');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    XBeeSerial.write('0');
    break;
    
    default:
    break;
  }
  XBeeSerial.write('\r');
  delay(500);
  XBeeSerial.write('A');  //Commande permettant de charger les changement en EEPROM
  XBeeSerial.write('T');
  XBeeSerial.write('W');
  XBeeSerial.write('R');
  XBeeSerial.write('\r');
  delay(500);
  XBeeSerial.write('A');  //Application des changements
  XBeeSerial.write('T');
  XBeeSerial.write('A');
  XBeeSerial.write('C');
  XBeeSerial.write('\r');
  delay(500);
  closeATmode();  //Quitter le mode de commande AT
}

void openATmode(void){  //Ouverture du mode AT
  while(XBeeSerial.available()){XBeeSerial.read();} //On vide le buffer SerialXBee
  while(!XBeeSerial.available()){ //Tant que le module ne répond pas OK, on continue à demander le mode AT
    XBeeSerial.write('+');  //Commande pour déclencher le mode AT
    XBeeSerial.write('+');
    XBeeSerial.write('+');
    delay(1200);
  }
}

void closeATmode(void){ //Fermeture du mode AT
  while(XBeeSerial.available()){XBeeSerial.read();}
  while(!XBeeSerial.available()){
    XBeeSerial.write('A');
    XBeeSerial.write('T');
    XBeeSerial.write('C');
    XBeeSerial.write('N');
    XBeeSerial.write('\r');
    delay(200);
  }
}


