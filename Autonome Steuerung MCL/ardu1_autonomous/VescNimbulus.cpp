#ifndef VESCNIMBULUS
#define VESCNIMBULUS

//  VescNimbulus.cpp
//  Created by Thomas Zimmermann on 25.07.24.
//  Objektiorientierter Ansatz VESC-Controller anzusteuern,
//  damit Programmcode in main() vereinfacht werden kann

#include <Arduino.h>
// VescUart ist fuer UART-Protokoll von VESC
#include <VescUart.h>

// Defines der VESC-IDs
// #define HL 0
// #define HR 1
// #define VL 2
// #define VR 3
// Baudrate der VESCs
#define baudrate 115200

class VescNimbulus {
private:
  // Globale Variablen
  int poles = 15;               // Polpaarzahl
  int erpmFactor = poles * 60;  // Umrechung ERPM in RPS
  // Deklarieren der eingebundenen Objekte
  // damit Klassenweit bekannt
  VescUart* vescuart;

public:
  // Konstruktor
  VescNimbulus() {
    // Erstelle das vescuart-Objekt
    vescuart = new VescUart();
  }

  // Initialisierungsmethode
  void init() {
    // initialisierung der seriellen Schnittselle zum VSEC 0
    Serial1.begin(baudrate);
    while (!Serial1)
      ;  // warten, bis UART initialierung fertig ist
    // initialisieren der Kommunikation mit VESC
    vescuart->setSerialPort(&Serial1);
  }

  // Methode zum Setzen von Drehzahlen in 1/s
  void setRPS(float rps0, float rps1, float rps2, float rps3)
  {
    vescuart->setRPM(rps0 * erpmFactor, 0);
    vescuart->setRPM(rps1 * erpmFactor, 1);
    vescuart->setRPM(rps2 * erpmFactor, 2);
    vescuart->setRPM(rps3 * erpmFactor, 3);
  }

  // Methode zum Erhalten genau einer Drehzahl in 1/s
  float getRPS(int vescID) {
    // hole die Werte des vesc-Controllers
    if (vescuart->getVescValues(vescID)) {
      // Gebe Drehzahl in 1/s zurueck
      return vescuart->data.rpm / erpmFactor;
    } else { // es ist keine Rueckgabe der Werte moeglich
      return 0.0 / 0.0;  // return NaN for errors
    }
  }
};

#endif  // VESCNIMBULUS
