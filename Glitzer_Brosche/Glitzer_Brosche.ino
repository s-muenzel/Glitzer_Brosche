// Brauchen wir um mit den Objekten reden zu koennen
#include <Wire.h>
// Generelle Unterstuetzung fuer Sensoren
#include <Adafruit_Sensor.h>
// unser (spezieller) Sensor
#include <Adafruit_LSM303_U.h>
// die 7 Pixel auf der Vorderseite
#include <Adafruit_NeoPixel.h>
// der einzelne Pixel auf der Rueckseite
#include <Adafruit_DotStar.h>

// OO-Ansatz mit den folgenden Klassen:
// Sensor: liest die Werte aus dem Sensor und prueft auf gefundene Aktionen
// Pixel_Strip: enthaelt alle Pixel (die 7 Pixel auf der Vorderseite
// Pixel: Pixel vorne, macht alles was blinken angeht
// Pixel_Hinten: Das einzelne Pixel auf der Rueckseite

// Hauptprogramm:
// Setup: alles vorbereiten
// Loop: Jedes Objekt triggern.
//		 1) Sensor auslesen und auf "Ereignisse" pruefen
//		 2) Je nach Ergebnis Aenderung anstossen:
//         - nix: kein neuer Sensorwert oder keine Aktion gefunden
//         - Tipp: Pixel_Strip "Tipp" anstossen
//         - Bewegt: Pixel_Strip ein freies Pixel blinken lassen
//		 3) Pixel_Strip updaten (Helligkeitsaenderungen)
//		 4) Pixel_Hinten updaten (Helligkeitsaenderungen)


// Hier sind alle generell wichtigen Werte:
#define SCHWELLWERT		10	// Quadrat der Laenge der Vektor-Aenderung, ab der ein Flackern passiert [(m s^2)^2]
#define SENSOR_ZEIT		100 // wie lange zwischen Auswertungen des Beschleunigungssensors [ms] --> genutzt in Klasse Sensor
#define TICK			5   // wie lange ist die Pause, aber auch die Schrittdauer beim Auf- bzw Abblenden [ms] --> genutzt in Klasse Pixel

#define PIXEL_ANZAHL	7 // Wieviele Pixel sind (vorne) vorhanden
#define PIXEL_HW_PIN	1 // An welchem PIN sind sie angeloetet

#define DATAPIN    3      // Das Pixel hinten braucht 2 PINs
#define CLOCKPIN   4      // Das Pixel hinten braucht 2 PINs

// Hier die Farben, die beim Blinken gezeigt werden sollen
//     R    G    B
static const uint8_t Lieblingsfarben[][3] = {
	{200,   0, 200},   // Lila
	{240,   0,   0},   // Rot
	{200, 200, 200},   // Weiss
};


class Pixel_Hinten {
private:
  Adafruit_DotStar RueckPixel = Adafruit_DotStar(  1, DATAPIN, CLOCKPIN);
public:
  Pixel_Hinten() {
    RueckPixel.begin(); // Initialize pins for output
    RueckPixel.show();  // Turn all LEDs off ASAP  
  }

  void Waagrecht(uint8_t Wieviel) {
    if (Wieviel == 255) {
      RueckPixel.setPixelColor(0,0x00200000);
    } else {if (Wieviel > 64) { // Wieviel liegt zwischen 0 und 255. 255 heisst, TIPP kann passieren
      uint8_t Rot = (Wieviel-64)/8; // mit ein bisschen Rot anfangen, dann Rot immer weiter ausblenden
      RueckPixel.setPixelColor(0, (Rot<<8) );
    } else
      RueckPixel.setPixelColor(0,0);
    }
    RueckPixel.show();  // Turn all LEDs off ASAP
  }

  void Update(unsigned long Zeit) {
  }
};

class Pixel {
private:
	
	uint8_t Id;
	boolean Aktiv;
	Adafruit_NeoPixel *Strip; // damit das Pixel selbst die HW ansteuern kann

#define BLENDSCHRITTE	5 // in wie vielen Schritten werden die Pixel auf- UND abgeblendet
	unsigned long	Zeiten[2*BLENDSCHRITTE];
	uint8_t 		Rot[2*BLENDSCHRITTE],
					Gruen[2*BLENDSCHRITTE],
					Blau[2*BLENDSCHRITTE];
	
public:
	Pixel() {
	}
	
	void Initialisiere(Adafruit_NeoPixel *strip, uint8_t id) {
		Strip = strip;
		Id = id;
		Aktiv = false;
	}
	
	boolean Bin_Aktiv() {
		return Aktiv;
	}
	
	void Blinke(unsigned long Startzeit, double Beschleunigung) {
		// Eine der Lieblingsfarben auswaehlen
		int c = random(sizeof(Lieblingsfarben) / 3);
		int red = Lieblingsfarben[c][0] * min(1,Beschleunigung/500);
		int green = Lieblingsfarben[c][1] * min(1,Beschleunigung/500);
		int blue = Lieblingsfarben[c][2] * min(1,Beschleunigung/500); 

		// jetzt die Zeit-Leuchtwert-Punkte anlegen..
		// Erst in 5 Schritten aufblenden
		for (uint8_t x=0; x < BLENDSCHRITTE; x++) {
			Zeiten[x] = Startzeit + TICK*x;
			Rot[x] = (red * (x+1)) / BLENDSCHRITTE;
			Gruen[x] = (green * (x+1)) / BLENDSCHRITTE;
			Blau[x] = (blue * (x+1)) / BLENDSCHRITTE;
		}
		// Dann in 5 Schritten ausblenden
		for (uint8_t x=BLENDSCHRITTE; x < 2*BLENDSCHRITTE; x++) {
			Zeiten[x] = Startzeit + TICK*x;
			Rot[x] = (red * (2*BLENDSCHRITTE-1-x)) / BLENDSCHRITTE;
			Gruen[x] = (green * (2*BLENDSCHRITTE-1-x)) / BLENDSCHRITTE;
			Blau[x] = (blue * (2*BLENDSCHRITTE-1-x)) / BLENDSCHRITTE;
		}
		Aktiv = true; // das Pixel ist jetzt aktiv
	  }

	void Tipp(unsigned long Startzeit) {
		for (uint8_t x=0; x < 2*BLENDSCHRITTE; x+=2) { // wir fuellen jeweils in jede 2. Zelle Hell  ein
			Zeiten[x]     = Startzeit + 100*TICK*x; // Blinkfrequenz ist 100 mal groesser als TICK (z.B. 500 ms)
			Rot[x]        = 200;
			Gruen[x]      = 200;
			Blau[x]       = 200;
		}
		for (uint8_t x=1; x < 2*BLENDSCHRITTE; x+=2) { // und jetzt in die anderen 2. Zellen Dunkel 
			Zeiten[x]     = Startzeit + 100*TICK*x;
			Rot[x]        = 0;
			Gruen[x]      = 0;
			Blau[x]       = 0;
		}
		Aktiv = true; // das Pixel ist jetzt aktiv
	}
	  
	boolean Update(unsigned long Zeit) {

		if(Aktiv == false) // nicht aktiv --> nichts zu tun
			return false;  // nix getan, return false

		boolean Neuer_Wert = false;
		boolean Ist_Aktiv = false;
			
		// durch den Array gehen und schauen ob ein neuer Helligkeitswert gesetzt werden muss
		for(uint8_t i=0;i<2*BLENDSCHRITTE;i++) {
			if(Zeiten[i] != 0) {
				if(Zeit > Zeiten[i]) {
					// Wert gefunden
					// HW auf neuen Wert setzen
					Strip->setPixelColor(Id, Strip->Color(Rot[i], Gruen[i], Blau[i]));
					// Merken, dass wir etwas veraendert haben
					Neuer_Wert = true;
					// Eintrag in Array "abgearbeitet", also leeren
					Zeiten[i] = 0;
				}
				Ist_Aktiv = true; // zumindest war mindestens ein Zeit/Farb-Paar vorhanden (also ist das Pixel aktiv)
				break;
			}
		}
		// Wenn alle Werte abgearbeitet wurden, wird das Pixel inaktiv
		if(!Ist_Aktiv)
			Aktiv = false;
		
		return Neuer_Wert;
	}
};


class Pixel_Strip {
	
private:
	// das ist der gesamte Strip, d.h. alle LEDs haengen an dem Strip.
	Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_ANZAHL /* Anzahl der LEDs */, PIXEL_HW_PIN /* an welchem PIN haengen sie */);
	// Jede LED ist ein eigenes Objekt
	Pixel Pixel_Array[PIXEL_ANZAHL];
	
	
	uint8_t Freies_Pixel() {
		// durch alle Pixel gehen und schauen ob einer nicht aktiv ist.
		uint8_t P = random(PIXEL_ANZAHL); //  wir wollen aber nicht immer das erste bekommen, bei einen zufaelligen Pixel anfangen
		for (uint8_t x=0;x<PIXEL_ANZAHL;x++) {
			P = (P+1)%PIXEL_ANZAHL; // damit wird P auf 0 zurueckgesetzt wenn wir bei PIXELANZAHL angekommen sind
			if (!Pixel_Array[P].Bin_Aktiv())
				return P;
		}
		return 255;
	}
	
public:
	Pixel_Strip() {
		// die Hardware initialisieren
		strip.begin();
		strip.show(); // noch nichts gesetzt: alles aus
		
		// Unsere Pixel initialisieren
		for (uint8_t x=0;x<PIXEL_ANZAHL;x++) {
			Pixel_Array[x].Initialisiere(&strip,x);
		}
	}
	
	void Update(unsigned long Zeit) {
		boolean Neuer_Wert = false;
		for (uint8_t x=0;x<PIXEL_ANZAHL;x++) {
			if (Pixel_Array[x].Update(Zeit))
				Neuer_Wert = true;
		}
		if (Neuer_Wert) // Nur wenn wir etwas geaendert haben reden wir mit der HW
			strip.show();
	}
	
	void Blinke(unsigned long Zeit, double Beschleunigung) {
		uint8_t P = Freies_Pixel();
		if ( P != 255) // ein freies Pixel gefunden
			Pixel_Array[P].Blinke(Zeit, Beschleunigung);
	}

	void Tipp(unsigned long Zeit) {
		for(uint8_t i=0;i<PIXEL_ANZAHL;i++) // Bei Tipp ueberschreiben wir alle Pixel, ob aktiv oder nicht
			Pixel_Array[i].Tipp(Zeit);
	}
	
};

class Sensor {
private:
// DEBUG ONLY
    uint8_t Debug_Schreiber = 0;
    void Debug_Schreiben(double x, double y, double z, unsigned long Zeit, double dx, double dy, double dz, double Delta) {
		if (Debug_Schreiber > 0){
			Debug_Schreiber--;
			Serial.print("X: "); Serial.print(x);
			Serial.print(" Y: "); Serial.print(y);
			Serial.print(" Z: "); Serial.print(z);
			Serial.print(" Waagrecht seit: "); Serial.print(Zeit);
			Serial.print(" D_X: "); Serial.print(dx);
			Serial.print(" D_Y: "); Serial.print(dy);
			Serial.print(" D_Z: "); Serial.print(dz);
			Serial.print(" Delta: "); Serial.print(Delta);
			Serial.println();
		}
    }
    void Debug_Triggern() {
      Debug_Schreiber = 4;
    }
    
#define TIPP_ZEIT 5000 // wie viele Millisekunden muss der Sensor waagrecht auf dem Ruecken liegen, bevor wir einen "Tipp" detektieren

	Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
	double alt_x, alt_y, alt_z; // Hier merken wir uns den vorherigen Beschleunigungswert
	unsigned long Sensor_Zeit; // Wann haben wir das letzte Mal den Sensor gelesen
	unsigned long Waagrechte_Zeit; // Wie lange ist der Sensor schon waagrecht auf dem Ruecken
	
public:
	Sensor() {
	}

  void Initialisiere() {// HW initialisieren
		if (!accel.begin())
		{
			Serial.println("LSM303 Sensor nicht gefunden");
			while (1) {
				digitalWrite(13,HIGH); // Das ist noch eine LED, allerdings einfarbig
				delay(100);
				digitalWrite(13,LOW);
				delay(100);
			}// Hier koennten wir LED 13 blinken lassen als Fehlermeldung..
		}

		alt_x = 0; // Letzten Vektor auf Null initialisieren
		alt_y = 0;
		alt_z = 0;

		Sensor_Zeit = 0; // sofort neuen Sensorwert lesen

	}
	
	typedef enum { NICHT_GELESEN, NIX, BEWEGT, TIPP } Aktionen;
	
	Aktionen  Check(unsigned long Zeit, double *Beschleunigung,  uint8_t *Waagrecht) {
		
		*Waagrecht = min(255,255*(Zeit - Waagrechte_Zeit)/TIPP_ZEIT);

		// Immer wenn SENSOR_ZEIT vergangen ist, einen neuen Wert lesen
		if (Sensor_Zeit <= Zeit) { // Zeit für einen neuen Sensor-Wert 
		    Sensor_Zeit = Zeit + SENSOR_ZEIT; // die naechste Messung erst nach SENSOR_ZEIT Millisekunden
			double x,y,z; // aktueller Vektor
			double diff_x, diff_y, diff_z; // Differenzvektor
			
			// Lese neue Werte
			sensors_event_t event; 
			accel.getEvent(&event);
			x = event.acceleration.x;
			y = event.acceleration.y;
			z = event.acceleration.z; 

			// Jetzt muessen wir rausfinden, was fuer eine Bewegung wir detektieren
			
			// Gesucht ist der Betrag der Aenderung, d.h. |Neuer Vektor - Alter Vektor|
			diff_x = x - alt_x;
			diff_y = y - alt_y;
			diff_z = z - alt_z;
			
  			// Jetzt die Laenge des Differenzvektors berechnen
			// Eigentlich muesste man die Wurzel nehmen, aber um Rechenaufwand zu sparen, vergleichen wir das Quadrat des Vektors
			double Delta = diff_x*diff_x + diff_y*diff_y + diff_z*diff_z;
			*Beschleunigung = Delta; // der Wert wird ausserhalb gebraucht..
    
			alt_x = x; // Jetzt den aktuellen Wert als alten Wert merken
			alt_y = y;
			alt_z = z;
  
			if ((Delta > SCHWELLWERT) && (Zeit - Waagrechte_Zeit >= TIPP_ZEIT)) { // TIPP
				Debug_Triggern();
				Debug_Schreiben(x,y,z,Zeit - Waagrechte_Zeit, diff_x,diff_y,diff_z,Delta);
				Serial.print("Tipp entdeckt: "); Serial.println(z);
				Waagrechte_Zeit = Zeit; // Nur als Sicherheit, dass wir nicht faelschlicherweise mehrfach TIPP melden
				return TIPP;
			} else if (Delta > SCHWELLWERT) { // BEWEGT
				Debug_Triggern();
				Debug_Schreiben(x,y,z,Zeit - Waagrechte_Zeit, diff_x,diff_y,diff_z,Delta);
				Waagrechte_Zeit = Zeit; // Nur als Sicherheit, dass wir nicht faelschlicherweise mehrfach TIPP melden
				Serial.print("Schwelle entdeckt: "); Serial.println(Delta);
				return BEWEGT;
			} else {
				// kein TIPP und kein BEWEGT gefunden.
				// Pruefen, ob die Brosche (noch) waagrecht liegt.
				//   Als Sensorwert  bedeutet das, dass x und y klein sind und z etwa Erdbeschleunigung (z.B. -1 < x,y < 1 und 9 < z < 11)
				//  Dazu merken wir uns, wann die Bedingung das letzt Mal NICHT erfuellt wurde
				if (!( (-2 < x) && (x < 2) && (-2 < y) && (y < 2) && (9 < z) && (z < 11)) ) {
					Waagrechte_Zeit = Zeit;
				}
				Debug_Schreiben(x,y,z,Zeit - Waagrechte_Zeit, diff_x,diff_y,diff_z,Delta);
				return NIX;
			}
		} else // keinen neuen Wert gelesen
			return NICHT_GELESEN;
	}
};

Pixel_Strip MeinePixel;
Pixel_Hinten MeinHinteresPixel;
Sensor MeinSensor;
		   
void setup() 
{
  Serial.begin(9600);
  Serial.println("Starte");

  MeinSensor.Initialisiere();
}

void loop() {
	unsigned long Jetzt = millis();
	double Beschleunigung;
	uint8_t Waagrecht;
	switch (MeinSensor.Check(Jetzt,&Beschleunigung,&Waagrecht)) {
	case Sensor::NIX: 
		// vielleicht haelt jemand die Brosche waagrecht und wartet, dass ein TIPP gemacht werdne kann.
		MeinHinteresPixel.Waagrecht(Waagrecht);
		break;
	case Sensor::BEWEGT:
		MeinePixel.Blinke(Jetzt, Beschleunigung);
		break;
	case Sensor::TIPP:
		MeinePixel.Tipp(Jetzt);
		break;
	case Sensor::NICHT_GELESEN:
	default:
		break;
	}
	MeinePixel.Update(Jetzt);
	MeinHinteresPixel.Update(Jetzt);
}


