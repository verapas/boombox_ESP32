# ガラクタ — Garakuta Boombox

> **ガラクタ** (Garakuta) ist Japanisch für **„Schrott"** oder **„Gerümpel"** — ein liebevoller Name für eine selbstgebaute Box aus zusammengekratzten Teilen, die klanglich alles andere als Schrott ist.

Ein komplett selbstgebauter Bluetooth-Lautsprecher auf Basis eines ESP32. Vom CAD-Modell über die Elektronik bis zur Firmware — alles von Grund auf selbst entwickelt.

---

## Fotos

| | | |
|:---:|:---:|:---:|
| ![Foto 1](photo_1.jpg) | ![Foto 2](photo_2.jpg) | ![Foto 3](photo_3.jpg) |

---

## Features auf einen Blick

- Bluetooth Audio (A2DP Sink) — verbindet sich wie ein normaler BT-Lautsprecher
- Digitaler Equalizer (Bass & Treble) via Biquad-IIR-Filter, live per Drehregler
- 8-Band Spektrum-Analysator, komplett in Software auf dem ESP32
- 16×8 WS2812B LED-Matrix mit mehreren Visualisierungs-Modi
- 3D-gedrucktes Gehäuse mit maßgeschneidertem Gitter-Frontpanel für Retro-Pixel-Look
- Einzel-Zellen Li-Ion Akku mit echtem BMS und Boost-Converter

---

## Hardware

### Mikrocontroller & Audio
| Komponente | Details |
|---|---|
| **MCU** | ESP32 (Dual-Core, Xtensa LX6) |
| **Verstärker** | 2× MAX98357A (I2S Class-D, 3W/Kanal) |
| **Lautsprecher** | 2× 5W Chassis |
| **Passivmembranen** | AliExpress Radiators, per Trial & Error ans Gehäusevolumen angepasst |

Der ESP32 agiert als reiner **A2DP Sink** — er empfängt den Bluetooth-Audiostream vom Handy als fertig dekodiertes PCM. Das digitale I2S-Signal geht direkt an die MAX98357A-Chips, die es intern verstärken. Ein separater DAC war nicht nötig.

### Stromversorgung
| Komponente | Details |
|---|---|
| **Akku** | 1× 18650 Li-Ion Zelle (3.7V) |
| **BMS / Laden** | TP4056 + DW01A Schutz-IC (Tiefentladungsschutz, Laderegler) |
| **Boost-Converter** | MT3608, 3.7V → ~5.09V für die Verstärker |
| **Puffer** | Parallelgeschaltetes Paket aus 470µF Elektrolytkondensatoren |
| **Hauptschalter** | Physischer Schalter, trennt Akku komplett vom Booster |

Der kniffligste Teil war das **Clipping** — bei harten Bassschlägen reagierte der MT3608 zu träge und die Versorgungsspannung brach kurz ein, was als hartes „KKKK"-Geräusch hörbar war. Die Lösung: 470µF Kondensatoren direkt parallel an VOUT+ / VOUT- des Boosters. Die Kondensatoren fangen die Lastspitzen ab und halten die Spannung stabil.

### Steuerung & Bedienung
| Komponente | GPIO | Funktion |
|---|---|---|
| Poti Bass | GPIO 33 | Bass ±12 dB |
| Poti Treble | GPIO 34 | Höhen ±12 dB |
| Poti Volume | GPIO 35 | Lautstärke (Hard-Limit: 70%) |
| Taster | GPIO 16 | Kurz: LED-Modus wechseln / Lang (≥1.5s): Reconnect |

### LED-Matrix
| Parameter | Wert |
|---|---|
| Typ | WS2812B (NeoPixel) |
| Auflösung | 16×8 Pixel (2× 8×8 Panel in Reihe) |
| Gesamt-LEDs | 128 |
| Datenpin | GPIO 23 (mit 470Ω Serienwiderstand) |

### Pin-Belegung (vollständig)
| Signal | GPIO |
|---|---|
| I2S DOUT (→ MAX98357A DIN) | 32 |
| I2S LRC / WS | 26 |
| I2S BCLK | 27 |
| AMP SD (Mute) | 17 |
| LED Data | 23 |
| Poti Bass | 33 |
| Poti Treble | 34 |
| Poti Volume | 35 |
| Taster | 16 |

---

## Software & Firmware

### Bluetooth — A2DP Sink

Der ESP32 registriert sich via `esp_a2dp_api` als Bluetooth Classic Audio Sink. Das Handy sieht ihn als normalen BT-Lautsprecher und streamt dekodiertes PCM direkt an den ESP32. Die Audiodaten laufen über einen FreeRTOS-Ringbuffer und werden in einem dedizierten I2S-Writer-Task an die Verstärker ausgegeben.

Der Bluetooth-Gerätename lautet **`ガラクタ`** — erscheint so auf jedem gekoppelten Gerät.

### Digitaler Equalizer (DSP)

Alle Audiobearbeitung passiert direkt auf den rohen PCM-Samples im I2S-Writer-Task, bevor die Daten an den Verstärker gehen:

- **Bass-Regler** → Biquad Low-Shelf-Filter @ 120 Hz, Slope 0.8, ±12 dB
- **Treble-Regler** → Biquad High-Shelf-Filter @ 6000 Hz, Slope 0.8, ±12 dB
- **Volume-Regler** → Lineare Skalierung der Samples, hard-limited auf 70% zur Sicherheit

Die Biquad-Koeffizienten werden bei jeder Poti-Bewegung neu berechnet. Die Filterstruktur ist eine direkte Transposed-Form-II IIR-Implementierung.

### 8-Band Spektrum-Analysator

Aus dem PCM-Stream werden in Echtzeit 8 Frequenzbänder extrahiert — ganz ohne FFT. Stattdessen wird eine Kaskade aus einfachen IIR-Tiefpass-Filtern genutzt:

- Band 0 ist der reine LP-Output (Bassbereich)
- Jedes höhere Band ergibt sich aus der **Differenz** zweier aufeinanderfolgender Tiefpass-Stufen
- Band 7 erfasst alles, was über alle LP-Stufen hinausgeht (Hochton)

Damit ergibt sich ein logarithmisch gestaffeltes Frequenzspektrum. Die Bandpegel werden per **Hüllkurvenfilter** geglättet (schneller Anstieg, langsamer Abfall) und die höheren Bänder werden verstärkt, da hochfrequente Inhalte von Natur aus weniger Energie tragen.

### LED-Visualisierungen

Per **kurzem Tastendruck** wird zwischen folgenden Modi durchgeschalten:

#### Modus 1 — Spektrum-Balken (VisualizerA)
8 vertikale Balken, einer pro Frequenzband. Die Balkenhöhe folgt der Bandenergie (`sqrt`-skaliert für bessere visuelle Balance). Jeder Balken hat einen Farbverlauf von unten nach oben. Bei Stille fallen die Balken langsam ab.

#### Modus 2 — Plasma (VisualizerC)
Eine sinusbasierte Plasma-Animation läuft über das gesamte 16×8 Display. Die **Animationsgeschwindigkeit** skaliert mit der aktuellen Gesamtlautstärke des Audiosignals — bei lautem, energiereichem Audio läuft das Plasma schneller.

#### Modus 3 — Poti-Anzeige
Zeigt die aktuelle Stellung aller drei Potentiometer als horizontale Balken an:
- Zeile 0–2: Bass (rot)
- Zeile 3–5: Treble (cyan)
- Zeile 6–7: Lautstärke (grün)

Dieser Modus wird automatisch aktiviert, sobald ein Regler bewegt wird, und kehrt nach kurzer Zeit automatisch zum vorherigen Visualisierungs-Modus zurück.

#### Modus 4 — Off
LEDs komplett aus.

### Intro-Text Scroller

Beim Einschalten scrollt eine zufällig gewählte Nachricht über das LED-Display. Darunter befinden sich unter anderem:

> *„GARAKUTA ONLINE."*
> *„GARAKUTA HEISST SCHROTTGERAET FALLS DU ES NICHT WUSSTEST"*
> *„GARAKUTA MAG DICH"*

### Taster-Funktionen

| Druck | Aktion |
|---|---|
| **Kurz** | LED-Modus weiterschalten |
| **Lang (≥ 1.5s)** | Bluetooth Reconnect + kurze Melodie als Bestätigung |

Die Reconnect-Melodie spielt drei Töne direkt über I2S (softwaregenerierter Sinus — kein zusätzlicher Chip).

---

## Gehäuse & Optik

Das Gehäuse wurde komplett selbst in CAD konstruiert und **3D-gedruckt**. Die interne Verkabelung nimmt ca. 30% des Innenvolumens ein.

Das Frontpanel ist kein simpler Diffusor, sondern eine maßgeschneiderte **Gitterstruktur**, die physisch vor jeder einzelnen LED sitzt. Dadurch entsteht die optische Illusion von Sub-Pixeln — jede LED wirkt wie ein kleines Fenster in einer Wabe. Das Ergebnis ist ein markanter **Retro-Pixel-Look**, der sich deutlich von einem normalen LED-Matrix-Display unterscheidet.

---

## Abhängigkeiten / Libraries

| Library | Zweck |
|---|---|
| `FastLED` | WS2812B LED-Steuerung |
| `esp_a2dp_api` (ESP-IDF) | Bluetooth A2DP Sink |
| `driver/i2s_std` (ESP-IDF) | I2S Ausgabe → MAX98357A |
| `Preferences` (Arduino ESP32) | Persistente Einstellungen im NVS-Flash |
| `FreeRTOS` | Ringbuffer & Tasks |

---

## Lizenz

Dieses Projekt ist ein privates Hobbyprojekt. Nutzung für eigene Bastelprojekte ist willkommen.
