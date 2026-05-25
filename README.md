# Segelboot_Autopilot

# ⛵ Segelboot Autopilot (ESP32)

Ein Open-Source-Autopilot für Segelboote, basierend auf dem leistungsstarken **ESP32**-Mikrocontroller. Das Projekt kombiniert hardwarenahe, präzise Kursberechnungen in C/C++ mit einer modernen, plattformunabhängigen Steuerung über ein integriertes HTML-Webinterface.

---

## ✨ Features

* **🧠 ESP32-Power:** Nutzt die Dual-Core-Rechenleistung sowie integriertes WLAN und Bluetooth.
* **📱 Webinterface (HTML/JS):** Kabellose Steuerung und Live-Datenanzeige auf dem Smartphone, Tablet oder Laptop.
* **🧭 Kursmodi:** Unterstützt Kompasskurs (Heading Hold) und ist vorbereitet für Windmodus (Apparent Wind Hold).
* **🔄 PID-Regelung:** Implementiert einen anpassbaren PID-Regelkreis für sanfte und präzise Ruderbewegungen bei Wellengang.

---

## 🛠️ Technische Struktur & Sprachen

Das Projekt setzt sich aus folgenden Kernkomponenten zusammen:
* **C++ (52.4%):** Hauptlogik des Autopiloten, Sensor-Auslesung und PID-Regelschleife.
* **HTML/JS (35.5%):** Benutzeroberfläche (Webinterface), die direkt vom ESP32 gehostet wird.
* **C (12.1%):** Low-Level-Treiber und optimierte mathematische Berechnungen.

---

## 📁 Projektstruktur

```text
├── ESP32_Autopilot/      # Quellcode für den ESP32 (C++/C)
│   ├── src/              # Core-Logik, PID-Regler und Sensor-Auswertung
│   └── data/             # Webserver-Dateien (HTML, CSS, JavaScript)
└── README.md             # Diese Dokumentationsdatei
```

---

## 🔧 Installation & Setup

### 1. Repository klonen
Klipse das Repository auf deinen lokalen Computer:
```bash
git clone https://github.com
```

### 2. Entwicklungsumgebung vorbereiten
* Öffne den Ordner `ESP32_Autopilot` vorzugsweise in **PlatformIO** (VS Code) oder richte die **Arduino IDE** für ESP32-Boards ein.
* Installiere die benötigten Bibliotheken für deine IMU-Sensorik und den Webserver (z. B. `ESPAsyncWebServer`).

### 3. Webinterface hochladen
* Lade den Inhalt des `data/`-Ordners über das Dateisystem-Tool deiner IDE (**SPIFFS** oder **LittleFS**) auf den Flash-Speicher des ESP32 hoch.

### 4. Code flashen
* Kompiliere das Projekt und lade den Hauptcode über USB auf deinen ESP32.

---

## 🕹️ Bedienung & Nutzung

1. Schalte die Stromversorgung des ESP32-Autopiloten ein.
2. Suche an deinem Smartphone oder Laptop nach dem WLAN-Netzwerk des Autopiloten und verbinde dich.
3. Öffne einen Webbrowser deiner Wahl und tippe die IP-Adresse ein (Standard: `192.168.4.1`).
4. Über das Interface kannst du nun:
   * Den Autopiloten aktivieren/deaktivieren (Standby / Auto).
   * Kurskorrekturen in Schritten von `+1°`, `-1°`, `+10°` oder `-10°` vornehmen.
   * Die PID-Empfindlichkeit live an dein Boot anpassen.

---

## 🤝 Beitragen

Beiträge, Fehlerberichte (Issues) und Verbesserungsvorschläge sind herzlich willkommen! Wenn du den Code optimieren oder neue Features hinzufügen möchtest:
1. Forke das Projekt.
2. Erstelle einen neuen Feature-Branch.
3. Sende einen Pull Request.
