#include <LoRa.h>
#include "boards.h"
#include "TinyGPSPlus.h"

TinyGPSPlus gps;

const bool test = true; // Flag to toggle test mode

// Constants for distance estimation and Earth's radius
const float PathLossExponent = 3.0; // Path loss exponent, adjust for your environment
const double earthRadiusKm = 6371.0; // Earth's radius in kilometers

// Function to estimate distance based on RSSI
double estimateDistance(int rssi) {
    // Constants for the Log-distance path loss model
    const double pathLossExponent = 2.7; // Typical urban area value; adjust based on your environment
    const double rssiAtOneMeter = -24;   // Example value; you need to measure this for your setup

    double distance = pow(10.0, (rssiAtOneMeter - rssi) / (10.0 * pathLossExponent));
    return distance;
}


// Converts degrees to radians
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Calculates the distance between two points on Earth using the Haversine formula
double haversineKm(double lat1, double lon1, double lat2, double lon2) {
    double dLat = degreesToRadians(lat2 - lat1);
    double dLon = degreesToRadians(lon2 - lon1);
    lat1 = degreesToRadians(lat1);
    lat2 = degreesToRadians(lat2);

    double a = sin(dLat/2) * sin(dLat/2) +
               sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
    double c = 2 * atan2(sqrt(a), sqrt(1-a)); 
    return earthRadiusKm * c;
}

void setup() {
    initBoard();
    delay(1500); // Delay for stability after power-on

    Serial.begin(9600);
    Serial.println("LoRa Receiver Initialization");
    Serial.println("Timestamp,RSSI,SNR,SpreadingFactor,Bandwidth,TransmitterLat,TransmitterLon,ReceiverLat,ReceiverLon,DistanceKm");

    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency)) {
        Serial.println("Starting LoRa failed!");
    } else {
        Serial.println("LoRa Initialized for Receiver");
    }
}

void loop() {
    if (test) {
        // Test mode operations
        processTestPacket();
    } else {
        // Operational mode with GPS data handling
        processGPSAndLoRaData();
    }
    delay(1000); // Adjust based on required duty cycle
}

void processTestPacket() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // Process and print packet details for test packets
        // processPacketDetails();
    }
}

void processGPSAndLoRaData() {
    double receiverLat = 0, receiverLon = 0;
    updateGPSLocation(receiverLat, receiverLon);

    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        handleReceivedPacket(receiverLat, receiverLon);
    }
}

void updateGPSLocation(double &lat, double &lon) {
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        gps.encode(c);
    }
    if (gps.location.isUpdated() && gps.location.isValid()) {
        lat = gps.location.lat();
        lon = gps.location.lng();
    }
}

void handleReceivedPacket(double receiverLat, double receiverLon) {
    String receivedData = "";
    while (LoRa.available()) {
        receivedData += (char)LoRa.read();
    }

    // Assuming the packet format is "lat,lon"
    double transmitterLat, transmitterLon;
    parseCoordinates(receivedData, transmitterLat, transmitterLon);

    if (transmitterLat == 0 || transmitterLon == 0) {
        Serial.println("Invalid coordinates received.");
        return;
    }

    double distanceKm = haversineKm(receiverLat, receiverLon, transmitterLat, transmitterLon);
    logPacketDetails(distanceKm, receiverLat, receiverLon);
}

void parseCoordinates(String data, double &lat, double &lon) {
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
        lat = data.substring(0, commaIndex).toDouble();
        lon = data.substring(commaIndex + 1).toDouble();
    }
}

void logPacketDetails(double distance, double receiverLat, double receiverLon) {
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    int spreadingFactor = LoRa.getSpreadingFactor();
    long signalBandwidth = LoRa.getSignalBandwidth();
    int codingRate = LoRa.getCodingRate();
    long frequency = LoRa.getFrequency();
    long preambleLength = LoRa.getPreambleLength();
    int packetLength = LoRa.parsePacket(0); // Assuming you call this at the right time to get packet length

    Serial.print(millis());
    Serial.print(",");
    Serial.print(rssi);
    Serial.print(",");
    Serial.print(snr);
    Serial.print(",");
    Serial.print(spreadingFactor);
    Serial.print(",");
    Serial.print(signalBandwidth);
    Serial.print(",");
    Serial.print(codingRate);
    Serial.print(",");
    Serial.print(frequency);
    Serial.print(",");
    Serial.print(preambleLength);
    Serial.print(",");
    Serial.print(packetLength);
    Serial.print(",");
    Serial.print(receiverLat, 6);
    Serial.print(",");
    Serial.print(receiverLon, 6);
    Serial.print(",");
    Serial.print(distance);
    Serial.println();
}


#ifdef HAS_DISPLAY
void displayPacketDetails(int spreadingFactor, long bandwidth) {
    u8g2->clearBuffer();
    char buf[256];
    snprintf(buf, sizeof(buf), "SF: %i BW: %li", spreadingFactor, bandwidth);
    u8g2->drawStr(0, 40, buf);
    u8g2->sendBuffer();
}
#endif
