#include <IRremote.h>

// === PIN SETUP ===
const int RECV_PIN = 2;      // Pin IR Receiver
const int LED_PIN = 13;      // LED bawaan Arduino (atau LED eksternal)

IRrecv irrecv(RECV_PIN);
decode_results results;

// Ganti dengan kode dari remote kamu
unsigned long codePower = 0xFFA25D;  // Contoh: Tombol Power

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();       // Mulai receiver
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.print("Kode IR: 0x");
    Serial.println(results.value, HEX);

    // Cek apakah tombol yang ditekan cocok
    if (results.value == codePower) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Toggle LED
      Serial.println("Tombol Power ditekan! LED di-toggle.");
    }

    irrecv.resume();  // Siap nerima sinyal lagi
  }
}
