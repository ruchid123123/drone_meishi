// ESP32 Drone Attitude Viewer for Processing
// Receives "ATT,roll,pitch,yaw" from serial port and displays 3D orientation

import processing.serial.*;

Serial serial;
float roll = 0;
float pitch = 0;
float yaw = 0;

// For latency measurement
long lastUpdateTime = 0;
int updateCount = 0;
float updateRate = 0;

// Serial port settings
// Set to "" for auto-detect, or specify port name like "COM4"
String portName = "COM4";
int baudRate = 115200;

void setup() {
  size(800, 600, P3D);
  textSize(16);

  // List available serial ports
  println("Available serial ports:");
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) {
    println("[" + i + "] " + ports[i]);
  }

  // Only auto-detect if portName is empty
  if (portName.equals("") && ports.length > 0) {
    // Try to find ESP32 port (usually higher COM number on Windows)
    for (String p : ports) {
      if (p.contains("COM") || p.contains("tty.usb") || p.contains("ttyUSB")) {
        portName = p;
        break;
      }
    }
    if (portName.equals("")) {
      portName = ports[ports.length - 1];  // Use last port as fallback
    }
  }

  if (!portName.equals("")) {
    println("Connecting to: " + portName);
    try {
      serial = new Serial(this, portName, baudRate);
      serial.bufferUntil('\n');
    } catch (Exception e) {
      println("Failed to open serial port: " + e.getMessage());
      println("Please modify portName variable manually.");
    }
  } else {
    println("No serial ports found!");
  }
}

void draw() {
  background(30);
  lights();

  // Draw title and info
  camera();
  hint(DISABLE_DEPTH_TEST);
  fill(255);
  textAlign(LEFT, TOP);
  text("ESP32 Drone Attitude Viewer", 10, 10);
  text("Roll:  " + nf(roll, 0, 2) + " deg", 10, 35);
  text("Pitch: " + nf(pitch, 0, 2) + " deg", 10, 55);
  text("Yaw:   " + nf(yaw, 0, 2) + " deg", 10, 75);
  text("Update rate: " + nf(updateRate, 0, 1) + " Hz", 10, 100);
  text("Port: " + portName, 10, 125);

  // Instructions
  textAlign(RIGHT, TOP);
  text("Press 'r' to reset yaw", width - 10, 10);
  text("Press 'q' to quit", width - 10, 30);
  hint(ENABLE_DEPTH_TEST);

  // 3D view
  translate(width / 2, height / 2, 0);

  // Apply rotations (convert degrees to radians)
  // Note: Processing uses different coordinate system
  rotateX(radians(-pitch));
  rotateZ(radians(-roll));
  rotateY(radians(yaw));

  // Draw drone body (box)
  drawDrone();

  // Update rate calculation
  updateCount++;
  long now = millis();
  if (now - lastUpdateTime > 1000) {
    updateRate = updateCount * 1000.0 / (now - lastUpdateTime);
    updateCount = 0;
    lastUpdateTime = now;
  }
}

void drawDrone() {
  // Main body
  fill(80, 80, 80);
  stroke(50);
  strokeWeight(1);
  box(200, 20, 200);

  // Front indicator (red)
  pushMatrix();
  translate(0, 0, -100);
  fill(255, 0, 0);
  noStroke();
  sphere(15);
  popMatrix();

  // Back indicator (blue)
  pushMatrix();
  translate(0, 0, 100);
  fill(0, 0, 255);
  noStroke();
  sphere(10);
  popMatrix();

  // Left indicator (green)
  pushMatrix();
  translate(-100, 0, 0);
  fill(0, 255, 0);
  noStroke();
  sphere(10);
  popMatrix();

  // Right indicator (yellow)
  pushMatrix();
  translate(100, 0, 0);
  fill(255, 255, 0);
  noStroke();
  sphere(10);
  popMatrix();

  // Draw arms
  stroke(100);
  strokeWeight(3);
  line(-100, 0, -100, 100, 0, 100);  // FL to RR
  line(100, 0, -100, -100, 0, 100);  // FR to RL

  // Draw motors
  fill(60);
  noStroke();

  // FL
  pushMatrix();
  translate(-100, -10, -100);
  cylinder(25, 15);
  popMatrix();

  // FR
  pushMatrix();
  translate(100, -10, -100);
  cylinder(25, 15);
  popMatrix();

  // RR
  pushMatrix();
  translate(100, -10, 100);
  cylinder(25, 15);
  popMatrix();

  // RL
  pushMatrix();
  translate(-100, -10, 100);
  cylinder(25, 15);
  popMatrix();

  // Draw coordinate axes
  strokeWeight(2);
  // X axis (red) - roll
  stroke(255, 0, 0);
  line(0, 0, 0, 150, 0, 0);
  // Y axis (green) - up
  stroke(0, 255, 0);
  line(0, 0, 0, 0, -150, 0);
  // Z axis (blue) - pitch
  stroke(0, 0, 255);
  line(0, 0, 0, 0, 0, -150);
}

void cylinder(float r, float h) {
  int sides = 16;
  float angle = TWO_PI / sides;

  beginShape(TRIANGLE_STRIP);
  for (int i = 0; i <= sides; i++) {
    float x = cos(i * angle) * r;
    float z = sin(i * angle) * r;
    vertex(x, 0, z);
    vertex(x, h, z);
  }
  endShape();

  // Top cap
  beginShape(TRIANGLE_FAN);
  vertex(0, 0, 0);
  for (int i = 0; i <= sides; i++) {
    float x = cos(i * angle) * r;
    float z = sin(i * angle) * r;
    vertex(x, 0, z);
  }
  endShape();
}

void serialEvent(Serial p) {
  try {
    String line = p.readStringUntil('\n');
    if (line != null) {
      line = line.trim();
      if (line.startsWith("ATT,")) {
        String[] parts = line.substring(4).split(",");
        if (parts.length >= 3) {
          roll = float(parts[0]);
          pitch = float(parts[1]);
          yaw = float(parts[2]);
        }
      }
    }
  } catch (Exception e) {
    // Ignore parse errors
  }
}

float yawOffset = 0;

void keyPressed() {
  if (key == 'r' || key == 'R') {
    // Reset yaw reference
    yawOffset = yaw;
    println("Yaw reset");
  }
  if (key == 'q' || key == 'Q') {
    exit();
  }
}
