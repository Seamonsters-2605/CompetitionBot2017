import java.util.*;

final boolean TEST_MODE = false;

float xMin = -50;
float xMax = 2000;
float xScale = 100;
float yMin = -1;
float yMax = 13;
float yScale = 1;

int graphNumber = 0;
float hue = 0;
float hueIncrement = (sqrt(5.0) - 1) / 2 * 255.0;

String[] lines;

void settings() {
  size(1800, 480);
}

void setup() {
  background(255,255,255);
  strokeWeight(3);
  colorMode(HSB);
  
  // draw axes
  stroke(0);
  line(0, valueToPixelY(0), width, valueToPixelY(0));
  line(valueToPixelX(0), 0, valueToPixelX(0), height);
  for(float y = yMin; y < yMax; y += yScale) {
    line(valueToPixelX(0) - 8, valueToPixelY(y), valueToPixelX(0) + 8, valueToPixelY(y));
  }
  for(float x = xMin; x < xMax; x += xScale) {
    line(valueToPixelX(x), valueToPixelY(0) - 8, valueToPixelX(x), valueToPixelY(0) + 8);
  }
  
  selectInput("Choose a log file", "fileSelected");
}

void fileSelected(File selection) {
  if(selection == null)
    return;
  else {
    lines = loadStrings(selection);
  }
}

float valueToPixelX(float x) {
  return (x - xMin) / (xMax - xMin) * float(width);
}

float valueToPixelY(float y) {
  return height - (y - yMin) / (yMax - yMin) * float(height);
}

void draw() {
  if(lines != null) {
    stroke(hue, 255, 191);
    
    if(TEST_MODE)
      noiseSeed(int(random(65535)));
    
    boolean first = true;
    float prevX = 0;
    float prevY = 0;
    for(String line : lines) {
      String[] words = line.split(" ");
      float x = float(words[0]);
      float y;
      if(!TEST_MODE)
        y = float(words[1]);
      else
        y = noise(x / 100.0) * (yMax - yMin) + yMin;
      
      if(first)
        first = false;
      else
        line(valueToPixelX(prevX), valueToPixelY(prevY), valueToPixelX(x), valueToPixelY(y));
      prevX = x;
      prevY = y;
    }
    
    line(width - 96, 16 + graphNumber * 8, width - 32, 16 + graphNumber * 8);
    
    lines = null;
    graphNumber++;
    hue += hueIncrement;
    if(hue > 255)
      hue -= 255.0;
    selectInput("Choose a log file", "fileSelected");
  }
}