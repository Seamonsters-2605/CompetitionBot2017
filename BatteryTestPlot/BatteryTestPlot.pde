import java.util.*;

final boolean TEST_MODE = false;

float xMin = 0;
float xMax = 2000;
float xScale = 100;
float yMin = -1;
float yMax = 13;
float yScale = 1;

float hue = 0;
float hueIncrement = (sqrt(5.0) - 1) / 2 * 255.0;

GraphData[] graphs;
int numGraphs = 0;
boolean newGraph = false;

class GraphData {
  float[] xCoords;
  float[] yCoords;
  
  GraphData(int numCoords) {
    xCoords = new float[numCoords];
    yCoords = new float[numCoords];
  }
}

void settings() {
  size(1800, 900);
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
  textAlign(CENTER, TOP);
  fill(0);
  for(float x = xMin; x < xMax; x += xScale) {
    stroke(0);
    line(valueToPixelX(x), valueToPixelY(0) - 8, valueToPixelX(x), valueToPixelY(0) + 8);
    noStroke();
    text(x, valueToPixelX(x), valueToPixelY(0) + 12);
  }
  
  graphs = new GraphData[10];
  
  strokeWeight(1);
  
  selectInput("Choose a log file", "fileSelected");
}

void fileSelected(File selection) {
  if(selection == null)
    return;
  else {
    String[] lines = loadStrings(selection);
    GraphData data = new GraphData(lines.length);
    
    int i = 0;
    for(String line : lines) {
      String[] words = line.split(" ");
      float x = float(words[0]);
      float y;
      if(!TEST_MODE)
        y = float(words[1]);
      else
        y = noise(x / 100.0) * (yMax - yMin) + yMin;
      
      data.xCoords[i] = x;
      data.yCoords[i] = y;
      i++;
    }
    
    graphs[numGraphs] = data;
    numGraphs++;
    newGraph = true;
  }
}

float valueToPixelX(float x) {
  return (x - xMin) / (xMax - xMin) * float(width);
}

float pixelXToValue(float x) {
  return x / float(width) * (xMax - xMin) + xMin;
}

float valueToPixelY(float y) {
  return height - (y - yMin) / (yMax - yMin) * float(height);
}

void draw() {
  if(newGraph) {
    stroke(hue, 255, 191);
    
    if(TEST_MODE)
      noiseSeed(int(random(65535)));
    
    boolean first = true;
    float prevX = 0;
    float prevY = 0;
    GraphData data = graphs[numGraphs - 1];
    for(int i = 0; i < data.xCoords.length; i++) {
      float x = data.xCoords[i];
      float y = data.yCoords[i];
      
      if(first)
        first = false;
      else
        line(valueToPixelX(prevX), valueToPixelY(prevY), valueToPixelX(x), valueToPixelY(y));
      prevX = x;
      prevY = y;
    }
    
    line(width - 96, 8 + (numGraphs - 1) * 16, width - 32, 8 + (numGraphs - 1) * 16);
    
    newGraph = false;
    hue += hueIncrement;
    if(hue > 255)
      hue -= 255.0;
    selectInput("Choose a log file", "fileSelected");
  }
  
  textAlign(RIGHT, CENTER);
  fill(0,0,255);
  noStroke();
  rect(width - 256, 0, 160, 128);
  for(int i = 0; i < numGraphs; i++) {
    fill(0);
    text(graphs[i].yCoords[int(pixelXToValue(mouseX))], width - 96, 8 + i * 16);
  }
}