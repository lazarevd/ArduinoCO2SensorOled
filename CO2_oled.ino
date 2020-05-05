/*********
  РАБОТАЕТ С ДИСПЛЕЕМ OLED 030175LA 128 30
  ДЛЯ SPARKFUN pro micro:

  Дисплей цепляется на i2c ноги: SCL - 3, SDA - 2
  Датчик CO2 (MH-Z19B) - подключение по цифре: RX - 14, TX - 15 - это Serial порты
*********/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MHZ19_uart.h>
MHZ19_uart mhz19;
int dispCO2;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define MHZ_RX 8
#define MHZ_TX 9
#define CHART_SIZE 70
#define KALMAN_ARRAY_SIZE 5


int const MEASURE_DELAY = 5500; // ms, minimum = 5000, быстрее mhx19 просто не обновляется
long const GRAPH_TICK_TIME_RANGE = 27500; //ms
int measuresNumber;//сколько проводить измерений на столбец графика !!! измерений должно быть БОЛЬШЕ, чем значений в массиве калмана
int kalmanMeasuresArray[KALMAN_ARRAY_SIZE];//массив для фильтра Калмана (сглаживание нескольких последних значений)
int currentMeasureCount = 0;//сколько всего изменений провели для текущего столбца
int chartValues[CHART_SIZE]; //70 столбцов в графике = 11 часов, если по 10 минут столбец
int chartValuesCount = 0; //считаем сколько столбцов заполнено, чтобы сдвигать график или дополнять


int CO2_BASE = 410;
int CO2_GREEN_MAX = 1000;
int CO2_YELLOW_MAX = 2000;
#define CO2_MAX_VALUE 5000


int redP = 5;
int greenP = 6;
int blueP = 10;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  measuresNumber = GRAPH_TICK_TIME_RANGE/MEASURE_DELAY;
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(100);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("Starting...");
  display.display(); 

// CO2 sensor 
  display.setCursor(0, 0);
  display.print(F("MHZ-19... "));
  Serial.print(F("MHZ-19... "));
  mhz19.begin(MHZ_TX, MHZ_RX);
  mhz19.setAutoCalibration(false);
  mhz19.getStatus();    // первый запрос, в любом случае возвращает -1
  delay(500);
  if (mhz19.getStatus() == 0) {
    display.print(F("OK"));
    Serial.println(F("OK"));
  } else {
    display.print(F("ERROR"));
    Serial.println(F("ERROR"));
  }
}

void loop() {
  delay(MEASURE_DELAY);//задержка нужна чтобы не было проблем с перезаливкой прошивки
  dispCO2 = mhz19.getPPM();
  updateKalmanArray(dispCO2, kalmanMeasuresArray);
  updateChartArray(chartValues, &chartValuesCount, &currentMeasureCount, measuresNumber);
  drawGraph(chartValues, CHART_SIZE, &chartValuesCount, dispCO2);
  setColorByCo2(dispCO2);
 }

 
void updateKalmanArray(int currentValue, int* kalmanArray)  {//добавляем измерение в хвост массива
    for (int i = 0; i < KALMAN_ARRAY_SIZE-1; i++) {
      kalmanArray[i] = kalmanArray[i+1];
    }
    kalmanArray[KALMAN_ARRAY_SIZE-1] = currentValue;

  String st = "kalman arr: cur measure: " + String(currentValue) + ": ";
  for (int i=0;i<KALMAN_ARRAY_SIZE;i++) {
    st = st + " " + kalmanArray[i];
  }
  Serial.println(st);
}
 
void updateChartArray(int* chart, int* chartCount, int* measureCount, int measuresForTick) {
    (*measureCount)++;
    if (*measureCount >= measuresForTick) {//когда измерений на столбец достаточно, смещаем предыдущие значения
      for (int i = CHART_SIZE-1 ; i >= 1; i--) {
        //Serial.println("ch: " + String(chart[i]) + " " + String(chart[i-1]));
        chart[i] = chart[i-1];
      }
    if (*chartCount >= CHART_SIZE) { *chartCount = CHART_SIZE; }
    *measureCount = 0;
    (*chartCount)++;
  }
  int filterResult = kalmanFilter(kalmanMeasuresArray, KALMAN_ARRAY_SIZE, 15);
  Serial.println("filterResult: " + String(filterResult) + " measureCount: " + String(*measureCount) + "/" + String(measuresNumber));
  if (*measureCount <= 1) {
    chart[0] = filterResult;
  } else {
    chart[0] = (chart[0]+filterResult)/2;//находим среднее арифметическое за период одного столбца (по умолчанию 10 минут). Если между изменениями у нас 5500 милиисекунд, то 600000мс/5500мс = 109 измерений
  }
    //////////
    
  String st = String(chart[0]) + "[0]" + " " + String(chart[1]) + "[1]" + " " + String(chart[2]) + "[2]" + " " + String(chart[3]) + "[3]";
  for(int i=5; i < CHART_SIZE; i = i + 5) {
    st = st + " " + String(chart[i]) + "["+String(i)+"]";
  }
  st = st + " " + String(chart[68]) + "[68]" + " " + String(chart[69]) + "[69]" + "\n";
  Serial.println("graph: " + st);
  
}

void drawGraph(int* values, int values_size, int* chartCount, int current_val) {

  int maxValue = 0;
  int lowestValueDifference = 100; //это будет минимальный разброс значений на графике между максимальным и минимальным
  int minValue = CO2_MAX_VALUE;
  int graphStartPositionX = 47;
  int graphStartPositionY = 0;


  

  for (int i = 0; i < CHART_SIZE && i < *chartCount; i++) {
    if (values[i] < minValue) {minValue = values[i];}
    if (values[i] > maxValue) {maxValue = values[i];}
  }

  Serial.println("heightRatio " + String(CHART_SIZE)+ " " + String(*chartCount)+ " " + String(maxValue)+ " " + String(minValue));

  if (minValue > lowestValueDifference && (maxValue-minValue < lowestValueDifference)) {
    minValue = minValue - lowestValueDifference;
  }
  
  float heightRatio = (float)SCREEN_HEIGHT/(maxValue-minValue);
  Serial.println("heightRatio " + String(heightRatio)+ " " + String(minValue)+ " " + String(maxValue)+ " ");

  display.clearDisplay();

  for (int i = 0; i < CHART_SIZE; i++) {
    int currentHeight = (float)(values[i]-minValue)*heightRatio;
    //if (currentHeight > 0) {Serial.println("cur h " + String(i) + "=" + String(currentHeight));}
    if (currentHeight < 1) {currentHeight=1;}
    display.fillRect(graphStartPositionX+i, SCREEN_HEIGHT-currentHeight, 1, currentHeight, 1);
  }
  
  display.fillRect(50, 20, 1, 21, 1);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 25);
  display.print(String(minValue));
  display.setCursor(20, 0);
  display.print(String(maxValue));
  display.setTextSize(2);
  display.setCursor(0, 9);
  display.print(String(current_val));

  display.display();    //draw
}



void setColorByCo2(int measure) {
  float k = 0.05;//яркость
  if (measure <= CO2_BASE) {
    setColor(0, 255, 0, k);
  }
  else if (measure > CO2_BASE and measure < CO2_GREEN_MAX) {
    setColor((int)(((float)255/590)*(measure-CO2_BASE)), 255, 0, k);
  } else if (measure >= CO2_GREEN_MAX and measure < CO2_YELLOW_MAX) {
    setColor(255, (int)(255-((float)255.0/(CO2_YELLOW_MAX-CO2_GREEN_MAX))*(measure-CO2_GREEN_MAX)), 0, k);
  } else {
    setColor(255, 0, 0, k);
  }
}



int kalmanFilter(int inputArray[], int arrSize, float r) {//r - тут степень сглаживания, 15 обычно дает приемлемый результат, увеличение сглаживает, более 100 делать не стоит т.к. смещается график
  int state = -1;
  int szo = sizeof(inputArray);
  int sz = arrSize * sizeof(inputArray[0]);
  if (sizeof(inputArray) > 0 && sizeof(inputArray) <= arrSize* sizeof(inputArray[0])) {
    float Q = 2.0f;
    float R = r;
    float F = 1.0f;
    float H = 1;
    float x0 = 0;
    float p0 = 0;
    float covar = 0.1f;
    state = inputArray[0];
    for (int i = 1; i < arrSize; i++) {//начинаем со второго элемента, первый у нас в x0;
      x0 = F*state;
      p0 = F*covar*F + Q;
      float k = H*p0 / (H*p0*H + R);
      state = x0 + k*(inputArray[i] - H*x0);
      covar = (1 - k*H)*p0;
    }
  }
  return state;
}


void setColor(int red, int green, int blue, float k)
{
analogWrite(redP, red*k);
analogWrite(greenP, green*k);
analogWrite(blueP, blue*k);
}