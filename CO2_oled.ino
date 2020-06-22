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
#include <avr/eeprom.h>
MHZ19_uart mhz19;
int dispCO2;

#define FLOAT_TO_INT(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define MHZ_RX 8
#define MHZ_TX 9
#define CHART_SIZE 70
#define KALMAN_ARRAY_SIZE 5
#define BUTTON_PIN 18

#define CO2_BASE 410
#define CO2_GREEN_MAX 1000
#define CO2_YELLOW_MAX 2000
#define CO2_MAX_VALUE 5000

int const MEASURE_DELAY = 5500; // ms, задержка между измерениями minimum = 5000, быстрее mhx19 просто не обновляется
unsigned long timer;//таймер замедления опроса датчика
unsigned long longPushTimer;//таймер для кнопки
int const LONG_PUSH_TIME = 5000;

long const GRAPH_TICK_TIME_RANGE = 27500; //ms
long const GRAPH_TICK_TIME_RANGE3 = 154000; //ms
long const GRAPH_TICK_TIME_RANGE24 = 600000; //ms 600000
int measuresForTick;//сколько проводить измерений на столбец графика  измерений должно быть БОЛЬШЕ!!!, чем значений в массиве калмана  = GRAPH_TICK_TIME_RANGE/MEASURE_DELAY;
int kalmanMeasuresArray[KALMAN_ARRAY_SIZE];//массив для фильтра Калмана (сглаживание нескольких последних значений)
int currentMeasureCount = 0;//сколько всего изменений провели для текущего столбца
int chartValues[CHART_SIZE]; //70 столбцов в графике = 11,6 часов, если по 10 минут столбец или около 32 минут, если по 27.5 секунд
int chartValuesCount = 0; //считаем сколько столбцов заполнено, чтобы сдвигать график или дополнять

int measuresForTick24;//сколько проводить измерений на столбец графика  измерений должно быть БОЛЬШЕ!!!, чем значений в массиве калмана
int kalmanMeasuresArray24[KALMAN_ARRAY_SIZE];//массив для фильтра Калмана (сглаживание нескольких последних значений)
int currentMeasureCount24 = 0;//сколько всего изменений провели для текущего столбца
int chartValues24[CHART_SIZE]; //70 столбцов в графике = 11 часов, если по 10 минут столбец или около 30 минут, если по 27.5 секунд
int chartValuesCount24 = 0; //считаем сколько столбцов заполнено, чтобы сдвигать график или дополнять

int measuresForTick3;//сколько проводить измерений на столбец графика  измерений должно быть БОЛЬШЕ!!!, чем значений в массиве калмана
int kalmanMeasuresArray3[KALMAN_ARRAY_SIZE];//массив для фильтра Калмана (сглаживание нескольких последних значений)
int currentMeasureCount3 = 0;//сколько всего изменений провели для текущего столбца
int chartValues3[CHART_SIZE]; //70 столбцов в графике = 11 часов, если по 10 минут столбец или около 30 минут, если по 27.5 секунд
int chartValuesCount3 = 0; //считаем сколько столбцов заполнено, чтобы сдвигать график или дополнять

boolean buttonWasPressed = false;
boolean buttonPressed = false;
boolean longPress = false;
int graphMode = 0;

int redP = 5;
int greenP = 6;
int blueP = 10;



  struct SaveStruct {//структура для сохранения суточного графика
    int values[CHART_SIZE];
    int graphPosition;
  };


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  measuresForTick = GRAPH_TICK_TIME_RANGE/MEASURE_DELAY;
  measuresForTick3 = GRAPH_TICK_TIME_RANGE3/MEASURE_DELAY;
  measuresForTick24 = GRAPH_TICK_TIME_RANGE24/MEASURE_DELAY;
  
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  loadEEPROMToArray(chartValues, &chartValuesCount, 0, CHART_SIZE);
  loadEEPROMToArray(chartValues3, &chartValuesCount3, 1, CHART_SIZE);
  loadEEPROMToArray(chartValues24, &chartValuesCount24, 2, CHART_SIZE);
  pinMode(BUTTON_PIN, INPUT);
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
  if (mhz19.getStatus() == 0) {
    display.print(F("OK"));
    Serial.println(F("OK"));
  } else {
    display.print(F("ERROR"));
    Serial.println(F("ERROR"));
  }
  timer = millis();//устанавливаем таймер
}


void loop() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
     if (!buttonWasPressed) {
      buttonPressed = true;
      longPushTimer = millis();
      switch(graphMode)
      {
        case 0:
        graphMode = 1;
        break;
        case 1:
        graphMode = 2;
        break;
        case 2:
        graphMode = 0;
        break;
      }
      buttonWasPressed = true;
     } else {
      buttonPressed = false;
      }
 } else {
  longPushTimer = millis();
  buttonWasPressed = false;
 }
if (millis()-longPushTimer > LONG_PUSH_TIME) {
  Serial.println(F("RESET SAVED VALUES"));
  setZeroToSavedArray(chartValues, CHART_SIZE, &chartValuesCount, &currentMeasureCount);
  saveArrayToEEPROM(chartValues, chartValuesCount, 0, CHART_SIZE);
  setZeroToSavedArray(chartValues3, CHART_SIZE, &chartValuesCount3, &currentMeasureCount3);
  saveArrayToEEPROM(chartValues3, chartValuesCount3, 1, CHART_SIZE);
  setZeroToSavedArray(chartValues24, CHART_SIZE, &chartValuesCount24, &currentMeasureCount24);
  saveArrayToEEPROM(chartValues24, chartValuesCount24, 2, CHART_SIZE);
}
  if (millis()-timer > MEASURE_DELAY) {//задержка нужна чтобы не опрашивать датчик слишком часто
  timer = millis();
  dispCO2 = mhz19.getPPM();
  updateKalmanArray(dispCO2, kalmanMeasuresArray);
  updateKalmanArray(dispCO2, kalmanMeasuresArray3);
  updateKalmanArray(dispCO2, kalmanMeasuresArray24);
  updateChartArray(chartValues, kalmanMeasuresArray, &chartValuesCount, &currentMeasureCount, measuresForTick);
  updateChartArray(chartValues3, kalmanMeasuresArray3, &chartValuesCount3, &currentMeasureCount3, measuresForTick3);
  updateChartArray(chartValues24, kalmanMeasuresArray24, &chartValuesCount24, &currentMeasureCount24, measuresForTick24);


  if (currentMeasureCount == 0) {
  saveArrayToEEPROM(chartValues, chartValuesCount, 0, CHART_SIZE);
  }
  if (currentMeasureCount3 == 0) {
  saveArrayToEEPROM(chartValues3, chartValuesCount3, 1, CHART_SIZE);
  }
  if (currentMeasureCount24 == 0) {
  saveArrayToEEPROM(chartValues24, chartValuesCount24, 2, CHART_SIZE);
  }
 
  setColorByCo2(dispCO2);
  }
  switch(graphMode) {
    case 0:
    drawGraph(chartValues, CHART_SIZE, &chartValuesCount, dispCO2, graphMode);  
    break;
    case 1:
    drawGraph(chartValues3, CHART_SIZE, &chartValuesCount3, dispCO2, graphMode);
    break;
    case 2:
    drawGraph(chartValues24, CHART_SIZE, &chartValuesCount24, dispCO2, graphMode);
    break;
   
  }
 }


void setZeroToSavedArray(int* arrray, int arrLen, int *chartCount, int *measuresCount) {
  for (int i = 0; i < arrLen; i++) {
      arrray[i] = 0;
    }
    *measuresCount = 0;
    *chartCount = 0;
}
 
void updateKalmanArray(int currentValue, int* kalmanArray)  {//добавляем измерение в хвост массива
    for (int i = 0; i < KALMAN_ARRAY_SIZE-1; i++) {
      kalmanArray[i] = kalmanArray[i+1];
    }
    kalmanArray[KALMAN_ARRAY_SIZE-1] = currentValue;
}


void saveArrayToEEPROM(int* arrray, int graphPosition, int address, int arrLen) {
  struct SaveStruct save;
  for (int i=0; i<arrLen; i++) {
    save.values[i] = arrray[i];
  }
  save.graphPosition = graphPosition;
  eeprom_write_block((void*)&save, address*sizeof(save), sizeof(save));
}

void loadEEPROMToArray(int* arrray, int* graphPosition, int address, int arrLen) {
  struct SaveStruct load;
  eeprom_read_block((void*)&load, address*sizeof(load), sizeof(load));
  for (int i=0; i<arrLen; i++) {
     arrray[i] = load.values[i];
  }
  *graphPosition = load.graphPosition;
  int pos = *graphPosition;
}

 
void updateChartArray(int* chart, int* kalmanArray, int* chartCount, int* measureCount, int measuresForTick) {
    (*measureCount)++;
    if (*measureCount >= measuresForTick) {//когда измерений на столбец достаточно, смещаем предыдущие значения
      for (int i = CHART_SIZE-1 ; i >= 1; i--) {
        chart[i] = chart[i-1];
      }
    if (*chartCount >= CHART_SIZE) { 
      *chartCount = CHART_SIZE; 
      } else {
      (*chartCount)++;
        }
    *measureCount = 0;

  }
  int filterResult = kalmanFilter(kalmanArray, KALMAN_ARRAY_SIZE, 15);
  if (*measureCount <= 1) {
    chart[0] = filterResult;
  } else {
    chart[0] = (chart[0]+filterResult)/2;//находим среднее арифметическое за период одного столбца (по умолчанию 10 минут). Если между изменениями у нас 5500 милиисекунд, то 600000мс/5500мс = 109 измерений
  }
}

void drawGraph(int* values, int values_size, int* chartCount, int current_val, int mode) {

  int maxValue = 0;
  int lowestValueDifference = 100; //это будет минимальный разброс значений на графике между максимальным и минимальным
  int minValue = CO2_MAX_VALUE;
  int graphStartPositionX = 47;
  int graphStartPositionY = 0;

  for (int i = 0; i < CHART_SIZE && i <= *chartCount; i++) {
    if (values[i] < minValue) {minValue = (int)(values[i]/100)*100;}
    if (values[i] > maxValue) {maxValue = (int)((values[i]/100)+1)*100;}
  }
  
  if (minValue > lowestValueDifference && (maxValue-minValue < lowestValueDifference)) {
    minValue = minValue - lowestValueDifference;
  }
  
  float heightRatio = (float)SCREEN_HEIGHT/(maxValue-minValue);

  display.clearDisplay();

  for (int i = 0; i < CHART_SIZE; i++) {
    int currentHeight = (float)(values[i]*heightRatio) - (minValue*heightRatio);
    if (currentHeight < 1) {currentHeight=1;}
    display.fillRect(graphStartPositionX+i, SCREEN_HEIGHT-currentHeight, 1, currentHeight, 1);
  }
  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 25);
  display.print(minValue);
  display.setCursor(20, 0);
  display.print(maxValue);
  display.setTextSize(2);
  display.setCursor(0, 9);
  display.print(current_val);

   display.setTextSize(1);
   display.setCursor(0, 25);
  
  switch(mode) {
  case 0:
  display.print("32m");
  break;
  case 1:
  display.print("3h");
  break;
  case 2:
  display.print("11h");
  break;
  }
  
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
