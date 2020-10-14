int rgbBaselines[4][4]; // Each row contains measurements for a different block, each column is a given set of conditions 
const int analogPR = 1;
const int outR = 6;
const int outG = 3;
const int outB = 5;
const char colourIdStrings[4][15] = {
  "Ambient Dtcd",
  "Red Dtcd",
  "Green Dtcd",
  "Blue Dtcd"
};

void colourId(){
  int SE[4] = {0,0,0,0};
  int sample;
  int numTestCases = 4;
  int T = 100;
  int testCases[4][3] = {
    {0, 0, 0}, //Ambient
    {255, 0 ,0}, //Max Red
    {0, 255, 0}, //Max Green 
    {0, 0, 255} // Max Blue
  };
  for (int i = 1; i<numTestCases;i++){//NOTE: here the tests start at RED and NOT Ambient - I expect little different between each block's ambient response
    sample = 0; // Important that sample is set to 0 for each testcase, or else it will accumulate
    analogWrite(outR,testCases[i][0]);
    analogWrite(outG,testCases[i][3]);
    analogWrite(outB,testCases[i][2]);
    for (int j = 0; j <5;j++){
       delay(T) //Comes before the reading so the PR has time to respond to the initial lighting conditions
       sample += analogRead(analogPR);
    }
    for (int c = 0; c < 4; c++){// For each possible colour, add the current squared error for the given test number (i)
      SE[c] += (sample/5 - rgbBaselines[c][i])^^2; // This may cause overflow, if it does then we might be able to use the absolute error
    }
  }
  // Find the minimum squared error 
  int minDex = 0;
  int minVal = SE[0];
  for (int c = 1; i<4;i++){
    if (SE[c] < minval){
      minDex = c;
      minVal = SE[c];
    }
  }
  Serial.println(colourIdStrings[minDex]);
}

void getBaseline(int *colourVec){ // Detects ambient conditions and gets a baseline value for the photoresistor
  int sample;
  int numTestCases = 4;
  int T = 100;
  int testCases[4][3] = {
    {0, 0, 0}, //Ambient
    {255, 0 ,0}, //Max Red
    {0, 255, 0}, //Max Green 
    {0, 0, 255} // Max Blue
  };
  for (int i = 0; i<numTestCases;i++){ 
    sample = 0;
    analogWrite(outR,testCases[i][0]);
    analogWrite(outG,testCases[i][1]);
    analogWrite(outB,testCases[i][2]);
    for (int j = 0; j <5;j++){
       delay(T); //Comes before the reading so the PR has time to respond to the initial lighting conditions
       sample += analogRead(analogPR);
    }
    colourVec[i] = sample/5; 
  }
}

void printBaselines(int *colourVec){
  for (int col = 0; col < 4; col++){
    Serial.print(colourVec[col]);
  }
}

void baselineSetUpRoutine(){
  Serial.println("Please remove all blocks from colour sensor, ambient test starts in 5 seconds")
  delay(5000);
  Serial.println("Ambient Test Starting");
  getBaseline(rgbBaselines[0]);
  Serial.println("Please place Red block under colour sensor, test starts in 5 seconds")
  delay(5000);
  Serial.println("Red Test Starting");
  getBaseline(rgbBaselines[1]);
  Serial.println("Please place Green block under colour sensor, test starts in 5 seconds")
  delay(5000);
  Serial.println("Green Test Starting");
  getBaseline(rgbBaselines[2]);
  Serial.println("Please place Blue block under colour sensor, test starts in 5 seconds")
  delay(5000);
  Serial.println("Blue Test Starting");
  getBaseline(rgbBaselines[3]);
  Serial.println("No Block Responses to Null, R, G ,B:");
  printBaselines(rgbBaselines[0]);
  Serial.println("Red Block Responses to Null, R, G ,B:");
  printBaselines(rgbBaselines[1]);
  Serial.println("Green Block Responses to Null, R, G, B");
  printBaselines(rgbBaselines[2]);
  Serial.println("Blue Block Responses to Null, R, G, B");
  printBaselines(rgbBaselines[3]);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  baselineSetUpRoutine();
  Serial.println("Please put down a block for testing")
  colourId(); 
}


void loop() {
  // put your main code here, to run repeatedly:

}
