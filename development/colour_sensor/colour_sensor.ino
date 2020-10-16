int argbCalibrationVals[4][4]; // Each row contains measurements for a different block, each column is a given set of conditions 
int testCases[4][3] = {{0, 0, 0},{1, 0 ,0}, {0, 1, 0},{0, 0, 1}};
const char colourIdStrings[4][15] = {"Ambient", "Red", "Green", "Blue"};

const int analogPR = A1;
const int outR = 2;
const int outG = 3;
const int outB = 4;

int arr_minDex(int arr[])
{
  // Find the minimum squared error 
  int minDex = 0;
  int minVal = arr[0];
  for (int c = 1; c<4;c++)
  {
    if (arr[c] < minVal)
    {
      minDex = c;
      minVal = arr[c];
    }
  }

  return minDex;
}

void colourId()
{
  int abs_error_vec[4] = {0,0,0,0};
  int sample;
  int numTestCases = 4;
  
  for(int i = 1; i < numTestCases; i++)
  {
    sample = 0; 
    digitalWrite(outR,testCases[i][0]);
    digitalWrite(outG,testCases[i][1]);
    digitalWrite(outB,testCases[i][2]);

    for (int j = 0; j < 15; j++)
    {
       delay(100); 
       sample += analogRead(analogPR);
    }

    for (int c = 0; c < 4; c++)
    {
      abs_error_vec[c] = abs(sample/15 - argbCalibrationVals[c][i]); //removed +=
    }
  }

  int minDex = arr_minDex(abs_error_vec);
  Serial.println(colourIdStrings[minDex]);
}

void getBaseline(int *colourVec)
{ 
  int sample;
  int numTestCases = 4;

  for (int i = 0; i<numTestCases;i++)
  { 
    sample = 0;
    digitalWrite(outR,testCases[i][0]);
    digitalWrite(outG,testCases[i][1]);
    digitalWrite(outB,testCases[i][2]);

    for (int j = 0; j <15;j++)
    {
       delay(100); 
       sample += analogRead(analogPR);
    }
    colourVec[i] = sample/15; 
  }
}

void printBaselines(int *colourVec)
{
  for (int col = 0; col < 4; col++)
  {
    Serial.println(colourVec[col]);
  }
}

void calibrate()
{
  Serial.println("Please remove all blocks from colour sensor, ambient test starts in 5 seconds");
  delay(5000);
  Serial.println("Ambient Test Starting");
  getBaseline(argbCalibrationVals[0]);
  Serial.println("Please place Red block under colour sensor, test starts in 5 seconds");
  delay(5000);
  Serial.println("Red Test Starting");
  getBaseline(argbCalibrationVals[1]);
  Serial.println("Please place Green block under colour sensor, test starts in 5 seconds");
  delay(5000);
  Serial.println("Green Test Starting");
  getBaseline(argbCalibrationVals[2]);
  Serial.println("Please place Blue block under colour sensor, test starts in 5 seconds");
  delay(5000);
  Serial.println("Blue Test Starting");
  getBaseline(argbCalibrationVals[3]);

  // calibration output
  Serial.println("No Block Responses to Null, R, G ,B:");
  printBaselines(argbCalibrationVals[0]);
  Serial.println("Red Block Responses to Null, R, G ,B:");
  printBaselines(argbCalibrationVals[1]);
  Serial.println("Green Block Responses to Null, R, G, B");
  printBaselines(argbCalibrationVals[2]);
  Serial.println("Blue Block Responses to Null, R, G, B");
  printBaselines(argbCalibrationVals[3]);
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(outR, OUTPUT);
  pinMode(outG, OUTPUT);
  pinMode(outB, OUTPUT);

  calibrate();
}


void loop() 
{
  // put your main code here, to run repeatedly:
  Serial.println("Please put down a block for testing");
  delay(5000);
  colourId(); 
}
