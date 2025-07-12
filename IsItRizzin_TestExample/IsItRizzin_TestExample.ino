void setup() {
Serial1.begin(115200);
}

void loop() {
float Value1 = 1.0;
float Value2 = 2.0;
float Value3 = 3.0;
float Value4 = 4.0;
float Value5 = 5.0;
float Value6 = 6.0;
float Value7 = 7.0;
float Value8 = 8.0;

Serial1.printf("Label1,%f,Label2,%f,Label3,%f,Label4,%f,Label5,%f,Label6,%f,Label7,%f,Label8,%f\n",Value1,Value2,Value3,Value4,Value5,Value6,Value7,Value8);
delay(500); 
  }
