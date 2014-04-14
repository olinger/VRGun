#include "PVision.h"
#include "Wire.h"

typedef struct
{
  int16_t blob1X;
  int16_t blob1Y;
  int16_t blob2X;
  int16_t blob2Y;
  int32_t a, b, c, d, e, f, g, h, i, j, k, l, m, n;
} buffer_t;

PVision ircam;
byte result;

buffer_t b;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  pinMode(13, OUTPUT);
  
  ircam.init();
}

void loop()
{
  //result = ircam.read();
  
  if (result & BLOB1)
  {
    b.blob1X = ircam.Blob1.X;
    b.blob1Y = ircam.Blob1.Y;
  }
  else
  {
    b.blob1X = -1;
    b.blob1Y = -1;
  }
  
  if (result & BLOB2)
  {
    b.blob2X = ircam.Blob2.X;
    b.blob2Y = ircam.Blob2.Y;
  }
  else
  {
    b.blob2X = -1;
    b.blob2Y = -1;
  }
  
  int n = RawHID.send(&b, 1);
  delay(100);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
}
