#include "Handterpret.h"

Handterpret::Handterpret(int16_t AX, int16_t AY, int16_t AZ, float GX, float GY, float GZ)
    : AccX(AX), AccY(AY), AccZ(AZ), GyroX(GX), GyroY(GY), GyroZ(GZ) {}

Handterpret::Handterpret()
{
    AccX = 0;
    AccY = 0;
    AccZ = 0;
    GyroX = 0;
    GyroY = 0;
    GyroZ = 0;
}

Handterpret::~Handterpret(){}

void Handterpret::get_metrics(float (&metrics)[6]) {
    metrics[0] = AccX;
    metrics[1] = AccY;
    metrics[2] = AccZ;
    metrics[3] = GyroX;
    metrics[4] = GyroY;
    metrics[5] = GyroZ;
}

void Handterpret::update_metrics(float metrics[6]) {
    AccX = metrics[0];
    AccY = metrics[1];
    AccZ = metrics[2];
    GyroX = metrics[3];
    GyroY = metrics[4];
    GyroZ = metrics[5];
}
void Handterpret::display_metrics() {
    //special value of tab to match all different consoles config for \t
    char * tab = "    ";
    Serial.print("sensor metrics : AccX ");
    Serial.print(AccX);Serial.print(tab);
    Serial.print(AccY);Serial.print(tab);
    Serial.print(AccZ);Serial.print(tab);
    Serial.print(GyroX);Serial.print(tab);
    Serial.print(GyroY);Serial.print(tab);
    Serial.print(GyroZ);Serial.println();
}
