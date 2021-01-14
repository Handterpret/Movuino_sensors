#ifndef HANDTERPRET_H
#define HANDTERPRET_H
#include "Arduino.h"
class Handterpret
{
    private:
        int16_t AccX;
        int16_t AccY;
        int16_t AccZ;
        float GyroX;
        float GyroY;
        float GyroZ;
    public:
        Handterpret(int16_t AX, int16_t AY, int16_t AZ, float GX, float GY, float GZ);
        Handterpret();
        ~Handterpret();
        void get_metrics(float (&metrics)[6]);
        void update_metrics(float metrics[6]);
        void display_metrics();
};
#endif /* HANDTERPRET_H */
