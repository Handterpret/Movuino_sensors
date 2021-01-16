#include "Handterpret.h"

//default constructor
Handterpret::Handterpret()
{
    hist_index = 0;
    for(int i = 0; i < 3; i++) {
        Acc[i] = 0;
        Gyro[i] = 0;
        prev_av_Gyro[i] = 0;
        rot_state[i] = 0;
        trans_state[i] = 0;
        for(int j = 0; j < 20; j++) {
            hist_Gyro[i][j] = 0;
            hist_Gyro[i][j] = 0;
            hist_Gyro[i][j] = 0;
        }
    }
}

Handterpret::~Handterpret(){}

void Handterpret::get_metrics(float (&metrics)[6]) {
    for(int i = 0; i < 3; i++) {
        metrics[i] = Acc[i];
        metrics[3+i] = Gyro[i];
    }
}

void Handterpret::update(float metrics[6]) {
    for(int i = 0; i < 3; i++) {
        Acc[i] = metrics[i];
        Gyro[i] = metrics[3+i];
        hist_Gyro[i][hist_index] = metrics[3+i];
    }
    hist_index++;
    if (hist_index == HISTORY_SIZE) {
        unsigned long CurrentTime = millis();
        unsigned long ElapsedTime = CurrentTime - StartTime;
        //Serial.print("Time elapsed ");Serial.println(ElapsedTime/1000);
        check_movement();
        hist_index = 0;
    }
}
void Handterpret::check_movement() {
    float av_Gyro[3] = {0, 0, 0};
    byte current_rot_state[3] = {0, 0, 0};
    byte current_trans_state[3] = {0, 0, 0};
    for(int i = 0; i < 3; i++) {
        av_Gyro[i] = average_historic(hist_Gyro[i]);
        current_rot_state[i] = detect_rotation_mov(av_Gyro[i], prev_av_Gyro[i]);
        current_trans_state[i] = detect_translation_mov(Acc[i]);
        print_translation(current_trans_state[i], i);
        //Serial.print(" ");Serial.print(current_rot_state[i]);
    }
    Serial.println();
    discrimine_rotation(current_rot_state);
    for(int i = 0; i < 3; i++) {
        rot_state[i] = current_rot_state[i];
        prev_av_Gyro[i] = av_Gyro[i];
    }
}
void Handterpret::discrimine_rotation(byte current_rot_state[3]) {
    if (current_rot_state[1] != rot_state[1]) {
            print_rotation(rot_state[1], 1);
    }
    else if (current_rot_state[2] != rot_state[2]) {
            print_rotation(rot_state[2], 2);
    }
    else if (current_rot_state[0] != rot_state[0]) {
            print_rotation(rot_state[0], 0);
    }
}
void Handterpret::print_rotation(byte rot, byte axis) {
    if (rot == 2) {
        switch(axis) {
            case 0 :
                Serial.println("adduction detected");
                break;
            case 1 :
                Serial.println("supination detected");
                break;
            case 2 :
                Serial.println("flexion detected");
                break;
        }
    }
    else if (rot == 1) {
        switch(axis) {
            case 0 :
                Serial.println("abduction detected");
                break;
            case 1 :
                Serial.println("pronation detected");
                break;
            case 2 :
                Serial.println("extension detected");
                break;
        }
    }
}
void Handterpret::print_translation(byte acc, byte axis) {
    if (acc == 2) {
        switch(axis) {
            case 0 :
                Serial.println("right translation detected");
                break;
            case 1 :
                Serial.println("forward translation detected");
                break;
            case 2 :
                Serial.println("upper translation detected");
                break;
        }
    }
    else if (acc == 1) {
        switch(axis) {
            case 0 :
                Serial.println("left translation detected");
                break;
            case 1 :
                Serial.println("backward translation detected");
                break;
            case 2 :
                Serial.println("lower translation detected");
                break;
        }
    }
}

byte Handterpret::detect_rotation_mov(float av_gyro, float prev_av_gyro) {
    if (av_gyro >= prev_av_gyro + ROTATION_SENSIBILITY) {
        /*Serial.print("Actual average : "); Serial.println(av_gyro);
        Serial.print("Previous average : "); Serial.println(prev_av_gyro);
        */
        return 2;
    }
    else if (av_gyro <= prev_av_gyro - ROTATION_SENSIBILITY) {
        /*Serial.print("Actual average : "); Serial.println(av_gyro);
        Serial.print("Previous average : "); Serial.println(prev_av_gyro);
        */
       return 1;
    }
    else return 0;
}
byte Handterpret::detect_translation_mov(int16_t acc_val) {
    if (acc_val >= ACCELERATION_SENSIBILITY) {
        return 2;
    }
    else if (acc_val <= -ACCELERATION_SENSIBILITY) {
       return 1;
    }
    else return 0;
}
float Handterpret::average_historic(float hist[HISTORY_SIZE]) {
    float sum = 0.;
    for (int i = 0 ; i < HISTORY_SIZE; i++) {
        sum += hist[i];
    }
    return sum/HISTORY_SIZE;
}
void Handterpret::display_metrics() {
    //special value of tab to match all different consoles config for \t
    char * tab = "    ";
    Serial.print("sensor metrics : Acc ");
    for(int i = 0; i < 3; i++) {
        Serial.print(Acc[i]);Serial.print(tab);
    }
    Serial.print("   Gyro   ");
    for(int i = 0; i < 3; i++) {
        Serial.print(Gyro[i]);Serial.print(tab);
    }
    Serial.println();
}
