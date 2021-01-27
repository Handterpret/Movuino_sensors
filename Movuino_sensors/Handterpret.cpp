#include "Handterpret.h"

//default constructor
Handterpret::Handterpret()
{
    measure_index = 0;
    for(int i = 0; i < 3; i++) {
        Acc[i] = 0;
        Gyro[i] = 0;
        prev_av_Gyro[i] = 0;
        rot_state[i] = 0;
        trans_state[i] = 0;
        trans_speed[i] = 0;
        hist_Gyro[i] = 0;
        num_translation[i] = 0;
        num_translation[3+i] = 0;
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
        hist_Gyro[i] += metrics[3+i];
        trans_speed[i] += filter_acc(metrics[i]);
    }
    //Serial.println(metrics[5]);
    measure_index++;
    if (measure_index == MEASURES_PER_PERIOD) {
        unsigned long CurrentTime = millis();
        unsigned long ElapsedTime = CurrentTime - StartTime;
        //Serial.print("Time elapsed ");Serial.println(ElapsedTime);
        check_movement();
        measure_index = 0;
        //reset each period because of Accelerometers noise
        for(int i = 0; i < 3; i++) {
            trans_speed[i] = 0;
        }
    }
}
void Handterpret::check_movement() {
    float av_Gyro[3] = {0, 0, 0};
    byte current_rot_state[3] = {0, 0, 0};
    byte current_trans_state[3] = {0, 0, 0};
    //display_metrics();
    //Serial.print("Average : ");
    for(int i = 0; i < 3; i++) {
        //av_Gyro[i] = average_historic(hist_Gyro[i]);
        //av_Gyro[i] = filter_rot(av_Gyro[i], prev_av_Gyro[i], i);
        //current_rot_state[i] = detect_rotation_mov(av_Gyro[i], prev_av_Gyro[i]);
        current_trans_state[i] = detect_translation_mov(trans_speed[i]);
        find_tranlation_movement(current_trans_state[i], trans_state[i], i);
    }
    //Serial.println();
    /*if (discrimine_rotation(current_rot_state)) {
        for(int i = 0; i < 3; i++)
            find_tranlation_movement(current_trans_state[i], trans_state[i], i);
    }*/
    // store new average rotation, rotation and translation status
    for(int i = 0; i < 3; i++) {
        rot_state[i] = current_rot_state[i];
        prev_av_Gyro[i] = av_Gyro[i];
        trans_state[i] = current_trans_state[i];
        hist_Gyro[i] = 0;
    }
}
int16_t Handterpret::filter_acc(int16_t acc) {
    if (acc > ACCELERATION_SENSIBILITY || acc < -ACCELERATION_SENSIBILITY)
        return acc;
    else return 0;
}
bool Handterpret::discrimine_rotation(byte current_rot_state[3]) {
    /*if (current_rot_state[1] != rot_state[1]) {
            print_rotation(rot_state[1], 1);
    }*/
    if (current_rot_state[2] != rot_state[2]) {
            print_rotation(rot_state[2], 2);
            return false;
    }/*
    else if (current_rot_state[0] != rot_state[0]) {
            print_rotation(rot_state[0], 0);
    }*/
    else return true;
}
void Handterpret::find_tranlation_movement(byte current_trans_state, byte post_trans_state, byte axis) {
    if (current_trans_state != post_trans_state) {
        if (current_trans_state == 2 && post_trans_state == 1)
            print_translation(post_trans_state, axis);
        else if (current_trans_state == 1 && post_trans_state == 2) {
            print_translation(post_trans_state, axis);
        }
    }
}
void Handterpret::print_rotation(byte rot, byte axis) {
    if (rot == 2) {
        switch(axis) {
            case 0 :
                Serial.println("adduction detected");
                break;
            case 1 :
                Serial.println("flexion detected");
                break;
            case 2 :
                Serial.println("supination detected");
                break;
        }
    }
    else if (rot == 1) {
        switch(axis) {
            case 0 :
                Serial.println("abduction detected");
                break;
            case 1 :
                Serial.println("extension detected");
                break;
            case 2 :
                Serial.println("pronation detected");
                break;
        }
    }
}
void Handterpret::print_translation(byte acc, byte axis) {
    if (acc == 2) {
        switch(axis) {
            case 0 :
                Serial.print(num_translation[0]);Serial.print(" ");Serial.println("left translation detected");
                num_translation[0]+=1;
                break;
            case 1 :
                Serial.print(num_translation[1]);Serial.print(" ");Serial.println("backward translation detected");
                num_translation[1]+=1;
                break;
            case 2 :
                Serial.print(num_translation[2]);Serial.print(" ");Serial.println("upper translation detected");
                num_translation[2]+=1;
                break;
        }
    }
    else if (acc == 1) {
        switch(axis) {
            case 0 :
                Serial.print(num_translation[3]);Serial.print(" ");Serial.println("right translation detected");
                num_translation[3]+=1;
                break;
            case 1 :
                Serial.print(num_translation[4]);Serial.print(" ");Serial.println("forward translation detected");
                num_translation[4]+=1;
                break;
            case 2 :
                Serial.print(num_translation[5]);Serial.print(" ");Serial.println("lower translation detected");
                num_translation[5]+=1;
                break;
        }
    }
}
byte Handterpret::detect_rotation_mov(float av_gyro, float prev_av_gyro) {
    if (av_gyro >= prev_av_gyro + ROTATION_SENSIBILITY) {
        return 2;
    }
    else if (av_gyro <= prev_av_gyro - ROTATION_SENSIBILITY) {
        return 1;
    }
    else return 0;
}
byte Handterpret::detect_translation_mov(float speed) {
    if (speed > 0) {
        return 2;
    }
    else if (speed < 0) {
       return 1;
    }
    else return 0;
}
float Handterpret::average_historic(float hist) {
    return hist/MEASURES_PER_PERIOD;
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
void Handterpret::display_speeds() {
    //special value of tab to match all different consoles config for \t
    char * tab = "    ";
    Serial.print("sensor speeds : ");
    for(int i = 0; i < 3; i++) {
        Serial.print(trans_speed[i]);Serial.print(tab);
    }
    Serial.println();
}
float Handterpret::filter_rot(float current_rot_av, float prev_rot_av, byte axis) {
    if (current_rot_av >= 80 || current_rot_av <= -80) {
        if ((current_rot_av < 0) == (prev_rot_av < 0)) {
            return current_rot_av;
        }
        else if (prev_rot_av >= 80 || prev_rot_av <= -80 ){
          Serial.println(current_rot_av);
          return prev_rot_av;
        }
    }
    else
        return current_rot_av;
}
