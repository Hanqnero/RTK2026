#include "encoder.h"

Encoder enc_left(LEFT_ENC_A,  LEFT_ENC_B);
Encoder enc_right(RIGHT_ENC_A, RIGHT_ENC_B);

void enc_left_isr()  { enc_left.on_change();  }
void enc_right_isr() { enc_right.on_change(); }

void encoders_begin() {
    enc_left.begin();
    enc_right.begin();
    // CHANGE — срабатывает и на передний и на задний фронт канала A (режим 2x)
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  enc_left_isr,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), enc_right_isr, CHANGE);
}
