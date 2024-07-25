#ifndef _InverseKinematics_H_
#define _InverseKinematics_H_

extern "C"{
  class Machine { //machine class
  public:
    //class functions
    Machine(float d, float e, float f, float g);
    double theta(int leg, float hz, float nx, float ny); //returns the value of theta a, b, or c
};
}

#endif
