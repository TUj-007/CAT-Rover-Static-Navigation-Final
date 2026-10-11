#pragma once
#include <stdint.h>

// 0: drive, 1: hold zero, -1: latch polarity/motion fault.
struct DirectionSafety {
  int direction=0;
  bool reversing=false;
  uint32_t until=0;
  int check(int requested,float speed,uint32_t now,uint16_t pause) {
    if(direction&&requested!=direction){reversing=true;until=now+pause;}
    direction=requested;
    if(reversing&&(int32_t)(until-now)>0)return 1;
    if(speed*requested<-.02f){
      if(reversing&&(int32_t)(until+300-now)>0)return 1;
      return -1;
    }
    reversing=false;return 0;
  }
};
