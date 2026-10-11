#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/rover_controller/CommandParser.h"
#include "../firmware/rover_controller/DirectionSafety.h"

bool feed(CommandGate &g,const char *payload,uint32_t now,uint32_t epoch) {
  char line[100]; snprintf(line,sizeof(line),"%s*%04X",payload,wireCRC(payload));
  return g.accept(line,now,epoch);
}
int main() {
  DirectionSafety d;
  assert(d.check(1,0,100,120)==0);
  assert(d.check(1,-.05f,120,120)==-1); // Steady reverse feedback faults.
  DirectionSafety r;
  assert(r.check(1,.08f,100,120)==0);
  assert(r.check(-1,.08f,120,120)==1); // Expected coast while reversing.
  assert(r.check(-1,.08f,250,120)==1);
  assert(r.check(-1,.08f,550,120)==-1); // Bounded coast interval.
  DirectionSafety s;
  s.check(1,.08f,100,120);s.check(-1,.08f,120,120);
  assert(s.check(-1,0,250,120)==0);
  assert(s.check(-1,-.08f,270,120)==0);
  assert(wireCRC("123456789")==0x29B1);
  CommandGate g;
  assert(!feed(g,"C,1,1,100,50,50",100,1));
  assert(feed(g,"C,1,1,100,0,0",100,1));
  assert(feed(g,"C,1,2,100,-80,80",100,1));
  assert(g.left==-80 && g.right==80);
  assert(!feed(g,"C,1,2,100,90,90",100,1));
  assert(!feed(g,"C,1,3,100,301,0",100,1));
  assert(!feed(g,"C,2,3,100,0,0",100,1));
  assert(!feed(g,"C,1,3,100,0,0",400,1));
  assert(!feed(g,"C,1,4294967296,100,0,0",100,1));
  assert(!feed(g,"C,1,3,100,,0",100,1));
  assert(!feed(g,"C,1,3,100,0,0,0",100,1));
  char corrupt[]="C,1,3,100,0,0*0000"; assert(!g.accept(corrupt,100,1));
  g.reset(); assert(!feed(g,"C,2,4,100,80,80",100,2));
  assert(feed(g,"C,2,4,100,0,0",100,2));
  puts("Firmware parser vectors passed"); return 0;
}
