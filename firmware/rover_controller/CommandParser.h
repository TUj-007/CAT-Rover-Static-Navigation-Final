#pragma once
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

inline uint16_t wireCRC(const char* s) {
  uint16_t crc=0xFFFF;
  while(*s) { crc^=(uint16_t)(uint8_t)*s++<<8;
    for(uint8_t i=0;i<8;i++) crc=(crc&0x8000)?(uint16_t)((crc<<1)^0x1021):(uint16_t)(crc<<1); }
  return crc;
}
inline bool wireBody(char* line) {
  char* star=strchr(line,'*');
  if(!star || strlen(star+1)!=4) return false;
  uint16_t want=0;
  for(uint8_t i=1;i<=4;i++) { char c=star[i]; uint8_t v;
    if(c>='0'&&c<='9')v=c-'0'; else if(c>='A'&&c<='F')v=c-'A'+10;
    else if(c>='a'&&c<='f')v=c-'a'+10; else return false;
    want=(want<<4)|v;
  }
  *star=0; return wireCRC(line)==want;
}
inline bool wireUnsigned(const char* s,uint32_t& v) {
  if(!s||!*s)return false; v=0;
  while(*s) { if(*s<'0'||*s>'9')return false; uint8_t d=*s++-'0';
    if(v>429496729UL||(v==429496729UL&&d>5))return false; v=v*10+d; }
  return true;
}
inline bool wireSpeed(const char* s,int16_t& v) {
  if(!s)return false; bool negative=*s=='-'; if(negative)s++;
  uint32_t n; if(!wireUnsigned(s,n)||n>300)return false;
  v=negative?-(int16_t)n:(int16_t)n; return true;
}
struct CommandGate {
  bool zeroSeen=false, hasSequence=false;
  uint32_t sequence=0, acceptedAt=0;
  int16_t left=0,right=0;
  void reset() {zeroSeen=false;hasSequence=false;left=right=0;}
  bool accept(char* line,uint32_t now,uint32_t epoch) {
    if(!wireBody(line))return false;
    uint8_t commas=0; for(char* p=line;*p;p++)if(*p==',')commas++;
    if(commas!=5||strstr(line,",,"))return false;
    char* fields[6]; fields[0]=line; uint8_t count=1;
    for(char* p=line;*p;p++)if(*p==','){*p=0;fields[count++]=p+1;}
    if(strcmp(fields[0],"C"))return false;
    uint32_t ep,seq,stamp; int16_t l,r;
    if(!wireUnsigned(fields[1],ep)||!wireUnsigned(fields[2],seq)||!wireUnsigned(fields[3],stamp)
      ||!wireSpeed(fields[4],l)||!wireSpeed(fields[5],r))return false;
    if(ep!=epoch || now-stamp>250 || (hasSequence&&(int32_t)(seq-sequence)<=0))return false;
    if(!zeroSeen&&(l||r))return false;
    zeroSeen=true;hasSequence=true;sequence=seq;acceptedAt=now;left=l;right=r;return true;
  }
};
