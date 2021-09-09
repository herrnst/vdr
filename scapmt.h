/*
 * scapmt.h: Static CaPmt pid mapping
 */

#ifndef __SCA_H
#define __SCA_H

///

#include "remux.h"

#define SCA_MAX_PIDS     MAXPID // real PIDs are 13 bit (0x0000 - 0x1FFF)
#define SCA_INVALID_PID  0xFFFF // see MTD_INVALID_ID in mtd.c

#define SCA_MAX_SERVICES 15 // 16 - 1 for a dummy service
#define SCA_MAX_CAPIDS   16
#define SCA_MAX_ESPIDS  (SCA_MAX_CAPIDS - 1) // -1 for ECM-Pid

#define SCA_SLOT_SHIFT   4
#define SCA_SLOT_MASK    0xF
#define SCA_INDEX_MASK   0xF

#define SCA_CAPID_BIT 0x1000 // 1 << 12
#define SCA_PID_BIT   0x800  // 1 << 11

#define SCA_PID(p)            (SCA_PID_BIT   | (p))
#define SCA_CAPID(slot,index) (SCA_CAPID_BIT | ((slot) << SCA_SLOT_SHIFT) | (index))

#define SCA_MAX_UNIQ_PIDS 0x800   // common uniq PIDs are 11 bit (0x0000 - 0x07FF)
#define SCA_UNIQ_PID_MASK 0x7FF

#define TSNULL 0x1FFF
#define SCAMAPPED(p)    ((p) >= SCA_PID_BIT && (p) != TSNULL)

class cCamSLot;
class cCiCaPmtList;

class cScaMapper {
private:
  cMutex mutex;
  uint16_t uniqPids[SCA_MAX_PIDS]; // maps a real PID to a unique PID
  uint16_t realPids[SCA_MAX_PIDS]; // maps a unique PID to a real PID
  bool uniqUsed[SCA_MAX_UNIQ_PIDS]; // availability of unique PIDs
  int nextUniqPid;

  uint16_t camServices[SCA_MAX_SERVICES][ 5 + SCA_MAX_ESPIDS + 1]; // caid + NumPids + flags + sid + ecm + pids.. zero terminated
  uint16_t pmtServices[SCA_MAX_SERVICES][ 5 + SCA_MAX_ESPIDS + 1];
  int numPmtServices;

  int maxSids;
  int maxPids;
  cCamSlot *masterSlot;
  cVector<uint32_t> vScaConf;
  bool caPmtSent;

  bool ecmAction;
  bool ecmShared;

  void Clear(void);
  uint16_t MapUniqCaPid(uint16_t RealPid, int Service, int Index);
  uint16_t MapUniqPid(uint16_t RealPid);

  bool EcmTrigger(uchar *TsPacket);
  bool HasSharedEcm(uint16_t CaId);
  void RemapSharedEcm(uint16_t CaId, uint16_t Ecm);

  int GetServiceSlot(uint16_t CaId, uint16_t Sid);
  void AddService(int ServiceSlot, uint16_t *sPmt);
  void DelService(int ServiceSlot);

  bool ParseCaPmts(cCiCaPmtList &CaPmtList);
  void ParseCaPmt(cDynamicBuffer *capmt, cVector<uint16_t> &esPids, cVector<uint16_t> &ecmPids, cVector<uint16_t> &caIds);

public:
  cScaMapper(cVector<uint32_t> &VscaConf, cCamSlot *MasterSlot);
  ~cScaMapper();
  uint16_t RealToUniqPid(uint16_t RealPid) { return uniqPids[RealPid] ? uniqPids[RealPid] : MapUniqPid(RealPid); }
  uint16_t UniqToRealPid(uint16_t UniqPid) { return realPids[UniqPid]; }
  uint16_t UniqCaPid(int ServiceSlot, int Index);
  uchar *TsPreProcess(uchar *Data, int &Count);
  void MapStaticCaPmts(cCiCaPmtList &CaPmtList);
  void SetPid(int RealPid, bool On);
  int NumSids(void) { return maxSids; }
  int NumPids(void) { return maxPids; }
  };

#endif //__SCA_H
