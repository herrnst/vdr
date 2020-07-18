/*
 * scapmt.c: Static CaPmt pid mapping
 */

#include "libsi/si.h"
#include "ci.h"
#include "scapmt.h"

 // set by camtweaks.conf
extern bool DebugCamtweaks;
extern bool DebugCamtweaksMtd;

#define DBGSTATICMAP(a...) if (DebugCamtweaks) dsyslog(a)
#define DBGSTATIC(a...) if (DebugCamtweaksMtd) dsyslog(a)

#define SCASLOT(u)       ((u >> SCA_SLOT_SHIFT) & SCA_SLOT_MASK)
#define IS_SCA_ECMPID(p) ((p & (SCA_CAPID_BIT | SCA_INDEX_MASK)) == SCA_CAPID_BIT)
#define IS_SCA_CAPID(u)  (u & SCA_CAPID_BIT)
#define IS_SCA_PID(u)    (u & SCA_PID_BIT)

// --- cScaMapper ------------------------------------------------------------

enum { oCAID    = 0,
       oNumPids = 1,
       oTrigger = 2,
       oSID     = 3,
       oECM     = 4,
       oPIDS    = 5
       };

cScaMapper::cScaMapper(cVector<uint32_t> &VscaConf, cCamSlot *MasterSlot)
{
  maxSids = 0;
  maxPids = 0;
  masterSlot = MasterSlot;
  caPmtSent = false;

  Clear();
  // initialize the service slots with CAID and number of pids
  for (int n = 0; n < VscaConf.Size(); n++) {
      uint32_t sca = VscaConf[n];
      vScaConf.Append(sca); // copy
      int caId = (sca >> 16) & 0xFFFF;
      int sids = (sca >>  8) & 0xFF;
      int pids = (sca      ) & 0xFF;
      dsyslog("CAM %d: activating static CaPmt mapping: CAID 0x%04X, %d services, %d pids", MasterSlot->SlotNumber(), caId, sids, pids);
      maxPids = max(maxPids, pids); // ?
      for (int s = 0; s < sids && maxSids < SCA_MAX_SERVICES; s++) {
          uint16_t *sCam = camServices[maxSids++];
          sCam[oCAID]    = caId;
          sCam[oNumPids] = pids;
          }
      }
}

cScaMapper::~cScaMapper()
{
}

void cScaMapper::Clear(void)
{
  memset(uniqPids, 0, sizeof(uniqPids));
  memset(realPids, SCA_INVALID_PID, sizeof(realPids));
  memset(uniqUsed, 0, sizeof(uniqUsed));
  nextUniqPid = 0;

  memset(camServices, 0, sizeof(camServices));
  memset(pmtServices, 0, sizeof(pmtServices));
  numPmtServices = 0;

  ecmAction = false;
  ecmShared = false;
}

uint16_t cScaMapper::UniqCaPid(int ServiceSlot, int Index)
{
  return SCA_CAPID(ServiceSlot,Index);
}

// map CAPMT Pids
uint16_t cScaMapper::MapUniqCaPid(uint16_t RealPid, int ServiceSlot, int Index)
{
  uint16_t uniq = SCA_CAPID(ServiceSlot,Index); // 0x1000...0x10FF
  uint16_t uniqCurr = uniqPids[RealPid];

  bool shared = false;
  if (IS_SCA_CAPID(uniqCurr)) {
     int slot = SCASLOT(uniqCurr);
     if (slot != ServiceSlot && camServices[slot][oSID]) { // uses by another active slot
        DBGSTATICMAP(">>> StatCapmt: shared CAPID %d(%04X) slot[%d] -> [%d]", RealPid, RealPid, slot, ServiceSlot);
        shared = true;
        }
     }
  if (!shared)
     uniqPids[RealPid] = uniq;
  realPids[uniq] = RealPid;
  DBGSTATICMAP(">>> StatCapmt: slot[%d] map CAPID %d(%04X) to %d(%04X)", ServiceSlot, RealPid, RealPid, uniq, uniq);
  return uniq;
}

// map common Pids
uint16_t cScaMapper::MapUniqPid(uint16_t RealPid)
{
  for (int i = 0, n = nextUniqPid; i < SCA_MAX_UNIQ_PIDS; i++, n++) {
      n %= SCA_MAX_UNIQ_PIDS;
      if (!uniqUsed[n]) { // inactive
         int uniq = SCA_PID(n); // 0x800...0xFFF

         int realCurr = realPids[uniq];
         if (realCurr != SCA_INVALID_PID && IS_SCA_PID(uniqPids[realCurr])) // old real <-> uniq connection
            uniqPids[realCurr] = 0; // unlink

         realPids[uniq] = RealPid;
         uniqPids[RealPid] = uniq;
         DBGSTATICMAP(">>> StatCapmt: mapped PID %d(%04X) to %d(%04X)", RealPid, RealPid, uniq, uniq);

         uniqUsed[n] = true; // mark active
         nextUniqPid = n + 1;
         return uniq;
         }
      }
  esyslog(">>> StatCapmt: ERROR - mapper ran out of unique PIDs");
  return 0;
}

// free unused common uniq Pid mapping
void cScaMapper::SetPid(int RealPid, bool On)
{
  DBGSTATICMAP("...cs:setpid %c %d(%X)", On ? '+' : '-', RealPid, RealPid);
  if (!On) { // disable
     int uniq = uniqPids[RealPid];
     if (IS_SCA_PID(uniq))
        uniqUsed[uniq & SCA_UNIQ_PID_MASK] = On;
     }
}

// --- static CAPMT mapping and shared ECM handling

#define RESET_ECM 1

uchar *cScaMapper::TsPreProcess(uchar *Data, int &Count)
{
  cMutexLock MutexLock(&mutex);
  int pid = TsPid(Data);
  int uniq = RealToUniqPid(pid);

  if (uniq == TSNULL) // unwanted Pid -> skip
     return NULL;

  TsSetPid(Data, uniq); // STATIC CAPMT pid mapping
  ////
  //// handle shared Ecm and clearing of the CAMs ecm bufferr
  ////
  if (ecmAction && IS_SCA_ECMPID(uniq)) {
     ecmAction = ecmShared;
     int slot = SCASLOT(uniq);
     for (int i = 0; i < maxSids; i++) {
         uint16_t *sCam = camServices[i];
         if (sCam[oSID] && sCam[oECM] == pid) { // active slot for this ECM Pid
            if (i != slot) { // slot uses a shared ECM
               //DBGSTATICMAP(">>> StatCapmt: Inject shared ECM pid 0x%X as 0x%X for slot[%d]", uniq, SCA_CAPID(i, 0), i);
               TsSetPid(Data, SCA_CAPID(i, 0));
#if RESET_ECM
               if (sCam[oTrigger]) // new service
                  sCam[oTrigger] = !EcmTrigger(Data);
#endif
               masterSlot->Inject(Data, TS_SIZE); // duplicate
               TsSetPid(Data, uniq); // restore pid
               }
#if RESET_ECM
            else if (sCam[oTrigger]) // new service
               sCam[oTrigger] = !EcmTrigger(Data);
            if (sCam[oTrigger])
               ecmAction = true; // need ecmTrigger
#endif
            }
         }
     }
  ////
  return Data;
}

bool cScaMapper::EcmTrigger(uchar *TsPacket)
{
  // inject with toggled ECM Table-Id to clear CAMs ECM buffer ?
  if (TsPacket[4] == 0) { // simple: expect payload at offset 0
     uint8_t tableId = TsPacket[5];
     if ((tableId & 0xFE) == 0x80) { // ECM odd/even data
        TsPacket[5] = tableId == 0x80 ? 0x81 : 0x80;
        DBGSTATICMAP(">>> StatCapmt: Trigger ECM data with TableId 0x%02X/0x%02X", TsPacket[5], tableId);
        // 3 times the 'other'
        masterSlot->Inject(TsPacket, TS_SIZE);
        masterSlot->Inject(TsPacket, TS_SIZE);
        masterSlot->Inject(TsPacket, TS_SIZE);
        // 2 + 1 times 'current'
        TsPacket[5] = tableId;
        masterSlot->Inject(TsPacket, TS_SIZE);
        masterSlot->Inject(TsPacket, TS_SIZE);
        return true;
        }
     }
  return false;
}

bool cScaMapper::HasSharedEcm(uint16_t CaId)
{
  for (int i = 0; i < maxSids; i++) {
      uint16_t *sCam1 = camServices[i];
      if (sCam1[oCAID] == CaId && sCam1[oSID]) { // active slot
         for (int j = i + 1; j < maxSids; j++) {
             uint16_t *sCam2 = camServices[j];
             if (sCam2[oCAID] == CaId && sCam2[oSID] && sCam2[oECM] == sCam1[oECM]) // shared Ecm
                return true;
             }
         }
      }
  return false;
}

void cScaMapper::RemapSharedEcm(uint16_t CaId, uint16_t Ecm)
{
  int slot = -1;
  int uniq = RealToUniqPid(Ecm); // current mapping

  for (int i = 0; i < maxSids; i++) {
      uint16_t *sCam = camServices[i];
      if (sCam[oCAID] == CaId && sCam[oSID] && sCam[oECM] == Ecm) { // ECM used by this active slot
         if (SCA_CAPID(i, 0) == uniq)  // match
            return;
         slot = i; // shared ECM, remap to this active slot if required
         }
      }
  if (slot >= 0)
     MapUniqCaPid(Ecm, slot, 0); // remap shared ECM
}

// ---

int cScaMapper::GetServiceSlot(uint16_t CaId, uint16_t Sid)
{
  int slot = -1;
  for (int i = 0; i < maxSids; i++) {
      uint16_t *sCam = camServices[i];
      if (sCam[oCAID] == CaId) {
         if (sCam[oSID] == Sid)
            return i; // active service found
         if (slot < 0 && !sCam[oSID])
            slot = i; // select first inactive slot
         }
      }
  return slot;
}

void cScaMapper::AddService(int ServiceSlot, uint16_t *sPmt)
{
  int i, j;
  uint16_t *sCam = camServices[ServiceSlot];

  if (!caPmtSent) {
     caPmtSent = masterSlot->SendStaticCaPmt(vScaConf);
     if (!caPmtSent) {
        esyslog(">>> StatCapmt: failed to send static CAPMT to CAM");
        return;
        }
     }

  DBGSTATICMAP(">>> StatCapmt: Link service %d(%X) to slot[%d]", sPmt[oSID], sPmt[oSID], ServiceSlot);

  ecmAction = sCam[oECM]; // ecm -> slot was in use before, do an ECM "trigger"
  sCam[oTrigger] = ecmAction;

  sCam[oSID] = sPmt[oSID]; // sid
  for (i = oECM, j = 0; sPmt[i]; i++, j++) { // ecm, pid1, pid2, ..
      int realPid = sPmt[i];
      if (j <= sCam[oNumPids]) {
         MapUniqCaPid(realPid, ServiceSlot, j);
         sCam[i] = realPid;
         }
      else { // too many pids - map to TSNULL and skip after Decrypt()
         dsyslog(">>> StatCapmt: Sid %d(%X): Pid %d(%X) over limit!", sPmt[oSID], sPmt[oSID], realPid, realPid);
         uniqPids[realPid] = TSNULL; // will not be sent to the CAM
         }
      }
  sCam[i] = 0; // zero termination
}

void cScaMapper::DelService(int ServiceSlot)
{
  uint16_t *sCam = camServices[ServiceSlot];
  DBGSTATICMAP(">>> StatCapmt: Unlink service %d(%X) from slot[%d]", sCam[oSID], sCam[oSID], ServiceSlot);

  sCam[oSID] = 0; // sid 0 to indicate inactive service slot
  RemapSharedEcm(sCam[oCAID], sCam[oECM]);
  }

void cScaMapper::MapStaticCaPmts(cCiCaPmtList &CaPmtList)
{
  if (!ParseCaPmts(CaPmtList))
     return;

  for (int j = 0; j < numPmtServices; j++) {
      uint16_t *sPmt = pmtServices[j];
      uint16_t sid = sPmt[oSID];
      uint16_t caId = sPmt[oCAID];
      int serviceSlot = caId ? GetServiceSlot(caId, sid) : -1;

      if (sPmt[oTrigger]) { // add / update
         if (serviceSlot >= 0)
            AddService(serviceSlot, sPmt); // activate or update this slot
         else
            dsyslog(">>> StatCapmt: *** ERROR *** No free slot for service %d(%X)", sid, sid);
         }
      else { // remove
         if (serviceSlot >= 0 && camServices[serviceSlot][oSID]) // active slot
            DelService(serviceSlot);
         }
      if (HasSharedEcm(caId))
         ecmShared = true;
      }
  ecmAction = ecmAction || ecmShared;
}

bool cScaMapper::ParseCaPmts(cCiCaPmtList &CaPmtList)
{
  cVector<uint16_t> ecmPids;
  cVector<uint16_t> esPids;
  cVector<uint16_t> caIds;

  if (!CaPmtList.caPmts.Size())
     return false; // no CAPMT changes

  numPmtServices = 0;
  for (int i = 0; i < CaPmtList.caPmts.Size() && numPmtServices < SCA_MAX_SERVICES; i++) {
      cDynamicBuffer *capmt = CaPmtList.CaPmt(i);
      uint16_t sid = capmt->Get(1) << 8 | capmt->Get(2);

      if (!sid)
         continue;

      esPids.Clear();
      ecmPids.Clear();
      caIds.Clear();
      ParseCaPmt(capmt, esPids, ecmPids, caIds); // changed services

      if (esPids.Size() && (ecmPids.Size() != 1 || caIds.Size() != 1))
         dsyslog(">>> StatCapmt: *** WARNING *** Sid %d(%X) EsPids/EcmPids/CaIds: %d/%d/%d", sid, sid, esPids.Size(), ecmPids.Size(), caIds.Size());

      uint16_t *sPmt = pmtServices[numPmtServices];
      sPmt[oCAID] = caIds.Size() ? caIds[0] : 0;
      sPmt[oTrigger] = (esPids.Size() && caIds.Size()) ? 1 : 0;  // Trigger 0 indicate a removed or FTA service
      sPmt[oSID] = sid;
      sPmt[oECM] = ecmPids.Size() ? ecmPids[0] : 0;
      int n = oPIDS;
      for (int j = 0; j < SCA_MAX_ESPIDS && j < esPids.Size(); j++)
          sPmt[n++] = esPids[j];
      sPmt[n] = 0; // zero termination

      numPmtServices++;
      DBGSTATICMAP(">>> StatCapmt: Sid %d(%X) EsPids/EcmPids/CaIds: %d/%d/%d", sid, sid, n - oPIDS, ecmPids.Size(), caIds.Size());
      }
   return true;
}

// --- PARSE CAPMT

#define CPCI_OK_DESCRAMBLING  0x01 // ci.c

uint8_t ParseCaDescriptors(uchar *p, int Length, cVector<uint16_t> &ecmPids, cVector<uint16_t> &caIds)
{
  uint8_t cmdid = 0; 
  if (Length >= 1) {
     cmdid = *p++;
     Length--;
     for (int l = 0; Length > 1 && *p == SI::CaDescriptorTag; Length -= l, p += l) {
         l = p[1] + 2;
         if (l >= 6) {
            uint16_t caid = p[2] << 8 | p[3];
            uint16_t ecm = Peek13(p + 4);
            DBGSTATICMAP(">>> StatCapmt: capmt   [%d] ca %X e %d(%X)", cmdid, caid, ecm, ecm);
            caIds.AppendUnique(caid);
            ecmPids.AppendUnique(ecm);
            }
         else
            break;
         }
     }
  return cmdid;
}

void ParseStreams(uchar *p, int Length, cVector<uint16_t> &esPids, cVector<uint16_t> &ecmPids, cVector<uint16_t> &caIds, uint8_t plCmdid)
{
  for (int l = 0; Length >= 5; Length -= l, p += l) {
      uint16_t pid = Peek13(p + 1);
      l = p[3] * 256 + p[4];
      uint8_t esCmdid = ParseCaDescriptors(p + 5, l, ecmPids, caIds);
      l += 5;
      bool Ok = esCmdid ? esCmdid == CPCI_OK_DESCRAMBLING : plCmdid == CPCI_OK_DESCRAMBLING;
      DBGSTATICMAP(">>> StatCapmt: capmt    p %c%d(%X)", Ok ? '+' : '-', pid, pid);
      if (Ok)
         esPids.AppendUnique(pid);
      }
}

void cScaMapper::ParseCaPmt(cDynamicBuffer *capmt, cVector<uint16_t> &esPids, cVector<uint16_t> &ecmPids, cVector<uint16_t> &caIds)
{
  uchar *p = capmt->Data();
  int Length = capmt->Length();

  if (Length >= 3) {
     uint8_t lm = capmt->Get(0);
     uint16_t sid = capmt->Get(1) << 8 | capmt->Get(2);
     DBGSTATICMAP(">>> StatCapmt: capmt <%d> %d(%X)", lm, sid, sid);
     if (Length >= 6) {
        int l = p[4] * 256 + p[5];
        uint8_t plCmdid = ParseCaDescriptors(p + 6, l, ecmPids, caIds);
        l += 6;
        ParseStreams(p + l, Length - l, esPids, ecmPids, caIds, plCmdid);
        }
     }
}

// ---
