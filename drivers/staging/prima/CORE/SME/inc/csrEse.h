/** ------------------------------------------------------------------------- * 
    ------------------------------------------------------------------------- *  

  
    \file csrEse.h
  
    Definitions used only by ESE extensions to CSR.
  
    Copyright (C) 2011,2014 Qualcomm, Incorporated
  
 
   ========================================================================== */
#ifndef CSR_ESE_H__
#define CSR_ESE_H__


#include "csrSupport.h"
#include "smeInside.h"
#include "vos_nvitem.h"

#ifdef FEATURE_WLAN_ESE

/*
 * ESE Defines
 */

//#define __ATTRIB_PACK _Packed
#ifndef __ATTRIB_PACK
#define __ATTRIB_PACK __attribute__((__packed__))
#endif

#define IEEE80211_IE_LEN(a) a[0]
#define IEEE80211_ELEM_LEN(a) (a-2)

#define ESE_CCKM_ELEMID                 0x9c
#define ESE_CCKM_REASSOC_REQ_IE_LEN     sizeof(tCsrCckmReassocReqIe)
#define ESE_CCKM_OUI_TYPE               0
#define ESE_TPC_IELEN                   6
#define ESE_AIRONET_ELEMID              0x85
#define ESE_AIRONET_DEVICEID            0x66
//#define ESE_AIRONET_IE_LEN              sizeof(ese_aironet_ie_t)
//#define ESE_RADIOMGMT_IE_LEN            sizeof(ese_radiomgmt_ie_t)
#define ESE_TSRS_OUI_TYPE 8
#define ESE_TSRS_MIN_LEN  6
#define ESE_TSRS_MAX_LEN  13

#define CAC_DATA_RATE               2000
#define QBSSLOAD_PER_2000_DATA_RATE 32   // For every 2000 kbps the QBss load is ~less by 32

/*
 * ESE Typedefs
 */

/*
 * CCKM Reassociation Request Element
 */

typedef struct tagCsrCckmReassocReqIe {
        tANI_U8  cckm_id;
        tANI_U8  cckm_len;
        tANI_U8  cckm_oui[3];
        tANI_U8  cckm_oui_type;
        union {
            tANI_U8     data[8];
            tANI_U64    tsf;
        } cur_tsf_timer;
        tANI_U32 cckm_reassoc_req_num;
        tANI_U8  cckm_mic[8];
} __ATTRIB_PACK tCsrCckmReassocReqIe;


/*
 * ESE Routines
 */

tANI_U8 csrConstructEseCckmIe( tHalHandle hHal, tCsrRoamSession *pSession, 
        tCsrRoamProfile *pProfile, tSirBssDescription *pBssDescription, 
        void *pRsnIe, tANI_U8 rsn_ie_len, void *pCckmIe);

void ProcessIAPPNeighborAPList(void *context);
void csrNeighborRoamIndicateVoiceBW( tpAniSirGlobal pMac, v_U32_t peak_data_rate, int AddmissionCheckFlag );

#endif // FEATURE_WLAN_ESE
#endif // !CSR_ESE_H__
