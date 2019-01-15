#if !defined( __ESEGLOBAL_H )
#define __ESEGLOBAL_H

/**=========================================================================

  \file  eseGlobal.h

  \brief Definitions for ESE specific defines.

  Copyright (c) 2012-2013 Qualcomm Atheros, Inc.
  All Rights Reserved.
  Qualcomm Atheros Confidential and Proprietary.

  ========================================================================*/


#define SIR_ESE_SNAP_HDR_LENGTH   8
#define SIR_MAC_ESE_OUI_LEN       4
#define SIR_MAC_ESE_MAX_SSID_LEN 32
//Currently we support only the following Types and SubTypes
//IAPP Frame Type: REPORT
#define SIR_ESE_IAPP_TYPE_REPORT                0x30
#define SIR_ESE_IAPP_SUBTYPE_ADJACENTAP         0x0

//IAPP Frame Type: RADIO
#define SIR_ESE_IAPP_TYPE_RADIO_MEAS             0x32
#define SIR_ESE_IAPP_SUBTYPE_RADIO_REQUEST       0x1
#define SIR_ESE_IAPP_SUBTYPE_REPORT              0x81

#define SIR_ESE_IAPP_RADIO_UNUSED_REQUEST         0x0
#define SIR_ESE_IAPP_RADIO_CHNL_LOAD_REQUEST      0x1
#define SIR_ESE_IAPP_RADIO_NOISE_HIST_REQUEST     0x2
#define SIR_ESE_IAPP_RADIO_BEACON_REQUEST         0x3
#define SIR_ESE_IAPP_RADIO_FRAME_REQUEST          0x4
#define SIR_ESE_IAPP_RADIO_HIDDEN_NODE_REQUEST    0x5
#define SIR_ESE_IAPP_RADIO_TSM_REQUEST            0x6

#define SIR_ESE_MAX_MEAS_REQ                      1
#define SIR_ESE_MAX_MEAS_IE_REQS                  8
#define SIR_ESE_MAX_MEAS_IE_REPS                  8
#define SIR_ESE_MAX_NEIGHBOR_IE_REPS              30
#define SIR_BCN_REPORT_MAX_BSS_DESC                4

//IAPP Frame Type: ROAM
#define SIR_ESE_IAPP_TYPE_ROAM                   0x33
#define SIR_ESE_IAPP_SUBTYPE_NEIGHBOR_LIST       0x81
#define SIR_ESE_IAPP_SUBTYPE_DIRECTED_ROAM       0x82

//Currently we support only the following ESE IAPP IE
#define SIR_ESE_EID_MEAS_REQUEST_IE              0x26
#define SIR_ESE_EID_MEAS_REPORT_IE               0x27
#define SIR_ESE_EID_NEIGHBOR_LIST_IE             0x28
#define SIR_ESE_EID_ADJACENT_AP_REPORT_IE        0x9b
#define SIR_ESE_EID_NEW_ASSOC_REASON_IE          0x9c
#define SIR_ESE_EID_NEIGHBOR_LIST_RF_SUBIE       0x1
#define SIR_ESE_EID_NEIGHBOR_LIST_TSF_SUBIE      0x2

#define SIR_ESE_ASSOC_REASON_UNSPECIFIED         0x0
#define SIR_ESE_ASSOC_REASON_NORMAL_ROAM         0x1
#define SIR_ESE_ASSOC_REASON_LOAD_BALANCE        0x2
#define SIR_ESE_ASSOC_REASON_TSPEC_REJECT        0x3
#define SIR_ESE_ASSOC_REASON_DIRECT_ROAM         0x4
#define SIR_ESE_ASSOC_REASON_FIRST_ASSOC         0x5
#define SIR_ESE_ASSOC_REASON_IN_CELLUAR_ROAM     0x6
#define SIR_ESE_ASSOC_REASON_OUT_CELLUAR_ROAM    0x7
#define SIR_ESE_ASSOC_REASON_BETTER_AP           0x8
#define SIR_ESE_ASSOC_REASON_DEAUTH_ROAM         0x9
//********************************************************************
//  SME/PE Shared Definitions
//********************************************************************
typedef struct sSirAdjacentApRepInd
{
   tANI_U16     messageType; // eWNI_SME_ESE_ADJACENT_AP_REPORT
   tANI_U16     length;
   tSirMacAddr  bssid;
   tSirMacAddr  prevApMacAddr;
   tANI_U8      channelNum;
   tSirMacSSid  prevApSSID;
   tANI_U16     clientDissSecs;
   tANI_U8      roamReason;
   tANI_U16     tsmRoamdelay;
} tSirAdjacentApRepInd, *tpSirAdjacentApRepInd;

//********************************************************************
//  ESE Protocol Definitions
//********************************************************************

typedef __ani_attr_pre_packed struct sEseIappHdr
{
    tANI_U8 AironetSnap[SIR_ESE_SNAP_HDR_LENGTH];
    tANI_U16 IappLen;
    tANI_U8  IappType;
    tANI_U8  FuncType;
    tANI_U8  DestMac[SIR_MAC_ADDR_LENGTH];
    tANI_U8  SrcMac[SIR_MAC_ADDR_LENGTH];
} __ani_attr_packed tEseIappHdr, *tpEseIappHdr;

typedef __ani_attr_pre_packed struct sAdjacentApRepIe 
{
    tANI_U16      Eid;
    tANI_U16      Length;
    tANI_U8       CiscoOui[SIR_MAC_ESE_OUI_LEN];
    tANI_U8       Bssid[SIR_MAC_ADDR_LENGTH];
    tANI_U16      ChannelNum;
    tANI_U16      SsidLen;
    tANI_U8       Ssid[SIR_MAC_ESE_MAX_SSID_LEN];
    tANI_U16      ClientDissSecs;
} __ani_attr_packed tAdjacentApRepIe, *tpAdjacentApRepIe;

typedef __ani_attr_pre_packed struct sAssocReasonIe 
{
    tANI_U16      Eid;
    tANI_U16      Length;
    tANI_U8       CiscoOui[SIR_MAC_ESE_OUI_LEN];
    tANI_U8       AssocReason;
} __ani_attr_packed tAssocReasonIe, *tpAssocReasonIe;

typedef __ani_attr_pre_packed struct sNeighborListIe 
{
    tANI_U8       Eid;
    tANI_U8       Length;
    tANI_U8       Bssid[SIR_MAC_ADDR_LENGTH];
    tANI_U8       CurChannel;
    tANI_U8       ChannelBand;
    tANI_U8       PhyType;
} __ani_attr_packed tNeighborListIe, *tpNeighborListIe;

typedef __ani_attr_pre_packed struct sNeighborListRfSubIe 
{
    tANI_U8       SubEid;
    tANI_U8       Length;
    tANI_U8       MinRecvSigPwr;
    tANI_U8       ApTxPwr;
    tANI_U8       RoamHys;
    tANI_U8       AdaptScanThres;
    tANI_U8       TransitionTime;
} __ani_attr_packed tNeighborListRfSubIe, *tpNeighborListRfSubIe;

typedef __ani_attr_pre_packed struct sNeighborListTsfSubIe 
{
    tANI_U8       SubEid;
    tANI_U8       Length;
    tANI_U16      TsfOffset;
    tANI_U16      BcnInterval;
} __ani_attr_packed tNeighborListTsfSubIe, *tpNeighborListTsfSubIe;

typedef __ani_attr_pre_packed struct sMeasReqMode 
{
    tANI_U8       Parallel: 1;
    tANI_U8       Enable: 1;
    tANI_U8       NotUsed: 1;
    tANI_U8       Report: 1;
    tANI_U8       Reserved: 4;
} __ani_attr_packed tMeasReqMode, *tpMeasReqMode;

typedef __ani_attr_pre_packed struct sMeasRepMode 
{
    tANI_U8       Parallel: 1;
    tANI_U8       Incapable: 1;
    tANI_U8       Refused: 1;
    tANI_U8       Reserved: 5;
} __ani_attr_packed tMeasRepMode, *tpMeasRepMode;

typedef __ani_attr_pre_packed struct sMeasRequestIe 
{
    tANI_U16      Eid;
    tANI_U16      Length;
    tANI_U16      MeasToken;
    tMeasReqMode  MeasReqMode;
    tANI_U8       MeasType;
} __ani_attr_packed tMeasRequestIe, *tpMeasRequestIe;

typedef __ani_attr_pre_packed struct sMeasReportIe 
{
    tANI_U16      Eid;
    tANI_U16      Length;
    tANI_U16      MeasToken;
    tMeasRepMode  MeasRepMode;
    tANI_U8       MeasType;
} __ani_attr_packed tMeasReportIe, *tpMeasReportIe;

typedef __ani_attr_pre_packed struct sBcnRequest 
{
    tMeasRequestIe MeasReqIe;
    tANI_U8        ChanNum;
    tANI_U8        ScanMode;
    tANI_U16       MeasDuration;
} __ani_attr_packed tBcnRequest, *tpBcnRequest;

typedef __ani_attr_pre_packed struct sBcnRequestFields 
{
    tANI_U8        ChanNum;
    tANI_U8        ScanMode;
    tANI_U16       MeasDuration;
} __ani_attr_packed tBcnRequestFields, *tpBcnRequestFields;

typedef __ani_attr_pre_packed struct sEseRadioMeasRequest
{
    tEseIappHdr   IappHdr;
    tANI_U16      DiagToken;
    tANI_U8       MeasDly;
    tANI_U8       ActivationOffset;
} __ani_attr_packed tEseRadioMeasRequest, *tpEseRadioMeasRequest;

typedef __ani_attr_pre_packed struct sEseRadioMeasReport
{
    tEseIappHdr   IappHdr;
    tANI_U16      DiagToken;
} __ani_attr_packed tEseRadioMeasReport, *tpEseRadioMeasReport;

typedef __ani_attr_pre_packed struct sEseNeighborListReport
{
    tEseIappHdr   IappHdr;
} __ani_attr_packed tEseNeighborListReport, *tpEseNeighborListReport;

typedef __ani_attr_pre_packed struct sEseAdjacentApReport
{
    tEseIappHdr   IappHdr;
} __ani_attr_packed tEseAdjacentApReport, *tpEseAdjacentApReport;


//********************************************************************
//  ESE Parser Definitions
//********************************************************************

typedef enum eEsePackUnpackStatus
{
    eESE_UNPACK_SUCCESS = 0,
    eESE_PACK_SUCCESS = 0,
    eESE_UNPACK_IE_ERR,
    eESE_UNPACK_FRM_ERR,
    eESE_PACK_FRM_ERR,
    eESE_PACK_IE_ERR,
    eESE_PACK_BUFF_OVERFLOW,
    eESE_ERROR_UNKNOWN
} eEsePackUnpackStatus;

//************************
//  ESE UnPack Definitions
//************************
typedef struct sEseMeasReqIeInfo
{
    tpMeasRequestIe    pMeasReqIe;
    tpBcnRequestFields pBcnReqFields;

} tEseMeasReqIeInfo, *tpEseMeasReqIeInfo;

typedef struct sEseNeighborIeInfo
{
    tpNeighborListIe       pNeighborIe;
    tpNeighborListRfSubIe  pRfSubIe;
    tpNeighborListTsfSubIe pTsfSubIe;
} tEseNeighborIeInfo, *tpEseNeighborIeInfo;

typedef struct sEseUnpackIappFrm
{
    tpEseRadioMeasRequest   pRadioMeasReqHdr;
    tEseMeasReqIeInfo       MeasReqInfo[SIR_ESE_MAX_MEAS_IE_REQS];
    tpEseNeighborListReport pNeighborListHdr;
    tEseNeighborIeInfo      NeighborInfo[SIR_ESE_MAX_NEIGHBOR_IE_REPS];
} tEseUnpackIappFrm, *tpEseUnpackIappFrm;

//************************
//  ESE Pack Definitions
//************************

typedef struct sEseMeasRepIeInfo
{
    tpMeasReportIe    pMeasRepIe;
    tpBcnReportFields pBcnRepFields;
    tpTrafStrmMetrics pTrafStrmFields;
    tANI_U8           *pBuf;
} tEseMeasRepIeInfo, *tpEseMeasRepIeInfo;

typedef struct sEseAdjApRepIeInfo
{
    tpAdjacentApRepIe pAdjApIe;
} tEseAdjApRepIeInfo, *tpEseAdjApRepIeInfo;

typedef struct sEseAssocReasonIeInfo
{
    tpAssocReasonIe pAssocReasonIe;
} tEseAssocReasonIeInfo, *tpEseAssocReasonIeInfo;

typedef struct sEsePackIappFrm
{
    tpEseRadioMeasReport  pRadioMeasRepHdr;
    tEseMeasRepIeInfo     MeasRepInfo[SIR_ESE_MAX_MEAS_IE_REPS];
    tpEseAdjacentApReport pAdjApRepHdr;
    tEseAdjApRepIeInfo    AdjApRepInfo;
    tEseAssocReasonIeInfo AssocReasonInfo;
} tEsePackIappFrm, *tpEsePackIappFrm;

//********************************************************************
//  ESE Context Definitions
//********************************************************************

typedef struct sEseMeasReq
{
    tANI_U8         isValid;
    tANI_U16        DiagToken;
    tANI_U8         MeasDly;
    tANI_U8         ActivationOffset;
    tANI_U8         numMeasReqIe;
    tAniBool        RepSent[SIR_ESE_MAX_MEAS_IE_REQS];
    tpMeasRequestIe pCurMeasReqIe[SIR_ESE_MAX_MEAS_IE_REQS];
} tEseMeasReq, *tpEseMeasReq;



#endif //#if defined __ESEGLOBAL_H
