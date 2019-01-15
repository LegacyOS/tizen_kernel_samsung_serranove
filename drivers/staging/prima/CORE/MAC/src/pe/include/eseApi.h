
/**=========================================================================
  
  \file  eseApi.h
  
  \brief ESE APIs
  
  Copyright (c) 2012 Qualcomm Atheros, Inc.
  All Rights Reserved.
  Qualcomm Atheros Confidential and Proprietary.
  
  ========================================================================*/

/* $Header$ */

#ifndef __ESE_API_H__
#define __ESE_API_H__

#ifdef WLAN_FEATURE_VOWIFI
extern tSirRetStatus
eseProcessBeaconReportXmit( tpAniSirGlobal pMac, tpSirBeaconReportXmitInd pBcnReport);
#endif

tSirRetStatus limActivateTSMStatsTimer(tpAniSirGlobal pMac,
                                            tpPESession psessionEntry);
tSirRetStatus limProcessTsmTimeoutHandler(tpAniSirGlobal pMac,tpSirMsgQ  limMsg);
void limProcessHalEseTsmRsp(tpAniSirGlobal pMac, tpSirMsgQ limMsg);

tSirRetStatus limProcessAdjacentAPRepMsg (tpAniSirGlobal pMac, tANI_U32 *pMsgBuf);

void limCleanupEseCtxt(tpAniSirGlobal pMac, tpPESession pSessionEntry);
#endif
