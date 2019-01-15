/*
 * Copyright (c) 2011,2014 Qualcomm Atheros, Inc. 
 * All Rights Reserved. 
 * Qualcomm Atheros Confidential and Proprietary. 
 * This file limProcessEseFrame.cc contains the code
 * for processing IAPP Frames.
 * Author:      Chas Mannemala
 * Date:        05/23/03
 * History:-
 * Date           Modified by    Modification Information
 * --------------------------------------------------------------------
 *
 */
#ifdef FEATURE_WLAN_ESE
#include "palTypes.h"
#include "wniApi.h"
#include "sirApi.h"
#include "aniGlobal.h"
#include "utilsApi.h"
#include "limTypes.h"
#include "limUtils.h"
#include "limAssocUtils.h"
#include "limSerDesUtils.h"
#include "limSendSmeRspMessages.h"
#include "limSendMessages.h"

tANI_U8 AironetSnapHdr[] = {0xAA, 0xAA, 0x03, 0x00, 0x40, 0x96, 0x00, 0x00};
tANI_U8 CiscoOui[] = {0x00, 0x40, 0x96, 0x00};

/**----------------------------------------------------------------
 * limCleanupIappPackFrm
 *
 *FUNCTION:
 * This function is used to cleanup populated IAPP frame.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  pPackedFrm - Pointer to populated frame
 * @return NONE
 *----------------------------------------------------------------*/

void  limCleanupIappPackFrm(tpAniSirGlobal pMac, tpEsePackIappFrm pPackedFrm) {

   tANI_U8 counter;

   if (!pPackedFrm)
       return;

   if (pPackedFrm->pRadioMeasRepHdr)
       vos_mem_free(pPackedFrm->pRadioMeasRepHdr);

   for (counter=0; counter < SIR_ESE_MAX_MEAS_IE_REPS; counter++)
        if( pPackedFrm->MeasRepInfo[counter].pMeasRepIe)
            vos_mem_free(pPackedFrm->MeasRepInfo[counter].pMeasRepIe);

   if (pPackedFrm->pAdjApRepHdr)
       vos_mem_free(pPackedFrm->pAdjApRepHdr);

   if (pPackedFrm->AdjApRepInfo.pAdjApIe)
       vos_mem_free(pPackedFrm->AdjApRepInfo.pAdjApIe);

   if (pPackedFrm->AssocReasonInfo.pAssocReasonIe)
       vos_mem_free(pPackedFrm->AssocReasonInfo.pAssocReasonIe);

   return;
}

/**----------------------------------------------------------------
 * limCleanupEseCtxt
 *
 *FUNCTION:
 * This function is used to cleanup ESE Ctxt.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  pPackedFrm - Pointer to populated frame
 * @return NONE
 *----------------------------------------------------------------*/

void  limCleanupEseCtxt(tpAniSirGlobal pMac, tpPESession pSessionEntry) {

   tpEseMeasReq pCurMeasReq = NULL; 
   tANI_U8 ieCounter;

   if(!pSessionEntry)
      return;

   pCurMeasReq = &(pSessionEntry->eseContext.curMeasReq);

   if (!pCurMeasReq->isValid)
       return;

   PELOGE(limLog( pMac, LOGE, "Cleaning up ESE RM context\n");)  

   for (ieCounter=0; ieCounter < pCurMeasReq->numMeasReqIe; ieCounter++)
        if(pCurMeasReq->pCurMeasReqIe[ieCounter])
           vos_mem_free(pCurMeasReq->pCurMeasReqIe[ieCounter]);
    
   pCurMeasReq->isValid = VOS_FALSE;

   return;
}
/**----------------------------------------------------------------
 * limUnPackMeasReqIe
 *
 *FUNCTION:
 * This function is used to unpack Measurement Req IE of IAPP frame.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pBuff - Pointer to Buffer where IE is present.
 * @param  *pNeighborIeInfo - A pointer to Ie info structure.
 * @return Status
 *----------------------------------------------------------------*/

static eEsePackUnpackStatus
limUnpackMeasReqIe(tANI_U8 *pBuf, tpEseMeasReqIeInfo pMeasReqIeInfo) {

   tpMeasRequestIe pMeasIe = (tpMeasRequestIe)pBuf;
   tpBcnRequestFields pBcnReqFields = NULL;
   tANI_U32 ieLen=0;

   ieLen = vos_le16_to_cpu(pMeasIe->Length);

   if (ieLen < sizeof(tMeasRequestIe) -sizeof(pMeasIe->Eid) -sizeof(pMeasIe->Length))
       return eESE_UNPACK_IE_ERR;

   pMeasIe->Eid = vos_le16_to_cpu(pMeasIe->Eid);
   pMeasIe->Length = vos_le16_to_cpu(pMeasIe->Length);
   pMeasIe->MeasToken = vos_le16_to_cpu(pMeasIe->MeasToken);
   pMeasReqIeInfo->pMeasReqIe = pMeasIe;

   if (pMeasIe->MeasType == SIR_ESE_IAPP_RADIO_BEACON_REQUEST) {
       pBcnReqFields = (tpBcnRequestFields)(pBuf + sizeof(tMeasRequestIe));
       pBcnReqFields->MeasDuration = vos_le16_to_cpu(pBcnReqFields->MeasDuration);
       pMeasReqIeInfo->pBcnReqFields = pBcnReqFields;
    }

    return eESE_UNPACK_SUCCESS;
}

/**----------------------------------------------------------------
 * limUnPackNeighborListIe
 *
 *FUNCTION:
 * This function is used to unpack Neighbor list IE of IAPP frame.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pBuff - Pointer to Buffer where IE is present.
 * @param  *pNeighborIeInfo - A pointer to Ie info structure.
 * @return Status
 *----------------------------------------------------------------*/

static eEsePackUnpackStatus
limUnpackNeighborListIe(tANI_U8 *pBuf, tpEseNeighborIeInfo pNeighborIeInfo) {

   tpNeighborListIe pNeighborIe = (tpNeighborListIe)pBuf;
   tANI_U32 remIeLen=0, ieLen=0;
   tANI_U8 *pIeBuf =NULL, *pSubIe=NULL;
   tANI_U8 subIeEidSize=1, subIeLenSize=1;
   tANI_U16 curSubEid, curSubIeSize;

   ieLen = pNeighborIe->Length;

   if (ieLen < sizeof(tNeighborListIe) -sizeof(pNeighborIe->Eid) -sizeof(pNeighborIe->Length))
       return eESE_UNPACK_IE_ERR;

   pNeighborIeInfo->pNeighborIe = pNeighborIe;
   remIeLen = pNeighborIe->Length - (sizeof(tNeighborListIe) -sizeof(pNeighborIe->Eid) -sizeof(pNeighborIe->Length));
   pIeBuf = pBuf + sizeof(tNeighborListIe);

   while (remIeLen) {

         if (remIeLen <= (subIeEidSize+subIeLenSize))
             return eESE_UNPACK_IE_ERR;

          pSubIe = pIeBuf;
          curSubEid = *((tANI_U8 *)pSubIe);
          curSubIeSize = *((tANI_U8 *)(pSubIe+subIeEidSize));

          remIeLen -= (subIeEidSize + subIeLenSize);
          pIeBuf += (subIeEidSize + subIeLenSize);

          if (remIeLen < (curSubIeSize))
              return eESE_UNPACK_IE_ERR;

           switch (curSubEid) {

                   case SIR_ESE_EID_NEIGHBOR_LIST_RF_SUBIE:
                        pNeighborIeInfo->pRfSubIe = (tpNeighborListRfSubIe)pSubIe;
                   break;
                   case SIR_ESE_EID_NEIGHBOR_LIST_TSF_SUBIE:
                        pNeighborIeInfo->pTsfSubIe = (tpNeighborListTsfSubIe)pSubIe;
                        pNeighborIeInfo->pTsfSubIe->TsfOffset = vos_le16_to_cpu(pNeighborIeInfo->pTsfSubIe->TsfOffset);
                        pNeighborIeInfo->pTsfSubIe->BcnInterval = vos_le16_to_cpu(pNeighborIeInfo->pTsfSubIe->BcnInterval);
                   break;
                   default:
                   break;
            }

            remIeLen -= curSubIeSize;
            pIeBuf +=curSubIeSize;

    }

    return eESE_UNPACK_SUCCESS;
}

// --------------------------------------------------------------------
/**
 * limUnpackIappFrame
 *
 * FUNCTION:  Unpack the IAPP frame.
 *
 * LOGIC:
 *
 * ASSUMPTIONS: Currently handles only 
 *              1. Meas Request Frm 
 *              2. Neighbor List frm
 *
 * NOTE:
 *
 * @param pIappFrm pointer to the Frm Buffer.
 * @param frameLen length of the frame.
 * @param pUnpackIappFrm unpacked structure.
 * @return Status
 */
// --------------------------------------------------------------------

static eEsePackUnpackStatus
limUnpackIappFrame(tANI_U8 *pIappFrm, tANI_U32 frameLen, tpEseUnpackIappFrm pUnpackIappFrm) 
{
   tpEseIappHdr pIappHdr = (tpEseIappHdr)pIappFrm;
   tANI_U16 ieLen=1, eidLen= 1, curEid, curIeSize;
   tANI_U16 remLen = 0;
   tANI_U8 *pBuf=NULL;
   tANI_U8 numCount=0;
   eEsePackUnpackStatus retStatus=eESE_UNPACK_SUCCESS, status=eESE_UNPACK_SUCCESS;

   if (frameLen <= sizeof(tEseIappHdr)) {
       return eESE_UNPACK_FRM_ERR;
   }

   pIappHdr->IappLen = vos_be16_to_cpu(pIappHdr->IappLen);
   if (pIappHdr->IappLen != (frameLen-SIR_ESE_SNAP_HDR_LENGTH)){
       return eESE_UNPACK_FRM_ERR;
   }

   remLen = frameLen;
   pBuf = pIappFrm;

   if (pIappHdr->IappType == SIR_ESE_IAPP_TYPE_RADIO_MEAS &&
       pIappHdr->FuncType == SIR_ESE_IAPP_SUBTYPE_RADIO_REQUEST) {
  
       pUnpackIappFrm->pRadioMeasReqHdr = (tpEseRadioMeasRequest)pIappHdr;
       //IAPP Header is Big endian
       pUnpackIappFrm->pRadioMeasReqHdr->DiagToken = vos_be16_to_cpu(pUnpackIappFrm->pRadioMeasReqHdr->DiagToken);
       remLen -= sizeof(tEseRadioMeasRequest);
       pBuf += sizeof(tEseRadioMeasRequest);
       eidLen=ieLen=2;
    }
       
   if (pIappHdr->IappType == SIR_ESE_IAPP_TYPE_ROAM &&
       pIappHdr->FuncType == SIR_ESE_IAPP_SUBTYPE_NEIGHBOR_LIST) {
       pUnpackIappFrm->pNeighborListHdr = (tpEseNeighborListReport)pIappHdr;
       remLen -= sizeof(tEseNeighborListReport);
       pBuf += sizeof(tEseNeighborListReport);
    }
 

    while (remLen) {

           if (remLen <= (eidLen+ieLen))
               return eESE_UNPACK_FRM_ERR;

           //IAPP IEs are Little Endian
           if (eidLen == 2) {
               curEid = vos_le16_to_cpu(*((tANI_U16 *)pBuf));
           } else 
               curEid = *((tANI_U8 *)pBuf);

           if (ieLen == 2) {
               curIeSize = vos_le16_to_cpu(*((tANI_U16 *)(pBuf+eidLen)));
           } else 
               curIeSize = *((tANI_U8 *)(pBuf+eidLen));


           if (remLen < (curIeSize + eidLen +ieLen))
               return eESE_UNPACK_FRM_ERR;

         
            switch (curEid) {

                    case SIR_ESE_EID_MEAS_REQUEST_IE:
                         numCount=0;
                         while ((numCount < SIR_ESE_MAX_MEAS_IE_REQS) && 
                                (pUnpackIappFrm->MeasReqInfo[numCount].pMeasReqIe))
                                 numCount++;

                          if (numCount < SIR_ESE_MAX_MEAS_IE_REQS)
                              status = limUnpackMeasReqIe(pBuf, &(pUnpackIappFrm->MeasReqInfo[numCount]));

                    break;
                    case SIR_ESE_EID_NEIGHBOR_LIST_IE:
                         numCount=0;
                         while ((numCount < SIR_ESE_MAX_NEIGHBOR_IE_REPS) && 
                                (pUnpackIappFrm->NeighborInfo[numCount].pNeighborIe))
                                 numCount++;

                          if (numCount < SIR_ESE_MAX_NEIGHBOR_IE_REPS)
                              status = limUnpackNeighborListIe(pBuf, &(pUnpackIappFrm->NeighborInfo[numCount]));
                    break;
                    default:
                    break;
            }

           if (status)
               retStatus = status;

           pBuf+= (eidLen+ieLen+curIeSize);
           remLen-= (eidLen+ieLen+curIeSize);
    }

    return retStatus;
}

// --------------------------------------------------------------------
/**
 * limProcessEseBeaconRequest
 *
 * FUNCTION:  Processes the Beacon report request from the peer AP.
 *
 * LOGIC:
 *
 * ASSUMPTIONS:
 *
 * NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param pCurrentReq pointer to the current Req comtext.
 * @param pSessionEntry session entry.
 * @return None
 */
// --------------------------------------------------------------------
static tSirRetStatus
limProcessEseBeaconRequest( tpAniSirGlobal pMac, 
                           tpEseMeasReq pCurMeasReq,
                           tpPESession pSessionEntry )
{
#ifdef WLAN_FEATURE_VOWIFI
   tSirMsgQ mmhMsg;
   tpSirBeaconReportReqInd pSmeBcnReportReq;
   tpBcnRequest pBeaconReq = NULL;
   tANI_U8 counter;

   //Prepare the request to send to SME.
   pSmeBcnReportReq = (tpSirBeaconReportReqInd)vos_mem_malloc((sizeof(tSirBeaconReportReqInd)+
                             pCurMeasReq->numMeasReqIe));

   if(NULL == pSmeBcnReportReq)
   {
      limLog( pMac, LOGP, FL( "Memory Allocation Failure!!! ESE  BcnReq Ind to SME"));
      return eSIR_FAILURE;
   }

   PELOGE(limLog(pMac, LOGE, FL("Sending Beacon Report Req to SME\n")););
   vos_mem_zero(pSmeBcnReportReq, (sizeof( tSirBeaconReportReqInd )+ pCurMeasReq->numMeasReqIe));

   pSmeBcnReportReq->messageType = eWNI_SME_BEACON_REPORT_REQ_IND;
   pSmeBcnReportReq->length = sizeof( tSirBeaconReportReqInd );
   vos_mem_copy(pSmeBcnReportReq->bssId, pSessionEntry->bssId, sizeof(tSirMacAddr) );
   pSmeBcnReportReq->uDialogToken = pCurMeasReq->DiagToken;
   pSmeBcnReportReq->channelInfo.channelNum = 255;
   pSmeBcnReportReq->msgSource = eRRM_MSG_SOURCE_LEGACY_ESE;
   pSmeBcnReportReq->channelList.numChannels = pCurMeasReq->numMeasReqIe;

   for (counter=0; counter < pCurMeasReq->numMeasReqIe; counter++) {
        pBeaconReq = (tpBcnRequest)pCurMeasReq->pCurMeasReqIe[counter];
        pSmeBcnReportReq->fMeasurementtype[counter] = pBeaconReq->ScanMode;
        pSmeBcnReportReq->measurementDuration[counter] = SYS_TU_TO_MS(pBeaconReq->MeasDuration);
        pSmeBcnReportReq->channelList.channelNumber[counter] = pBeaconReq->ChanNum;
   }

   //Send request to SME.
   mmhMsg.type    = eWNI_SME_BEACON_REPORT_REQ_IND;
   mmhMsg.bodyptr = pSmeBcnReportReq;
   MTRACE(macTraceMsgTx(pMac, pSessionEntry->peSessionId, mmhMsg.type));
   limSysProcessMmhMsgApi(pMac, &mmhMsg,  ePROT);
#endif
   return eSIR_SUCCESS;
}

/**----------------------------------------------------------------
 * limPackRadioMeasRepHdr
 *
 *FUNCTION:
 * This function is used to pack Meas Rep IE of IAPP frame.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  pFrmBuff - Pointer to Buffer where IE needs to present.
 * @param  *pMeasRepHdr - A pointer to Ie info structure.
 * @param  totalPayload - total buffer size.
 * @return Status
 *----------------------------------------------------------------*/


static eEsePackUnpackStatus
limPackRadioMeasRepHdr(tpAniSirGlobal pMac, tpPESession pSessionEntry, tANI_U8 *pFrmBuf, tpEseRadioMeasReport pMeasRepHdr, tANI_U16 totalPayload) {

   tANI_U16 totalSize=0;

   vos_mem_copy(pMeasRepHdr->IappHdr.AironetSnap, AironetSnapHdr, SIR_ESE_SNAP_HDR_LENGTH);
   sirCopyMacAddr(pMeasRepHdr->IappHdr.SrcMac, pSessionEntry->selfMacAddr);
   pMeasRepHdr->DiagToken =vos_cpu_to_be16(pMeasRepHdr->DiagToken); 

   totalSize = totalPayload + sizeof(tEseRadioMeasReport)- SIR_ESE_SNAP_HDR_LENGTH;
   pMeasRepHdr->IappHdr.IappLen = vos_cpu_to_be16(totalSize);

   vos_mem_copy(pFrmBuf, pMeasRepHdr, sizeof(tEseRadioMeasReport)); 

   return eESE_PACK_SUCCESS;
}

/**----------------------------------------------------------------
 * limPackMeasRepIe
 *
 *FUNCTION:
 * This function is used to pack Meas Rep IE of IAPP frame.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  pFrmBuff - Pointer to Buffer where IE needs to present.
 * @param  *pMeasRepIeInfo - A pointer to Ie info structure.
 * @param  nRemBuffSize - Rem buffer size available for this IE.
 * @param  ieBuffSize - total buffer size consumed by IE (Return param).
 * @return Status
 *----------------------------------------------------------------*/

static eEsePackUnpackStatus
limPackMeasRepIe(tpAniSirGlobal pMac, tANI_U8 *pFrmBuf, tpEseMeasRepIeInfo pMeasRepIeInfo, tANI_U16 remBuffSize, tANI_U16 *ieBuffSize) {

   tpMeasReportIe pMeasRepIe = (tpMeasReportIe)pMeasRepIeInfo->pMeasRepIe;
   tpBcnReportFields pBcnRepFields = pMeasRepIeInfo->pBcnRepFields;
   tpTrafStrmMetrics pTsmFields = pMeasRepIeInfo->pTrafStrmFields;
   tANI_U16 ieSize = 0, ieTotalSize=0;

   ieSize = pMeasRepIe->Length;
   ieTotalSize = ieSize + sizeof(pMeasRepIe->Eid)+ sizeof(pMeasRepIe->Length);

   if (ieTotalSize > remBuffSize)
       return eESE_PACK_BUFF_OVERFLOW;

   pMeasRepIe->Eid = vos_cpu_to_le16(SIR_ESE_EID_MEAS_REPORT_IE);
   pMeasRepIe->MeasToken = vos_cpu_to_le16(pMeasRepIe->MeasToken);

   if (pBcnRepFields) {
       pBcnRepFields->MeasDuration = vos_cpu_to_le16(pBcnRepFields->MeasDuration);
       pBcnRepFields->BcnInterval = vos_cpu_to_le16(pBcnRepFields->BcnInterval);
       pBcnRepFields->CapabilityInfo = vos_cpu_to_le16(pBcnRepFields->CapabilityInfo);
       pBcnRepFields->ParentTsf = vos_cpu_to_le32(pBcnRepFields->ParentTsf);
   } else if (pTsmFields) {
       pTsmFields->UplinkPktQueueDly = vos_cpu_to_le16(pTsmFields->UplinkPktQueueDly);
       pTsmFields->UplinkPktQueueDlyHist[0] = vos_cpu_to_le16(pTsmFields->UplinkPktQueueDlyHist[0]);
       pTsmFields->UplinkPktQueueDlyHist[1] = vos_cpu_to_le16(pTsmFields->UplinkPktQueueDlyHist[1]);
       pTsmFields->UplinkPktQueueDlyHist[2] = vos_cpu_to_le16(pTsmFields->UplinkPktQueueDlyHist[2]);
       pTsmFields->UplinkPktQueueDlyHist[3] = vos_cpu_to_le16(pTsmFields->UplinkPktQueueDlyHist[3]);
       pTsmFields->UplinkPktTxDly = vos_cpu_to_le32(pTsmFields->UplinkPktTxDly);
       pTsmFields->UplinkPktLoss = vos_cpu_to_le16(pTsmFields->UplinkPktLoss);
       pTsmFields->UplinkPktCount = vos_cpu_to_le16(pTsmFields->UplinkPktCount);
       pTsmFields->RoamingDly = vos_cpu_to_le16(pTsmFields->RoamingDly);
    }

    pMeasRepIe->Length = vos_cpu_to_le16(ieSize);
    vos_mem_copy(pFrmBuf, (unsigned char *)pMeasRepIeInfo->pMeasRepIe, ieTotalSize); 
    
    if (ieBuffSize)
        *ieBuffSize = ieTotalSize;

    return eESE_PACK_SUCCESS;
}

/**----------------------------------------------------------------
 * limPackAdjacentApRepHdr
 *
 *FUNCTION:
 * This function is used to pack Meas Rep IE of IAPP frame.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  pFrmBuff - Pointer to Buffer where IE needs to present.
 * @param  *pMeasRepHdr - A pointer to Ie info structure.
 * @param  totalPayload - total buffer size.
 * @return Status
 *----------------------------------------------------------------*/


static eEsePackUnpackStatus
limPackAdjacentApRepHdr(tpAniSirGlobal pMac, tpPESession pSessionEntry, tANI_U8 *pFrmBuf, tpEseAdjacentApReport pAdjRepHdr, tANI_U16 totalPayload) {

   tANI_U16 totalSize=0;

   vos_mem_copy(pAdjRepHdr->IappHdr.AironetSnap, AironetSnapHdr, SIR_ESE_SNAP_HDR_LENGTH);
   sirCopyMacAddr(pAdjRepHdr->IappHdr.SrcMac, pSessionEntry->selfMacAddr);

   totalSize = totalPayload + sizeof(tEseAdjacentApReport)- SIR_ESE_SNAP_HDR_LENGTH;
   pAdjRepHdr->IappHdr.IappLen = vos_cpu_to_be16(totalSize);

   vos_mem_copy(pFrmBuf, pAdjRepHdr, sizeof(tEseAdjacentApReport)); 

   return eESE_PACK_SUCCESS;
}
/**----------------------------------------------------------------
 * limPackAdjApRepIe
 *
 *FUNCTION:
 * This function is used to pack Meas Rep IE of IAPP frame.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  pFrmBuff - Pointer to Buffer where IE needs to present.
 * @param  *pAdjApRepIeInfo - A pointer to Ie info structure.
 * @param  nRemBuffSize - Rem buffer size available for this IE.
 * @param  ieBuffSize - total buffer size consumed by IE (Return param).
 * @return Status
 *----------------------------------------------------------------*/

static eEsePackUnpackStatus
limPackAdjApRepIe(tpAniSirGlobal pMac, tANI_U8 *pFrmBuf, tpEseAdjApRepIeInfo pAdjApIeInfo, tANI_U16 remBuffSize, tANI_U16 *ieBuffSize) {

   tpAdjacentApRepIe pAdjApRepIe = pAdjApIeInfo->pAdjApIe;
   tANI_U16 ieSize = 0, ieTotalSize=0;

   ieSize = pAdjApRepIe->Length;
   ieTotalSize = ieSize + sizeof(pAdjApRepIe->Eid)+ sizeof(pAdjApRepIe->Length);

   if (ieTotalSize > remBuffSize)
       return eESE_PACK_BUFF_OVERFLOW;

   pAdjApRepIe->Eid = vos_cpu_to_be16(SIR_ESE_EID_ADJACENT_AP_REPORT_IE);
   pAdjApRepIe->Length = vos_cpu_to_be16(ieSize);
   pAdjApRepIe->ChannelNum = vos_cpu_to_be16(pAdjApRepIe->ChannelNum);
   pAdjApRepIe->SsidLen = vos_cpu_to_be16(pAdjApRepIe->SsidLen);
   pAdjApRepIe->ClientDissSecs = vos_cpu_to_be16(pAdjApRepIe->ClientDissSecs);
   vos_mem_copy(pAdjApRepIe->CiscoOui, CiscoOui, sizeof(CiscoOui)); 
   vos_mem_copy(pFrmBuf, (unsigned char *)pAdjApRepIe, ieTotalSize); 
    
    if (ieBuffSize)
        *ieBuffSize = ieTotalSize;

    return eESE_PACK_SUCCESS;
}  

/**----------------------------------------------------------------
 * limPackAssocReasonIe
 *
 *FUNCTION:
 * This function is used to pack Assoc Reason IE of IAPP frame.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  pFrmBuff - Pointer to Buffer where IE needs to present.
 * @param  *pAssocReasonIeInfo - A pointer to Ie info structure.
 * @param  nRemBuffSize - Rem buffer size available for this IE.
 * @param  ieBuffSize - total buffer size consumed by IE (Return param).
 * @return Status
 *----------------------------------------------------------------*/

static eEsePackUnpackStatus
limPackAssocReasonIe(tpAniSirGlobal pMac, tANI_U8 *pFrmBuf, tpEseAssocReasonIeInfo pAssocReasonInfo, tANI_U16 remBuffSize, tANI_U16 *ieBuffSize) {

   tpAssocReasonIe pAssocReasonIe = pAssocReasonInfo->pAssocReasonIe;
   tANI_U16 ieSize = 0, ieTotalSize=0;

   ieSize = pAssocReasonIe->Length;
   ieTotalSize = ieSize + sizeof(pAssocReasonIe->Eid)+ sizeof(pAssocReasonIe->Length);

   if (ieTotalSize > remBuffSize)
       return eESE_PACK_BUFF_OVERFLOW;

   pAssocReasonIe->Eid = vos_cpu_to_be16(SIR_ESE_EID_NEW_ASSOC_REASON_IE);
   pAssocReasonIe->Length = vos_cpu_to_be16(ieSize);
   vos_mem_copy(pAssocReasonIe->CiscoOui, CiscoOui, sizeof(CiscoOui)); 
   vos_mem_copy(pFrmBuf, (unsigned char *)pAssocReasonIe, ieTotalSize); 
    
    if (ieBuffSize)
        *ieBuffSize = ieTotalSize;

    return eESE_PACK_SUCCESS;
}
/**----------------------------------------------------------------
 * limPackIappFrame
 *
 *FUNCTION:
 * This function is used to pack IAPP frame for the session passed.
 * This calls helper routines per IE to pack the frame.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  pSessionEntry - Pointer to Session
 * @param  *pPopFrm - A pointer to populated Frm with all Ies and Hdr.
 * @param  pBuff - Pointer to Buffer.
 * @param  nBuffSize - total buffer size.
 * @return Status
 *----------------------------------------------------------------*/

static eEsePackUnpackStatus
limPackIappFrm(tpAniSirGlobal pMac, tpPESession pSessionEntry, tANI_U8 *pBuf,  tpEsePackIappFrm pPopFrm, tANI_U16 nBuffSize ) {

   tANI_U8 *pFrmBuf = pBuf;
   tANI_U16 buffConsumed=0, totalPayload=0;
   tANI_U16 remSize=nBuffSize, nStatus, totalFrmSize=0;
   tANI_U8 counter=0;

   if( pPopFrm->pRadioMeasRepHdr )
   {
      if (sizeof(tEseRadioMeasReport) > remSize) {
          limLog( pMac, LOGE, FL("Failure in packing Meas Rep Hdr in Iapp Frm\n") );
          return eESE_PACK_BUFF_OVERFLOW;
      }

      pFrmBuf += sizeof(tEseRadioMeasReport);
      remSize -= sizeof(tEseRadioMeasReport);

      while (pPopFrm->MeasRepInfo[counter].pMeasRepIe) {

         nStatus = limPackMeasRepIe(pMac, pFrmBuf, &pPopFrm->MeasRepInfo[counter], remSize, &buffConsumed);

         if (nStatus) {
             limLog( pMac, LOGE, FL("Failure in packing Meas Report IE in Iapp Frm\n") );
             return nStatus;
         }

         pFrmBuf += buffConsumed;
         totalPayload += buffConsumed;
         remSize -= buffConsumed;
         counter++;

         limLog( pMac, LOGE, FL("Meas IE %d\n\n"),buffConsumed );
         if (counter >= SIR_ESE_MAX_MEAS_IE_REPS)
             break;
       }

      nStatus = limPackRadioMeasRepHdr(pMac, pSessionEntry, pBuf, pPopFrm->pRadioMeasRepHdr, totalPayload);

      totalFrmSize = totalPayload + sizeof(tEseRadioMeasReport);

      limLog( pMac, LOGE, FL("totalFrmSize %d\n\n"),totalFrmSize);

      if (nStatus) {
          limLog( pMac, LOGE, FL("Failure in packing Meas Rep Hdr in Iapp Frm\n") );
          return nStatus;
      }

    } else if( pPopFrm->pAdjApRepHdr)
    {

      if (sizeof(tEseAdjacentApReport) > remSize) {
          limLog( pMac, LOGE, FL("Failure in packing Adjacent Ap report in Iapp Frm\n") );
          return eESE_PACK_BUFF_OVERFLOW;
      }

      pFrmBuf += sizeof(tEseAdjacentApReport);
      remSize -= sizeof(tEseAdjacentApReport);

      if (pPopFrm->AdjApRepInfo.pAdjApIe) {

          nStatus = limPackAdjApRepIe(pMac, pFrmBuf, &pPopFrm->AdjApRepInfo, remSize, &buffConsumed);
          if (nStatus) {
              limLog( pMac, LOGE, FL("Failure in packing Adjacent AP Report IE in Iapp Frm\n") );
              return nStatus;
          }

          pFrmBuf += buffConsumed;
          remSize -= buffConsumed;
          totalPayload += buffConsumed;
      }

      if (pPopFrm->AssocReasonInfo.pAssocReasonIe) {

          nStatus = limPackAssocReasonIe(pMac, pFrmBuf, &pPopFrm->AssocReasonInfo, remSize, &buffConsumed);
          if (nStatus) {
              limLog( pMac, LOGE, FL("Failure in packing Assoc Reason IE in Iapp Frm\n") );
              return nStatus;
          }

          pFrmBuf += buffConsumed;
          remSize -= buffConsumed;
          totalPayload += buffConsumed;
      }

      nStatus = limPackAdjacentApRepHdr(pMac, pSessionEntry, pBuf, pPopFrm->pAdjApRepHdr, totalPayload);

      if (nStatus) {
          limLog( pMac, LOGE, FL("Failure in packing Adjacent Ap Rep Hdr in Iapp Frm\n") );
          return nStatus;
      }

    }

    return eESE_PACK_SUCCESS;
}
 
/**----------------------------------------------------------------
 * limSendIappFrame
 *
 *FUNCTION:
 * This function is used send IAPP frame for the session passed.
 * Currently this handles both Adjacent AP report, Meas Report.
 * Allocates VOSS buffer, calls pack routine, sends it to HAL for
 * further sending.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  pSessionEntry - Pointer to Session
 * @param  *pPopFrm - A pointer to populated Frm with all Ies and Hdr.
 * @param  nPayload - total payload of data frame.
 * @return Status
 *----------------------------------------------------------------*/
   
tSirRetStatus
limSendIappFrame(tpAniSirGlobal pMac, tpPESession pSessionEntry, tpEsePackIappFrm pPopFrm, tANI_U16 nPayload) {

   tSirRetStatus statusCode = eSIR_SUCCESS;
   tpSirMacDataHdr3a pMacHdr;
   tANI_U16 nBytes, nStatus;
   void *pPacket;
   tANI_U8 *pFrame;
   eHalStatus halstatus;
   tANI_U8              txFlag = 0;

   if ( pSessionEntry == NULL ){
      limLog( pMac, LOGE, FL("(psession == NULL) in limSendIappFrame\n") );
      return eSIR_SUCCESS;//FAILURE;
   }

   nBytes = nPayload + sizeof(tSirMacDataHdr3a);
   //Allocate Max size Buffer for sending IAPP frame
   halstatus = palPktAlloc( pMac->hHdd, HAL_TXRX_FRM_802_11_MGMT, ( tANI_U16 )nBytes, 
                            ( void** ) &pFrame, ( void** ) &pPacket );

   if ( ! HAL_STATUS_SUCCESS ( halstatus ) )
   {
      limLog( pMac, LOGP, FL("Failed to allocate %d bytes for a IAPP Frm\n"), nBytes);
      return eSIR_FAILURE;
   }

   // Paranoia:
   vos_mem_zero(pFrame, nBytes);

   // Prepare FC
   pMacHdr = ( tpSirMacDataHdr3a ) pFrame;
   pMacHdr->fc.protVer = SIR_MAC_PROTOCOL_VERSION;
   pMacHdr->fc.type    = SIR_MAC_DATA_FRAME;
   pMacHdr->fc.subType = SIR_MAC_DATA_QOS_DATA;
   pMacHdr->fc.toDS = 1;
   pMacHdr->fc.wep = (pSessionEntry->encryptType == eSIR_ED_NONE)? 0 : 1;

   // Prepare Address 1
   sirCopyMacAddr(pMacHdr->addr1,pSessionEntry->bssId);
   // Prepare Address 2
   sirCopyMacAddr(pMacHdr->addr2,pSessionEntry->selfMacAddr);
   // Prepare Address 3
   sirCopyMacAddr(pMacHdr->addr3,pSessionEntry->bssId);
 
   // Now, we're ready to "pack" the frames
   nStatus = limPackIappFrm(pMac, pSessionEntry, pFrame + sizeof(tSirMacDataHdr3a), pPopFrm, nPayload );

   if (nStatus) {
       limLog( pMac, LOGE, FL( "Failure in packing IAPP Frame %d\n" ), nStatus);
       palPktFree( pMac->hHdd, HAL_TXRX_FRM_802_11_MGMT, ( void* ) pFrame, ( void* ) pPacket );
       return eSIR_FAILURE;
   }

#if 0
{

   tANI_U16 i;
   tANI_U8 *pBuff;
   pBuff = pFrame;
   printk("FRM\n");
   for (i=0;i<nBytes;i++)
        printk("%x ", pBuff[i]);
   printk("\n");
}
#endif

   limLog( pMac, LOGW, FL( "Sending IAPP Frame %d\n" ), nBytes);

   if( eHAL_STATUS_SUCCESS !=
         (halstatus = halTxFrame( pMac,
                                  pPacket,
                                  (tANI_U16) nBytes,
                                  HAL_TXRX_FRM_802_11_DATA,
                                  ANI_TXDIR_TODS,
                                  7,//SMAC_SWBD_TX_TID_MGMT_HIGH,
                                  limTxComplete,
                                  pFrame, txFlag )))
   {
      PELOGE(limLog( pMac, LOGE, FL( "halTxFrame FAILED! Status [%d]\n" ), halstatus );)
      //Pkt will be freed up by the callback
      return eSIR_FAILURE;
   }

   return statusCode;
} // End limSendIappFrm.


/**-----------------------------------------------------------------
\fn     limProcessIappRadioMeasFrame
\brief  Process Radio Measurement IAPP frame.

\param  pMac
\return NONE
-----------------------------------------------------------------*/
tSirRetStatus limProcessIappRadioMeasFrame(tpAniSirGlobal pMac, tpPESession pSessionEntry, tANI_U8 *pBd) {

   tANI_U8 *pIappFrm = WDA_GET_RX_MPDU_DATA(pBd);
   tANI_U32 frameLen = WDA_GET_RX_PAYLOAD_LEN(pBd);
   tpEseRadioMeasRequest pRadioMeasFrm = (tpEseRadioMeasRequest)pIappFrm;
   tEseUnpackIappFrm unpackedRadioMeasFrm;
   tpMeasRequestIe pCurCacheIe=NULL, pCurMeasReqIe=NULL;
   tEsePackIappFrm popRadioMeasRepFrm;
   tpMeasReportIe pFailedMeasRepIe=NULL;
   tpEseRadioMeasReport pRadioMeasRepHdr=NULL;
   tpEsePEContext pEseContext = &pSessionEntry->eseContext;
   tpEseMeasReq pCurMeasReqCtxt= NULL;
   eEsePackUnpackStatus parseStatus;
   tANI_U8 counter, numReqCount=0,numFailedReqs=0;
   tANI_U32 numBytes=0;
   tANI_U16 iappFrmSize=0;
   
   PELOGW(limLog(pMac, LOGW, FL("Procesing ESE Radio Meas Request...\n")););

   //Sanity check for Radio Measurements Frame
   VOS_ASSERT(pRadioMeasFrm->IappHdr.IappType == SIR_ESE_IAPP_TYPE_RADIO_MEAS);

   if (pRadioMeasFrm->IappHdr.FuncType != SIR_ESE_IAPP_SUBTYPE_RADIO_REQUEST)
       return eSIR_SUCCESS;

   vos_mem_zero(&popRadioMeasRepFrm, sizeof(popRadioMeasRepFrm));
   vos_mem_zero(&unpackedRadioMeasFrm, sizeof(unpackedRadioMeasFrm));
   parseStatus = limUnpackIappFrame(pIappFrm, frameLen, &unpackedRadioMeasFrm);
   
   if (parseStatus) {
       PELOGE(limLog(pMac, LOGE, FL("Parsing Error for ESE Radio Meas Req %d\n"), parseStatus););
       return eSIR_SUCCESS;
   } 

   /* Check if there is already Radio Meas Req pending
    * if so, the ctxt will be NULL, which make below
    * while loop to send Failure Report to Cisco AP.
    */
   if (!pEseContext->curMeasReq.isValid) {
       pCurMeasReqCtxt = &pEseContext->curMeasReq;
       vos_mem_zero(pCurMeasReqCtxt, sizeof(tEseMeasReq));
       pCurMeasReqCtxt->DiagToken = unpackedRadioMeasFrm.pRadioMeasReqHdr->DiagToken;
       pCurMeasReqCtxt->MeasDly = unpackedRadioMeasFrm.pRadioMeasReqHdr->MeasDly;
       pCurMeasReqCtxt->ActivationOffset = unpackedRadioMeasFrm.pRadioMeasReqHdr->ActivationOffset;
   }

   while (unpackedRadioMeasFrm.MeasReqInfo[numReqCount].pMeasReqIe) {

      pCurMeasReqIe = unpackedRadioMeasFrm.MeasReqInfo[numReqCount].pMeasReqIe;
          
      if ((pCurMeasReqIe->MeasType == SIR_ESE_IAPP_RADIO_BEACON_REQUEST) &&
          ((pCurMeasReqCtxt) && (pCurMeasReqCtxt->numMeasReqIe < SIR_ESE_MAX_MEAS_IE_REQS))){

           tpBcnRequest pBeaconReq = (tpBcnRequest)pCurMeasReqIe;
               
           numBytes = (pCurMeasReqIe->Length + sizeof(pCurMeasReqIe->Eid)+ sizeof(pCurMeasReqIe->Length));
           pCurMeasReqCtxt->pCurMeasReqIe[pCurMeasReqCtxt->numMeasReqIe] =
                                   (tpMeasRequestIe)vos_mem_malloc(numBytes);

           if (NULL == pCurMeasReqCtxt->pCurMeasReqIe[pCurMeasReqCtxt->numMeasReqIe]) {
               limLog( pMac, LOGP, FL( "Memory Allocation Failure!!! ESE Beacon Request IE" ));
               goto ESE_RADIO_CLEANUP;
            }

            pCurCacheIe = pCurMeasReqCtxt->pCurMeasReqIe[pCurMeasReqCtxt->numMeasReqIe];
            vos_mem_copy(pCurCacheIe, pBeaconReq, numBytes );
            pEseContext->curMeasReq.numMeasReqIe++;

            PELOGW(limLog(pMac, LOGW, FL("Recvd ESEBcnReq token %d scanMode %d chan %d dur %d\n"), pBeaconReq->MeasReqIe.MeasToken,
                              pBeaconReq->ScanMode, pBeaconReq->ChanNum, pBeaconReq->MeasDuration););
       } else {
               //Send Refusal or Incapable Report here.
               popRadioMeasRepFrm.MeasRepInfo[numFailedReqs].pMeasRepIe = (tpMeasReportIe)vos_mem_malloc(sizeof(tMeasReportIe));
               if (NULL == popRadioMeasRepFrm.MeasRepInfo[numFailedReqs].pMeasRepIe) {
                   limLog( pMac, LOGP, FL( "Memory Allocation Failure!!! ESE Meas Report IE" ));
                   goto ESE_RADIO_CLEANUP;
                }

                iappFrmSize += sizeof(tMeasReportIe);
                pFailedMeasRepIe = popRadioMeasRepFrm.MeasRepInfo[numFailedReqs].pMeasRepIe;
                vos_mem_zero(pFailedMeasRepIe, sizeof(tMeasReportIe));
                pFailedMeasRepIe->MeasToken = pCurMeasReqIe->MeasToken; 
                pFailedMeasRepIe->MeasType = pCurMeasReqIe->MeasType; 
                pFailedMeasRepIe->Length = (sizeof(tMeasReportIe)
                                            - sizeof(pFailedMeasRepIe->Length)
                                            - sizeof(pFailedMeasRepIe->Eid));

                if ((pCurMeasReqIe->MeasType != SIR_ESE_IAPP_RADIO_BEACON_REQUEST))
                    pFailedMeasRepIe->MeasRepMode.Incapable  = 1;
                else
                    pFailedMeasRepIe->MeasRepMode.Refused  = 1;

                numFailedReqs++;
                PELOGW(limLog(pMac, LOGW, FL("Recvd ESERadioMeasReq type %d token %d isRef %d isIn %d\n"), pCurMeasReqIe->MeasType,
                              pCurMeasReqIe->MeasToken, pFailedMeasRepIe->MeasRepMode.Refused, pFailedMeasRepIe->MeasRepMode.Incapable););
       }
       numReqCount++;

       if (numReqCount >= SIR_ESE_MAX_MEAS_IE_REQS)
           break;
   }


   if (pCurMeasReqCtxt && pCurMeasReqCtxt->numMeasReqIe) {
   
       if (eSIR_SUCCESS != limProcessEseBeaconRequest(pMac, pCurMeasReqCtxt, pSessionEntry)) {
           PELOGE(limLog(pMac, LOGE, FL("Process Beacon Request Returned Failure \n")););

           for (counter=0; counter < SIR_ESE_MAX_MEAS_IE_REQS; counter++) {
                pCurCacheIe = pCurMeasReqCtxt->pCurMeasReqIe[counter];
                if (pCurCacheIe)
                    vos_mem_free(pCurCacheIe);
           }
           goto ESE_RADIO_CLEANUP;
       }
       pCurMeasReqCtxt->isValid = VOS_TRUE;
   }
 
   if (numFailedReqs) {
       popRadioMeasRepFrm.pRadioMeasRepHdr = (tpEseRadioMeasReport)vos_mem_malloc(sizeof(tEseRadioMeasReport));
       if (NULL == popRadioMeasRepFrm.pRadioMeasRepHdr) {
           limLog( pMac, LOGP, FL( "Memory Allocation Failure!!! ESE Meas Report Hdr"));
           goto ESE_RADIO_CLEANUP;
        }

       iappFrmSize += sizeof(tEseRadioMeasReport);
       pRadioMeasRepHdr = popRadioMeasRepFrm.pRadioMeasRepHdr;
       vos_mem_zero(pRadioMeasRepHdr, sizeof(tEseRadioMeasReport));
       pRadioMeasRepHdr->DiagToken = unpackedRadioMeasFrm.pRadioMeasReqHdr->DiagToken;
       pRadioMeasRepHdr->IappHdr.IappType = SIR_ESE_IAPP_TYPE_RADIO_MEAS;
       pRadioMeasRepHdr->IappHdr.FuncType = SIR_ESE_IAPP_SUBTYPE_REPORT;
       sirCopyMacAddr(pRadioMeasRepHdr->IappHdr.SrcMac, pSessionEntry->selfMacAddr);
       
       limSendIappFrame(pMac, pSessionEntry, &popRadioMeasRepFrm, iappFrmSize);
   }

  ESE_RADIO_CLEANUP:
  limCleanupIappPackFrm(pMac, &popRadioMeasRepFrm);

   return eSIR_SUCCESS;
}

/**----------------------------------------------------------------
 * limProcessIappRoamFrame
 *
 *FUNCTION:
 * This function is called by limProcessIappFrame() upon
 * Neighbor List IAPP Data frame reception.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  *pBd - A pointer to Buffer descriptor + associated PDUs
 * @return Status
 *----------------------------------------------------------------*/
tSirRetStatus limProcessIappRoamFrame(tpAniSirGlobal pMac, tpPESession pSessionEntry, tANI_U8 *pBd) 
{
#ifdef WLAN_FEATURE_VOWIFI
   tANI_U8 *pIappFrm = WDA_GET_RX_MPDU_DATA(pBd);
   tANI_U32 frameLen = WDA_GET_RX_PAYLOAD_LEN(pBd);
   tpEseNeighborListReport pNeighborFrm = (tpEseNeighborListReport)pIappFrm;
   tEseUnpackIappFrm unpackedNeighborFrm;
   tpNeighborListIe pCurNeighborIe=NULL;
   eEsePackUnpackStatus unpackStatus;
   tpSirNeighborReportInd pSmeNeighborRpt = NULL;
   tpSirNeighborBssDescripton pCurBssDesc=NULL;
   tpNeighborListRfSubIe  pCurRfSubIe=NULL;
   tpNeighborListTsfSubIe pCurTsfSubIe=NULL;
   tSirMsgQ mmhMsg;
   tANI_U8 i,numNeighborCount=0;
   tANI_U32 length;

   limLog(pMac, LOGE, FL("Procesing ESE Roam Frame...\n"));
   //Sanity check for Roam Frame
   VOS_ASSERT(pNeighborFrm->IappHdr.IappType == SIR_ESE_IAPP_TYPE_ROAM);

   /*Ignore Silently if not Neighbor list Frame*/
   if (pNeighborFrm->IappHdr.FuncType != SIR_ESE_IAPP_SUBTYPE_NEIGHBOR_LIST)
       return eSIR_SUCCESS;

   vos_mem_zero(&unpackedNeighborFrm, sizeof(unpackedNeighborFrm));
   unpackStatus = limUnpackIappFrame(pIappFrm, frameLen, &unpackedNeighborFrm);
   
   if (unpackStatus) {
       limLog(pMac, LOGE, FL("Parser Returned Error Status for Neighbor List Report %d\n"), unpackStatus);
       return eSIR_SUCCESS;
   } 

   while (unpackedNeighborFrm.NeighborInfo[numNeighborCount].pNeighborIe)
          numNeighborCount++;

   if (!numNeighborCount) {
       limLog(pMac, LOGE, FL("No Neighbors found in Neighbor List Frm\n"));
       return eSIR_SUCCESS;
   }

   length = (sizeof( tSirNeighborReportInd )) +
            (sizeof( tSirNeighborBssDescription ) * (numNeighborCount - 1) ) ; 

   //Prepare the request to send to SME.
   pSmeNeighborRpt = (tpSirNeighborReportInd)vos_mem_malloc(length);
   if (NULL == pSmeNeighborRpt)
   {
      limLog( pMac, LOGP, "%s:%d:Unable to allocate memory", __func__, __LINE__ );
      return eSIR_MEM_ALLOC_FAILED;

   }
   vos_mem_zero(pSmeNeighborRpt, length ); 

            
   for( i = 0 ; i < numNeighborCount ; i++ )
   {
      pCurNeighborIe = unpackedNeighborFrm.NeighborInfo[i].pNeighborIe;
      pCurRfSubIe = unpackedNeighborFrm.NeighborInfo[i].pRfSubIe;
      pCurTsfSubIe = unpackedNeighborFrm.NeighborInfo[i].pTsfSubIe;
      pCurBssDesc = &pSmeNeighborRpt->sNeighborBssDescription[i];

      pCurBssDesc->length = sizeof( tSirNeighborBssDescription ); /*+ any optional ies */
      vos_mem_copy(pCurBssDesc->bssId, pCurNeighborIe->Bssid, sizeof(tSirMacAddr) );
      pCurBssDesc->channel = pCurNeighborIe->CurChannel;
      pCurBssDesc->phyType = pCurNeighborIe->PhyType;
      pCurBssDesc->bssidInfo.eseInfo.channelBand = pCurNeighborIe->ChannelBand;

      if (pCurTsfSubIe) {
          pCurBssDesc->bssidInfo.eseInfo.tsfOffset = pCurTsfSubIe->TsfOffset;
          pCurBssDesc->bssidInfo.eseInfo.beaconInterval = pCurTsfSubIe->BcnInterval;
      }
      if (pCurRfSubIe) {
          pCurBssDesc->bssidInfo.eseInfo.minRecvSigPower = pCurRfSubIe->MinRecvSigPwr;
          pCurBssDesc->bssidInfo.eseInfo.apTxPower = pCurRfSubIe->ApTxPwr;
          pCurBssDesc->bssidInfo.eseInfo.roamHysteresis = pCurRfSubIe->RoamHys;
          pCurBssDesc->bssidInfo.eseInfo.adaptScanThres = pCurRfSubIe->AdaptScanThres;
          pCurBssDesc->bssidInfo.eseInfo.transitionTime = pCurRfSubIe->TransitionTime;
      }

      limLog(pMac, LOGE, FL("Neighbor:\n"));
      limPrintMacAddr(pMac, pCurBssDesc->bssId, LOGE);
      limLog(pMac, LOGE, FL("Chan: %d PhyType: %d BcnInt: %d minRecvSigPwr: %d\n"),
                             pCurBssDesc->channel,
                             pCurBssDesc->phyType,
                             pCurBssDesc->bssidInfo.eseInfo.beaconInterval,
                             pCurBssDesc->bssidInfo.eseInfo.minRecvSigPower);
   }

   pSmeNeighborRpt->messageType = eWNI_SME_NEIGHBOR_REPORT_IND;
   pSmeNeighborRpt->length = length;
   pSmeNeighborRpt->numNeighborReports = numNeighborCount;
   vos_mem_copy(pSmeNeighborRpt->bssId, pSessionEntry->bssId, sizeof(tSirMacAddr) );

   //Send request to SME.
   mmhMsg.type    = pSmeNeighborRpt->messageType;
   mmhMsg.bodyptr = pSmeNeighborRpt;
   MTRACE(macTraceMsgTx(pMac, pSessionEntry->peSessionId, mmhMsg.type));
   limSysProcessMmhMsgApi(pMac, &mmhMsg,  ePROT);
#endif
   return eHAL_STATUS_SUCCESS;
}

/**----------------------------------------------------------------
 * limProcessIappFrame
 *
 *FUNCTION:
 * This function is called by limProcessMessageQueue() upon
 * IAPP Data frame reception.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac - Pointer to Global MAC structure
 * @param  *pBd - A pointer to Buffer descriptor + associated PDUs
 * @return None
 *----------------------------------------------------------------*/

void
limProcessIappFrame(tpAniSirGlobal pMac, tANI_U8 *pBd,tpPESession psessionEntry)
{
    tANI_U8 *pBody;
    tpEseIappHdr pIappHdr;

  
    if (!psessionEntry || !pMac || !pBd) {
        PELOGE(limLog(pMac, LOGE, FL("Invalid Context passed on to ProcessIappFrame\n"));)
        VOS_ASSERT(0);
        return;
    }

    pBody = WDA_GET_RX_MPDU_DATA(pBd);
    pIappHdr = (tpEseIappHdr) pBody;

    switch (pIappHdr->IappType)
    {
/* This is a featurization to not support RM Reports.
 *  The gerrit https://review-android.quicinc.com/#/c/299374
 *  will take care of not advertizing the support in IE.This
 *  change will make the feature complete by not processing 
 *  the RM Measurement Requests (Even though we do not advertise
 *  the support,if we receive a request from any of the AP, we
 *  ignore it.).*/
#ifndef FEATURE_DISABLE_RM
        case SIR_ESE_IAPP_TYPE_RADIO_MEAS:
             limProcessIappRadioMeasFrame(pMac, psessionEntry, pBd);
           break;
#endif /* FEATURE_DISABLE_RM */
        case SIR_ESE_IAPP_TYPE_ROAM:
             limProcessIappRoamFrame(pMac, psessionEntry, pBd);
            break;
        case SIR_ESE_IAPP_TYPE_REPORT:
        default:
             PELOGE(limLog(pMac, LOGE, FL("Dropping IappFrameType %d\n"), pIappHdr->IappType);)
            break;
    }

    return;
}

// --------------------------------------------------------------------
/**
 * eseProcessBeaconReportXmit
 *
 * FUNCTION:  
 *
 * LOGIC: Create a Radio measurement report action frame and send it to peer.
 *
 * ASSUMPTIONS:
 *
 * NOTE:
 *
 * @param pBcnReport Data for beacon report IE from SME.
 * @return None
 *----------------------------------------------------------------------*/
#ifdef WLAN_FEATURE_VOWIFI
tSirRetStatus
eseProcessBeaconReportXmit( tpAniSirGlobal pMac, tpSirBeaconReportXmitInd pBcnReport)
{
   tSirRetStatus status = eSIR_SUCCESS;
   tEsePackIappFrm packRadioMeasRepFrm;
   tpEseRadioMeasReport pRadioMeasRepHdr = NULL;
   tpEsePEContext pEseContext = NULL;
   tpEseMeasReq pCurMeasReq = NULL; 
   tpEseMeasRepIeInfo pMeasRepIeInfo = NULL; 
   tpBcnReportFields pBcnRepFields = NULL;
   tpMeasRequestIe pCurMeasReqIe=NULL;
   tpPESession pSessionEntry ;
   tANI_U8 sessionId,bssCounter,ieCounter, repCounter=0;
   tANI_U16 totalIeSize=0,totalFrmSize=0, totalIeLength=0;


   if(NULL == pBcnReport)
      return eSIR_FAILURE;

  if ((pSessionEntry = peFindSessionByBssid(pMac,pBcnReport->bssId,&sessionId))==NULL){
       PELOGE(limLog(pMac, LOGE,FL("session does not exist for given bssId\n"));)
       return eSIR_FAILURE;
   }

   pEseContext = &pSessionEntry->eseContext;

   if (pEseContext->curMeasReq.isValid)
       pCurMeasReq = &pEseContext->curMeasReq;

   if ( pCurMeasReq == NULL ){
       PELOGE(limLog( pMac, LOGE, "Received report xmit while there is no request pending in PE\n");)
       return eSIR_FAILURE;
   }
 
   if ( pCurMeasReq->DiagToken != pBcnReport->uDialogToken  ){
       PELOGE(limLog( pMac, LOGE, FL( "Received DiagToken doesnt match, SME: %d PE: %d\n"), pBcnReport->uDialogToken, pCurMeasReq->DiagToken););
       return eSIR_FAILURE;
   }

   PELOGE(limLog( pMac, LOGE, "Recvd ESEBcnRepXmit numBss %d\n", pBcnReport->numBssDesc);)  

   vos_mem_zero(&packRadioMeasRepFrm, sizeof(tEsePackIappFrm));
   packRadioMeasRepFrm.pRadioMeasRepHdr = (tpEseRadioMeasReport)vos_mem_malloc(sizeof(tEseRadioMeasReport));
   if (NULL ==  packRadioMeasRepFrm.pRadioMeasRepHdr)
   {
       limLog( pMac, LOGP, FL( "Memory Allocation Failure!!! ESE Meas Report Hdr"));
       goto BCN_REP_CLEANUP;
    }

    totalFrmSize += sizeof(tEseRadioMeasReport); 
    pRadioMeasRepHdr = packRadioMeasRepFrm.pRadioMeasRepHdr;
    vos_mem_zero(pRadioMeasRepHdr, sizeof(tEseRadioMeasReport));
    pRadioMeasRepHdr->DiagToken = pCurMeasReq->DiagToken;
    pRadioMeasRepHdr->IappHdr.IappType = SIR_ESE_IAPP_TYPE_RADIO_MEAS;
    pRadioMeasRepHdr->IappHdr.FuncType = SIR_ESE_IAPP_SUBTYPE_REPORT;
    sirCopyMacAddr(pRadioMeasRepHdr->IappHdr.SrcMac, pSessionEntry->selfMacAddr);

   for (bssCounter=0; bssCounter < pBcnReport->numBssDesc; bssCounter++) {

        if (!pBcnReport->pBssDescription[bssCounter]) {
            PELOGE(limLog( pMac, LOGE, "BSS Description NOT filled\n");)  
            break;
        }

        if (repCounter >= SIR_ESE_MAX_MEAS_IE_REPS){
            PELOGE(limLog( pMac, LOGE, "Exceeded MAX Meas Rep %d\n", repCounter);)  
            break;
        }

        totalIeSize = (tANI_U8)GET_IE_LEN_IN_BSS( pBcnReport->pBssDescription[bssCounter]->length );
        totalIeLength = sizeof(tMeasReportIe)+ sizeof(tBcnReportFields)+ totalIeSize;

        pMeasRepIeInfo = &packRadioMeasRepFrm.MeasRepInfo[repCounter];

        pCurMeasReqIe = NULL;
        for (ieCounter=0; ieCounter < pCurMeasReq->numMeasReqIe; ieCounter++) {
            if((pCurMeasReq->pCurMeasReqIe[ieCounter]) && 
               (((tpBcnRequest)(pCurMeasReq->pCurMeasReqIe[ieCounter]))->ChanNum == 
                  pBcnReport->pBssDescription[bssCounter]->channelId)) { 
               pCurMeasReqIe = pCurMeasReq->pCurMeasReqIe[ieCounter];
               break;
            }
        }
 
        if (!pCurMeasReqIe){
            PELOGE(limLog( pMac, LOGE, "No Matching RRM Req. Skipping BssDesc\n");)  
            continue;
        }

        pMeasRepIeInfo->pMeasRepIe = (tpMeasReportIe)vos_mem_malloc(totalIeLength);
        if (NULL == pMeasRepIeInfo->pMeasRepIe)
        {
            limLog( pMac, LOGP, FL( "Memory Allocation Failure!!! ESE Meas Report IE"));
            goto BCN_REP_CLEANUP;
        }

        PELOGE(limLog( pMac, LOGE, "Including BssDescription:\n");)  
        limPrintMacAddr(pMac, pBcnReport->pBssDescription[bssCounter]->bssId, LOGE);

        vos_mem_zero(pMeasRepIeInfo->pMeasRepIe, sizeof(tMeasReportIe)+sizeof(tBcnReportFields));
        pMeasRepIeInfo->pBcnRepFields = (tpBcnReportFields)((tANI_U8 *)pMeasRepIeInfo->pMeasRepIe + sizeof(tMeasReportIe));
        pMeasRepIeInfo->pBuf = ((tANI_U8 *)pMeasRepIeInfo->pBcnRepFields + sizeof(tBcnReportFields));

        pMeasRepIeInfo->pMeasRepIe->MeasToken = pCurMeasReqIe->MeasToken; 
        pMeasRepIeInfo->pMeasRepIe->MeasType = SIR_ESE_IAPP_RADIO_BEACON_REQUEST;
        pMeasRepIeInfo->pMeasRepIe->Length = (totalIeLength  
                                              - sizeof(pMeasRepIeInfo->pMeasRepIe->Length)
                                              - sizeof(pMeasRepIeInfo->pMeasRepIe->Eid));

        pBcnRepFields = pMeasRepIeInfo->pBcnRepFields;
        pBcnRepFields->ChanNum = pBcnReport->pBssDescription[bssCounter]->channelId;
        pBcnRepFields->Spare = 0;
        pBcnRepFields->MeasDuration = ((tpBcnRequest)pCurMeasReqIe)->MeasDuration;
        pBcnRepFields->PhyType = pBcnReport->pBssDescription[bssCounter]->nwType;
        pBcnRepFields->RecvSigPower = pBcnReport->pBssDescription[bssCounter]->rssi;
        pBcnRepFields->ParentTsf = pBcnReport->pBssDescription[bssCounter]->parentTSF; 
        pBcnRepFields->TargetTsf[0] = pBcnReport->pBssDescription[bssCounter]->timeStamp[0];
        pBcnRepFields->TargetTsf[1] = pBcnReport->pBssDescription[bssCounter]->timeStamp[1];
        pBcnRepFields->BcnInterval = pBcnReport->pBssDescription[bssCounter]->beaconInterval;
        pBcnRepFields->CapabilityInfo = pBcnReport->pBssDescription[bssCounter]->capabilityInfo;
        vos_mem_copy(pBcnRepFields->Bssid, pBcnReport->pBssDescription[bssCounter]->bssId, sizeof(tSirMacAddr));
        vos_mem_copy(pMeasRepIeInfo->pBuf, pBcnReport->pBssDescription[bssCounter]->ieFields, totalIeSize);

        totalFrmSize += totalIeLength;
        pCurMeasReq->RepSent[ieCounter] = VOS_TRUE; 
        repCounter++;
   }

   if (pBcnReport->fMeasureDone) {

       for (ieCounter=0; ieCounter < pCurMeasReq->numMeasReqIe; ieCounter++) {

            if((pCurMeasReq->RepSent[ieCounter]))
               continue; 

            if (repCounter >= SIR_ESE_MAX_MEAS_IE_REPS){
                PELOGE(limLog( pMac, LOGE, "Exceeded MAX Meas Rep %d\n", repCounter);)  
                break;
            }

            pMeasRepIeInfo = &packRadioMeasRepFrm.MeasRepInfo[repCounter];
            pCurMeasReqIe = pCurMeasReq->pCurMeasReqIe[ieCounter];
            pMeasRepIeInfo->pMeasRepIe = (tpMeasReportIe)vos_mem_malloc(sizeof(tMeasReportIe));
            if (NULL == pMeasRepIeInfo->pMeasRepIe)
            {
               limLog( pMac, LOGP, FL("Memory Allocation Failure!!! ESE Meas Report IE"));
               goto BCN_REP_CLEANUP;
            }

            vos_mem_zero(pMeasRepIeInfo->pMeasRepIe, sizeof(tMeasReportIe));
            pMeasRepIeInfo->pMeasRepIe->MeasToken = pCurMeasReqIe->MeasToken; 
            pMeasRepIeInfo->pMeasRepIe->MeasType = SIR_ESE_IAPP_RADIO_BEACON_REQUEST;
            pMeasRepIeInfo->pMeasRepIe->Length = (sizeof(tMeasReportIe) 
                                              - sizeof(pMeasRepIeInfo->pMeasRepIe->Length)
                                              - sizeof(pMeasRepIeInfo->pMeasRepIe->Eid));
            PELOGE(limLog( pMac, LOGE, "Refused MeasToken: %d\n", pCurMeasReqIe->MeasToken);)  
            pMeasRepIeInfo->pMeasRepIe->MeasRepMode.Refused  = 1;
            totalFrmSize += sizeof(tMeasReportIe);
            repCounter++;
      }
   }

   status = limSendIappFrame(pMac, pSessionEntry, &packRadioMeasRepFrm, totalFrmSize);

BCN_REP_CLEANUP:

   if ( pCurMeasReq && pBcnReport->fMeasureDone) {
       limCleanupEseCtxt(pMac, pSessionEntry); 
   }

   for (bssCounter=0; bssCounter < pBcnReport->numBssDesc; bssCounter++) 
        vos_mem_free(pBcnReport->pBssDescription[bssCounter]);

   limCleanupIappPackFrm(pMac, &packRadioMeasRepFrm);
   return status;
}
#endif

/**----------------------------------------------------------------------------------
 * limActivateTSMStatsTimer
 *
 * FUNCTION:  
 *
 * LOGIC: This function activates the TSM Timer if the state is enabled in the tsm IE.
 *
 * ASSUMPTIONS:
 *
 * NOTE:
 *
 * @param pMac -Pointer to Global MAC structure.
 *.@param psessionEntry Pointer to psessionEntry.
 * @return eSIR_SUCCESS/eSIR_FAILURE
 *----------------------------------------------------------------------------------*/

tSirRetStatus limActivateTSMStatsTimer(tpAniSirGlobal pMac,
                                            tpPESession psessionEntry)
{
    if(psessionEntry->eseContext.tsm.tsmInfo.state)
    {
        tANI_U32   val;
        val = SYS_TU_TO_MS(psessionEntry->eseContext.tsm.tsmInfo.msmt_interval);
        val = SYS_MS_TO_TICKS(val);
        PELOGW(limLog(pMac, LOGW, FL("ESE:Start the TSM Timer=%u"),val);)
        if(TX_SUCCESS != tx_timer_change (
                         &pMac->lim.limTimers.gLimEseTsmTimer,
                         val,val ))
        {
            PELOGE(limLog(pMac, LOGE, "Unable to change ESE TSM Stats timer\n");)
            return eSIR_FAILURE;
        }
        pMac->lim.limTimers.gLimEseTsmTimer.sessionId = 
                                          psessionEntry->peSessionId;
        if (TX_SUCCESS != 
                 tx_timer_activate(&pMac->lim.limTimers.gLimEseTsmTimer))
        {
            PELOGE(limLog(pMac, LOGP, "could not activate TSM Stats timer\n");)

            return eSIR_FAILURE;
        }
    }
    else 
    {
        limDeactivateAndChangeTimer(pMac,eLIM_TSM_TIMER);
    }
    return eSIR_SUCCESS;
}

/*------------------------------------------------------------------------
 * limProcessTsmTimeoutHandler
 *
 * FUNCTION:  
 *
 * LOGIC: This function posts the ESE TSM STATS Request message to the HAL.
 *
 * ASSUMPTIONS:
 *
 * NOTE:
 *
 * @param pMac -Pointer to Global MAC structure.
 *.@param limMsg Pointer to lim Msg Queue.
 * @return eSIR_SUCCESS/eSIR_FAILURE
 *-----------------------------------------------------------------------*/

tSirRetStatus limProcessTsmTimeoutHandler(tpAniSirGlobal pMac,tpSirMsgQ  limMsg)
{
    tANI_U8  sessionId = pMac->lim.limTimers.gLimEseTsmTimer.sessionId;
    tpPESession psessionEntry;
    tpAniGetTsmStatsReq pEseTSMStats = NULL;
    tSirMsgQ msg;

    PELOGW(limLog(pMac, LOG2, FL("Entering limProcessTsmTimeoutHandler\n"));)
    psessionEntry = peFindSessionBySessionId(pMac,sessionId);

    if(NULL==psessionEntry)
    {
        PELOGE(limLog(pMac, LOGE, FL("Invalid  Session ID\n"));)
        return eSIR_FAILURE;
    }

    pEseTSMStats = (tpAniGetTsmStatsReq)vos_mem_malloc(sizeof(tAniGetTsmStatsReq));
    if (NULL == pEseTSMStats)
    {
       PELOGE(limLog(pMac, LOGP, FL("vos_mem_malloc() failed"));)
       return eSIR_MEM_ALLOC_FAILED;
    }

    vos_mem_zero((tANI_U8 *)pEseTSMStats, sizeof(tAniGetTsmStatsReq));

    pEseTSMStats->tid= psessionEntry->eseContext.tsm.tid;
    vos_mem_copy(pEseTSMStats->bssId,psessionEntry->bssId,sizeof(tSirMacAddr));
    pEseTSMStats->tsmStatsCallback = NULL;

    msg.type = WDA_TSM_STATS_REQ;
    msg.bodyptr = pEseTSMStats;
    msg.bodyval = 0;
    PELOGW(limLog(pMac, LOG2, FL("Posting ESE TSM STATS REQ to WDA\n"));)
    if(eSIR_SUCCESS != wdaPostCtrlMsg(pMac, &msg))
    {
       PELOGW(limLog(pMac, LOGE, FL("wdaPostCtrlMsg() failed\n"));)
       if (NULL != pEseTSMStats)
       {
           vos_mem_free((tANI_U8*)pEseTSMStats);
       }
       return eSIR_FAILURE;
    }
    return eSIR_SUCCESS;
}


/**----------------------------------------------------------------
 * limProcessHalEseTsmRsp
 *
 * FUNCTION:  
 *
 * LOGIC: This function process the ESE TSM STATS Response from HAL 
 * and sends the TSM stats to the AP in the IAPP frame.
 *
 * ASSUMPTIONS:
 *
 * NOTE:
 *
 * @param pMac -Pointer to Global MAC structure.
 *.@param limMsg Pointer to lim Msg Queue.
 * @return eSIR_SUCCESS/eSIR_FAILURE
 *---------------------------------------------------------------*/

void limProcessHalEseTsmRsp(tpAniSirGlobal pMac, tpSirMsgQ limMsg)
{
    tpAniGetTsmStatsRsp pEseTSMStats = (tpAniGetTsmStatsRsp)limMsg->bodyptr;
    tEsePackIappFrm packRadioMeasRepFrm;
    tpEseRadioMeasReport pRadioMeasRepHdr = NULL;
    tpPESession pSessionEntry;
    tANI_U16 totalFrmSize=0,totalIeLength=0;
    tANI_U8  sessionId = pMac->lim.limTimers.gLimEseTsmTimer.sessionId;
    tpEseMeasRepIeInfo pMeasRepIeInfo = NULL; 
    tSirRetStatus status;

    if(!pEseTSMStats)
    {
        PELOGE(limLog( pMac, LOGE,"pEseTSMStats is NULL" );)
        return;
    }
    pSessionEntry = peFindSessionBySessionId(pMac,sessionId);
    vos_mem_copy(&pSessionEntry->eseContext.tsm.tsmMetrics,
                       &pEseTSMStats->tsmMetrics, offsetof(tTrafStrmMetrics,RoamingCount)); 

    PELOGE(limLog( pMac, LOG2,"--> Entering limProcessHalEseTsmRsp" );)
    PELOGE(limLog( pMac, LOGW,"TSM Report:" );)
    PELOGE(limLog( pMac, LOGW,"----------:\n" );)
    PELOGE(limLog( pMac, LOGW,"UplinkPktQueueDly = %d:\n",
               pSessionEntry->eseContext.tsm.tsmMetrics.UplinkPktQueueDly );)
               
    PELOGE(limLog( pMac, LOGW,FL("UplinkPktQueueDlyHist = %d %d %d %d:\n"),
               pSessionEntry->eseContext.tsm.tsmMetrics.UplinkPktQueueDlyHist[0],
               pSessionEntry->eseContext.tsm.tsmMetrics.UplinkPktQueueDlyHist[1],
               pSessionEntry->eseContext.tsm.tsmMetrics.UplinkPktQueueDlyHist[2],
               pSessionEntry->eseContext.tsm.tsmMetrics.UplinkPktQueueDlyHist[3]);)

    PELOGE(limLog( pMac, LOGW,"UplinkPktTxDly = %d:\n",
               pSessionEntry->eseContext.tsm.tsmMetrics.UplinkPktTxDly );)

    PELOGE(limLog( pMac, LOGW,"UplinkPktLoss = %d:\n",
               pSessionEntry->eseContext.tsm.tsmMetrics.UplinkPktLoss );)

    PELOGE(limLog( pMac, LOGW,"UplinkPktCount = %d:\n",
               pSessionEntry->eseContext.tsm.tsmMetrics.UplinkPktCount );)

    PELOGE(limLog( pMac, LOGW,"RoamingCount = %d:\n",
               pSessionEntry->eseContext.tsm.tsmMetrics.RoamingCount );)

    PELOGE(limLog( pMac, LOGW,"RoamingDly = %d:\n",
               pSessionEntry->eseContext.tsm.tsmMetrics.RoamingDly );)

    vos_mem_zero(&packRadioMeasRepFrm, sizeof(tEsePackIappFrm));
    packRadioMeasRepFrm.pRadioMeasRepHdr = (tpEseRadioMeasReport)vos_mem_malloc(sizeof(tEseRadioMeasReport));
    if (NULL == packRadioMeasRepFrm.pRadioMeasRepHdr)
    {
       PELOGE(limLog( pMac, LOGP,  
                     "Memory Allocation Failure!!! ESE Meas Report Hdr" );)
       return;
    }

    totalFrmSize += sizeof(tEseRadioMeasReport);

    pRadioMeasRepHdr = packRadioMeasRepFrm.pRadioMeasRepHdr;
    vos_mem_zero(pRadioMeasRepHdr, sizeof(tEseRadioMeasReport));
    pRadioMeasRepHdr->DiagToken = 0x0;
    pRadioMeasRepHdr->IappHdr.IappType = SIR_ESE_IAPP_TYPE_RADIO_MEAS;
    pRadioMeasRepHdr->IappHdr.FuncType = SIR_ESE_IAPP_SUBTYPE_REPORT;
    sirCopyMacAddr(pRadioMeasRepHdr->IappHdr.SrcMac, pSessionEntry->selfMacAddr);
    totalIeLength = sizeof(tMeasReportIe) + sizeof(tTrafStrmMetrics);

    pMeasRepIeInfo = &packRadioMeasRepFrm.MeasRepInfo[0];
    pMeasRepIeInfo->pMeasRepIe = (tpMeasReportIe)vos_mem_malloc(totalIeLength);
    if (NULL == pMeasRepIeInfo->pMeasRepIe)
    {
        PELOGE(limLog( pMac, LOGP, 
                   "Memory Allocation Failure!!! ESE Meas Report IE" );)
        return;
    }
    vos_mem_zero((void*)pMeasRepIeInfo->pMeasRepIe, totalIeLength);
    pMeasRepIeInfo->pMeasRepIe->Eid = SIR_ESE_EID_MEAS_REPORT_IE;
    pMeasRepIeInfo->pMeasRepIe->Length = (totalIeLength
                            - sizeof(pMeasRepIeInfo->pMeasRepIe->Length)
                            - sizeof(pMeasRepIeInfo->pMeasRepIe->Eid));
    pMeasRepIeInfo->pMeasRepIe->MeasToken =0x0; 
    pMeasRepIeInfo->pMeasRepIe->MeasType = SIR_ESE_IAPP_RADIO_TSM_REQUEST;
    pMeasRepIeInfo->pTrafStrmFields= (tTrafStrmMetrics *)
                                     ((tANI_U8*)pMeasRepIeInfo->pMeasRepIe
                                      + sizeof(tMeasReportIe));

    vos_mem_copy(pMeasRepIeInfo->pTrafStrmFields,
                       &pSessionEntry->eseContext.tsm.tsmMetrics,
                       sizeof(tTrafStrmMetrics));

    totalFrmSize += totalIeLength;
    status = limSendIappFrame(pMac, pSessionEntry, 
                                &packRadioMeasRepFrm, totalFrmSize);

    /*Reset the TSM stats to zero*/
    vos_mem_zero(&pSessionEntry->eseContext.tsm.tsmMetrics,
                                                 sizeof(tTrafStrmMetrics));

    limActivateTSMStatsTimer(pMac, pSessionEntry);

    if(pEseTSMStats != NULL)
        vos_mem_free((void *)pEseTSMStats );
    
    if (packRadioMeasRepFrm.pRadioMeasRepHdr)
        vos_mem_free(packRadioMeasRepFrm.pRadioMeasRepHdr);

    if(pMeasRepIeInfo->pMeasRepIe)
        vos_mem_free(pMeasRepIeInfo->pMeasRepIe);
}

tSirRetStatus 
limProcessAdjacentAPRepMsg (tpAniSirGlobal pMac,tANI_U32 *pMsgBuf)
{
   tpSirAdjacentApRepInd pAdjApRep = (tpSirAdjacentApRepInd )pMsgBuf;
   tSirRetStatus status = eSIR_SUCCESS;
   tEsePackIappFrm packAdjApFrm;
   tpEseAdjacentApReport pAdjRepHdr = NULL;
   tpAdjacentApRepIe pAdjRepIe = NULL;
   tpAssocReasonIe pAssocIe = NULL;
   tpPESession pSessionEntry ;
   tANI_U8 sessionId;
   tANI_U16 totalFrmSize=0;

   if(NULL == pAdjApRep)
      return eSIR_FAILURE;

   if ((pSessionEntry = peFindSessionByBssid(pMac,pAdjApRep->bssid,&sessionId))==NULL){
       PELOGE(limLog(pMac, LOGE,FL("session does not exist for given bssId\n"));)
       return eSIR_FAILURE;
   }

   PELOGE(limLog( pMac, LOGE, "Recvd ESE Adjacent AP Report\n");)  

   vos_mem_zero(&packAdjApFrm, sizeof(tEsePackIappFrm));
   packAdjApFrm.pAdjApRepHdr = (tpEseAdjacentApReport)vos_mem_malloc(sizeof(tEseRadioMeasReport));
   if (NULL == packAdjApFrm.pAdjApRepHdr)
   {
       limLog( pMac, LOGP, FL( "Memory Allocation Failure!!! ESE Adj Report Hdr" ));
       goto ADJ_REP_CLEANUP;
   }
   /*Update the TSM roam delay*/
   pSessionEntry->eseContext.tsm.tsmMetrics.RoamingDly = pAdjApRep->tsmRoamdelay;
   // Fill in IAPP Header
   totalFrmSize += sizeof(tEseAdjacentApReport); 
   pAdjRepHdr = packAdjApFrm.pAdjApRepHdr;
   vos_mem_zero(pAdjRepHdr, sizeof(tEseAdjacentApReport));
   pAdjRepHdr->IappHdr.IappType = SIR_ESE_IAPP_TYPE_REPORT;
   pAdjRepHdr->IappHdr.FuncType = SIR_ESE_IAPP_SUBTYPE_ADJACENTAP;
   sirCopyMacAddr(pAdjRepHdr->IappHdr.SrcMac, pSessionEntry->selfMacAddr);

   packAdjApFrm.AdjApRepInfo.pAdjApIe = (tpAdjacentApRepIe)vos_mem_malloc(sizeof(tAdjacentApRepIe));
   if (NULL == packAdjApFrm.AdjApRepInfo.pAdjApIe)
   {
       limLog( pMac, LOGP, FL( "Memory Allocation Failure!!! ESE Adjacent AP report IE"));
       goto ADJ_REP_CLEANUP;
   }

   // Fill in Adjacent Report IE
   totalFrmSize += sizeof(tAdjacentApRepIe); 
   pAdjRepIe = packAdjApFrm.AdjApRepInfo.pAdjApIe;
   vos_mem_zero(pAdjRepIe, sizeof(tAdjacentApRepIe));
   pAdjRepIe->Eid = SIR_ESE_EID_ADJACENT_AP_REPORT_IE;
   pAdjRepIe->Length = sizeof(tAdjacentApRepIe) -
                       sizeof(pAdjRepIe->Eid)-sizeof(pAdjRepIe->Length);
   sirCopyMacAddr(pAdjRepIe->Bssid, pAdjApRep->prevApMacAddr);
   pAdjRepIe->ChannelNum = pAdjApRep->channelNum;
   pAdjRepIe->SsidLen = pAdjApRep->prevApSSID.length;
   vos_mem_copy(pAdjRepIe->Ssid, pAdjApRep->prevApSSID.ssId, pAdjRepIe->SsidLen );
   pAdjRepIe->ClientDissSecs = pAdjApRep->clientDissSecs;

   packAdjApFrm.AssocReasonInfo.pAssocReasonIe = (tpAssocReasonIe)vos_mem_malloc(sizeof(tAssocReasonIe));
   if (NULL == packAdjApFrm.AssocReasonInfo.pAssocReasonIe)
   {
       limLog( pMac, LOGP, FL( "Memory Allocation Failure!!! ESE Adjacent AP report IE" ));
       goto ADJ_REP_CLEANUP;
   }

   // Fill in Assoc Reason IE
   totalFrmSize += sizeof(tAssocReasonIe); 
   pAssocIe = packAdjApFrm.AssocReasonInfo.pAssocReasonIe;
   vos_mem_zero(pAssocIe, sizeof(tAssocReasonIe));
   pAssocIe->Eid = SIR_ESE_EID_NEW_ASSOC_REASON_IE;
   pAssocIe->Length = sizeof(tAssocReasonIe) - 
                      sizeof(pAssocIe->Eid)-sizeof(pAssocIe->Length);
   pAssocIe->AssocReason = pAdjApRep->roamReason;
 
   status = limSendIappFrame(pMac, pSessionEntry, &packAdjApFrm, totalFrmSize);

ADJ_REP_CLEANUP:

   limCleanupIappPackFrm(pMac, &packAdjApFrm);

   return status;
}    

#endif //FEATURE_WLAN_ESE
