/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "AgroPlanCreator.h"
#include "PlanMasterController.h"
#include "AgroComplexItem.h"

AgroPlanCreator::AgroPlanCreator(PlanMasterController* planMasterController, QObject* parent)
    : PlanCreator(planMasterController, AgroComplexItem::name, QStringLiteral("/qmlimages/PlanCreator/AgroPlanCreator.png"), parent)
{

}

void AgroPlanCreator::createPlan(const QGeoCoordinate& mapCenterCoord)
{
    _planMasterController->removeAll();
    VisualMissionItem* takeoffItem = _missionController->insertTakeoffItem(mapCenterCoord, -1);
    _missionController->insertComplexMissionItem(AgroComplexItem::name, mapCenterCoord, -1);
    _missionController->insertLandItem(mapCenterCoord, -1);
    _missionController->setCurrentPlanViewSeqNum(takeoffItem->sequenceNumber(), true);
}
