/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "AgriculturePlanCreator.h"
#include "PlanMasterController.h"
#include "AgricultureComplexItem.h"

AgriculturePlanCreator::AgriculturePlanCreator(PlanMasterController* planMasterController, QObject* parent)
    : PlanCreator(planMasterController, AgricultureComplexItem::name, QStringLiteral("/qmlimages/PlanCreator/AgriculturePlanCreator.png"), parent)
{

}

void AgriculturePlanCreator::createPlan(const QGeoCoordinate& mapCenterCoord)
{
    _planMasterController->removeAll();
    VisualMissionItem* takeoffItem = _missionController->insertTakeoffItem(mapCenterCoord, -1);
    _missionController->insertComplexMissionItem(AgricultureComplexItem::name, mapCenterCoord, -1);
    _missionController->insertLandItem(mapCenterCoord, -1);
    _missionController->setCurrentPlanViewSeqNum(takeoffItem->sequenceNumber(), true);
}

