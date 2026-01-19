/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/


#include "AgroComplexItem.h"
#include "JsonHelper.h"
#include "QGCGeo.h"
#include "QGCQGeoCoordinate.h"
#include "SettingsManager.h"
#include "AppSettings.h"
#include "PlanMasterController.h"
#include "MissionItem.h"
#include "QGCApplication.h"
#include "Vehicle.h"
#include "QGCLoggingCategory.h"

#include <QtGui/QPolygonF>
#include <QtCore/QJsonArray>
#include <QtCore/QLineF>

#include "MissionController.h"

#include <algorithm>

#include "clipper.hpp"
bool AgroComplexItem::_ignoreGlobalUpdate = false;

QGC_LOGGING_CATEGORY(AgroComplexItemLog, "Plan.AgroComplexItem")

const QString AgroComplexItem::name(AgroComplexItem::tr("Agro"));

const double ClipperScale = 1000.0; // Millimeter accuracy for meters (NED)

ClipperLib::Path AgroComplexItem::_toClipperPath(const QPolygonF& poly) const {
    ClipperLib::Path path;
    for (const QPointF& pt : poly) {
        path.push_back(ClipperLib::IntPoint(
            static_cast<ClipperLib::cInt>(std::round(pt.x() * _clipperScale)),
            static_cast<ClipperLib::cInt>(std::round(pt.y() * _clipperScale))
        ));
    }
    return path;
}

QPolygonF AgroComplexItem::_fromClipperPath(const ClipperLib::Path& path) const {
    QPolygonF poly;
    for (const ClipperLib::IntPoint& pt : path) {
        poly << QPointF(static_cast<double>(pt.X) / _clipperScale,
                        static_cast<double>(pt.Y) / _clipperScale);
    }
    if (!poly.isEmpty() && poly.first() != poly.last()) poly << poly.first();
    return poly;
}

QGeoCoordinate AgroComplexItem::_toGeo(const QPointF& pt, const QGeoCoordinate& origin) {
    QGeoCoordinate coord;
    QGCGeo::convertNedToGeo(pt.y(), pt.x(), 0, origin, coord);
    return coord;
}

AgroComplexItem::AgroComplexItem(PlanMasterController* masterController, bool flyView, const QString& kmlOrShpFile)
    : TransectStyleComplexItem  (masterController, flyView, settingsGroup)
    , _metaDataMap              (FactMetaData::createMapFromJsonFile(QStringLiteral(":/json/Agro.SettingsGroup.json"), this))
    , _gridAngleFact            (settingsGroup, _metaDataMap[gridAngleName])
    , _flyAlternateTransectsFact(settingsGroup, _metaDataMap[flyAlternateTransectsName])
    , _splitConcavePolygonsFact (settingsGroup, _metaDataMap[splitConcavePolygonsName])
    , _isExclusionZoneFact      (settingsGroup, _metaDataMap[isExclusionZoneName])
    , _entryPoint               (EntryLocationTopLeft)
{
    _editorQml = "qrc:/qml/QGroundControl/Controls/AgroItemEditor.qml";

    if (_controllerVehicle && !(_controllerVehicle->fixedWing() || _controllerVehicle->vtol())) {
        // Only fixed wing flight paths support alternate transects
        _flyAlternateTransectsFact.setRawValue(false);
    }

    // We override the altitude to the mission default
    if (_cameraCalc.isManualCamera() || !_cameraCalc.valueSetIsDistance()->rawValue().toBool()) {
        _cameraCalc.distanceToSurface()->setRawValue(SettingsManager::instance()->appSettings()->defaultMissionItemAltitude()->rawValue());
    }

    connect(&_gridAngleFact,            &Fact::valueChanged,                        this, &AgroComplexItem::_setDirty);
    connect(&_flyAlternateTransectsFact,&Fact::valueChanged,                        this, &AgroComplexItem::_setDirty);
    connect(&_splitConcavePolygonsFact, &Fact::valueChanged,                        this, &AgroComplexItem::_setDirty);
    connect(&_isExclusionZoneFact,      &Fact::valueChanged,                        this, &AgroComplexItem::_setDirty);
    connect(this,                       &AgroComplexItem::refly90DegreesChanged,  this, &AgroComplexItem::_setDirty);

    connect(&_gridAngleFact,            &Fact::valueChanged,                        this, &AgroComplexItem::_rebuildTransects);
    connect(&_flyAlternateTransectsFact,&Fact::valueChanged,                        this, &AgroComplexItem::_rebuildTransects);
    connect(&_splitConcavePolygonsFact, &Fact::valueChanged,                        this, &AgroComplexItem::_rebuildTransects);
    connect(&_isExclusionZoneFact,      &Fact::valueChanged,                        this, &AgroComplexItem::_rebuildTransects);
    connect(this,                       &AgroComplexItem::refly90DegreesChanged,  this, &AgroComplexItem::_rebuildTransects);

    connect(&_surveyAreaPolygon,        &QGCMapPolygon::isValidChanged,             this, &AgroComplexItem::_updateWizardMode);
    connect(&_surveyAreaPolygon,        &QGCMapPolygon::traceModeChanged,           this, &AgroComplexItem::_updateWizardMode);

    connect(&_surveyAreaPolygon,        &QGCMapPolygon::pathChanged,                this, &AgroComplexItem::_rebuildTransects);

    if (!kmlOrShpFile.isEmpty()) {
        _surveyAreaPolygon.loadKMLOrSHPFile(kmlOrShpFile);
        _surveyAreaPolygon.setDirty(false);
    }
    setDirty(false);
}

void AgroComplexItem::_appendSprayerCommand(QList<MissionItem*>& items, QObject* missionItemParent, int& seqNum, bool active)
{
    float servoInstance = 5.0f;
    float turnOn = 1900.0f;
    float turnOff = 1100.0f;

    float pwmValue = active ? 1900.0f : 1100.0f;

    MissionItem* item = new MissionItem(seqNum++,
                                        MAV_CMD_DO_SET_SERVO,
                                        MAV_FRAME_MISSION,
                                        servoInstance,  // Param 1: Servo number (Instance/Channel)
                                        pwmValue,       // Param 2: PWM (microsecond)
                                        0,              // Param 3:
                                        0, 0, 0, 0,     // Param 4-7
                                        true,           // autoContinue
                                        false,          // isCurrentItem
                                        missionItemParent);
    items.append(item);
}

void AgroComplexItem::_appendVisualAction(QList<MissionItem*>& items, QObject* missionItemParent, int& seqNum, MAV_FRAME frame, const QGeoCoordinate& coord)
{
    // Take the current height and add 10
    double jumpAltitude = coord.altitude() + 10.0;

    // ВHover time at top point (seconds)
    float holdTime = 5.0f;

    MissionItem* item = new MissionItem(seqNum++,
                                        MAV_CMD_NAV_WAYPOINT, // Standard flight command (100% supported)
                                        frame,
                                        holdTime,       // Param 1:Hold time
                                        0,              // Param 2: Radius (0 = default)
                                        0,              // Param 3: Pass Radius
                                        0,              // Param 4: Yaw (0 = не менять)
                                        coord.latitude(),  // Param 5: The same latitude
                                        coord.longitude(), // Param 6: Same longitude
                                        jumpAltitude,      // Param 7: NEW HEIGHT (+10m)
                                        true,           // autoContinue
                                        false,          // isCurrentItem
                                        missionItemParent);
    items.append(item);
}

void AgroComplexItem::appendMissionItems(QList<MissionItem*>& items, QObject* missionItemParent)
{
    // If the mission is loaded from a file, we use the parent loading logic
    if (_loadedMissionItems.count()) {
        TransectStyleComplexItem::appendMissionItems(items, missionItemParent);
        return;
    }

    if (_isExclusionZoneFact.rawValue().toBool()) {
        return;
    }

    int seqNum = _sequenceNumber;

    // Determining the type of height
    MAV_FRAME mavFrame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    if (_cameraCalc.distanceMode() == QGroundControlQmlGlobal::AltitudeModeAbsolute ||
        _cameraCalc.distanceMode() == QGroundControlQmlGlobal::AltitudeModeCalcAboveTerrain) {
        mavFrame = MAV_FRAME_GLOBAL;
    } else if (_cameraCalc.distanceMode() == QGroundControlQmlGlobal::AltitudeModeTerrainFrame) {
        mavFrame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
    }

    // We go through the calculated points (they are available because they are protected in the parent)
    for (int i = 0; i < _rgFlightPathCoordInfo.count(); i++) {
        const CoordInfo_t& coordInfo = _rgFlightPathCoordInfo[i];

        // Using the parent method to add a flight point
        // (it is protected, so it is available to us here)
        _appendWaypoint(items, missionItemParent, seqNum, mavFrame, 0, coordInfo.coord);

        // Adding sprayer logic
        switch (coordInfo.coordType) {
            case CoordTypeSurveyEntry:
                // Turn it on AFTER arriving at the entry point
                _appendSprayerCommand(items, missionItemParent, seqNum, true);
                // _appendVisualAction(items, missionItemParent, seqNum, mavFrame, coordInfo.coord);
                break;

            case CoordTypeSurveyExit:
                // Turn it off AFTER arriving at the exit point
                _appendSprayerCommand(items, missionItemParent, seqNum, false);
                break;

            default:
                break;
        }
    }
}

void AgroComplexItem::save(QJsonArray&  planItems)
{
    QJsonObject saveObject;

    _saveCommon(saveObject);
    planItems.append(saveObject);
}

void AgroComplexItem::savePreset(const QString& name)
{
    QJsonObject saveObject;

    _saveCommon(saveObject);
    _savePresetJson(name, saveObject);
}

void AgroComplexItem::_saveCommon(QJsonObject& saveObject)
{
    TransectStyleComplexItem::_save(saveObject);

    saveObject[JsonHelper::jsonVersionKey] =                    5;
    saveObject[VisualMissionItem::jsonTypeKey] =                VisualMissionItem::jsonTypeComplexItemValue;
    saveObject[ComplexMissionItem::jsonComplexItemTypeKey] =    jsonComplexItemTypeValue;
    saveObject[_jsonGridAngleKey] =                             _gridAngleFact.rawValue().toDouble();
    saveObject[_jsonFlyAlternateTransectsKey] =                 _flyAlternateTransectsFact.rawValue().toBool();
    saveObject[_jsonSplitConcavePolygonsKey] =                  _splitConcavePolygonsFact.rawValue().toBool();

    // Saving the flag
    saveObject[_jsonIsExclusionZoneKey] =                       _isExclusionZoneFact.rawValue().toBool();

    saveObject[_jsonEntryPointKey] =                            _entryPoint;

    _surveyAreaPolygon.saveToJson(saveObject);
}

void AgroComplexItem::loadPreset(const QString& name)
{
    QString errorString;

    QJsonObject presetObject = _loadPresetJson(name);
    if (!_loadV4V5(presetObject, 0, errorString, 5, true /* forPresets */)) {
        qgcApp()->showAppMessage(QStringLiteral("Internal Error: Preset load failed. Name: %1 Error: %2").arg(name).arg(errorString));
    }
    _rebuildTransects();
}

bool AgroComplexItem::load(const QJsonObject& complexObject, int sequenceNumber, QString& errorString)
{
    // We need to pull version first to determine what validation/conversion needs to be performed
    QList<JsonHelper::KeyValidateInfo> versionKeyInfoList = {
        { JsonHelper::jsonVersionKey, QJsonValue::Double, true },
    };
    if (!JsonHelper::validateKeys(complexObject, versionKeyInfoList, errorString)) {
        return false;
    }

    int version = complexObject[JsonHelper::jsonVersionKey].toInt();
    if (version < 2 || version > 5) {
        errorString = tr("Survey items do not support version %1").arg(version);
        return false;
    }

    if (version == 4 || version == 5) {
        if (!_loadV4V5(complexObject, sequenceNumber, errorString, version, false /* forPresets */)) {
            return false;
        }

        _recalcComplexDistance();
        if (_cameraShots == 0) {
            // Shot count was possibly not available from plan file
            _recalcCameraShots();
        }
    } else {
        // Must be v2 or v3
        QJsonObject v3ComplexObject = complexObject;
        if (version == 2) {
            // Convert to v3
            if (v3ComplexObject.contains(VisualMissionItem::jsonTypeKey) && v3ComplexObject[VisualMissionItem::jsonTypeKey].toString() == QStringLiteral("survey")) {
                v3ComplexObject[VisualMissionItem::jsonTypeKey] = VisualMissionItem::jsonTypeComplexItemValue;
                v3ComplexObject[ComplexMissionItem::jsonComplexItemTypeKey] = jsonComplexItemTypeValue;
            }
        }
        if (!_loadV3(complexObject, sequenceNumber, errorString)) {
            return false;
        }

        // V2/3 doesn't include individual items so we need to rebuild manually
        _rebuildTransects();
    }

    return true;
}

bool AgroComplexItem::_loadV4V5(const QJsonObject& complexObject, int sequenceNumber, QString& errorString, int version, bool forPresets)
{
    QList<JsonHelper::KeyValidateInfo> keyInfoList = {
        { VisualMissionItem::jsonTypeKey,               QJsonValue::String, true },
        { ComplexMissionItem::jsonComplexItemTypeKey,   QJsonValue::String, true },
        { _jsonEntryPointKey,                           QJsonValue::Double, true },
        { _jsonGridAngleKey,                            QJsonValue::Double, true },
        { _jsonFlyAlternateTransectsKey,                QJsonValue::Bool,   false },

    };

    if(version == 5) {
        JsonHelper::KeyValidateInfo jSplitPolygon = { _jsonSplitConcavePolygonsKey, QJsonValue::Bool, true };
        keyInfoList.append(jSplitPolygon);
    }

    if (!JsonHelper::validateKeys(complexObject, keyInfoList, errorString)) {
        return false;
    }

    QString itemType = complexObject[VisualMissionItem::jsonTypeKey].toString();
    QString complexType = complexObject[ComplexMissionItem::jsonComplexItemTypeKey].toString();
    if (itemType != VisualMissionItem::jsonTypeComplexItemValue || complexType != jsonComplexItemTypeValue) {
        errorString = tr("%1 does not support loading this complex mission item type: %2:%3").arg(qgcApp()->applicationName()).arg(itemType).arg(complexType);
        return false;
    }

    _ignoreRecalc = !forPresets;

    if (!forPresets) {
        setSequenceNumber(sequenceNumber);

        if (!_surveyAreaPolygon.loadFromJson(complexObject, true /* required */, errorString)) {
            _surveyAreaPolygon.clear();
            return false;
        }
    }

    if (!TransectStyleComplexItem::_load(complexObject, forPresets, errorString)) {
        _ignoreRecalc = false;
        return false;
    }

    _gridAngleFact.setRawValue              (complexObject[_jsonGridAngleKey].toDouble());
    _flyAlternateTransectsFact.setRawValue  (complexObject[_jsonFlyAlternateTransectsKey].toBool(false));
    _isExclusionZoneFact.setRawValue        (complexObject[_jsonIsExclusionZoneKey].toBool(false));

    if (version == 5) {
        _splitConcavePolygonsFact.setRawValue   (complexObject[_jsonSplitConcavePolygonsKey].toBool(true));
    }

    _entryPoint = complexObject[_jsonEntryPointKey].toInt();

    _ignoreRecalc = false;

    return true;
}

bool AgroComplexItem::_loadV3(const QJsonObject& complexObject, int sequenceNumber, QString& errorString)
{
    QList<JsonHelper::KeyValidateInfo> mainKeyInfoList = {
        { VisualMissionItem::jsonTypeKey,               QJsonValue::String, true },
        { ComplexMissionItem::jsonComplexItemTypeKey,   QJsonValue::String, true },
        { QGCMapPolygon::jsonPolygonKey,                QJsonValue::Array,  true },
        { _jsonV3GridObjectKey,                         QJsonValue::Object, true },
        { _jsonV3CameraObjectKey,                       QJsonValue::Object, false },
        { _jsonV3CameraTriggerDistanceKey,              QJsonValue::Double, true },
        { _jsonV3ManualGridKey,                         QJsonValue::Bool,   true },
        { _jsonV3FixedValueIsAltitudeKey,               QJsonValue::Bool,   true },
        { _jsonV3HoverAndCaptureKey,                    QJsonValue::Bool,   false },
        { _jsonV3Refly90DegreesKey,                     QJsonValue::Bool,   false },
        { _jsonV3CameraTriggerInTurnaroundKey,          QJsonValue::Bool,   false },    // Should really be required, but it was missing from initial code due to bug
    };
    if (!JsonHelper::validateKeys(complexObject, mainKeyInfoList, errorString)) {
        return false;
    }

    QString itemType = complexObject[VisualMissionItem::jsonTypeKey].toString();
    QString complexType = complexObject[ComplexMissionItem::jsonComplexItemTypeKey].toString();
    if (itemType != VisualMissionItem::jsonTypeComplexItemValue || complexType != jsonV3ComplexItemTypeValue) {
        errorString = tr("%1 does not support loading this complex mission item type: %2:%3").arg(qgcApp()->applicationName()).arg(itemType).arg(complexType);
        return false;
    }

    _ignoreRecalc = true;

    setSequenceNumber(sequenceNumber);

    _hoverAndCaptureFact.setRawValue            (complexObject[_jsonV3HoverAndCaptureKey].toBool(false));
    _refly90DegreesFact.setRawValue             (complexObject[_jsonV3Refly90DegreesKey].toBool(false));
    _cameraTriggerInTurnAroundFact.setRawValue  (complexObject[_jsonV3CameraTriggerInTurnaroundKey].toBool(true));

    _cameraCalc.valueSetIsDistance()->setRawValue   (complexObject[_jsonV3FixedValueIsAltitudeKey].toBool(true));
    _cameraCalc.setDistanceMode(complexObject[_jsonV3GridAltitudeRelativeKey].toBool(true) ? QGroundControlQmlGlobal::AltitudeModeRelative : QGroundControlQmlGlobal::AltitudeModeAbsolute);

    bool manualGrid = complexObject[_jsonV3ManualGridKey].toBool(true);

    QList<JsonHelper::KeyValidateInfo> gridKeyInfoList = {
        { _jsonV3GridAltitudeKey,           QJsonValue::Double, true },
        { _jsonV3GridAltitudeRelativeKey,   QJsonValue::Bool,   true },
        { _jsonV3GridAngleKey,              QJsonValue::Double, true },
        { _jsonV3GridSpacingKey,            QJsonValue::Double, true },
        { _jsonEntryPointKey,               QJsonValue::Double, false },
        { _jsonV3TurnaroundDistKey,         QJsonValue::Double, true },
    };
    QJsonObject gridObject = complexObject[_jsonV3GridObjectKey].toObject();
    if (!JsonHelper::validateKeys(gridObject, gridKeyInfoList, errorString)) {
        _ignoreRecalc = false;
        return false;
    }

    _gridAngleFact.setRawValue          (gridObject[_jsonV3GridAngleKey].toDouble());
    _turnAroundDistanceFact.setRawValue (gridObject[_jsonV3TurnaroundDistKey].toDouble());

    if (gridObject.contains(_jsonEntryPointKey)) {
        _entryPoint = gridObject[_jsonEntryPointKey].toInt();
    } else {
        _entryPoint = EntryLocationTopRight;
    }

    _cameraCalc.distanceToSurface()->setRawValue        (gridObject[_jsonV3GridAltitudeKey].toDouble());
    _cameraCalc.adjustedFootprintSide()->setRawValue    (gridObject[_jsonV3GridSpacingKey].toDouble());
    _cameraCalc.adjustedFootprintFrontal()->setRawValue (complexObject[_jsonV3CameraTriggerDistanceKey].toDouble());

    if (manualGrid) {
        _cameraCalc.setCameraBrand(CameraCalc::canonicalManualCameraName());
    } else {
        if (!complexObject.contains(_jsonV3CameraObjectKey)) {
            errorString = tr("%1 but %2 object is missing").arg("manualGrid = false").arg("camera");
            _ignoreRecalc = false;
            return false;
        }

        QJsonObject cameraObject = complexObject[_jsonV3CameraObjectKey].toObject();

        // Older code had typo on "imageSideOverlap" incorrectly being "imageSizeOverlap"
        QString incorrectImageSideOverlap = "imageSizeOverlap";
        if (cameraObject.contains(incorrectImageSideOverlap)) {
            cameraObject[_jsonV3SideOverlapKey] = cameraObject[incorrectImageSideOverlap];
            cameraObject.remove(incorrectImageSideOverlap);
        }

        QList<JsonHelper::KeyValidateInfo> cameraKeyInfoList = {
            { _jsonV3GroundResolutionKey,           QJsonValue::Double, true },
            { _jsonV3FrontalOverlapKey,             QJsonValue::Double, true },
            { _jsonV3SideOverlapKey,                QJsonValue::Double, true },
            { _jsonV3CameraSensorWidthKey,          QJsonValue::Double, true },
            { _jsonV3CameraSensorHeightKey,         QJsonValue::Double, true },
            { _jsonV3CameraResolutionWidthKey,      QJsonValue::Double, true },
            { _jsonV3CameraResolutionHeightKey,     QJsonValue::Double, true },
            { _jsonV3CameraFocalLengthKey,          QJsonValue::Double, true },
            { _jsonV3CameraNameKey,                 QJsonValue::String, true },
            { _jsonV3CameraOrientationLandscapeKey, QJsonValue::Bool,   true },
            { _jsonV3CameraMinTriggerIntervalKey,   QJsonValue::Double, false },
        };
        if (!JsonHelper::validateKeys(cameraObject, cameraKeyInfoList, errorString)) {
            _ignoreRecalc = false;
            return false;
        }

        _cameraCalc.landscape()->setRawValue            (cameraObject[_jsonV3CameraOrientationLandscapeKey].toBool(true));
        _cameraCalc.frontalOverlap()->setRawValue       (cameraObject[_jsonV3FrontalOverlapKey].toInt());
        _cameraCalc.sideOverlap()->setRawValue          (cameraObject[_jsonV3SideOverlapKey].toInt());
        _cameraCalc.sensorWidth()->setRawValue          (cameraObject[_jsonV3CameraSensorWidthKey].toDouble());
        _cameraCalc.sensorHeight()->setRawValue         (cameraObject[_jsonV3CameraSensorHeightKey].toDouble());
        _cameraCalc.focalLength()->setRawValue          (cameraObject[_jsonV3CameraFocalLengthKey].toDouble());
        _cameraCalc.imageWidth()->setRawValue           (cameraObject[_jsonV3CameraResolutionWidthKey].toInt());
        _cameraCalc.imageHeight()->setRawValue          (cameraObject[_jsonV3CameraResolutionHeightKey].toInt());
        _cameraCalc.minTriggerInterval()->setRawValue   (cameraObject[_jsonV3CameraMinTriggerIntervalKey].toDouble(0));
        _cameraCalc.imageDensity()->setRawValue         (cameraObject[_jsonV3GroundResolutionKey].toDouble());
        _cameraCalc.fixedOrientation()->setRawValue     (false);
        _cameraCalc._setCameraNameFromV3TransectLoad    (cameraObject[_jsonV3CameraNameKey].toString());
    }

    // Polygon shape
    /// Load a polygon from json
    ///     @param json Json object to load from
    ///     @param required true: no polygon in object will generate error
    ///     @param errorString Error string if return is false
    /// @return true: success, false: failure (errorString set)
    if (!_surveyAreaPolygon.loadFromJson(complexObject, true /* required */, errorString)) {
        _surveyAreaPolygon.clear();
        _ignoreRecalc = false;
        return false;
    }

    _ignoreRecalc = false;

    return true;
}

/// Reverse the order of the transects. First transect becomes last and so forth.
void AgroComplexItem::_reverseTransectOrder(QList<QList<QGeoCoordinate>>& transects)
{
    QList<QList<QGeoCoordinate>> rgReversedTransects;
    for (int i=transects.count() - 1; i>=0; i--) {
        rgReversedTransects.append(transects[i]);
    }
    transects = rgReversedTransects;
}

/// Reverse the order of all points withing each transect, First point becomes last and so forth.
void AgroComplexItem::_reverseInternalTransectPoints(QList<QList<QGeoCoordinate>>& transects)
{
    for (int i=0; i<transects.count(); i++) {
        QList<QGeoCoordinate> rgReversedCoords;
        QList<QGeoCoordinate>& rgOriginalCoords = transects[i];
        for (int j=rgOriginalCoords.count()-1; j>=0; j--) {
            rgReversedCoords.append(rgOriginalCoords[j]);
        }
        transects[i] = rgReversedCoords;
    }
}

/// Reorders the transects such that the first transect is the shortest distance to the specified coordinate
/// and the first point within that transect is the shortest distance to the specified coordinate.
///     @param distanceCoord Coordinate to measure distance against
///     @param transects Transects to test and reorder
void AgroComplexItem::_optimizeTransectsForShortestDistance(const QGeoCoordinate& distanceCoord, QList<QList<QGeoCoordinate>>& transects)
{
    double rgTransectDistance[4];
    rgTransectDistance[0] = transects.first().first().distanceTo(distanceCoord);
    rgTransectDistance[1] = transects.first().last().distanceTo(distanceCoord);
    rgTransectDistance[2] = transects.last().first().distanceTo(distanceCoord);
    rgTransectDistance[3] = transects.last().last().distanceTo(distanceCoord);

    int shortestIndex = 0;
    double shortestDistance = rgTransectDistance[0];
    for (int i=1; i<3; i++) {
        if (rgTransectDistance[i] < shortestDistance) {
            shortestIndex = i;
            shortestDistance = rgTransectDistance[i];
        }
    }

    if (shortestIndex > 1) {
        // We need to reverse the order of segments
        _reverseTransectOrder(transects);
    }
    if (shortestIndex & 1) {
        // We need to reverse the points within each segment
        _reverseInternalTransectPoints(transects);
    }
}

qreal AgroComplexItem::_ccw(QPointF pt1, QPointF pt2, QPointF pt3)
{
    return (pt2.x()-pt1.x())*(pt3.y()-pt1.y()) - (pt2.y()-pt1.y())*(pt3.x()-pt1.x());
}

qreal AgroComplexItem::_dp(QPointF pt1, QPointF pt2)
{
    return (pt2.x()-pt1.x())/qSqrt((pt2.x()-pt1.x())*(pt2.x()-pt1.x()) + (pt2.y()-pt1.y())*(pt2.y()-pt1.y()));
}

void AgroComplexItem::_swapPoints(QList<QPointF>& points, int index1, int index2)
{
    QPointF temp = points[index1];
    points[index1] = points[index2];
    points[index2] = temp;
}

/// Returns true if the current grid angle generates north/south oriented transects
bool AgroComplexItem::_gridAngleIsNorthSouthTransects()
{
    // Grid angle ranges from -360<->360
    double gridAngle = qAbs(_gridAngleFact.rawValue().toDouble());
    return gridAngle < 45.0 || (gridAngle > 360.0 - 45.0) || (gridAngle > 90.0 + 45.0 && gridAngle < 270.0 - 45.0);
}

void AgroComplexItem::_adjustTransectsToEntryPointLocation(QList<QList<QGeoCoordinate>>& transects)
{
    if (transects.count() == 0) {
        return;
    }

    bool reversePoints = false;
    bool reverseTransects = false;

    if (_entryPoint == EntryLocationBottomLeft || _entryPoint == EntryLocationBottomRight) {
        reversePoints = true;
    }
    if (_entryPoint == EntryLocationTopRight || _entryPoint == EntryLocationBottomRight) {
        reverseTransects = true;
    }

    if (reversePoints) {
        qCDebug(AgroComplexItemLog) << "_adjustTransectsToEntryPointLocation Reverse Points";
        _reverseInternalTransectPoints(transects);
    }
    if (reverseTransects) {
        qCDebug(AgroComplexItemLog) << "_adjustTransectsToEntryPointLocation Reverse Transects";
        _reverseTransectOrder(transects);
    }

    qCDebug(AgroComplexItemLog) << "_adjustTransectsToEntryPointLocation Modified entry point:entryLocation" << transects.first().first() << _entryPoint;
}

QPointF AgroComplexItem::_rotatePoint(const QPointF& point, const QPointF& origin, double angle)
{
    QPointF rotated;
    double radians = (M_PI / 180.0) * -angle;

    rotated.setX(((point.x() - origin.x()) * cos(radians)) - ((point.y() - origin.y()) * sin(radians)) + origin.x());
    rotated.setY(((point.x() - origin.x()) * sin(radians)) + ((point.y() - origin.y()) * cos(radians)) + origin.y());

    return rotated;
}

void AgroComplexItem::_intersectLinesWithRect(const QList<QLineF>& lineList, const QRectF& boundRect, QList<QLineF>& resultLines)
{
    QLineF topLine      (boundRect.topLeft(),       boundRect.topRight());
    QLineF bottomLine   (boundRect.bottomLeft(),    boundRect.bottomRight());
    QLineF leftLine     (boundRect.topLeft(),       boundRect.bottomLeft());
    QLineF rightLine    (boundRect.topRight(),      boundRect.bottomRight());

    for (int i=0; i<lineList.count(); i++) {
        QPointF intersectPoint;
        QLineF intersectLine;
        const QLineF& line = lineList[i];

        auto isLineBoundedIntersect = [&line, &intersectPoint](const QLineF& linePosition) {
            return line.intersects(linePosition, &intersectPoint) == QLineF::BoundedIntersection;
        };

        int foundCount = 0;
        if (isLineBoundedIntersect(topLine)) {
            intersectLine.setP1(intersectPoint);
            foundCount++;
        }
        if (isLineBoundedIntersect(rightLine)) {
            if (foundCount == 0) {
                intersectLine.setP1(intersectPoint);
            } else {
                if (foundCount != 1) {
                    qWarning() << "Found more than two intersecting points";
                }
                intersectLine.setP2(intersectPoint);
            }
            foundCount++;
        }
        if (isLineBoundedIntersect(bottomLine)) {
            if (foundCount == 0) {
                intersectLine.setP1(intersectPoint);
            } else {
                if (foundCount != 1) {
                    qWarning() << "Found more than two intersecting points";
                }
                intersectLine.setP2(intersectPoint);
            }
            foundCount++;
        }
        if (isLineBoundedIntersect(leftLine)) {
            if (foundCount == 0) {
                intersectLine.setP1(intersectPoint);
            } else {
                if (foundCount != 1) {
                    qWarning() << "Found more than two intersecting points";
                }
                intersectLine.setP2(intersectPoint);
            }
            foundCount++;
        }

        if (foundCount == 2) {
            resultLines += intersectLine;
        }
    }
}

void AgroComplexItem::_intersectLinesWithPolygon(const QList<QLineF>& lineList, const QList<QPolygonF>& allowedPolygons, QList<QLineF>& resultLines)
{
    resultLines.clear();
    if (allowedPolygons.isEmpty() || lineList.isEmpty()) return;

    ClipperLib::Paths subjectPaths;
    for (const auto& poly : allowedPolygons) {
        subjectPaths.push_back(_toClipperPath(poly));
    }

    // We use strictly the same scale as in _toClipperPath
    const double scale = ClipperScale;

    for (const QLineF& line : lineList) {
        ClipperLib::Path linePath;
        linePath.push_back(ClipperLib::IntPoint(static_cast<long long>(std::round(line.p1().x() * scale)),
                                                static_cast<long long>(std::round(line.p1().y() * scale))));
        linePath.push_back(ClipperLib::IntPoint(static_cast<long long>(std::round(line.p2().x() * scale)),
                                                static_cast<long long>(std::round(line.p2().y() * scale))));

        ClipperLib::Clipper c;
        c.AddPath(linePath, ClipperLib::ptSubject, false); // false = open path (line)
        c.AddPaths(subjectPaths, ClipperLib::ptClip, true); // true = closed (polygon)

        ClipperLib::PolyTree solutionTree;
        // Using pftNonZero for more reliable clipping
        c.Execute(ClipperLib::ctIntersection, solutionTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        ClipperLib::Paths intersectedPaths;
        ClipperLib::OpenPathsFromPolyTree(solutionTree, intersectedPaths);

        for (const auto& path : intersectedPaths) {
            if (path.size() < 2) continue;
            for (size_t i = 0; i < path.size() - 1; ++i) {
                resultLines.append(QLineF(
                    static_cast<double>(path[i].X) / scale, static_cast<double>(path[i].Y) / scale,
                    static_cast<double>(path[i+1].X) / scale, static_cast<double>(path[i+1].Y) / scale
                ));
            }
        }
    }
}

// Adjust the line segments such that they are all going the same direction with respect to going from P1->P2
void AgroComplexItem::_adjustLineDirection(const QList<QLineF>& lineList, QList<QLineF>& resultLines)
{
    qreal firstAngle = 0;
    for (int i=0; i<lineList.count(); i++) {
        const QLineF& line = lineList[i];
        QLineF adjustedLine;

        if (i == 0) {
            firstAngle = line.angle();
        }

        if (qAbs(line.angle() - firstAngle) > 1.0) {
            adjustedLine.setP1(line.p2());
            adjustedLine.setP2(line.p1());
        } else {
            adjustedLine = line;
        }

        resultLines += adjustedLine;
    }
}

double AgroComplexItem::_clampGridAngle90(double gridAngle)
{
    // Clamp grid angle to -90<->90. This prevents transects from being rotated to a reversed order.
    if (gridAngle > 90.0) {
        gridAngle -= 180.0;
    } else if (gridAngle < -90.0) {
        gridAngle += 180;
    }
    return gridAngle;
}

bool AgroComplexItem::_nextTransectCoord(const QList<QGeoCoordinate>& transectPoints, int pointIndex, QGeoCoordinate& coord)
{
    if (pointIndex > transectPoints.count()) {
        qWarning() << "Bad grid generation";
        return false;
    }

    coord = transectPoints[pointIndex];
    return true;
}

bool AgroComplexItem::_hasTurnaround(void) const
{
    return _turnAroundDistance() > 0;
}

double AgroComplexItem::_turnaroundDistance(void) const
{
    return _turnAroundDistanceFact.rawValue().toDouble();
}

void AgroComplexItem::_rebuildTransectsPhase1(void)
{
    _rebuildTransectsPhase1WorkerSinglePolygon(false /* refly */);
    if (_refly90DegreesFact.rawValue().toBool()) {
        _rebuildTransectsPhase1WorkerSinglePolygon(true /* refly */);
    }
}

void AgroComplexItem::_rebuildTransectsPhase1WorkerSinglePolygon(bool refly) {

    if (_isExclusionZoneFact.rawValue().toBool()) {
        _transects.clear();
        // If the parent class has path lists, they should be cleared too
        return;
    }

    if (_ignoreRecalc || _surveyAreaPolygon.count() < 3) return;

    if (_surveyAreaPolygon.pathModel().count() == 0) return;
    // Coordinates (NED)
    QGeoCoordinate tangentOrigin = _surveyAreaPolygon.pathModel().value<QGCQGeoCoordinate*>(0)->coordinate();

    // Preparing the main training ground
    QPolygonF mainPolyNED;
    for (int i=0; i<_surveyAreaPolygon.count(); i++) {
        double y, x, d;
        QGCGeo::convertGeoToNed(_surveyAreaPolygon.vertexCoordinate(i), tangentOrigin, y, x, d);
        mainPolyNED << QPointF(x, y);
    }

    // We collect all the “red zones” in their original form
    QList<QPolygonF> rawExclusionPolysNED;
    if (_masterController && _masterController->missionController()) {
        QmlObjectListModel* items = _masterController->missionController()->visualItems();
        for (int i = 0; i < items->count(); i++) {
            AgroComplexItem* agroItem = qobject_cast<AgroComplexItem*>(items->get(i));
            if (agroItem && agroItem != this && agroItem->isExclusionZone()->rawValue().toBool()) {
                QPolygonF exPoly;
                for (int j=0; j < agroItem->surveyAreaPolygon()->count(); j++) {
                    double y, x, d;
                    QGCGeo::convertGeoToNed(agroItem->surveyAreaPolygon()->vertexCoordinate(j), tangentOrigin, y, x, d);
                    exPoly << QPointF(x, y);
                }
                if (exPoly.count() >= 3) rawExclusionPolysNED.append(exPoly);
            }
        }
    }

    // We inflate the restricted areas (for example, by 2 meters)
    // In the future, 'margin' can be added to the interface settings
    double margin = 2.0;
    QList<QPolygonF> inflatedExclusionPolysNED;
    _inflateExclusionZones(rawExclusionPolysNED, margin, inflatedExclusionPolysNED);

    // Subtract the BOLDED zones from the field
    ClipperLib::Clipper c;
    c.StrictlySimple(true);

    ClipperLib::Path subjectPath = _toClipperPath(mainPolyNED);
    if (!ClipperLib::Orientation(subjectPath)) ClipperLib::ReversePath(subjectPath);
    c.AddPath(subjectPath, ClipperLib::ptSubject, true);

    for (const auto& poly : inflatedExclusionPolysNED) {
        ClipperLib::Path path = _toClipperPath(poly);
        if (ClipperLib::Orientation(path)) ClipperLib::ReversePath(path);
        c.AddPath(path, ClipperLib::ptClip, true);
    }

    ClipperLib::Paths solution;
    c.Execute(ClipperLib::ctDifference, solution, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
    QList<QPolygonF> allowedPolygons;
    for (const auto& path : solution) {
        allowedPolygons << _fromClipperPath(path);
    }

    _generateTransectsForAllowedPolygons(refly, allowedPolygons, tangentOrigin, inflatedExclusionPolysNED);
}

for (const QLineF& gridLine : rawLines) {
        QList<QLineF> segmentsOnThisLine;
        _intersectLinesWithPolygon({gridLine}, allowedPolygons, segmentsOnThisLine);

        if (segmentsOnThisLine.isEmpty()) continue;

        // We sort the segments according to their REAL position on the line
        // compare the distance from the beginning of gridLine.p1() to the MIDDLE of the segment
        std::sort(segmentsOnThisLine.begin(), segmentsOnThisLine.end(), [&gridLine](const QLineF& a, const QLineF& b) {
            QPointF midA = (a.p1() + a.p2()) / 2.0;
            QPointF midB = (b.p1() + b.p2()) / 2.0;
            return QLineF(gridLine.p1(), midA).length() < QLineF(gridLine.p1(), midB).length();
        });

        // If this is a reverse snake pass, we reverse the order of segments on the line
        if (reverseLineOrder) {
            std::reverse(segmentsOnThisLine.begin(), segmentsOnThisLine.end());
        }

        for (QLineF& segment : segmentsOnThisLine) {
            // Direct the segment vector in the direction of movement
            bool needsFlip = false;
            double distP1 = QLineF(gridLine.p1(), segment.p1()).length();
            double distP2 = QLineF(gridLine.p1(), segment.p2()).length();

            if (!reverseLineOrder) {
                if (distP1 > distP2) needsFlip = true;
            } else {
                if (distP1 < distP2) needsFlip = true;
            }

            if (needsFlip) {
                segment = QLineF(segment.p2(), segment.p1());
            }

            // Add Bypass before the segment
            if (!_transects.isEmpty()) {
                _appendBypassIfNecessary(_transects.last().last().coord,
                                         _toGeo(segment.p1(), tangentOrigin),
                                         tangentOrigin,
                                         exclusionPolygons);
            }

            // Adding the working passage itself
            QList<TransectStyleComplexItem::CoordInfo_t> transect;
            transect.append({_toGeo(segment.p1(), tangentOrigin), CoordTypeSurveyEntry});
            transect.append({_toGeo(segment.p2(), tangentOrigin), CoordTypeSurveyExit});
            _transects.append(transect);
        }
        reverseLineOrder = !reverseLineOrder;
    }

void AgroComplexItem::_rebuildTransectsFromPolygon(bool refly, const QPolygonF& polygon, const QGeoCoordinate& tangentOrigin, const QPointF* const transitionPoint)
{
    // Generate transects

    double gridAngle = _gridAngleFact.rawValue().toDouble();
    double gridSpacing = _cameraCalc.adjustedFootprintSide()->rawValue().toDouble();

    gridAngle = _clampGridAngle90(gridAngle);
    gridAngle += refly ? 90 : 0;
    qCDebug(AgroComplexItemLog) << "_rebuildTransectsPhase1 Clamped grid angle" << gridAngle;

    qCDebug(AgroComplexItemLog) << "_rebuildTransectsPhase1 gridSpacing:gridAngle:refly" << gridSpacing << gridAngle << refly;

    // Convert polygon to bounding rect

    qCDebug(AgroComplexItemLog) << "_rebuildTransectsPhase1 Polygon";
    QRectF boundingRect = polygon.boundingRect();
    QPointF boundingCenter = boundingRect.center();
    qCDebug(AgroComplexItemLog) << "Bounding rect" << boundingRect.topLeft().x() << boundingRect.topLeft().y() << boundingRect.bottomRight().x() << boundingRect.bottomRight().y();

    // Create set of rotated parallel lines within the expanded bounding rect. Make the lines larger than the
    // bounding box to guarantee intersection.

    QList<QLineF> lineList;

    // Transects are generated to be as long as the largest width/height of the bounding rect plus some fudge factor.
    // This way they will always be guaranteed to intersect with a polygon edge no matter what angle they are rotated to.
    // They are initially generated with the transects flowing from west to east and then points within the transect north to south.
    double maxWidth = qMax(boundingRect.width(), boundingRect.height()) + 2000.0;
    double halfWidth = maxWidth / 2.0;
    double transectX = boundingCenter.x() - halfWidth;
    double transectXMax = transectX + maxWidth;
    while (transectX < transectXMax) {
        double transectYTop = boundingCenter.y() - halfWidth;
        double transectYBottom = boundingCenter.y() + halfWidth;

        lineList += QLineF(_rotatePoint(QPointF(transectX, transectYTop), boundingCenter, gridAngle), _rotatePoint(QPointF(transectX, transectYBottom), boundingCenter, gridAngle));
        transectX += gridSpacing;
    }

    // Now intersect the lines with the polygon
    QList<QLineF> intersectLines;
#if 1
    _intersectLinesWithPolygon(lineList, {polygon}, intersectLines);
#else
    // This is handy for debugging grid problems, not for release
    intersectLines = lineList;
#endif

    // Less than two transects intersected with the polygon:
    //      Create a single transect which goes through the center of the polygon
    //      Intersect it with the polygon
    if (intersectLines.count() < 2) {
        _surveyAreaPolygon.center();
        QLineF firstLine = lineList.first();
        QPointF lineCenter = firstLine.pointAt(0.5);
        QPointF centerOffset = boundingCenter - lineCenter;
        firstLine.translate(centerOffset);
        lineList.clear();
        lineList.append(firstLine);
        intersectLines = lineList;
        _intersectLinesWithPolygon(lineList, {polygon}, intersectLines);
    }

    // Make sure all lines are going the same direction. Polygon intersection leads to lines which
    // can be in varied directions depending on the order of the intesecting sides.
    QList<QLineF> resultLines;
    _adjustLineDirection(intersectLines, resultLines);

    // Convert from NED to Geo
    QList<QList<QGeoCoordinate>> transects;

    if (transitionPoint != nullptr) {
        QList<QGeoCoordinate>   transect;
        QGeoCoordinate          coord;
        QGCGeo::convertNedToGeo(transitionPoint->y(), transitionPoint->x(), 0, tangentOrigin, coord);
        transect.append(coord);
        transect.append(coord); //TODO
        transects.append(transect);
    }

    for (const QLineF& line: resultLines) {
        QList<QGeoCoordinate>   transect;
        QGeoCoordinate          coord;

        QGCGeo::convertNedToGeo(line.p1().y(), line.p1().x(), 0, tangentOrigin, coord);
        transect.append(coord);
        QGCGeo::convertNedToGeo(line.p2().y(), line.p2().x(), 0, tangentOrigin, coord);
        transect.append(coord);

        transects.append(transect);
    }

    _adjustTransectsToEntryPointLocation(transects);

    if (!_transects.isEmpty()) {
        _optimizeTransectsForShortestDistance(_transects.last().last().coord, transects);
    }

    if (_flyAlternateTransectsFact.rawValue().toBool()) {
        QList<QList<QGeoCoordinate>> alternatingTransects;
        for (int i=0; i<transects.count(); i++) {
            if (!(i & 1)) {
                alternatingTransects.append(transects[i]);
            }
        }
        for (int i=transects.count()-1; i>0; i--) {
            if (i & 1) {
                alternatingTransects.append(transects[i]);
            }
        }
        transects = alternatingTransects;
    }

    // Adjust to lawnmower pattern
    bool reverseVertices = false;
    for (int i=0; i<transects.count(); i++) {
        // We must reverse the vertices for every other transect in order to make a lawnmower pattern
        QList<QGeoCoordinate> transectVertices = transects[i];
        if (reverseVertices) {
            reverseVertices = false;
            QList<QGeoCoordinate> reversedVertices;
            for (int j=transectVertices.count()-1; j>=0; j--) {
                reversedVertices.append(transectVertices[j]);
            }
            transectVertices = reversedVertices;
        } else {
            reverseVertices = true;
        }
        transects[i] = transectVertices;
    }

    // Convert to CoordInfo transects and append to _transects
    for (const QList<QGeoCoordinate>& transect: transects) {
        QGeoCoordinate                                  coord;
        QList<TransectStyleComplexItem::CoordInfo_t>    coordInfoTransect;
        TransectStyleComplexItem::CoordInfo_t           coordInfo;

        coordInfo = { transect[0], CoordTypeSurveyEntry };
        coordInfoTransect.append(coordInfo);
        coordInfo = { transect[1], CoordTypeSurveyExit };
        coordInfoTransect.append(coordInfo);

        // For hover and capture we need points for each camera location within the transect
        if (triggerCamera() && hoverAndCaptureEnabled()) {
            double transectLength = transect[0].distanceTo(transect[1]);
            double transectAzimuth = transect[0].azimuthTo(transect[1]);
            if (triggerDistance() < transectLength) {
                int cInnerHoverPoints = static_cast<int>(floor(transectLength / triggerDistance()));
                qCDebug(AgroComplexItemLog) << "cInnerHoverPoints" << cInnerHoverPoints;
                for (int i=0; i<cInnerHoverPoints; i++) {
                    QGeoCoordinate hoverCoord = transect[0].atDistanceAndAzimuth(triggerDistance() * (i + 1), transectAzimuth);
                    TransectStyleComplexItem::CoordInfo_t coordInfo = { hoverCoord, CoordTypeInteriorHoverTrigger };
                    coordInfoTransect.insert(1 + i, coordInfo);
                }
            }
        }

        // Extend the transect ends for turnaround
        if (_hasTurnaround()) {
            QGeoCoordinate turnaroundCoord;
            double turnAroundDistance = _turnAroundDistanceFact.rawValue().toDouble();

            double azimuth = transect[0].azimuthTo(transect[1]);
            turnaroundCoord = transect[0].atDistanceAndAzimuth(-turnAroundDistance, azimuth);
            turnaroundCoord.setAltitude(qQNaN());
            TransectStyleComplexItem::CoordInfo_t coordInfo = { turnaroundCoord, CoordTypeTurnaround };
            coordInfoTransect.prepend(coordInfo);

            azimuth = transect.last().azimuthTo(transect[transect.count() - 2]);
            turnaroundCoord = transect.last().atDistanceAndAzimuth(-turnAroundDistance, azimuth);
            turnaroundCoord.setAltitude(qQNaN());
            coordInfo = { turnaroundCoord, CoordTypeTurnaround };
            coordInfoTransect.append(coordInfo);
        }

        _transects.append(coordInfoTransect);
    }
    qCDebug(AgroComplexItemLog) << "_transects.size() " << _transects.size();
}

void AgroComplexItem::_recalcCameraShots(void)
{
    double triggerDistance = this->triggerDistance();
    if (triggerDistance <= 0) {
        _cameraShots = 0;
        emit cameraShotsChanged();
        return;
    }

    _cameraShots = 0;

    // Safe bypass of all transects
    for (const QList<TransectStyleComplexItem::CoordInfo_t>& transect : _transects) {
        // We skip empty or single dots that break the logic
        if (transect.count() < 2) continue;

        QGeoCoordinate firstCameraCoord, lastCameraCoord;

        if (_hasTurnaround() && !hoverAndCaptureEnabled()) {
            // If there are turns, the camera turns on at the 2nd point and turns off at the penultimate one
            if (transect.count() >= 4) { // Minimum required: Turnaround, Entry, Exit, Turnaround
                firstCameraCoord = transect[1].coord;
                lastCameraCoord = transect[transect.count() - 2].coord;
            } else {
                firstCameraCoord = transect.first().coord;
                lastCameraCoord = transect.last().coord;
            }
        } else {
            firstCameraCoord = transect.first().coord;
            lastCameraCoord = transect.last().coord;
        }

        double dist = firstCameraCoord.distanceTo(lastCameraCoord);
        _cameraShots += qCeil(dist / triggerDistance);
    }

    emit cameraShotsChanged();
}

AgroComplexItem::ReadyForSaveState AgroComplexItem::readyForSaveState(void) const
{
    return TransectStyleComplexItem::readyForSaveState();
}

void AgroComplexItem::rotateEntryPoint(void)
{
    if (_entryPoint == EntryLocationLast) {
        _entryPoint = EntryLocationFirst;
    } else {
        _entryPoint++;
    }

    _rebuildTransects();

    setDirty(true);
}

double AgroComplexItem::timeBetweenShots(void)
{
    return _vehicleSpeed == 0 ? 0 : triggerDistance() / _vehicleSpeed;
}

double AgroComplexItem::additionalTimeDelay (void) const
{
    double hoverTime = 0;

    if (hoverAndCaptureEnabled()) {
        for (const QList<TransectStyleComplexItem::CoordInfo_t>& transect: _transects) {
            hoverTime += _hoverAndCaptureDelaySeconds * transect.count();
        }
    }

    return hoverTime;
}

void AgroComplexItem::_updateWizardMode(void)
{
    if (_surveyAreaPolygon.isValid() && !_surveyAreaPolygon.traceMode()) {
        setWizardMode(false);
    }
}


QList<QLineF> AgroComplexItem::_subtractPolygonFromLine(const QLineF& line, const QPolygonF& polygon)
{
    QList<QLineF> resultLines;
    QList<QPointF> points;
    points << line.p1() << line.p2();

    // We find all the intersection points of the line with the edges of the forbidden polygon
    for (int i = 0; i < polygon.count() - 1; i++) {
        QPointF p;
        if (line.intersects(QLineF(polygon[i], polygon[i+1]), &p) == QLineF::BoundedIntersection) {
            points << p;
        }
    }

    // Sort the points along the line from p1 to p2
    std::sort(points.begin(), points.end(), [&](const QPointF& a, const QPointF& b) {
        return QLineF(line.p1(), a).length() < QLineF(line.p1(), b).length();
    });

    // We check each received segment
    for (int i = 0; i < points.count() - 1; i++) {
        QLineF seg(points[i], points[i+1]);
        if (seg.length() < 0.1) continue;

        // Если середина отрезка СНАРУЖИ запретной зоны (exclusion), мы его берем
        if (!polygon.containsPoint(seg.pointAt(0.5), Qt::WindingFill)) {
            resultLines.append(seg);
        }
    }
    return resultLines;
}

void AgroComplexItem::_updateOtherAgroItems()
{
    if (_ignoreGlobalUpdate) {
        return;
    }

    // If we are not the controller or there is no list of items, exit
    if (!_masterController || !_masterController->missionController()) {
        return;
    }

    // Set a lock flag to prevent falling into an infinite recursion
    _ignoreGlobalUpdate = true;

    QmlObjectListModel* items = _masterController->missionController()->visualItems();
    for (int i = 0; i < items->count(); i++) {
        AgroComplexItem* agroItem = qobject_cast<AgroComplexItem*>(items->get(i));
        // If this is another AgroItem (not ourselves) and it is NOT an exclusion zone (i.e. it's a field that needs to be recalculated)
        if (agroItem && agroItem != this && !agroItem->isExclusionZone()->rawValue().toBool()) {
            // Force a recalculation
            agroItem->_rebuildTransects();
        }
    }

    _ignoreGlobalUpdate = false;
}

// Checking if a polygon is a circle (QGC approximation)
bool AgroComplexItem::_isPolygonCircle(const QPolygonF& polygon, QPointF& center) {
    if (polygon.count() < 10) return false; // Круги обычно имеют 30+ точек
    QRectF br = polygon.boundingRect();
    double ratio = br.width() / br.height();
    if (ratio < 0.8 || ratio > 1.2) return false; // Должен быть примерно квадратным
    center = br.center();
    return true;
}

// Cutting a polygon into top and bottom parts along the Y coordinate (NED)
QList<QPolygonF> AgroComplexItem::_splitPolygonHorizontal(const QPolygonF& polygon, double splitY) {
    QList<QPolygonF> results;
    QRectF br = polygon.boundingRect();

    // Create two rectangles for the intersection: above the line and below the line
    // Use very large widths to ensure the polygon overlaps
    QRectF topRect(br.x() - 1000, br.y() - 1000, br.width() + 2000, (splitY - (br.y() - 1000)));
    QRectF bottomRect(br.x() - 1000, splitY, br.width() + 2000, (br.bottom() + 1000) - splitY);

    QPolygonF topPart = polygon.intersected(topRect);
    QPolygonF bottomPart = polygon.intersected(bottomRect);

    if (!topPart.isEmpty()) results.append(topPart);
    if (!bottomPart.isEmpty()) results.append(bottomPart);

    return results;
}

// Вспомогательная функция: проверяет, пересекает ли прямой отрезок запретные зоны
bool AgroComplexItem::_isPathClear(const QPointF& start, const QPointF& end, const QList<QPolygonF>& exclusionPolygons)
{
    if (QLineF(start, end).length() < 0.1) return true;

    QLineF path(start, end);
    for (const QPolygonF& poly : exclusionPolygons) {
        // Checking for intersection with edges (mathematical)
        for (int i = 0; i < poly.count() - 1; i++) {
            QPointF p;
            if (path.intersects(QLineF(poly[i], poly[i+1]), &p) == QLineF::BoundedIntersection) {
                // If the intersection is not at the very ends of the segment (with a tolerance of 0.05)
                if (QLineF(start, p).length() > 0.05 && QLineF(end, p).length() > 0.05) {
                    return false;
                }
            }
        }
        // Check: is the middle of the segment inside the zone?
        if (poly.containsPoint(path.pointAt(0.5), Qt::WindingFill)) {
            return false;
        }
    }
    return true;
}

void AgroComplexItem::_appendBypassIfNecessary(const QGeoCoordinate& start, const QGeoCoordinate& end, const QGeoCoordinate& tangentOrigin, const QList<QPolygonF>& exclusionPolygons)
{
    if (!start.isValid() || !end.isValid()) return;

    double y1, x1, y2, x2, d;
    QGCGeo::convertGeoToNed(start, tangentOrigin, y1, x1, d);
    QGCGeo::convertGeoToNed(end, tangentOrigin, y2, x2, d);

    QPointF pStart(x1, y1);
    QPointF pEnd(x2, y2);

    // If the path is clear, we just go out
    if (_isPathClear(pStart, pEnd, exclusionPolygons)) return;

    for (const QPolygonF& poly : exclusionPolygons) {
        QLineF jumpPath(pStart, pEnd);
        QList<QPointF> intersections;

        // We find all entry and exit points
        for (int i = 0; i < poly.count() - 1; i++) {
            QPointF p;
            if (jumpPath.intersects(QLineF(poly[i], poly[i+1]), &p) == QLineF::BoundedIntersection) {
                intersections << p;
            }
        }

        if (intersections.count() < 2) continue;

        // Sort the points from the beginning to the end of the path
        std::sort(intersections.begin(), intersections.end(), [&](const QPointF& a, const QPointF& b) {
            return QLineF(pStart, a).length() < QLineF(pStart, b).length();
        });

        QPointF pIn = intersections.first();
        QPointF pOut = intersections.last();

        // Finding the nearest VERTS of the polygon for pIn and pOut
        int idxIn = -1, idxOut = -1;
        double minDistIn = 1e10, minDistOut = 1e10;

        for (int i = 0; i < poly.count() - 1; i++) {
            double dIn = QLineF(pIn, poly[i]).length();
            if (dIn < minDistIn) { minDistIn = dIn; idxIn = i; }
            double dOut = QLineF(pOut, poly[i]).length();
            if (dOut < minDistOut) { minDistOut = dOut; idxOut = i; }
        }

        if (idxIn == -1 || idxOut == -1) continue;

        // We build two traversals along the vertices
        auto getPath = [&](bool cw) {
            QList<QPointF> pts;
            pts << pIn;
            int cur = idxIn;
            int safety = poly.count();
            while (cur != idxOut && safety-- > 0) {
                pts << poly[cur];
                cur = cw ? (cur + 1) % (poly.count() - 1) : (cur - 1 + (poly.count() - 1)) % (poly.count() - 1);
            }
            pts << poly[idxOut] << pOut;
            return pts;
        };

        auto pathCW = getPath(true);
        auto pathCCW = getPath(false);

        auto len = [](const QList<QPointF>& p) {
            double s = 0;
            for(int i=0; i<p.size()-1; i++) s += QLineF(p[i], p[i+1]).length();
            return s;
        };

        const QList<QPointF>& bestPath = (len(pathCW) < len(pathCCW)) ? pathCW : pathCCW;

        // Optimization (Shortcut) -now as careful as possible
        QList<QPointF> optimized;
        optimized << bestPath.first();
        int curr = 0;
        while (curr < bestPath.size() - 1) {
            int nextBest = curr + 1;
            // We are trying to find the farthest visible point
            for (int lookAhead = curr + 2; lookAhead < bestPath.size(); lookAhead++) {
                // Strict _isPathClear now works here
                if (_isPathClear(bestPath[curr], bestPath[lookAhead], exclusionPolygons)) {
                    nextBest = lookAhead;
                }
            }
            optimized << bestPath[nextBest];
            curr = nextBest;
        }

        // Adding points to transects
        QList<TransectStyleComplexItem::CoordInfo_t> bypass;
        for (const auto& pt : optimized) {
            bypass.append({_toGeo(pt, tangentOrigin), CoordTypeInteriorHoverTrigger});
        }
        _transects.append(bypass);
        // break;
    }
}

void AgroComplexItem::_inflateExclusionZones(const QList<QPolygonF>& exclusionPolygonsNED, double marginMeters, QList<QPolygonF>& inflatedPolygonsNED)
{
    inflatedPolygonsNED.clear();

    if (exclusionPolygonsNED.isEmpty()) {
        return;
    }

    // If the indentation is almost zero, simply copy the original zones
    if (marginMeters < 0.05) {
        inflatedPolygonsNED = exclusionPolygonsNED;
        return;
    }

    // Clipper works with integers. 1000.0 provides millimeter accuracy for meters.
    const double scale = 1000.0;
    ClipperLib::ClipperOffset offsetter;

    // Set the limit for sharp corners (MiterLimit).
    // A value of 2.0 means that if a corner is too sharp, it will be cut off to avoid going to infinity.
    offsetter.MiterLimit = 2.0;

    for (const QPolygonF& polygon : exclusionPolygonsNED) {
        if (polygon.count() < 3) continue;

        ClipperLib::Path path;
        for (const QPointF& pt : polygon) {
            path.push_back(ClipperLib::IntPoint(
                static_cast<ClipperLib::cInt>(std::round(pt.x() * scale)),
                static_cast<ClipperLib::cInt>(std::round(pt.y() * scale))
            ));
        }

        // ClipperOffset requires correct orientation (clockwise for outer contours)
        if (!ClipperLib::Orientation(path)) {
            ClipperLib::ReversePath(path);
        }

        // This removes the "teeth" (dozens of dots on a fillet) and replaces them with one sharp corner.
        offsetter.AddPath(path, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    }

    ClipperLib::Paths solution;
    // Performing bloat (indentation)
    offsetter.Execute(solution, marginMeters * scale);

    for (const ClipperLib::Path& outPath : solution) {
        if (outPath.size() < 3) continue;

        QPolygonF inflatedPoly;
        for (const ClipperLib::IntPoint& pt : outPath) {
            inflatedPoly << QPointF(
                static_cast<double>(pt.X) / scale,
                static_cast<double>(pt.Y) / scale
            );
        }

        // Closing the polygon for correct display in QGC
        if (!inflatedPoly.isEmpty()) {
            if (inflatedPoly.first() != inflatedPoly.last()) {
                inflatedPoly << inflatedPoly.first();
            }
            inflatedPolygonsNED.append(inflatedPoly);
        }
    }

    qCDebug(AgroComplexItemLog) << "Inclusion margin applied (Miter):" << marginMeters
                                << "Original zones:" << exclusionPolygonsNED.count()
                                << "Inflated zones:" << inflatedPolygonsNED.count();
}

QPointF AgroComplexItem::_getAndOrderFirstPoint(const QList<QLineF>& segments, const QRectF& boundingRect)
{
    if (segments.isEmpty()) return QPointF(0,0);

    QPointF target;
    switch (_entryPoint) {
        case EntryLocationTopLeft:     target = boundingRect.topLeft(); break;
        case EntryLocationTopRight:    target = boundingRect.topRight(); break;
        case EntryLocationBottomLeft:  target = boundingRect.bottomLeft(); break;
        case EntryLocationBottomRight: target = boundingRect.bottomRight(); break;
        default: target = boundingRect.topLeft();
    }

    // We are looking for the closest point of any segment to the selected corner
    QPointF closest;
    double minDist = std::numeric_limits<double>::max();
    for (const QLineF& s : segments) {
        double d1 = QLineF(target, s.p1()).length();
        double d2 = QLineF(target, s.p2()).length();
        if (d1 < minDist) { minDist = d1; closest = s.p1(); }
        if (d2 < minDist) { minDist = d2; closest = s.p2(); }
    }
    return target; // Or return target, the greedy algorithm itself will find the nearest start
}
