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
                                        servoInstance,  // Param 1: Номер сервопривода (Instance/Channel)
                                        pwmValue,       // Param 2: PWM (микросекунды)
                                        0,              // Param 3: (не используется)
                                        0, 0, 0, 0,     // Param 4-7
                                        true,           // autoContinue
                                        false,          // isCurrentItem
                                        missionItemParent);
    items.append(item);
}

void AgroComplexItem::_appendVisualAction(QList<MissionItem*>& items, QObject* missionItemParent, int& seqNum, MAV_FRAME frame, const QGeoCoordinate& coord)
{
    // === ВИЗУАЛЬНЫЙ ТЕСТ: "ПРЫЖОК" ===

    // 1. Берем текущую высоту и добавляем 10 метров
    double jumpAltitude = coord.altitude() + 10.0;

    // 2. Время зависания в верхней точке (в секундах)
    float holdTime = 5.0f;

    MissionItem* item = new MissionItem(seqNum++,
                                        MAV_CMD_NAV_WAYPOINT, // Стандартная команда полета (поддерживается 100%)
                                        frame,
                                        holdTime,       // Param 1: Время ожидания (Hold time)
                                        0,              // Param 2: Радиус (0 = по умолчанию)
                                        0,              // Param 3: Pass Radius
                                        0,              // Param 4: Yaw (0 = не менять)
                                        coord.latitude(),  // Param 5: Та же широта
                                        coord.longitude(), // Param 6: Та же долгота
                                        jumpAltitude,      // Param 7: НОВАЯ ВЫСОТА (+10м)
                                        true,           // autoContinue
                                        false,          // isCurrentItem
                                        missionItemParent);
    items.append(item);
}

void AgroComplexItem::appendMissionItems(QList<MissionItem*>& items, QObject* missionItemParent)
{
    // Если миссия загружена из файла - используем родительскую логику загрузки
    if (_loadedMissionItems.count()) {
        TransectStyleComplexItem::appendMissionItems(items, missionItemParent);
        return;
    }

    if (_isExclusionZoneFact.rawValue().toBool()) {
        return;
    }

    int seqNum = _sequenceNumber;

    // Определяем тип высоты
    MAV_FRAME mavFrame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    if (_cameraCalc.distanceMode() == QGroundControlQmlGlobal::AltitudeModeAbsolute ||
        _cameraCalc.distanceMode() == QGroundControlQmlGlobal::AltitudeModeCalcAboveTerrain) {
        mavFrame = MAV_FRAME_GLOBAL;
    } else if (_cameraCalc.distanceMode() == QGroundControlQmlGlobal::AltitudeModeTerrainFrame) {
        mavFrame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
    }

    // Проходим по рассчитанным точкам (они доступны, т.к. protected в родителе)
    for (int i = 0; i < _rgFlightPathCoordInfo.count(); i++) {
        const CoordInfo_t& coordInfo = _rgFlightPathCoordInfo[i];

        // Используем метод родителя для добавления точки полета
        // (он protected, поэтому доступен нам здесь)
        _appendWaypoint(items, missionItemParent, seqNum, mavFrame, 0, coordInfo.coord);

        // Добавляем логику опрыскивателя
        switch (coordInfo.coordType) {
            case CoordTypeSurveyEntry:
                // Включаем ПОСЛЕ прилета в точку входа
                _appendSprayerCommand(items, missionItemParent, seqNum, true);
                // _appendVisualAction(items, missionItemParent, seqNum, mavFrame, coordInfo.coord);
                break;

            case CoordTypeSurveyExit:
                // Выключаем ПОСЛЕ прилета в точку выхода
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

void AgroComplexItem::_intersectLinesWithPolygon(const QList<QLineF>& lineList, const QPolygonF& polygon, QList<QLineF>& resultLines)
{
    resultLines.clear();
    for (const QLineF& line : lineList) {
        QList<QPointF> intersections;
        intersections.append(line.p1());
        intersections.append(line.p2());

        for (int j = 0; j < polygon.count() - 1; j++) {
            QPointF p;
            if (line.intersects(QLineF(polygon[j], polygon[j+1]), &p) == QLineF::BoundedIntersection) {
                if (!intersections.contains(p)) intersections.append(p);
            }
        }

        std::sort(intersections.begin(), intersections.end(), [&](const QPointF& a, const QPointF& b) {
            return QLineF(line.p1(), a).length() < QLineF(line.p1(), b).length();
        });

        for (int i = 0; i < intersections.count() - 1; i++) {
            QLineF segment(intersections[i], intersections[i+1]);
            if (segment.length() < 0.1) continue;
            // Проверяем, что середина сегмента внутри рабочего полигона
            if (polygon.containsPoint(segment.pointAt(0.5), Qt::OddEvenFill)) {
                resultLines.append(segment);
            }
        }
    }
}

/// Adjust the line segments such that they are all going the same direction with respect to going from P1->P2
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
    if (_ignoreRecalc || _surveyAreaPolygon.count() < 3) return;

    QGeoCoordinate tangentOrigin = _surveyAreaPolygon.pathModel().value<QGCQGeoCoordinate*>(0)->coordinate();

    QPolygonF mainPolyNED;
    for (int i=0; i<_surveyAreaPolygon.count(); i++) {
        double y, x, d; QGCGeo::convertGeoToNed(_surveyAreaPolygon.vertexCoordinate(i), tangentOrigin, y, x, d);
        mainPolyNED << QPointF(x, y);
    }

    ClipperLib::Paths clipPaths;         // Для Clipper
    QList<QPolygonF> exclusionPolysNED; // ДЛЯ ГЕНЕРАТОРА (Qt формат)

    if (_masterController && _masterController->missionController()) {
        QmlObjectListModel* items = _masterController->missionController()->visualItems();
        for (int i = 0; i < items->count(); i++) {
            AgroComplexItem* agroItem = qobject_cast<AgroComplexItem*>(items->get(i));
            if (agroItem && agroItem != this && agroItem->isExclusionZone()->rawValue().toBool()) {
                QPolygonF exPoly;
                for (int j=0; j < agroItem->surveyAreaPolygon()->count(); j++) {
                    double y, x, d; QGCGeo::convertGeoToNed(agroItem->surveyAreaPolygon()->vertexCoordinate(j), tangentOrigin, y, x, d);
                    exPoly << QPointF(x, y);
                }
                if (exPoly.count() >= 3) {
                    clipPaths.push_back(_toClipperPath(exPoly)); // Добавляем в Clipper формат
                    exclusionPolysNED.append(exPoly);           // ДОБАВЛЯЕМ В QT ФОРМАТ
                }
            }
        }
    }

    // Вычитание
    ClipperLib::Clipper c;
    c.AddPath(_toClipperPath(mainPolyNED), ClipperLib::ptSubject, true);
    c.AddPaths(clipPaths, ClipperLib::ptClip, true);
    ClipperLib::Paths solution;
    c.Execute(ClipperLib::ctDifference, solution, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    QList<QPolygonF> allowedPolygons;
    for (const auto& path : solution) allowedPolygons << _fromClipperPath(path);

    // ВАЖНО: Передаем exclusionPolysNED вместо clipPaths
    _generateTransectsForAllowedPolygons(refly, allowedPolygons, tangentOrigin, exclusionPolysNED);
}

void AgroComplexItem::_generateTransectsForAllowedPolygons(bool refly, const QList<QPolygonF>& allowedPolygons, const QGeoCoordinate& tangentOrigin, const QList<QPolygonF>& exclusionPolygons)
{
    _transects.clear();
    if (allowedPolygons.isEmpty()) return;

    double gridAngle = _clampGridAngle90(_gridAngleFact.rawValue().toDouble() + (refly ? 90 : 0));
    double gridSpacing = _cameraCalc.adjustedFootprintSide()->rawValue().toDouble();

    QRectF totalBr;
    for (const auto& p : allowedPolygons) totalBr = totalBr.united(p.boundingRect());
    QPointF center = totalBr.center();
    double maxDim = qMax(totalBr.width(), totalBr.height()) + 2000.0;

    QList<QLineF> rawLines;
    double xStart = center.x() - maxDim/2.0;
    while (xStart < center.x() + maxDim/2.0) {
        rawLines << QLineF(_rotatePoint(QPointF(xStart, center.y() - maxDim/2.0), center, gridAngle),
                           _rotatePoint(QPointF(xStart, center.y() + maxDim/2.0), center, gridAngle));
        xStart += gridSpacing;
    }

    QList<QLineF> candidateSegments;
    for (const auto& poly : allowedPolygons) {
        _intersectLinesWithPolygon(rawLines, poly, candidateSegments);
    }

    if (candidateSegments.isEmpty()) return;

    // СТАРТ: Берем ближайший к выбранному углу сегмент
    QPointF currentPos = _getAndOrderFirstPoint(candidateSegments, totalBr);

    while (!candidateSegments.isEmpty()) {
        int bestIdx = -1;
        bool reverse = false;
        double minScore = 1e12; // Используем "Score" вместо просто дистанции

        for (int i = 0; i < candidateSegments.size(); ++i) {
            for (bool rev : {false, true}) {
                QPointF testPt = rev ? candidateSegments[i].p2() : candidateSegments[i].p1();
                double dist = QLineF(currentPos, testPt).length();

                // ШТРАФ: Если переход к этой точке пересекает красную зону,
                // мы искусственно увеличиваем дистанцию в 100 раз,
                // чтобы алгоритм сначала выбрал сегменты на своей стороне.
                QLineF jumpPath(currentPos, testPt);
                bool hitsHole = false;
                for(const auto& hole : exclusionPolygons) {
                    for(int j=0; j<hole.count()-1; j++) {
                        if(jumpPath.intersects(QLineF(hole[j], hole[j+1]), nullptr) == QLineF::BoundedIntersection) {
                            hitsHole = true; break;
                        }
                    }
                    if(hitsHole) break;
                }

                double score = hitsHole ? dist * 100.0 : dist;

                if (score < minScore) {
                    minScore = score;
                    bestIdx = i;
                    reverse = rev;
                }
            }
        }

        QLineF line = candidateSegments.takeAt(bestIdx);
        if (reverse) line = QLineF(line.p2(), line.p1());

        // Если все же пришлось прыгать через дырку - строим Bypass
        if (!_transects.isEmpty()) {
            _appendBypassIfNecessary(_transects.last().last().coord, _toGeo(line.p1(), tangentOrigin), tangentOrigin, exclusionPolygons);
        }

        // Добавляем сегмент
        QList<TransectStyleComplexItem::CoordInfo_t> transect;
        TransectStyleComplexItem::CoordInfo_t p1, p2;
        p1.coord = _toGeo(line.p1(), tangentOrigin); p1.coordType = CoordTypeSurveyEntry;
        p2.coord = _toGeo(line.p2(), tangentOrigin); p2.coordType = CoordTypeSurveyExit;
        transect.append(p1); transect.append(p2);

        _transects.append(transect);
        currentPos = line.p2();
    }
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
    _intersectLinesWithPolygon(lineList, polygon, intersectLines);
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
        _intersectLinesWithPolygon(lineList, polygon, intersectLines);
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

    if (triggerDistance == 0) {
        _cameraShots = 0;
    } else {
        if (_cameraTriggerInTurnAroundFact.rawValue().toBool()) {
            _cameraShots = qCeil(_complexDistance / triggerDistance);
        } else {
            _cameraShots = 0;

            if (_loadedMissionItemsParent) {
                // We have to do it the hard way based on the mission items themselves
                if (hoverAndCaptureEnabled()) {
                    // Count the number of camera triggers in the mission items
                    for (const MissionItem* missionItem: _loadedMissionItems) {
                        _cameraShots += missionItem->command() == MAV_CMD_IMAGE_START_CAPTURE ? 1 : 0;
                    }
                } else {
                    bool waitingForTriggerStop = false;
                    QGeoCoordinate distanceStartCoord;
                    QGeoCoordinate distanceEndCoord;
                    for (const MissionItem* missionItem: _loadedMissionItems) {
                        if (missionItem->command() == MAV_CMD_NAV_WAYPOINT) {
                            if (waitingForTriggerStop) {
                                distanceEndCoord = QGeoCoordinate(missionItem->param5(), missionItem->param6());
                            } else {
                                distanceStartCoord = QGeoCoordinate(missionItem->param5(), missionItem->param6());
                            }
                        } else if (missionItem->command() == MAV_CMD_DO_SET_CAM_TRIGG_DIST) {
                            if (missionItem->param1() > 0) {
                                // Trigger start
                                waitingForTriggerStop = true;
                            } else {
                                // Trigger stop
                                waitingForTriggerStop = false;
                                _cameraShots += qCeil(distanceEndCoord.distanceTo(distanceStartCoord) / triggerDistance);
                                distanceStartCoord = QGeoCoordinate();
                                distanceEndCoord = QGeoCoordinate();
                            }
                        }
                    }

                }
            } else {
                // We have transects available, calc from those
                for (const QList<TransectStyleComplexItem::CoordInfo_t>& transect: _transects) {
                    QGeoCoordinate firstCameraCoord, lastCameraCoord;
                    if (_hasTurnaround() && !hoverAndCaptureEnabled()) {
                        firstCameraCoord = transect[1].coord;
                        lastCameraCoord = transect[transect.count() - 2].coord;
                    } else {
                        firstCameraCoord = transect.first().coord;
                        lastCameraCoord = transect.last().coord;
                    }
                    _cameraShots += qCeil(firstCameraCoord.distanceTo(lastCameraCoord) / triggerDistance);
                }
            }
        }
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

    // Находим все точки пересечения линии с ребрами запретного полигона
    for (int i = 0; i < polygon.count() - 1; i++) {
        QPointF p;
        if (line.intersects(QLineF(polygon[i], polygon[i+1]), &p) == QLineF::BoundedIntersection) {
            points << p;
        }
    }

    // Сортируем точки вдоль линии от p1 к p2
    std::sort(points.begin(), points.end(), [&](const QPointF& a, const QPointF& b) {
        return QLineF(line.p1(), a).length() < QLineF(line.p1(), b).length();
    });

    // Проверяем каждый полученный отрезок
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

void AgroComplexItem::_appendBypassIfNecessary(const QGeoCoordinate& start, const QGeoCoordinate& end, const QGeoCoordinate& tangentOrigin, const QList<QPolygonF>& exclusionPolygons)
{
    double y1, x1, y2, x2, d;
    QGCGeo::convertGeoToNed(start, tangentOrigin, y1, x1, d);
    QGCGeo::convertGeoToNed(end, tangentOrigin, y2, x2, d);
    QLineF path(x1, y1, x2, y2);

    for (const auto& poly : exclusionPolygons) {
        QList<QPointF> intersectPts;
        for (int i = 0; i < poly.count() - 1; i++) {
            QPointF p;
            if (path.intersects(QLineF(poly[i], poly[i+1]), &p) == QLineF::BoundedIntersection) intersectPts << p;
        }

        if (intersectPts.count() < 2 && !poly.containsPoint(path.center(), Qt::WindingFill)) continue;

        // Если пересекаем дырку, строим путь по вершинам
        std::sort(intersectPts.begin(), intersectPts.end(), [&](const QPointF& a, const QPointF& b) {
            return QLineF(path.p1(), a).length() < QLineF(path.p1(), b).length();
        });

        QPointF pIn = intersectPts.isEmpty() ? path.p1() : intersectPts.first();
        QPointF pOut = intersectPts.isEmpty() ? path.p2() : intersectPts.last();

        int i1 = 0, i2 = 0;
        double d1 = 1e10, d2 = 1e10;
        for(int i=0; i<poly.count()-1; i++) {
            double dist1 = QLineF(pIn, poly[i]).length(); if(dist1 < d1) { d1 = dist1; i1 = i; }
            double dist2 = QLineF(pOut, poly[i]).length(); if(dist2 < d2) { d2 = dist2; i2 = i; }
        }

        auto build = [&](bool cw) {
            QList<QPointF> r; r << pIn;
            int cur = i1; int safety = 0;
            while(cur != i2 && safety++ < 500) {
                r << poly[cur];
                cur = cw ? (cur + 1) % (poly.count()-1) : (cur - 1 + (poly.count()-1)) % (poly.count()-1);
            }
            r << poly[i2] << pOut; return r;
        };

        auto pCW = build(true); auto pCCW = build(false);
        auto dist = [](const QList<QPointF>& l) { double s=0; for(int i=0; i<l.size()-1; i++) s += QLineF(l[i], l[i+1]).length(); return s; };

        QList<TransectStyleComplexItem::CoordInfo_t> bypass;
        for(const auto& pt : (dist(pCW) < dist(pCCW) ? pCW : pCCW)) {
            TransectStyleComplexItem::CoordInfo_t bp;
            bp.coord = _toGeo(pt, tangentOrigin);
            bp.coordType = CoordTypeInteriorHoverTrigger;
            bypass.append(bp);
        }
        _transects.append(bypass);
        break; // Облетаем первую встреченную дырку
    }
}


void AgroComplexItem::_inflateExclusionZones(const QList<QPolygonF>& exclusionPolygonsNED, double marginMeters, QList<QPolygonF>& inflatedPolygonsNED)
{
    inflatedPolygonsNED.clear();

    if (exclusionPolygonsNED.isEmpty()) {
        return;
    }

    if (marginMeters < 0.1) { // Если отступ слишком мал, просто копируем
        inflatedPolygonsNED = exclusionPolygonsNED;
        return;
    }

    // Clipper работает с целыми числами. 1000.0 = миллиметровая точность для метров.
    const double scale = 1000.0;
    ClipperLib::ClipperOffset offsetter;

    for (const QPolygonF& polygon : exclusionPolygonsNED) {
        if (polygon.count() < 3) continue;

        ClipperLib::Path path;
        for (const QPointF& pt : polygon) {
            path.push_back(ClipperLib::IntPoint(
                static_cast<ClipperLib::cInt>(std::round(pt.x() * scale)),
                static_cast<ClipperLib::cInt>(std::round(pt.y() * scale))
            ));
        }

        // ВАЖНО: ClipperOffset ожидает корректную ориентацию.
        // Если полигон вывернут, отступ может уйти "внутрь" вместо "наружу".
        if (!ClipperLib::Orientation(path)) {
            ClipperLib::ReversePath(path);
        }

        offsetter.AddPath(path, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
    }

    ClipperLib::Paths solution;
    // Execute принимает (результат, дельта * масштаб)
    offsetter.Execute(solution, marginMeters * scale);

    for (const ClipperLib::Path& outPath : solution) {
        QPolygonF inflatedPoly;
        for (const ClipperLib::IntPoint& pt : outPath) {
            inflatedPoly << QPointF(
                static_cast<double>(pt.X) / scale,
                static_cast<double>(pt.Y) / scale
            );
        }

        if (!inflatedPoly.isEmpty()) {
            if (inflatedPoly.first() != inflatedPoly.last()) {
                inflatedPoly << inflatedPoly.first(); // Замыкаем для QGC
            }
            inflatedPolygonsNED.append(inflatedPoly);
        }
    }

    qCDebug(AgroComplexItemLog) << "Inclusion margin applied:" << marginMeters
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

    // Ищем самую близкую точку любого сегмента к выбранному углу
    QPointF closest;
    double minDist = std::numeric_limits<double>::max();
    for (const QLineF& s : segments) {
        double d1 = QLineF(target, s.p1()).length();
        double d2 = QLineF(target, s.p2()).length();
        if (d1 < minDist) { minDist = d1; closest = s.p1(); }
        if (d2 < minDist) { minDist = d2; closest = s.p2(); }
    }
    return target; // Или возвращаем target, жадный алгоритм сам найдет ближайший старт
}
