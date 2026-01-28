/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "TransectStyleComplexItem.h"
#include "SettingsFact.h"

#include <QtCore/QLoggingCategory>

Q_DECLARE_LOGGING_CATEGORY(SurveyComplexItemLog)

class PlanMasterController;
class MissionItem;

class AgroComplexItem : public TransectStyleComplexItem
{
    Q_OBJECT

public:
    /// @param flyView true: Created for use in the Fly View, false: Created for use in the Plan View
    /// @param kmlOrShpFile Polygon comes from this file, empty for default polygon
    AgroComplexItem(PlanMasterController* masterController, bool flyView, const QString& kmlOrShpFile);

    Q_PROPERTY(Fact*            gridAngle              READ gridAngle              CONSTANT)
    Q_PROPERTY(Fact*            flyAlternateTransects  READ flyAlternateTransects  CONSTANT)
    Q_PROPERTY(Fact*            splitConcavePolygons   READ splitConcavePolygons   CONSTANT)
    Q_PROPERTY(QGeoCoordinate   centerCoordinate       READ centerCoordinate       WRITE setCenterCoordinate)
    Q_PROPERTY(Fact*            vehicleSpeed           READ vehicleSpeed           CONSTANT)
    Q_PROPERTY(Fact*            sprayEnabled           READ sprayEnabled           CONSTANT)
    Q_PROPERTY(Fact*            pumpActuatorId         READ pumpActuatorId         CONSTANT)
    Q_PROPERTY(Fact*            spinnerActuatorId      READ spinnerActuatorId      CONSTANT)
    Q_PROPERTY(Fact*            pumpFixedValue         READ pumpFixedValue         CONSTANT)
    Q_PROPERTY(Fact*            pumpRate               READ pumpRate               CONSTANT)
    Q_PROPERTY(Fact*            spinnerPWM             READ spinnerPWM             CONSTANT)
    Q_PROPERTY(Fact*            minPump                READ minPump                CONSTANT)
    Q_PROPERTY(Fact*            minSpeed               READ minSpeed               CONSTANT)
    Q_PROPERTY(Fact*            calcModeEnabled        READ calcModeEnabled        CONSTANT)
    Q_PROPERTY(Fact*            targetRate             READ targetRate             CONSTANT)
    Q_PROPERTY(Fact*            flowRateMax            READ flowRateMax            CONSTANT)
    Q_PROPERTY(Fact*            swathWidth             READ swathWidth             CONSTANT)

    Fact* gridAngle             (void) { return &_gridAngleFact; }
    Fact* flyAlternateTransects (void) { return &_flyAlternateTransectsFact; }
    Fact* splitConcavePolygons  (void) { return &_splitConcavePolygonsFact; }
    Fact* vehicleSpeed          (void) { return &_vehicleSpeedFact; }
    Fact* sprayEnabled          (void) { return &_sprayEnabledFact; }
    Fact* pumpActuatorId        (void) { return &_pumpActuatorIdFact; }
    Fact* spinnerActuatorId     (void) { return &_spinnerActuatorIdFact; }
    Fact* pumpFixedValue        (void) { return &_pumpFixedValueFact; }
    Fact* pumpRate              (void) { return &_pumpRateFact; }
    Fact* spinnerPWM            (void) { return &_spinnerPWMFact; }
    Fact* minPump               (void) { return &_minPumpFact; }
    Fact* minSpeed              (void) { return &_minSpeedFact; }
    Fact* calcModeEnabled       (void) { return &_calcModeEnabledFact; }
    Fact* targetRate            (void) { return &_targetRateFact; }
    Fact* flowRateMax           (void) { return &_flowRateMaxFact; }
    Fact* swathWidth            (void) { return &_swathWidthFact; }

    Q_INVOKABLE void rotateEntryPoint(void);

    // Overrides from ComplexMissionItem
    QString         patternName         (void) const final { return name; }
    bool            load                (const QJsonObject& complexObject, int sequenceNumber, QString& errorString) final;
    QString         mapVisualQML        (void) const final { return QStringLiteral("SurveyMapVisual.qml"); }
    QString         presetsSettingsGroup(void) { return settingsGroup; }
    void            savePreset          (const QString& name);
    void            loadPreset          (const QString& name);
    bool            isSurveyItem        (void) const final { return true; }
    QGeoCoordinate  centerCoordinate    (void) const { return _surveyAreaPolygon.center(); }
    void            setCenterCoordinate (const QGeoCoordinate& coordinate) { _surveyAreaPolygon.setCenter(coordinate); }

    // Overrides from TransectStyleComplexItem
    void    save                (QJsonArray&  planItems) final;
    bool    specifiesCoordinate (void) const final { return true; }
    double  timeBetweenShots    (void) final;
    void    appendMissionItems  (QList<MissionItem*>& items, QObject* missionItemParent);
    void    _appendVisualAction(QList<MissionItem*>& items, QObject* missionItemParent, int& seqNum, MAV_FRAME frame, const QGeoCoordinate& coord);

    // Overrides from VisualMissionionItem
    QString             commandDescription  (void) const final { return tr("Survey"); }
    QString             commandName         (void) const final { return tr("Survey"); }
    QString             abbreviation        (void) const final { return tr("S"); }
    ReadyForSaveState   readyForSaveState    (void) const final;
    double              additionalTimeDelay (void) const final;

    // Must match json spec for GridEntryLocation
    enum EntryLocation {
        EntryLocationFirst,
        EntryLocationTopLeft = EntryLocationFirst,
        EntryLocationTopRight,
        EntryLocationBottomLeft,
        EntryLocationBottomRight,
        EntryLocationLast = EntryLocationBottomRight
    };

    static const QString name;

    static constexpr const char* jsonComplexItemTypeValue =   "agro";
    static constexpr const char* jsonV3ComplexItemTypeValue = "agro";

    static constexpr const char* settingsGroup =              "Agro";
    static constexpr const char* gridAngleName =              "GridAngle";
    static constexpr const char* gridEntryLocationName =      "GridEntryLocation";
    static constexpr const char* flyAlternateTransectsName =  "FlyAlternateTransects";
    static constexpr const char* splitConcavePolygonsName =   "SplitConcavePolygons";
    static constexpr const char* vehicleSpeedName =           "VehicleSpeed";
    static constexpr const char* sprayEnabledName =           "SprayEnabled";
    static constexpr const char* pumpActuatorIdName =         "PumpActuatorId";
    static constexpr const char* spinnerActuatorIdName =      "SpinnerActuatorId";
    static constexpr const char* pumpFixedValueName =         "PumpFixedValue";
    static constexpr const char* pumpRateName =               "PumpRate";
    static constexpr const char* spinnerPWMName =             "SpinnerPWM";
    static constexpr const char* minPumpName =                "MinPump";
    static constexpr const char* minSpeedName =               "MinSpeed";
    static constexpr const char* calcModeEnabledName =        "CalcModeEnabled";
    static constexpr const char* targetRateName =             "TargetRate";
    static constexpr const char* flowRateMaxName =            "FlowRateMax";
    static constexpr const char* swathWidthName =             "SwathWidth";

signals:
    void refly90DegreesChanged(bool refly90Degrees);

private slots:
    void _updateWizardMode              (void);

    // Overrides from TransectStyleComplexItem
    void _rebuildTransectsPhase1        (void) final;
    void _recalcCameraShots             (void) final;

    void _recalcSpeedFromRate           (void);

private:
    enum CameraTriggerCode {
        CameraTriggerNone,
        CameraTriggerOn,
        CameraTriggerOff,
        CameraTriggerHoverAndCapture
    };

    QPointF _rotatePoint(const QPointF& point, const QPointF& origin, double angle);
    void _intersectLinesWithRect(const QList<QLineF>& lineList, const QRectF& boundRect, QList<QLineF>& resultLines);
    void _intersectLinesWithPolygon(const QList<QLineF>& lineList, const QPolygonF& polygon, QList<QLineF>& resultLines);
    void _adjustLineDirection(const QList<QLineF>& lineList, QList<QLineF>& resultLines);
    bool _nextTransectCoord(const QList<QGeoCoordinate>& transectPoints, int pointIndex, QGeoCoordinate& coord);
    bool _appendMissionItemsWorker(QList<MissionItem*>& items, QObject* missionItemParent, int& seqNum, bool hasRefly, bool buildRefly);
    void _optimizeTransectsForShortestDistance(const QGeoCoordinate& distanceCoord, QList<QList<QGeoCoordinate>>& transects);
    qreal _ccw(QPointF pt1, QPointF pt2, QPointF pt3);
    qreal _dp(QPointF pt1, QPointF pt2);
    void _swapPoints(QList<QPointF>& points, int index1, int index2);
    void _reverseTransectOrder(QList<QList<QGeoCoordinate>>& transects);
    void _reverseInternalTransectPoints(QList<QList<QGeoCoordinate>>& transects);
    void _adjustTransectsToEntryPointLocation(QList<QList<QGeoCoordinate>>& transects);
    bool _gridAngleIsNorthSouthTransects();
    double _clampGridAngle90(double gridAngle);
    bool _imagesEverywhere(void) const;
    bool _triggerCamera(void) const;
    bool _hasTurnaround(void) const;
    double _turnaroundDistance(void) const;
    bool _hoverAndCaptureEnabled(void) const;
    bool _loadV3(const QJsonObject& complexObject, int sequenceNumber, QString& errorString);
    bool _loadV4V5(const QJsonObject& complexObject, int sequenceNumber, QString& errorString, int version, bool forPresets);
    void _saveCommon(QJsonObject& complexObject);
    void _rebuildTransectsPhase1Worker(bool refly);
    void _rebuildTransectsPhase1WorkerSinglePolygon(bool refly);
    /// Adds to the _transects array from one polygon
    void _rebuildTransectsFromPolygon(bool refly, const QPolygonF& polygon, const QGeoCoordinate& tangentOrigin, const QPointF* const transitionPoint);

    void _appendSprayerCommand(QList<MissionItem*>& items, QObject* missionItemParent, int& seqNum, bool active);

#if 0
    // Splitting polygons is not supported since this code would get stuck in a infinite loop
    // Code is left here in case someone wants to try to resurrect it

    void _rebuildTransectsPhase1WorkerSplitPolygons(bool refly);

    // Decompose polygon into list of convex sub polygons
    void _PolygonDecomposeConvex(const QPolygonF& polygon, QList<QPolygonF>& decomposedPolygons);
    // return true if vertex a can see vertex b
    bool _VertexCanSeeOther(const QPolygonF& polygon, const QPointF* vertexA, const QPointF* vertexB);
    bool _VertexIsReflex(const QPolygonF& polygon, QList<QPointF>::const_iterator& vertexIter);
#endif

    QMap<QString, FactMetaData*> _metaDataMap;

    SettingsFact    _gridAngleFact;
    SettingsFact    _flyAlternateTransectsFact;
    SettingsFact    _splitConcavePolygonsFact;
    SettingsFact    _vehicleSpeedFact;
    SettingsFact    _sprayEnabledFact;
    SettingsFact    _pumpActuatorIdFact;
    SettingsFact    _spinnerActuatorIdFact;
    SettingsFact    _pumpFixedValueFact;
    SettingsFact    _pumpRateFact;
    SettingsFact    _spinnerPWMFact;
    SettingsFact    _minPumpFact;
    SettingsFact    _minSpeedFact;
    SettingsFact    _calcModeEnabledFact;
    SettingsFact    _targetRateFact;
    SettingsFact    _flowRateMaxFact;
    SettingsFact    _swathWidthFact;
    int             _entryPoint;

    static constexpr const char* _jsonGridAngleKey =          "angle";
    static constexpr const char* _jsonEntryPointKey =         "entryLocation";

    static constexpr const char* _jsonV3GridObjectKey =                   "grid";
    static constexpr const char* _jsonV3GridAltitudeKey =                 "altitude";
    static constexpr const char* _jsonV3GridAltitudeRelativeKey =         "relativeAltitude";
    static constexpr const char* _jsonV3GridAngleKey =                    "angle";
    static constexpr const char* _jsonV3GridSpacingKey =                  "spacing";
    static constexpr const char* _jsonV3EntryPointKey =                   "entryLocation";
    static constexpr const char* _jsonV3TurnaroundDistKey =               "turnAroundDistance";
    static constexpr const char* _jsonV3CameraTriggerDistanceKey =        "cameraTriggerDistance";
    static constexpr const char* _jsonV3CameraTriggerInTurnaroundKey =    "cameraTriggerInTurnaround";
    static constexpr const char* _jsonV3HoverAndCaptureKey =              "hoverAndCapture";
    static constexpr const char* _jsonV3GroundResolutionKey =             "groundResolution";
    static constexpr const char* _jsonV3FrontalOverlapKey =               "imageFrontalOverlap";
    static constexpr const char* _jsonV3SideOverlapKey =                  "imageSideOverlap";
    static constexpr const char* _jsonV3CameraSensorWidthKey =            "sensorWidth";
    static constexpr const char* _jsonV3CameraSensorHeightKey =           "sensorHeight";
    static constexpr const char* _jsonV3CameraResolutionWidthKey =        "resolutionWidth";
    static constexpr const char* _jsonV3CameraResolutionHeightKey =       "resolutionHeight";
    static constexpr const char* _jsonV3CameraFocalLengthKey =            "focalLength";
    static constexpr const char* _jsonV3CameraMinTriggerIntervalKey =     "minTriggerInterval";
    static constexpr const char* _jsonV3CameraObjectKey =                 "camera";
    static constexpr const char* _jsonV3CameraNameKey =                   "name";
    static constexpr const char* _jsonV3ManualGridKey =                   "manualGrid";
    static constexpr const char* _jsonV3CameraOrientationLandscapeKey =   "orientationLandscape";
    static constexpr const char* _jsonV3FixedValueIsAltitudeKey =         "fixedValueIsAltitude";
    static constexpr const char* _jsonV3Refly90DegreesKey =               "refly90Degrees";
    static constexpr const char* _jsonFlyAlternateTransectsKey =          "flyAlternateTransects";
    static constexpr const char* _jsonSplitConcavePolygonsKey =           "splitConcavePolygons";
    static constexpr const char* _jsonVehicleSpeedKey =                   "vehicleSpeed";
    static constexpr const char* _jsonSprayEnabledKey =                   "sprayEnabled";
    static constexpr const char* _jsonPumpActuatorIdKey =                 "pumpActuatorId";
    static constexpr const char* _jsonSpinnerActuatorIdKey =              "spinnerActuatorId";
    static constexpr const char* _jsonPumpFixedValueKey =                 "pumpFixedValue";
    static constexpr const char* _jsonPumpRateKey =                       "pumpRate";
    static constexpr const char* _jsonSpinnerPWMKey =                     "spinnerPWM";
    static constexpr const char* _jsonMinPumpKey =                        "minPump";
    static constexpr const char* _jsonMinSpeedKey =                       "minSpeed";
    static constexpr const char* _jsonCalcModeEnabledKey =                "calcModeEnabled";
    static constexpr const char* _jsonTargetRateKey =                     "targetRate";
    static constexpr const char* _jsonFlowRateMaxKey =                    "flowRateMax";
    static constexpr const char* _jsonSwathWidthKey =                     "swathWidth";
};
