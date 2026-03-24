import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FactControls
import QGroundControl.FlightMap

TransectStyleComplexItemEditor {
    transectAreaDefinitionComplete: missionItem.surveyAreaPolygon.isValid
    transectAreaDefinitionHelp:     qsTr("Use the Polygon Tools to create the polygon which outlines your survey area.")
    transectValuesHeaderName:       qsTr("Transects")
    transectValuesComponent:        _transectValuesComponent
    presetsTransectValuesComponent: _transectValuesComponent

    // The following properties must be available up the hierarchy chain
    //  property real   availableWidth    ///< Width for control
    //  property var    missionItem       ///< Mission Item for editor

    property real   _margin:        ScreenTools.defaultFontPixelWidth / 2
    property var    _missionItem:   missionItem

    property var ctrlVehicle: missionItem.masterController.controllerVehicle
    property bool isArduPilot: ctrlVehicle ? ctrlVehicle.apmFirmware : true
    property bool isPX4:       ctrlVehicle ? ctrlVehicle.px4Firmware : false

    Component {
        id: _transectValuesComponent

        GridLayout {
            Layout.fillWidth:   true
            columnSpacing:      _margin
            rowSpacing:         _margin
            columns:            2

            QGCLabel { text: qsTr("Angle") }
            FactTextField {
                fact:                   missionItem.gridAngle
                Layout.fillWidth:       true
                onUpdated:              angleSlider.value = missionItem.gridAngle.value
                Layout.minimumWidth:    40
                // Disable corner editing if this is an exclusion zone
                enabled:                !missionItem.isExclusionZone.value
            }

            QGCSlider {
                id:                     angleSlider
                from:           0
                to:           359
                stepSize:               1
                Layout.fillWidth:       true
                Layout.columnSpan:      2
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 1.5
                onValueChanged:         missionItem.gridAngle.value = value
                Component.onCompleted:  value = missionItem.gridAngle.value
                live: true
                // Disable the slider if it is an exclusion zone
                enabled:                !missionItem.isExclusionZone.value
            }
 
            Rectangle {
                Layout.columnSpan:      2;
                Layout.fillWidth:       true;
                height:                 1
                color:                  QGroundControl.globalPalette.text;
                opacity:                0.5
                Layout.topMargin:       _margin;
                Layout.bottomMargin:    _margin
            }

            // --- Flight Speed ---
            QGCLabel { text: qsTr("Flight Speed") }
            FactTextField {
                fact:                   missionItem.vehicleSpeed
                Layout.fillWidth:       true
                Layout.minimumWidth:    40
                unitsLabel:             "m/s"
            }

            Rectangle {
                Layout.columnSpan:      2;
                Layout.fillWidth:       true;
                height:                 1
                color:                  QGroundControl.globalPalette.text;
                opacity:                0.5
                Layout.topMargin:       _margin;
                Layout.bottomMargin:    _margin
            }

            // --- Sprayer Settings ---
            QGCLabel {
                text: isPX4 ? qsTr("Sprayer") : qsTr("Sprayer")
                font.bold: true
            }

            FactCheckBox {
                text: qsTr("Enable")
                fact: missionItem.sprayEnabled
            }

            // --- BLOCK FOR PX4 ---
            QGCLabel { 
                text:               qsTr("Actuator ID")
                visible:            isPX4 
            }
            FactTextField { 
                fact:               missionItem.actuatorId
                Layout.fillWidth:   true
                visible:            isPX4 
            }

            // --- BLOCK FOR ARDUPILOT ---
            QGCLabel {
                text: qsTr("Pump Pin ID");
                visible: isArduPilot
            }
            FactTextField {
                fact: missionItem.pumpActuatorId;
                Layout.fillWidth: true; visible: isArduPilot
            }

            QGCLabel {
                text: qsTr("Spinner Pin ID");
                visible: isArduPilot
            }
            FactTextField {
                fact: missionItem.spinnerActuatorId;
                Layout.fillWidth: true; visible: isArduPilot
            }

            QGCLabel {
                text: qsTr("Pump Rate (%)");
                visible: isArduPilot
            }
            FactTextField {
                fact: missionItem.pumpRate;
                Layout.fillWidth: true; visible: isArduPilot
            }

            QGCLabel {
                text: qsTr("Spinner PWM");
                visible: isArduPilot
            }
            FactTextField {
                fact: missionItem.spinnerPWM;
                Layout.fillWidth: true;
                visible: isArduPilot
            }

            QGCLabel {
                text: qsTr("Min Speed (cm/s)");
                visible: isArduPilot
            }
            FactTextField {
                fact: missionItem.minSpeed;
                Layout.fillWidth: true;
                visible: isArduPilot
            }

            QGCLabel {
                text: qsTr("Min Pump (%)");
                visible: isArduPilot
            }
            FactTextField {
                fact: missionItem.minPump;
                Layout.fillWidth: true;
                visible: isArduPilot
            }

            Rectangle {
                Layout.columnSpan:      2;
                Layout.fillWidth:       true;
                height:                 1
                color:                  QGroundControl.globalPalette.text;
                opacity:                0.5
                Layout.topMargin:       _margin;
                Layout.bottomMargin:    _margin
            }

            QGCLabel {
                text:       qsTr("Turnaround dist")
                visible:    !forPresets
            }
            FactTextField {
                Layout.fillWidth:   true
                fact:               missionItem.turnAroundDistance
                visible:            !forPresets
                // Disable the field if it is an exclusion zone
                enabled:            !missionItem.isExclusionZone.value
            }

            QGCOptionsComboBox {
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                visible:            !forPresets

                model: [
                    {
                        text:       qsTr("Is Keep-Out Zone (Exclusion)"),
                        fact:       missionItem.isExclusionZone,
                        enabled:    true,
                        visible:    true
                    },
                    {
                        text:       qsTr("Hover and capture image"),
                        fact:       missionItem.hoverAndCapture,
                        enabled:    missionItem.cameraCalc.distanceMode === QGroundControl.AltitudeFrameRelative || missionItem.cameraCalc.distanceMode === QGroundControl.AltitudeFrameAbsolute,
                        visible:    missionItem.hoverAndCaptureAllowed
                    },
                    {
                        text:       qsTr("Refly at 90 deg offset"),
                        fact:       missionItem.refly90Degrees,
                        enabled:    missionItem.cameraCalc.distanceMode !== QGroundControl.AltitudeFrameCalcAboveTerrain,
                        visible:    true
                    },
                    {
                        text:       qsTr("Images in turnarounds"),
                        fact:       missionItem.cameraTriggerInTurnAround,
                        enabled:    missionItem.hoverAndCaptureAllowed ? !missionItem.hoverAndCapture.rawValue : true,
                        visible:    true
                    },
                    {
                        text:       qsTr("Fly alternate transects"),
                        fact:       missionItem.flyAlternateTransects,
                        enabled:    true,
                        visible:    _vehicle ? (_vehicle.fixedWing || _vehicle.vtol) : false
                    }
                ]
            }
        }
    }

    KMLOrSHPFileDialog {
        id:             kmlOrSHPLoadDialog
        title:          qsTr("Select Polygon File")

        onAcceptedForLoad: (file) => {
            missionItem.surveyAreaPolygon.loadKMLOrSHPFile(file)
            missionItem.resetState = false
            //editorMap.mapFitFunctions.fitMapViewportTomissionItems()
            close()
        }
    }
}