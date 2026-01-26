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
    transectValuesHeaderName:       qsTr("Agro Settings")
    transectValuesComponent:        _transectValuesComponent
    presetsTransectValuesComponent: _transectValuesComponent

    property real   _margin:        ScreenTools.defaultFontPixelWidth / 2
    property var    _missionItem:   missionItem

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
            }

            QGCLabel { text: qsTr("Flight Speed") }
            FactTextField {
                fact:               missionItem.vehicleSpeed
                Layout.fillWidth:   true
                unitsLabel:         "m/s"
            }

            Rectangle {
                Layout.columnSpan: 2; Layout.fillWidth: true; height: 1
                color: QGroundControl.globalPalette.text; opacity: 0.5
                Layout.topMargin: _margin; Layout.bottomMargin: _margin
            }
            QGCLabel { text: qsTr("Sprayer Setup (PX4 Actuators)"); font.bold: true; Layout.columnSpan: 2 }

            QGCCheckBox {
                text: qsTr("Enable Sprayer"); checked: missionItem.sprayEnabled.value
                onClicked: missionItem.sprayEnabled.value = checked
                Layout.columnSpan: 2
            }

            QGCLabel { text: qsTr("Pump Actuator ID (1-6)") }
            FactTextField {
                fact: missionItem.pumpActuatorId
                Layout.fillWidth: true
                enabled: missionItem.sprayEnabled.value
            }

            QGCLabel { text: qsTr("Spinner Actuator ID (1-6)") }
            FactTextField {
                fact: missionItem.spinnerActuatorId
                Layout.fillWidth: true
                enabled: missionItem.sprayEnabled.value
            }

            QGCLabel {
                text: qsTr("Pump Logic Mode"); font.bold: true; Layout.columnSpan: 2; Layout.topMargin: _margin
            }

            QGCCheckBox {
                id: varRateCheck
                text: qsTr("Use Speed Dependent Rate")
                checked: missionItem.pumpRate.value > 0.001
                onClicked: {
                    if (checked) {
                        missionItem.pumpRate.value = 0.2
                    } else {
                        missionItem.pumpRate.value = 0.0
                    }
                }
                Layout.columnSpan: 2
                enabled: missionItem.sprayEnabled.value
            }

            QGCLabel { text: qsTr("Pump ON Value (-1 to 1)") }
            FactTextField {
                fact: missionItem.pumpFixedValue
                Layout.fillWidth: true
                enabled: missionItem.sprayEnabled.value && !varRateCheck.checked
            }

            QGCLabel { text: qsTr("Pump Rate (% per m/s)") }
            FactTextField {
                fact: missionItem.pumpRate
                Layout.fillWidth: true
                enabled: missionItem.sprayEnabled.value && varRateCheck.checked
            }

            QGCLabel { text: qsTr("Min Pump Output (%)") }
            FactTextField {
                fact: missionItem.minPump
                Layout.fillWidth: true
                enabled: missionItem.sprayEnabled.value && varRateCheck.checked
            }

            QGCLabel { text: qsTr("Min Speed (m/s)") }
            FactTextField {
                fact: missionItem.minSpeed
                Layout.fillWidth: true
                enabled: missionItem.sprayEnabled.value && varRateCheck.checked
            }

            QGCLabel { text: qsTr("Spinner Speed") }
            FactTextField {
                fact: missionItem.spinnerPWM
                Layout.fillWidth: true
                enabled: missionItem.sprayEnabled.value
            }

            Rectangle {
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                height:             1
                color:              QGroundControl.globalPalette.text
                opacity:            0.2
            }

            QGCLabel {
                text:       qsTr("Turnaround dist")
                visible:    !forPresets
            }
            FactTextField {
                Layout.fillWidth:   true
                fact:               missionItem.turnAroundDistance
                visible:            !forPresets
            }

            QGCOptionsComboBox {
                Layout.columnSpan:  2
                Layout.fillWidth:   true
                visible:            !forPresets

                model: [
                    {
                        text:       qsTr("Hover and capture image"),
                        fact:       missionItem.hoverAndCapture,
                        enabled:    missionItem.cameraCalc.distanceMode === QGroundControl.AltitudeModeRelative || missionItem.cameraCalc.distanceMode === QGroundControl.AltitudeModeAbsolute,
                        visible:    missionItem.hoverAndCaptureAllowed
                    },
                    {
                        text:       qsTr("Refly at 90 deg offset"),
                        fact:       missionItem.refly90Degrees,
                        enabled:    missionItem.cameraCalc.distanceMode !== QGroundControl.AltitudeModeCalcAboveTerrain,
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
