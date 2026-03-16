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

            QGCLabel {
                text:                   qsTr("Application Rate Control");
                font.bold:              true;
                Layout.columnSpan:      2
                Layout.fillWidth:       true
                wrapMode:               Text.WordWrap 
            }
            FactCheckBox {
                id:                     calcModeCheck
                text:                   qsTr("Auto-Calculate Speed")
                fact:                   missionItem.calcModeEnabled
                Layout.columnSpan:      2
                Layout.fillWidth:       true
            }

            QGCLabel {
                text:                   qsTr("Target Rate (L/ha)")
                visible:                calcModeCheck.checked
            }
            FactTextField {
                fact:                   missionItem.targetRate
                Layout.fillWidth:       true
                Layout.minimumWidth:    40
                visible:                calcModeCheck.checked
            }

            QGCSlider {
                from:                   1
                to:                     100
                stepSize:               1
                value:                  missionItem.targetRate.value
                onValueChanged:         missionItem.targetRate.value = value
                Layout.columnSpan:      2
                Layout.fillWidth:       true
                visible:                calcModeCheck.checked
                live:                   true
            }

            QGCLabel {
                text:                   qsTr("Swath Width (m)")
                visible:                calcModeCheck.checked
            }
            FactTextField {
                fact:                   missionItem.swathWidth
                Layout.fillWidth:       true
                Layout.minimumWidth:    40
                visible:                calcModeCheck.checked
            }

            QGCLabel {
                text:                   qsTr("Max Flow (L/min)")
                visible:                calcModeCheck.checked
            }
            FactTextField {
                fact:                   missionItem.flowRateMax
                Layout.fillWidth:       true
                Layout.minimumWidth:    40
                visible:                calcModeCheck.checked
            }

            QGCLabel {
                text:                   qsTr("Flight Speed")
            }
            FactTextField {
                fact:                   missionItem.vehicleSpeed
                Layout.fillWidth:       true
                Layout.minimumWidth:    40
                unitsLabel:             "m/s"
                enabled:                !calcModeCheck.checked
                readOnly:               calcModeCheck.checked
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

            QGCCheckBox {
                id:                     showAdvanced
                text:                   qsTr("Show Advanced Setup")
                checked:                false
                Layout.columnSpan:      2
                Layout.fillWidth:       true
            }

            ColumnLayout {
                Layout.columnSpan:      2
                Layout.fillWidth:       true
                visible:                showAdvanced.checked

                FactCheckBox {
                    text:               qsTr("Enable Sprayer");
                    fact:               missionItem.sprayEnabled
                }

                GridLayout {
                    columns:            2
                    Layout.fillWidth:   true

                    QGCLabel {
                        text:           qsTr("Pump ID")
                    }
                    FactTextField {
                        fact:           missionItem.pumpActuatorId;
                        Layout.fillWidth: true
                        Layout.minimumWidth: 40
                    }

                    QGCLabel {
                        text:           qsTr("Spinner ID")
                    }
                    FactTextField {
                        fact:           missionItem.spinnerActuatorId;
                        Layout.fillWidth: true
                        Layout.minimumWidth: 40
                    }

                    QGCLabel {
                        text:           "Manual Logic:";
                        font.bold:      true;
                        Layout.fillWidth: true
                        Layout.columnSpan: 2;
                        visible:        !calcModeCheck.checked
                    }

                    QGCLabel {
                        text:           qsTr("Pump Fixed Val");
                        visible:        !calcModeCheck.checked
                    }
                    FactTextField {
                        fact:           missionItem.pumpFixedValue;
                        Layout.fillWidth: true;
                        Layout.minimumWidth: 40
                        visible:        !calcModeCheck.checked
                    }

                    QGCLabel {
                        text:           qsTr("Pump Rate %");
                        visible:        !calcModeCheck.checked
                    }
                    FactTextField {
                        fact:           missionItem.pumpRate;
                        Layout.fillWidth: true;
                        Layout.minimumWidth: 40
                        visible:        !calcModeCheck.checked
                    }

                    QGCLabel {
                        text:           qsTr("Spinner Speed");
                        visible:        !calcModeCheck.checked
                    }
                    FactTextField {
                        fact:           missionItem.spinnerPWM;
                        Layout.fillWidth: true;
                        Layout.minimumWidth: 40
                        visible:        !calcModeCheck.checked
                    }
                }
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