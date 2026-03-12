/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Controls
import QtLocation
import QtPositioning

import QGroundControl


import QGroundControl.Controls
import QGroundControl.FlightMap

/// Survey Complex Mission Item visuals
TransectStyleMapVisuals {
    polygonInteractive: true

    // redefining color
    polygonColor: {
        // Checking whether the object has the isExclusionZone property and whether it is enabled
        if (_missionItem && _missionItem.isExclusionZone && _missionItem.isExclusionZone.value === true) {
            return "red"   // Or "#FF4444" for a softer red
        }
        return "#228B22"   // ForestGreen (или просто "green")
    }
}
