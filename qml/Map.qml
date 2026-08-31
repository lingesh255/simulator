import QtQuick
import QtLocation
import QtPositioning

// Map Viewer (SRS §3.2.2). Context properties `droneModel`, `markerModel`,
// `pathModel`, `theme`, `tileCacheDir` and `offlineMode` are injected from
// Python (gui/map_viewer.py) before this file is loaded. `theme` (a
// gui.theme.ThemeBridge) exposes the active light/dark palette's colors -
// bind to it instead of hardcoding colors, so a theme toggle updates the
// map's chrome/markers/overlays live. Raw OSM map tiles themselves are not
// themeable and are left as-is.
Item {
    id: root

    signal mapClicked(real lat, real lon)

    Plugin {
        id: osmPlugin
        name: "osm"
        PluginParameter { name: "osm.useragent"; value: "DroneSwarmSimulatorGUI/0.1" }
        PluginParameter { name: "osm.mapping.cache.directory"; value: tileCacheDir }
        PluginParameter { name: "osm.mapping.offline.directory"; value: tileCacheDir }
        // The plugin's online provider repository resolves most map types to
        // Thunderforest, which serves "API Key Required" placeholder tiles
        // without a paid key. Disable it and serve the standard (key-free)
        // OpenStreetMap raster tiles through the plugin's custom map type.
        PluginParameter { name: "osm.mapping.providersrepository.disabled"; value: "true" }
        PluginParameter { name: "osm.mapping.custom.host"; value: "https://tile.openstreetmap.org/" }
        PluginParameter { name: "osm.mapping.custom.mapcopyright"; value: "© OpenStreetMap contributors" }
        PluginParameter { name: "osm.mapping.custom.datacopyright"; value: "© OpenStreetMap contributors" }
    }

    Map {
        id: map
        objectName: "map"
        anchors.fill: parent
        plugin: osmPlugin
        center: QtPositioning.coordinate(20.5937, 78.9629)  // default: India, per the SRS example swarm region
        zoomLevel: 4
        copyrightsVisible: true

        // Pick the CustomMap style, i.e. the plain OSM tiles configured on the
        // plugin above, rather than the key-gated default.
        Component.onCompleted: {
            for (var i = 0; i < map.supportedMapTypes.length; ++i) {
                if (map.supportedMapTypes[i].style === MapType.CustomMap) {
                    map.activeMapType = map.supportedMapTypes[i]
                    break
                }
            }
        }

        // Bounding-box working-area overlay (§3.2.2)
        MapRectangle {
            id: regionRect
            visible: false
            color: theme.accent
            opacity: 0.12
            border.color: theme.accent
            border.width: 2
        }

        // Flight-path vector overlays, one polyline per drone (§3.2.2)
        MapItemView {
            model: pathModel
            delegate: MapPolyline {
                line.width: 2
                line.color: theme.accent
                path: model.points
            }
        }

        // Restricted-area polygon being marked for the mission planner
        MapItemView {
            model: restrictedAreaModel
            delegate: MapPolygon {
                color: theme.warning
                opacity: 0.25
                border.color: theme.warning
                border.width: 2
                path: model.points
            }
        }

        // Live drone positions (§3.2.2) - color reflects fault/flight state
        MapItemView {
            model: droneModel
            delegate: MapQuickItem {
                coordinate: QtPositioning.coordinate(model.lat, model.lon)
                anchorPoint: Qt.point(droneDot.width / 2, droneDot.height / 2)

                // `model.lat/lon` here are already a smoothed, gradually
                // advancing position - see `DroneMarkerModel` in
                // map_models.py, which eases towards each real telemetry fix
                // at a speed controlled by the map's Speed slider. This small
                // fixed Behavior only blends the ~30Hz steps that model
                // produces into a continuous glide instead of a flicker of
                // tiny snaps; it has no bearing on the actual glide speed.
                Behavior on coordinate {
                    CoordinateAnimation {
                        duration: 40
                        easing.type: Easing.Linear
                    }
                }

                sourceItem: Rectangle {
                    id: droneDot
                    width: 20; height: 20; radius: 10
                    // Border stays fixed white regardless of app theme: it is a
                    // contrast device against the (untheme-able) map tiles
                    // underneath, not app chrome.
                    color: model.hasFault ? theme.critical : (model.status === "IN_FLIGHT" ? theme.nominal : theme.accent)
                    border.color: "white"
                    border.width: 2
                    Text {
                        anchors.centerIn: parent
                        text: model.sysid
                        font.pixelSize: 8
                        color: "white"
                    }
                }
            }
        }

        // Click-to-set start (green) / destination (red) / restricted-area
        // centre (amber) markers (§3.2.2)
        MapItemView {
            model: markerModel
            delegate: MapQuickItem {
                coordinate: QtPositioning.coordinate(model.lat, model.lon)
                anchorPoint: Qt.point(pin.width / 2, pin.height)
                sourceItem: Rectangle {
                    id: pin
                    width: 16; height: 16; radius: 8
                    // Border stays fixed black for the same reason as the
                    // drone dot's border above.
                    color: model.role === "start" ? theme.nominal
                           : (model.role === "no_fly_zone" ? theme.warning : theme.critical)
                    border.color: "black"
                    border.width: 1
                }
            }
        }

        MouseArea {
            id: mapMouseArea
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton
            property bool isDragging: false
            property point lastDragPos: Qt.point(0, 0)
            property bool suppressClick: false

            onPressed: (mouse) => {
                if (mouse.button === Qt.LeftButton) {
                    mapMouseArea.isDragging = false
                    mapMouseArea.lastDragPos = Qt.point(mouse.x, mouse.y)
                    mapMouseArea.suppressClick = false
                }
            }

            onPositionChanged: (mouse) => {
                if (mouse.buttons !== Qt.LeftButton) {
                    return
                }

                var dx = mouse.x - mapMouseArea.lastDragPos.x
                var dy = mouse.y - mapMouseArea.lastDragPos.y
                if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
                    mapMouseArea.isDragging = true
                    mapMouseArea.suppressClick = true
                    map.pan(-dx, -dy)
                    mapMouseArea.lastDragPos = Qt.point(mouse.x, mouse.y)
                }
            }

            onReleased: (mouse) => {
                if (mouse.button === Qt.LeftButton) {
                    mapMouseArea.isDragging = false
                }
            }

            onClicked: (mouse) => {
                if (mapMouseArea.suppressClick) {
                    mouse.accepted = true
                    return
                }

                var coord = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                root.mapClicked(coord.latitude, coord.longitude)
            }

            onWheel: (wheel) => {
                if (wheel.pixelDelta.x === 0 && wheel.pixelDelta.y === 0 && Math.abs(wheel.angleDelta.y) < 1) {
                    return
                }

                var coordUnderMouse = map.toCoordinate(Qt.point(wheel.x, wheel.y))
                var delta = wheel.angleDelta.y > 0 ? 1 : -1
                var targetZoom = Math.max(map.minimumZoomLevel, Math.min(map.maximumZoomLevel, map.zoomLevel + delta))
                if (targetZoom === map.zoomLevel) {
                    wheel.accepted = true
                    return
                }

                var currentPixel = map.fromCoordinate(coordUnderMouse, false)
                map.zoomLevel = targetZoom
                var updatedPixel = map.fromCoordinate(coordUnderMouse, false)
                map.pan(updatedPixel.x - currentPixel.x, updatedPixel.y - currentPixel.y)
                wheel.accepted = true
            }
        }
    }

    function zoomIn() {
        map.zoomLevel = Math.min(map.maximumZoomLevel, map.zoomLevel + 1)
    }

    function zoomOut() {
        map.zoomLevel = Math.max(map.minimumZoomLevel, map.zoomLevel - 1)
    }

    function setRegion(minLat, minLon, maxLat, maxLon) {
        regionRect.topLeft = QtPositioning.coordinate(maxLat, minLon)
        regionRect.bottomRight = QtPositioning.coordinate(minLat, maxLon)
        regionRect.visible = true
        map.center = QtPositioning.coordinate((minLat + maxLat) / 2, (minLon + maxLon) / 2)
        map.fitViewportToMapItems()
    }

    function panTo(lat, lon, zoom) {
        map.center = QtPositioning.coordinate(lat, lon)
        if (zoom !== undefined) {
            map.zoomLevel = zoom
        }
    }
}
