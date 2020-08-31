# import gmplot

class MapHTMLgenerator():
    def __init__(self, center_lat, center_lng, zoom, apikey='',
                 map_type='satellite', tilt=0):

        self.center = [center_lat, center_lng]
        self.zoom = zoom
        self.apikey = apikey
        self.map_type = map_type
        self.tilt = tilt
        assert(self.map_type in ['roadmap', 'satellite', 'hybrid', 'terrain'])
        assert(self.tilt in [0, 45])

    def write_map(self,  f):
        f.write('\t\tvar centerlatlng = new google.maps.LatLng(%f, %f);\n' %
                (self.center[0], self.center[1]))
        f.write('\t\tvar myOptions = {\n')
        f.write('\t\t\tzoom: %d,\n' % (self.zoom))
        f.write('\t\t\tcenter: centerlatlng,\n')

        # These are the only changes (override)
        f.write('\t\t\tmapTypeId: \'%s\',\n' % (self.map_type))
        f.write('\t\t\ttilt: %d\n' % (self.tilt))

        f.write('\t\t};\n')
        f.write(
            '\t\tmap = new google.maps.Map(document.getElementById("map_canvas"), myOptions);\n')
        f.write('\n')
    
    def write_mouseListener(self, f):
        f.write("\t\tmap.addListener('mousemove', function (event) {\n")
        f.write("\t\t\ttitleJSON['mouseCoords'] = event.latLng.toJSON();\n")
        f.write("\t\t\ttitleJSON['origin'] = 'mousemove';\n")
        f.write('\t\t\tdocument.title = JSON.stringify(titleJSON);\n')
        f.write('\t\t});\n\n')

    def write_polygonDragListener(self, f):
        f.write("\t\tpolygon.addListener('drag', function() {\n")
        f.write('\t\t\tvar polypath = polygon.getPath().getArray();\n')
        f.write('\t\t\tfor(var i = 0; i < polypath.length; i++){\n')
        f.write('\t\t\t\tmarkers[i].setPosition(polypath[i]);\n')
        f.write('\t\t\t}\n')
        f.write('\t\t})\n\n')

    # create the html file which include one google map and all points and
    # paths
    def draw(self, htmlfile):
        f = open(htmlfile, 'w')
        f.write('<html>\n')
        f.write('<head>\n')
        f.write(
            '<meta name="viewport" content="initial-scale=1.0, user-scalable=no" />\n')
        f.write(
            '<meta http-equiv="content-type" content="text/html; charset=UTF-8"/>\n')
        f.write('<title>Google Maps - pygmaps </title>\n')
        if self.apikey:
            f.write('<script defer type="text/javascript" src="https://maps.googleapis.com/maps/api/js?libraries=visualization&sensor=true_or_false&key=%s"></script>\n' % self.apikey )
        else:
            f.write('<script defer type="text/javascript" src="https://maps.googleapis.com/maps/api/js?libraries=visualization&sensor=true_or_false"></script>\n' )

        f.write('<script type="text/javascript">\n')
        f.write('\tvar markers = [];\n')
        f.write('\tvar markers_custom = [];\n')
        f.write('\tvar markers_uav = {};\n')
        f.write('\tvar polygon;\n')
        f.write('\tvar map;\n\n')
        f.write('\tvar titleJSON = {\n')
        f.write('\t\torigin: "",\n')
        f.write('\t\tmouseCoords: {},\n')
        f.write('\t\tmarkerCoords: [],\n')
        f.write('\t\tcustomMarkerCoords: []\n')
        f.write('\t};\n\n')
        f.write('\tfunction initialize() {\n')
        self.write_map(f)

        # these are the only changes (override)
        self.write_mouseListener(f)
        # self.write_polygonDragListener(f)

        f.write('\t}\n')
        f.write('</script>\n')
        f.write('</head>\n')
        f.write(
            '<body style="margin:0px; padding:0px;" onload="initialize()">\n')
        f.write(
            '\t<div id="map_canvas" style="width: 100%; height: 100%;"></div>\n')
        f.write('</body>\n')
        f.write('</html>\n')
        f.close()

class GoogleMapsJSWrapper:
    def __init__(self, map_holder):
        self.js_code = ""
        self.map_holder = map_holder
    
    def add_markers(self, coords, title="", color="blue", size=1.0):
        self.js_code += "var color = '%s';\n" % color
        self.js_code += "var size = %f;\n" % size
        for i in range(len(coords)):
            if title == "":
                self.js_code += "var title = 'Marker%d';\n" % (i+1)
            else:
                self.js_code += "var title = '%s';\n" % title
            self.js_code += "var latlng = new google.maps.LatLng(%f, %f);\n" % (coords[i][1], coords[i][0])            
            self.js_code += \
                '''
                var marker = new google.maps.Marker({
                    title: title,
                    icon: {
                        path: google.maps.SymbolPath.CIRCLE,
                        scale: 4 * size,
                        fillColor: color,
                        fillOpacity: 1.0,
                        strokeWeight: 0
                    },
                    position: latlng,
                    zIndex: 10
                });
                marker.setMap(map);
                markers.push(marker);
                '''

    def add_polygon(self, coords, color="#6495ED", opacity=0.3):
        self.js_code += "var color = '%s';\n" % color
        self.js_code += "var opacity = %f;\n" % opacity
        self.js_code += "var coords = [\n"
        for coord in coords:
            self.js_code += "\tnew google.maps.LatLng(%f, %f),\n" % (coord[1], coord[0])
        self.js_code += "];\n"

        self.js_code += \
            '''
            polygon = new google.maps.Polygon({
                clickable: false, 	// onemogucuje prikaz koordinata, draggable ne radi bez toga
                geodesic: true,
                fillColor: color,
                fillOpacity: opacity,
                path: coords,
                strokeColor: color,
                strokeOpacity: 1.000000,
                strokeWeight: 1,
                draggable: true
            });
            polygon.setMap(map);

            polygon.addListener('drag', function() {
                var polypath = polygon.getPath().getArray();
                for(var i = 0; i < polypath.length-1; i++){
                    markers[i].setPosition(polypath[i]);
                }
            });

            polygon.addListener('dragend', function() {
                titleJSON['markerCoords'] = [];
                polygon.getPath().forEach(latlng => 
                        titleJSON['markerCoords'].push([latlng.lng(), latlng.lat()]));
                titleJSON['origin'] = 'polygon_drag_end';
                document.title = JSON.stringify(titleJSON);
            });
            '''

    def add_UAV(self, color, size=1.0):
        self.js_code += "var title = 'UAV-" + color + "';\n"
        self.js_code += "var path = window.location.pathname;\n"
        self.js_code += "var dir = path.substring(0, path.lastIndexOf('/'));\n"
        self.js_code += "var icon_path = dir + " + "'/resources/UAV-" + color + ".svg';\n"
        self.js_code += "var size = %f;\n" % size
        self.js_code += \
                '''
                var marker = new google.maps.Marker({
                    title: title,
                    icon: {
                        url: icon_path,
                        anchor: new google.maps.Point(20*size, 20*size),
                        scaledSize: new google.maps.Size(40*size, 40*size)
                    },
                    visible: false,
                    optimized: false,
                    zIndex: 100
                });
                marker.setMap(map);
                '''
        self.js_code += "markers_uav['UAV-" + color + "'] = marker;\n"
        

    def clear_map(self):
        self.js_code += \
            '''
            for(var i = 0; i < markers.length; i++){
                markers[i].setMap(null);
            }
            markers = [];

            if(polygon != null){
                polygon.setMap(null);
            }
            '''
        
    def set_map(self, center_coords, zoom):
        self.js_code += "var centerlatlng = new google.maps.LatLng(%f, %f);\n" % (center_coords[1], center_coords[0])
        self.js_code += "map.setCenter(centerlatlng);\n"
        self.js_code += "map.setZoom(%d);\n" % zoom

    
    def set_UAV_position(self, color, coords):
        self.js_code += "if(!markers_uav['UAV-" + color + "'].getVisible()){\n"
        self.js_code += "\tmarkers_uav['UAV-" + color + "'].setVisible(true);\n"
        self.js_code += "}\n"
        self.js_code += "var latLng = new google.maps.LatLng(%f, %f);\n" % (coords[1], coords[0])
        self.js_code += "markers_uav['UAV-" + color + "'].setPosition(latLng);\n"
    

    def set_polygon_drag(self, drag_flag):
        if drag_flag:
            self.js_code += "polygon.setOptions({clickable: true});\n"
        else:
            self.js_code += "polygon.setOptions({clickable: false});\n"

    
    def custom_markers(self, generate_markers_flag, size=1.0):
        self.js_code += "var size = %f;\n" % size
        if generate_markers_flag:
            self.js_code += \
                '''
                map.addListener('click', function(event){
                    var marker = new google.maps.Marker({
                        title: 'Custom Marker',
                        icon: {
                            path: google.maps.SymbolPath.CIRCLE,
                            scale: 4 * size,
                            fillColor: 'orange',
                            fillOpacity: 1.0,
                            strokeWeight: 0
                        },
                        position: event.latLng,
                        draggable: true,
                        zIndex: 10
                    });
                    marker.setMap(map);
                    markers_custom.push(marker);

                    marker.addListener('dragend', function(event) {
                        titleJSON['origin'] = 'markers_custom';
                        titleJSON['customMarkerCoords'] = [];
                        markers_custom.forEach(marker => 
                            titleJSON['customMarkerCoords'].push([marker.getPosition().lng(), 
                                                                  marker.getPosition().lat()]));
                        document.title = JSON.stringify(titleJSON);
                    });

                    titleJSON['origin'] = 'markers_custom';
                    titleJSON['customMarkerCoords'] = [];
                    markers_custom.forEach(marker => 
                        titleJSON['customMarkerCoords'].push([marker.getPosition().lng(), 
                                                              marker.getPosition().lat()]));
                    document.title = JSON.stringify(titleJSON);
                });
                '''
        else:
            self.js_code += "google.maps.event.clearListeners(map, 'click');"

    def remove_custom_markers(self):
        self.js_code += \
            '''
            for(var i in markers_custom){
                markers_custom[i].setMap(null);
            }
            markers_custom = [];

            titleJSON['origin'] = 'markers_custom';
            titleJSON['customMarkerCoords'] = [];
            markers_custom.forEach(marker => 
                titleJSON['customMarkerCoords'].push([marker.getPosition().lng(), 
                                                      marker.getPosition().lat()]));
            document.title = JSON.stringify(titleJSON);
            '''

    def delete_last_custom_marker(self):
        self.js_code += \
            '''
            if(markers_custom.length > 0)
                markers_custom.pop().setMap(null);
            
            titleJSON['origin'] = 'markers_custom';
            titleJSON['customMarkerCoords'] = [];
            markers_custom.forEach(marker => 
                titleJSON['customMarkerCoords'].push([marker.getPosition().lng(), 
                                                      marker.getPosition().lat()]));
            document.title = JSON.stringify(titleJSON);
            '''

    
    def add_marker_rotation_listener(self, centerOfRotationMarkerID):
        self.js_code += \
            '''
            function find_farthest_marker(coords){
                var maxDist = 0.0;
                var farthestMarkerID;

                // transformation
                var R = 6370000;
                var phi0 = coords.lat()*Math.PI/180;
                var xID = R*coords.lng()*Math.PI/180*Math.cos(phi0);
                var yID = R*coords.lat()*Math.PI/180;

                for(var i in markers){
                    coords = markers[i].getPosition();
                    var x = R*coords.lng()*Math.PI/180*Math.cos(phi0);
                    var y = R*coords.lat()*Math.PI/180;
                    var dist = Math.sqrt( Math.pow(xID-x, 2) + Math.pow(yID-y, 2) );
                    if(dist > maxDist){
                        maxDist = dist;
                        farthestMarkerID = i;
                    }
                }
                return farthestMarkerID;
            }
            '''
        self.js_code += "var centerOfRotationMarkerID = %d;\n" % centerOfRotationMarkerID
        self.js_code += \
            '''
            var markerStartCoords;
            var markerCenterCoords;
            var listenerHandle;
            var markerStaticCoords;

            for(var i in markers){
                markers[i].addListener('mousedown', function(event) {
                    map.setOptions({draggable: false});
                    polygon.setOptions({clickable: false});

                    markerStaticCoords = [];
                    markers.forEach(marker => markerStaticCoords.push(marker.getPosition()));

                    if(centerOfRotationMarkerID == -1) markerCenterCoords = markers[find_farthest_marker(event.latLng)].getPosition();
                    else markerCenterCoords = markers[centerOfRotationMarkerID].getPosition();

                    // transformation to planar coordinates (equirectengular projection - simplified)
                    var phi0 = markerCenterCoords.lat()*Math.PI/180;
                    var R = 6370000; // earth radius
                     
                    markerCenterCoords = [ R*markerCenterCoords.lng()*Math.PI/180*Math.cos(phi0), 
                                           R*markerCenterCoords.lat()*Math.PI/180 ];
                    markerStartCoords = [ R*event.latLng.lng()*Math.PI/180*Math.cos(phi0), 
                                          R*event.latLng.lat()*Math.PI/180 ];

                    listenerHandle = google.maps.event.addListener(map, 'mousemove', function(event) {
                        endCoords = [ R*event.latLng.lng()*Math.PI/180*Math.cos(phi0),
                                      R*event.latLng.lat()*Math.PI/180 ];
                        var v1 = [ markerStartCoords[0] - markerCenterCoords[0], 
                                   markerStartCoords[1] - markerCenterCoords[1] ];
                        var v1_angle = Math.atan2(v1[1], v1[0]);
                        var v2 = [ endCoords[0] - markerCenterCoords[0], 
                                   endCoords[1] - markerCenterCoords[1] ];
                        var v2_angle = Math.atan2(v2[1],v2[0]);
                        var alpha = v2_angle - v1_angle;

                        var coords = [];
                        for(var j in markers){
                            var markerCoords = markerStaticCoords[j];
                            markerCoords = [ R*markerCoords.lng()*Math.PI/180*Math.cos(phi0),
                                             R*markerCoords.lat()*Math.PI/180 ];
                            var rm = [[Math.cos(alpha), -Math.sin(alpha)], 
                                      [Math.sin(alpha),  Math.cos(alpha)]]; // rotation matrix
                            var v = [markerCoords[0] - markerCenterCoords[0], 
                                     markerCoords[1] - markerCenterCoords[1]];
                            var v_rotated = [ rm[0][0]*v[0]+rm[0][1]*v[1], rm[1][0]*v[0]+rm[1][1]*v[1] ];
                            var markerEndCoords = new google.maps.LatLng(
                                (v_rotated[1]+markerCenterCoords[1]) / (R*Math.PI/180), 
                                (v_rotated[0]+markerCenterCoords[0]) / (R*Math.PI/180*Math.cos(phi0))
                            );
                            markers[j].setPosition(markerEndCoords);
                            coords.push(markerEndCoords);
                        }
                        coords.push(coords[0]);
                        polygon.setPath(coords);
                    });
                });

                markers[i].addListener('mouseup', function(event) {
                    markerStaticCoords = [];
                    markers.forEach(marker => markerStaticCoords.push(marker.getPosition()));
                    google.maps.event.removeListener(listenerHandle);
                    map.setOptions({draggable: true});

                    titleJSON['markerCoords'] = [];
                    polygon.getPath().forEach(latlng => 
                            titleJSON['markerCoords'].push([latlng.lng(), latlng.lat()]));
                    titleJSON['origin'] = 'marker_rotate_mouseup';
                    document.title = JSON.stringify(titleJSON);
                });
            }
            '''

    def remove_marker_rotation_listener(self):
        self.js_code += \
            '''
            for(var i in markers){
                google.maps.event.clearListeners(markers[i], 'mousedown');
                google.maps.event.clearListeners(markers[i], 'mouseup');
            }
            google.maps.event.removeListener(listenerHandle);
            '''

    
    def hide_markers(self, hide_flag):
        self.js_code += "for(var i = 0; i<markers.length; i++){\n"
        if hide_flag:
            self.js_code += "\tmarkers[i].setVisible(false);\n"
        else:
            self.js_code += "\tmarkers[i].setVisible(true);\n"
        self.js_code += "}\n"

    def hide_polygon(self, hide_flag):
        self.js_code += "if(polygon != null) {\n"
        if hide_flag:
            self.js_code += "\tpolygon.setVisible(false);\n"
        else:
            self.js_code += "\tpolygon.setVisible(true);\n"
        self.js_code += "}\n"

    def set_UAV_marker_size(self, size):
        self.js_code += "var size = %f;\n" % size
        self.js_code += \
            '''
            for(var uav in markers_uav){
                var marker = markers_uav[uav];
                var icon = marker.getIcon();
                var newIcon = {
                    url: icon.url,
                    anchor: new google.maps.Point(20*size, 20*size),
                    scaledSize: new google.maps.Size(40*size, 40*size)
                }
                marker.setIcon(newIcon);
            }
            '''

    def set_marker_size(self, size):
        self.js_code += "var size = %f;\n" % size
        self.js_code += \
            '''
            for(var i in markers){
                marker = markers[i];
                var icon = marker.getIcon();
                icon.scale = 4 * size;
                marker.setIcon(icon);
            }
            '''
    
    def set_marker_color(self, color):
        self.js_code += "var color = '%s';\n" % color
        self.js_code += \
            '''
            for(var i in markers){
                marker = markers[i];
                var icon = marker.getIcon();
                icon.fillColor = color;
                marker.setIcon(icon);
            }
            '''

    def set_polygon_color(self, color):
        self.js_code += "var color = '%s';\n" % color
        self.js_code += "polygon.setOptions({fillColor: color});\n"

    def set_polygon_opacity(self, opacity):
        self.js_code += "polygon.setOptions({fillOpacity: %f});\n" % opacity
        
    
    def execute(self):
        self.map_holder.run_javascript(self.js_code)
        self.js_code = ""


if __name__ == "__main__":
    # gmap=HTMLgenerator(40.414619186803854, -3.712474573081552, 19)
    gmap=MapHTMLgenerator(0, 0, 1)

    # uav_long, uav_lat = -3.712474573091552, 40.414618186903854
    # gmap.marker( uav_lat, uav_long, color='#0000FF', title='UAV1')

    # API KEY
    gmap.apikey = ""
    gmap.draw("map.html")

