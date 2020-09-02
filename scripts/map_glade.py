#!/usr/bin/env python
# -*- coding: utf-8 -*- 

## Here we imported both Gtk library and the WebKit2 engine. 
import gi
gi.require_version('Gtk', '3.0')
gi.require_version('WebKit2', '4.0')
from gi.repository import Gtk, GdkPixbuf, Gdk, GLib, WebKit2

from time import sleep
import threading
import json
import os.path

from math import pi, sin, cos, atan2

import roslib; roslib.load_manifest('graupner_serial')
import rospy
import roslaunch
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped, GeoPath

# import local files
from GMLParser import GMLParser
from GoogleMapsWebWrapper import GoogleMapsJSWrapper, MapHTMLgenerator

# All points are in the form of (Longitude, Latitude) !!!!!
class MapApp(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

#       /* GTK */
        self.builder = Gtk.Builder()
        self.builder.add_from_file(roslib.packages.get_pkg_dir('graupner_serial')+'/scripts/map.glade')
        self.window = self.builder.get_object('map_window')
        self.builder.connect_signals(self)

        # GTK elements
        self.parcelEntry = self.builder.get_object('dropdown_parcels_entry')
        self.parcelListStore = self.builder.get_object('dropdown_parcels_liststore')
        self.parcelComboBox = self.builder.get_object('parcel_combobox')
        self.mouseLatLabel = self.builder.get_object('mouse_latitude_label')
        self.mouseLongLabel = self.builder.get_object('mouse_longitude_label')
        self.mouseCoordSystemLabel = self.builder.get_object('coordinate_system_label')
        self.mouseLatLabelFix = self.builder.get_object('mouse_latitude_label_fix')
        self.mouseLongLabelFix = self.builder.get_object('mouse_longitude_label_fix')
        self.mouseCoordSystemLabelFix = self.builder.get_object('coordinate_system_label_fix')
        self.mouseLatLabelCustom = self.builder.get_object('mouse_latitude_label_custom')
        self.mouseLongLabelCustom = self.builder.get_object('mouse_longitude_label_custom')
        self.mouseCoordSystemLabelCustom = self.builder.get_object('coordinate_system_label_custom')
        self.hideMarkersCheckbox = self.builder.get_object('hideMarkers_cb')
        self.hidePolygonCheckbox = self.builder.get_object('hidePolygon_cb')
        self.UAVredImg = self.builder.get_object('uav_red_img')
        self.UAVgreenImg = self.builder.get_object('uav_green_img')
        self.UAVblueImg = self.builder.get_object('uav_blue_img')
        self.UAVredLatLabel = self.builder.get_object('uav_red_lat_label')
        self.UAVredLongLabel = self.builder.get_object('uav_red_long_label')
        self.UAVgreenLatLabel = self.builder.get_object('uav_green_lat_label')
        self.UAVgreenLongLabel = self.builder.get_object('uav_green_long_label')
        self.UAVblueLatLabel = self.builder.get_object('uav_blue_lat_label')
        self.UAVblueLongLabel = self.builder.get_object('uav_blue_long_label')
        self.UAVsizeSpinButton = self.builder.get_object('uav_icon_size_spinbutton')
        self.markerSizeSpinButton = self.builder.get_object('marker_size_spinbutton')
        self.trajectorySizeSpinButton = self.builder.get_object('trajectory_marker_size_spinbutton')
        self.markerColorButton = self.builder.get_object('marker_color_colorbutton')
        self.polygonColorButton = self.builder.get_object('polygon_color_colorbutton')
        self.trajectoryColorButton = self.builder.get_object('trajectory_colorbutton')
        self.polygonOpacityAdjustment = self.builder.get_object('polygon_opacity_adjustment')
        self.centerOfRotationLabel = self.builder.get_object('center_of_rotation_label')
        self.centerOfRotationCombobox = self.builder.get_object('center_of_rotation_combobox')
        self.centerOfRotationEntry = self.builder.get_object('center_of_rotation_entry')
        self.markerListStore = self.builder.get_object('dropdown_markers_liststore')
        self.polygonDragCheckButton = self.builder.get_object('polygon_drag_checkbutton')
        self.polygonRotateCheckButton = self.builder.get_object('polygon_rotate_checkbutton')
        self.addMarkersToggleButton = self.builder.get_object('add_markers_togglebutton')
        self.deleteLastMarkerButton = self.builder.get_object('delete_last_marker_button')
        self.resetNewMarkersButton = self.builder.get_object('reset_new_markers_button')
        self.acceptNewMarkersButton = self.builder.get_object('accept_new_markers_button')

        fileChooser = self.builder.get_object('file_chooser')
        GMLFileFilter = self.builder.get_object('GMLfile_filter')
        fileChooser.add_filter(GMLFileFilter)

        # scale and add images
        height = -1
        width = 30
        imgPath = roslib.packages.get_pkg_dir('graupner_serial')+"/scripts/resources/UAV-red.svg"
        pixbuf = GdkPixbuf.Pixbuf.new_from_file_at_scale(imgPath, width, height, True)
        self.UAVredImg.set_from_pixbuf(pixbuf)
        imgPath = roslib.packages.get_pkg_dir('graupner_serial')+"/scripts/resources/UAV-green.svg"
        pixbuf = GdkPixbuf.Pixbuf.new_from_file_at_scale(imgPath, width, height, True)
        self.UAVgreenImg.set_from_pixbuf(pixbuf)
        imgPath = roslib.packages.get_pkg_dir('graupner_serial')+"/scripts/resources/UAV-blue.svg"
        pixbuf = GdkPixbuf.Pixbuf.new_from_file_at_scale(imgPath, width, height, True)
        self.UAVblueImg.set_from_pixbuf(pixbuf)

        rgba = self.markerColorButton.get_rgba()
        self.markerColorHex = "#{:02x}{:02x}{:02x}".format(int(rgba.red*255), int(rgba.green*255), int(rgba.blue*255))
        rgba = self.polygonColorButton.get_rgba()
        self.polygonColorHex = "#{:02x}{:02x}{:02x}".format(int(rgba.red*255), int(rgba.green*255), int(rgba.blue*255))
        rgba = self.trajectoryColorButton.get_rgba()
        self.trajectoryColorHex = "#{:02x}{:02x}{:02x}".format(int(rgba.red*255), int(rgba.green*255), int(rgba.blue*255))
        self.markerColorButton.connect('notify::color', self.onMarkerColorChange)
        self.polygonColorButton.connect('notify::color', self.onPolygonColorChange)
        self.trajectoryColorButton.connect('notify::color', self.onTrajectoryColorChange)

        styleProvider = Gtk.CssProvider()
        styleProvider.load_from_data('''
            #add_markers_togglebutton { 
                background: #DA3030; 
                color: #C5B9C4;
            }
        ''')
        Gtk.StyleContext.add_provider(
            self.addMarkersToggleButton.get_style_context(),
            styleProvider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

        self.activeParcelTreeIter = None


#       /* Gmaps and WebKit2 */
        self.mapHolder = WebKit2.WebView()
        self.jsWrapper = GoogleMapsJSWrapper(self.mapHolder)

        self.coords = []
        self.adjustedCoords = []
        self.customCoords = []
        self.trajectory_coords = []
        self.phi0 = 0; self.R = 6370000 # earth radius
        self.v_translate = [0, 0]; self.alpha_rotate = 0

        # add UAVs
        self.jsWrapper.add_UAV('red')
        self.jsWrapper.add_UAV('green')
        self.jsWrapper.add_UAV('blue')
        self.jsWrapper.execute()

        # generate map.html
        HTMLgenerator = MapHTMLgenerator(0, 0, 1, apikey='')
        HTMLfile = os.path.dirname(os.path.abspath(__file__)) + "/map.html"
        HTMLgenerator.draw(HTMLfile)

        # print console log
        settings = self.mapHolder.get_settings()
        settings.set_enable_write_console_messages_to_stdout(True)
        settings.set_allow_universal_access_from_file_urls(True)
        self.mapHolder.set_settings(settings)

        # JS -> Python connection
        self.mapHolder.connect('notify::title', self.js2py)
        self.mapHolder.connect('load-changed', self.load_finished)   # connect() je iz GObject-a

        # mapHolder.load_html(open('map.html').read(), None) # ne radi
        # mapHolder.load_uri("http://127.0.0.1:5000")
        self.mapHolder.load_uri("file://" + HTMLfile)

        self.mapBox = self.builder.get_object('map_box')
        self.mapBox.pack_start(self.mapHolder, True, True, 0)

#       /* ROS */
        self.UAV_names = ["UAV", "red", "blue", "yellow"]
        rospy.init_node('MultiUAV_GUI_Map_Node')
        rospy.Subscriber("red/mavros/global_position/global", NavSatFix, self.global_odometry_callback, callback_args="red")
        rospy.Subscriber("blue/mavros/global_position/global", NavSatFix, self.global_odometry_callback, callback_args="green")
        rospy.Subscriber("yellow/mavros/global_position/global", NavSatFix, self.global_odometry_callback, callback_args="blue")
        # rospy.Subscriber("", coordinates_global, self.trajectory_callback)
        # self.building_points_pub = rospy.Publisher("BuildingPoints", GeoPath, queue_size=10)

        self.mapHolder.show_all()

        self.window.connect('delete-event', self.destroy) 
        self.window.show_all() 



    def load_finished(self, webview, event):
        if event == WebKit2.LoadEvent.FINISHED:
            print("Map loaded")

#   /* ROS */
    def global_odometry_callback(self, data, UAV):
		self.set_UAV_position(UAV, [data.longitude, data.latitude])

    def trajectory_callback(self, data):
        self.trajectory_coords = []
        for i in range(len(data.latitude)):
            self.trajectory_coords.append([data.longitude[i], data.latitude[i]])
        
        angles = []
        phi0 = self.trajectory_coords[0][1] * pi/180
        for i in range(len(self.trajectory_coords-1)):
            x = self.R*self.trajectory_coords[i][0]*pi/180*cos(phi0)
            y = self.R*self.trajectory_coords[i][1]*pi/180
            x_next = self.R*self.trajectory_coords[i+1][0]*pi/180*cos(phi0)
            y_next = self.R*self.trajectory_coords[i][1]*pi/180
            alpha = atan2(y_next, x_next) - atan2(y, x)
            angles.append(alpha)
        angles.append(alpha)

        self.jsWrapper.add_markers_trajectory(self.trajectory_coords, angles)
        self.jsWrapper.execute()

    def ROSSendBuildingPoints(self, coords):
        geoPath = GeoPath()
        geoPoseStamped = GeoPoseStamped()
        for coord in coords:
            geoPoseStamped.pose.position.latitude = coord[1]
            geoPoseStamped.pose.position.longitude = coord[0]
            geoPath.append(geoPoseStamped)
        self.building_points_pub.publish(geoPath)
    

    def set_UAV_position(self, color, coords):
        coords = self.apply_adjustment(coords)
        if color == 'red':
            self.jsWrapper.set_UAV_position('red', coords)
            self.UAVredLatLabel.set_text("%14.10f" % coords[1])
            self.UAVredLongLabel.set_text("%14.10f" % coords[0])
        elif color == 'green':
            self.jsWrapper.set_UAV_position('green', coords)
            self.UAVgreenLatLabel.set_text("%14.10f" % coords[1])
            self.UAVgreenLongLabel.set_text("%14.10f" % coords[0])
        elif color == 'blue':
            self.jsWrapper.set_UAV_position('blue', coords)
            self.UAVblueLatLabel.set_text("%14.10f" % coords[1])
            self.UAVblueLongLabel.set_text("%14.10f" % coords[0])
        else:
            print("ERROR: MapApp.set_UAV_position: Referencing non existing UAV")
            return
        self.jsWrapper.execute()


    def js2py(self, webview, event):
        try:
            msg = json.loads(self.mapHolder.get_title())
        except:
            print("Warning: No JSON can be loaded from document title")
            return
        self.mouseLatLabel.set_text("%16.12f" % msg['mouseCoords']['lat'])
        self.mouseLongLabel.set_text("%16.12f" % msg['mouseCoords']['lng'])
        self.mouseLatLabelFix.set_text("%16.12f" % msg['mouseCoords']['lat'])
        self.mouseLongLabelFix.set_text("%16.12f" % msg['mouseCoords']['lng'])
        self.mouseLatLabelCustom.set_text("%16.12f" % msg['mouseCoords']['lat'])
        self.mouseLongLabelCustom.set_text("%16.12f" % msg['mouseCoords']['lng'])

        if msg['origin'] == 'marker_rotate_mouseup':
            self.adjustedCoords = msg['markerCoords']
            self.jsWrapper.set_polygon_drag(self.polygonDragCheckButton.get_active())
            self.jsWrapper.execute()
            self.set_transformation_parameters()
        elif msg['origin'] == 'polygon_drag_end':
            self.adjustedCoords = msg['markerCoords']
            self.set_transformation_parameters()
        elif msg['origin'] == 'markers_custom':
            self.customCoords = msg['customMarkerCoords']


    def set_transformation_parameters(self):
        assert(self.activeParcelTreeIter is not None)
        parcelID = self.parcelListStore[self.activeParcelTreeIter][0]
        coords = self.coords[parcelID]

        center = self.center_coords(coords)

        self.phi0 = center[1]*pi/180

        centerX = self.R*center[0]*pi/180*cos(self.phi0)
        centerY = self.R*center[1]*pi/180

        adjustedCenter = self.center_coords(self.adjustedCoords)

        adjustedCenterX = self.R*adjustedCenter[0]*pi/180*cos(self.phi0)
        adjustedCenterY = self.R*adjustedCenter[1]*pi/180

        self.v_translate = [adjustedCenterX - centerX, adjustedCenterY - centerY]
        
        markerX = self.R*coords[0][0]*pi/180*cos(self.phi0)
        markerY = self.R*coords[0][1]*pi/180
        angle = atan2(markerY - centerY, markerX - centerX)

        adjustedMarkerX = self.R*self.adjustedCoords[0][0]*pi/180*cos(self.phi0)
        adjustedMarkerY = self.R*self.adjustedCoords[0][1]*pi/180
        adjustedAngle = atan2(adjustedMarkerY - adjustedCenterY, 
                              adjustedMarkerX - adjustedCenterX)

        self.alpha_rotate = adjustedAngle - angle

    def apply_adjustment(self, coords):
        if self.activeParcelTreeIter is None:
            return coords
        parcelID = self.parcelListStore[self.activeParcelTreeIter][0]
        markerCoords = self.coords[parcelID]

        center = self.center_coords(markerCoords)
        centerX = self.R*center[0]*pi/180*cos(self.phi0)
        centerY = self.R*center[1]*pi/180

        posPlanarX = self.R*coords[0]*pi/180*cos(self.phi0)
        posPlanarY = self.R*coords[1]*pi/180

        rm = [[cos(self.alpha_rotate), -sin(self.alpha_rotate)],
              [sin(self.alpha_rotate),  cos(self.alpha_rotate)]] # rotation matrix
        v = [posPlanarX - centerX, posPlanarY - centerY]         # vector to rotate

        v_rotated = [rm[0][0]*v[0] + rm[0][1]*v[1], rm[1][0]*v[0] + rm[1][1]*v[1]]
        pos_rotated = [centerX + v_rotated[0], centerY + v_rotated[1]]

        pos_rotated_translated = [pos_rotated[0] + self.v_translate[0], 
                                  pos_rotated[1] + self.v_translate[1]]
        adjustedPosition = [pos_rotated_translated[0] / (self.R*pi/180*cos(self.phi0)),
                            pos_rotated_translated[1] / (self.R*pi/180)]
        return adjustedPosition

    def center_coords(self, coords):
        center = [0, 0]
        for c in coords[:-1]:
            center[0] += c[0]
            center[1] += c[1]
        center[0] /= len(coords[:-1])
        center[1] /= len(coords[:-1])
        return center


    def reset_config(self, coords):
        self.hideMarkersCheckbox.set_active(False)
        self.hidePolygonCheckbox.set_active(False)
        self.polygonDragCheckButton.set_active(False)
        self.polygonRotateCheckButton.set_active(False)

        self.markerListStore.clear()
        self.markerListStore.append([-1, "Automatic"])
        for i in range(len(coords)-1):
            s = "Marker" + str(i+1)
            self.markerListStore.append([i, s])
        treeModel = self.centerOfRotationCombobox.get_model()
        self.centerOfRotationCombobox.set_active_iter(treeModel.get_iter_first())

        self.alpha_rotate = 0
        self.v_translate = [0,0]

    def parcel_changed(self):
        assert(self.activeParcelTreeIter is not None)
        parcelID = self.parcelListStore[self.activeParcelTreeIter][0]
        coords = self.coords[parcelID]        

        self.reset_config(coords)
        
        center = self.center_coords(coords)
        
        self.jsWrapper.clear_map()
        self.jsWrapper.set_map(center, 19)    # zoom = 19
        self.jsWrapper.add_markers(coords[:-1], color=self.markerColorHex, size=self.markerSizeSpinButton.get_value())
        self.jsWrapper.add_polygon(coords, color=self.polygonColorHex, opacity=self.polygonOpacityAdjustment.get_value())
        self.jsWrapper.execute()

        self.ROSSendBuildingPoints(coords[:-1])

    
##########################   GTK Handlers  ############################
    def onDropdownEntryActivate(self, parcelEntry):
        if self.activeParcelTreeIter is not None:
            treeModel = self.parcelComboBox.get_model()
            treeModel[self.activeParcelTreeIter][1] = parcelEntry.get_text()
        

    def onComboChange(self, parcelComboBox):
        treeIter = parcelComboBox.get_active_iter()
        if treeIter is not None:
            treeModel = parcelComboBox.get_model()
            parcelID, parcelName = treeModel[treeIter][:2]
            print("parcel ID: %d, parcel name: %s" % (parcelID, parcelName))
            tmpActiveParcelTreeIter = self.activeParcelTreeIter
            self.activeParcelTreeIter = treeIter
            if tmpActiveParcelTreeIter is not None:
                if parcelID != treeModel[tmpActiveParcelTreeIter][0]:
                    self.parcel_changed()
            else:
                self.parcel_changed()


    def onGMLFileUpload(self, fileChooser):
        GMLfile = fileChooser.get_filename()
        parser = GMLParser()
        parser.parse(GMLfile)
        coords_dict = parser.getCoordinatesDictionary()
        self.coords = coords_dict['coordinates']
        print(self.coords)
        self.parcelListStore.clear()
        for i in range(len(self.coords)):
            s = "Parcel" + str(i+1)
            self.parcelListStore.append([i, s])
        
        self.parcelEntry.set_text("")
        self.reset_config([])
        self.activeParcelTreeIter = None

    
    def onPolygonDragToggle(self, cbButton):
        if self.activeParcelTreeIter is None:
            cbButton.set_active(False)
        else:
            self.jsWrapper.set_polygon_drag(cbButton.get_active())
            self.jsWrapper.execute()

    
    def onPolygonRotateToggle(self, cbButton):
        if self.activeParcelTreeIter is None:
            cbButton.set_active(False)
        elif cbButton.get_active():
            self.centerOfRotationLabel.set_sensitive(True)
            self.centerOfRotationCombobox.set_sensitive(True)
            treeModel = self.centerOfRotationCombobox.get_model()
            treeIter = self.centerOfRotationCombobox.get_active_iter()
            centerOfRotationMarkerID = treeModel[treeIter][0]
            self.jsWrapper.add_marker_rotation_listener(centerOfRotationMarkerID)
            self.jsWrapper.execute()
        else:
            self.centerOfRotationLabel.set_sensitive(False)
            self.centerOfRotationCombobox.set_sensitive(False)
            self.jsWrapper.remove_marker_rotation_listener()
            self.jsWrapper.execute()


    def onMarkerComboChange(self, markerCombobox):
        if markerCombobox.get_active_iter() is None or not self.polygonRotateCheckButton.get_active():
            return
        self.jsWrapper.remove_marker_rotation_listener()
        treeModel = markerCombobox.get_model()
        treeIter = markerCombobox.get_active_iter()
        centerOfRotationMarkerID = treeModel[treeIter][0]
        self.jsWrapper.add_marker_rotation_listener(centerOfRotationMarkerID)
        self.jsWrapper.execute()
        

#   /* Fix Markers */
    def onFitBuildingPlanResetClicked(self, button):
        if self.activeParcelTreeIter is not None:
            self.parcel_changed()


    def onHideMarkersCheckboxToggle(self, cbButton):
        if self.activeParcelTreeIter is None:
            cbButton.set_active(False)
        else:
            self.jsWrapper.hide_markers(cbButton.get_active())
            self.jsWrapper.execute()

    def onHidePolygonCheckboxToggle(self, cbButton):
        if self.activeParcelTreeIter is None:
            cbButton.set_active(False)
        else:
            self.jsWrapper.hide_polygon(cbButton.get_active())
            self.jsWrapper.execute()

    def onLoadConfig(self, button):
        dialog = Gtk.FileChooserDialog(
            title="Load Configuration File", 
            action=Gtk.FileChooserAction.OPEN,
            parent=self.window
        )
        dialog.add_buttons(
            Gtk.STOCK_CANCEL,
            Gtk.ResponseType.CANCEL,
            Gtk.STOCK_OPEN,
            Gtk.ResponseType.OK,
        )
        configFileFilter = self.builder.get_object('config_file_filter')
        dialog.add_filter(configFileFilter)
        dialog.run()
        configFile = dialog.get_filename()
        dialog.destroy()
        try:
            config = json.loads(open(configFile, 'r').read())
        except:
            print("MapError: %s is corrupted, cannot parse as JSON" % configFile)
            return
        self.coords = config['coordinates']
        self.adjustedCoords = config['adjusted_coordinates']
        parcelID = config['parcelID']
        self.coords[parcelID] = self.adjustedCoords

        self.parcelListStore.clear()
        for i in range(len(self.coords)):
            s = "Parcel" + str(i+1)
            self.parcelListStore.append([i, s])
        self.activeParcelTreeIter = None
        self.parcelComboBox.set_active(parcelID)
        
        center = self.center_coords(self.coords[parcelID])
        
        self.jsWrapper.clear_map()
        self.jsWrapper.set_map(center, 19)    # zoom = 19
        self.jsWrapper.add_markers(self.adjustedCoords[:-1], color=self.markerColorHex, size=self.markerSizeSpinButton.get_value())
        self.jsWrapper.add_polygon(self.adjustedCoords, color=self.polygonColorHex, opacity=self.polygonOpacityAdjustment.get_value())
        self.jsWrapper.execute()

        sleep(0.1)
        self.v_translate = config['v_translate']
        self.alpha_rotate = config['alpha_rotate']
        self.phi0 = config['phi0']


    def onSaveConfig(self, button):
        dialog = Gtk.FileChooserDialog(
            title="Save Configuration File", 
            action=Gtk.FileChooserAction.SAVE,
            parent=self.window
        )
        dialog.add_buttons(
            Gtk.STOCK_CANCEL,
            Gtk.ResponseType.CANCEL,
            Gtk.STOCK_SAVE,
            Gtk.ResponseType.ACCEPT,
        )
        dialog.set_do_overwrite_confirmation(True)
        dialog.run()
        configFile = dialog.get_filename()
        dialog.destroy()

        if configFile is None:
            return
        if not configFile.endswith('.config'):
            configFile = configFile + '.config'
        
        parcelID = self.parcelListStore[self.activeParcelTreeIter][0]
        config = {
            'coordinates': self.coords,
            'adjusted_coordinates': self.adjustedCoords,
            'parcelID': parcelID,
            'phi0': self.phi0,
            'v_translate': self.v_translate,
            'alpha_rotate': self.alpha_rotate
        }
        with open(configFile, 'w') as f:
            f.write( json.dumps(config, indent=4) )



    def onAddMarkersToggle(self, togglebutton):
        styleProvider = Gtk.CssProvider()
        if togglebutton.get_active():
            styleProvider.load_from_data('#add_markers_togglebutton { background: #3B54DC; }')
        else:
            styleProvider.load_from_data('#add_markers_togglebutton { background: #DA3030; }')
        Gtk.StyleContext.add_provider(
            togglebutton.get_style_context(),
            styleProvider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

        self.hideMarkersCheckbox.set_active(togglebutton.get_active())
        self.hidePolygonCheckbox.set_active(togglebutton.get_active())
        self.jsWrapper.custom_markers(togglebutton.get_active())
        self.jsWrapper.execute()

    def onResetNewMarkersClick(self, button):
        self.jsWrapper.remove_custom_markers()
        self.jsWrapper.execute()

    def onDeleteLastNewMarkerClick(self, button):
        self.jsWrapper.delete_last_custom_marker()
        self.jsWrapper.execute()

    def onAcceptNewMarkersClick(self, button):
        if(len(self.customCoords) < 3):
            infodialog = Gtk.MessageDialog(
                transient_for=self.window,
                flags=0,
                message_type=Gtk.MessageType.INFO,
                buttons=Gtk.ButtonsType.OK,
                text="Not enough points selected"
            )
            infodialog.format_secondary_text("At least 3 points are needed to construct Building Ground Plan")
            infodialog.run()
            infodialog.destroy()
            return

        dialog = Gtk.MessageDialog(
            transient_for=self.window,
            flags=0,
            message_type=Gtk.MessageType.INFO,
            buttons=Gtk.ButtonsType.OK_CANCEL,
            text='Your selection will be saved as building'
        )
        dialog.format_secondary_text("Selection is saved temporarily, if you want to save it permenantly save it as configuration file (Save Config)")
        response = dialog.run()
        dialog.destroy()

        if response == Gtk.ResponseType.OK:
            newParcelID = len(self.coords)
            newParcelName = "Parcel" + str(newParcelID+1)
            self.parcelListStore.append([newParcelID, newParcelName])
            self.customCoords.append(self.customCoords[0])
            self.coords.append(self.customCoords)
            self.activeParcelTreeIter = None
            self.parcelComboBox.set_active(newParcelID)

            infodialog = Gtk.MessageDialog(
                transient_for=self.window,
                flags=0,
                message_type=Gtk.MessageType.INFO,
                buttons=Gtk.ButtonsType.OK,
                text="New Building Ground Plan is saved as " + newParcelName
            )
            infodialog.run()
            infodialog.destroy()

            self.onResetNewMarkersClick(None)
            self.addMarkersToggleButton.set_active(False)


#   /* Customization */
    def onUAVMarkerSizeChange(self, spinButton):
        self.jsWrapper.set_UAV_marker_size(spinButton.get_value())
        self.jsWrapper.execute()
    
    def onMarkerSizeChange(self, spinButton):
        self.jsWrapper.set_marker_size(spinButton.get_value())
        self.jsWrapper.execute()

    def onMarkerColorChange(self, colorButton, *args):
        rgba = colorButton.get_rgba()
        self.markerColorHex = "#{:02x}{:02x}{:02x}".format(int(rgba.red*255), int(rgba.green*255), int(rgba.blue*255))
        if self.activeParcelTreeIter is not None:
            self.jsWrapper.set_marker_color(self.markerColorHex)
            self.jsWrapper.execute()
        
    def onPolygonColorChange(self, colorButton, *args):
        rgba = colorButton.get_rgba()
        self.polygonColorHex = "#{:02x}{:02x}{:02x}".format(int(rgba.red*255), int(rgba.green*255), int(rgba.blue*255))
        if self.activeParcelTreeIter is not None:
            self.jsWrapper.set_polygon_color(self.polygonColorHex)
            self.jsWrapper.execute()

    def onPolygonOpacityChange(self, ranger):
        if self.activeParcelTreeIter is not None:
            self.jsWrapper.set_polygon_opacity(ranger.get_value())
            self.jsWrapper.execute()

    def onTrajectoryMarkerSizeChange(self, spinButton):
        self.jsWrapper.set_trajectory_marker_size(spinButton.get_value())
        self.jsWrapper.execute()

    def onTrajectoryColorChange(self, colorButton, *args):
        rgba = colorButton.get_rgba()
        self.markerColorHex = "#{:02x}{:02x}{:02x}".format(int(rgba.red*255), int(rgba.green*255), int(rgba.blue*255))
        if self.trajectory_coords:
            self.jsWrapper.set_trajectory_marker_color(self.markerColorHex)
            self.jsWrapper.execute()

    def onResetTemplate(self, button):
        self.UAVsizeSpinButton.set_value(1.0)
        self.markerSizeSpinButton.set_value(1.0)
        markerColor = Gdk.RGBA()
        markerColor.parse("#0000FF")
        self.markerColorButton.set_rgba(markerColor)
        polygonColor = Gdk.RGBA()
        polygonColor.parse("#6495ED")
        self.polygonColorButton.set_rgba(polygonColor)
        self.polygonOpacityAdjustment.set_value(0.3)

    def onLoadTemplate(self, button):
        dialog = Gtk.FileChooserDialog(
            title="Load Template File", 
            action=Gtk.FileChooserAction.OPEN,
            parent=self.window
        )
        dialog.add_buttons(
            Gtk.STOCK_CANCEL,
            Gtk.ResponseType.CANCEL,
            Gtk.STOCK_OPEN,
            Gtk.ResponseType.OK,
        )
        templateFileFilter = self.builder.get_object('template_file_filter')
        dialog.add_filter(templateFileFilter)
        dialog.run()
        templateFile = dialog.get_filename()
        dialog.destroy()
        try:
            template = json.loads(open(templateFile, 'r').read())
        except:
            print("MapError: %s is corrupted, cannot parse as JSON" % templateFile)
            return
        rgba = Gdk.RGBA()
        self.markerSizeSpinButton.set_value(template['marker_size'])
        rgba.parse(template['marker_color'])
        self.markerColorButton.set_rgba(rgba)
        self.UAVsizeSpinButton.set_value(template['uav_size'])
        rgba.parse(template['polygon_color'])
        self.polygonColorButton.set_rgba(rgba)
        self.polygonOpacityAdjustment.set_value(template['polygon_opacity'])

    def onSaveTemplate(self, button):
        dialog = Gtk.FileChooserDialog(
            title="Save Template File", 
            action=Gtk.FileChooserAction.SAVE,
            parent=self.window
        )
        dialog.add_buttons(
            Gtk.STOCK_CANCEL,
            Gtk.ResponseType.CANCEL,
            Gtk.STOCK_SAVE,
            Gtk.ResponseType.ACCEPT,
        )
        dialog.set_do_overwrite_confirmation(True)
        dialog.run()
        templateFile = dialog.get_filename()
        dialog.destroy()

        if templateFile is None:
            return
        if not templateFile.endswith('.temp'):
            templateFile = templateFile + '.temp'

        template = {
            'marker_size': self.markerSizeSpinButton.get_value(),
            'marker_color': self.markerColorHex,
            'uav_size': self.UAVsizeSpinButton.get_value(),
            'polygon_color': self.polygonColorHex,
            'polygon_opacity': self.polygonOpacityAdjustment.get_value()
        }
        with open(templateFile, 'w') as f:
            f.write( json.dumps(template, indent=4) )


    def destroy(self, *args):
        Gtk.main_quit(*args)

    def run(self):
        Gtk.main()


if __name__ == "__main__":
    app = MapApp()
    # app.start()
    Gtk.main()
    # while(app.is_alive()):
        # sleep(1)

# builder.connect_signals(Handler())  --> handlers = {"onDestroy": Gtk.main_quit \n ...}
## To disallow editing the webpage. 
# mapHolder.set_editable(True)