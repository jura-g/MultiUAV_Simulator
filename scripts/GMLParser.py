from osgeo import ogr
from osgeo.osr import SpatialReference
import json

class GMLParser:
    def __init__(self):
        self.points_dict = {}

    '''
    All functions that start with "getPoints" return list of lists of coordinates
    MultiPolygon and MultiLineString may have few parcels within them (few lists of coordinates)
    while MultiPoint, Polygon and LineString, describe only one parcel (one list of coordinates)

    !!!!! Points are in the form of (Longitude, Latitude) !!!!!
    '''
    def getPointsFromMultipolygon(self, geometry):
        polygonCount = geometry.GetGeometryCount()
        points = []
        for i in range(polygonCount):
            polygon = geometry.GetGeometryRef(i)
            points.append(self.getPointsFromPolygon(polygon)[0])
        return points

    def getPointsFromMultilinestring(self, geometry):         #not sure
        lineStringCount = geometry.GetGeometryCount()
        points = []
        for i in range(lineStringCount):
            lineString = geometry.GetGeometryRef(i)
            points.append(self.getPointsFromLineString(lineString)[0])
        return  [points]

    def getPointsFromPolygon(self, geometry):
        linearRing = geometry.GetGeometryRef(0)
        points = linearRing.GetPoints()
        return [points]

    def getPointsFromLineString(self, geometry):  # not sure
        line = geometry.GetGeometryRef(0)
        points = line.GetPoints()
        return [points]

    def getPointsFromMultipoint(self, geometry):  #not sure
        points = geometry.GetPoints()
        return [points]

    def getPointFromPoint(self, geometry):
        point = (geometry.getX(), geometry.getY())
        return [[point]]

    def getPoints(self, geometry):
        gtype = geometry.GetGeometryType()
        name = geometry.GetGeometryName()
        if gtype == 6 and name == "MULTIPOLYGON":
            return self.getPointsFromMultipolygon(geometry)
        elif gtype == 5 and name == "MULTILINESTRING":  #not sure
            return self.getPointsFromMultilinestring(geometry)
        elif gtype == 4 and name == "MULTIPOINT":       #not sure
            return self.getPointsFromMultipoint(geometry)
        elif gtype == 3 and name == "POLYGON":
            return self.getPointsFromPolygon(geometry)
        elif gtype == 2 and name == "LINESTRING":       #not sure
            return self.getPointsFromLineString(geometry)
        elif gtype == 1 and name == "POINT":            #not sure
            return self.getPointFromPoint(geometry)
        else:
            print("GMLParser: Unrecognized geometry type: ", name)
        return -1


    def getCoordinatesDictionary(self):
        return self.points_dict


    def parse(self, GMLfile):
        ogr.RegisterAll()
        inSource = ogr.Open(GMLfile)
        self.points_dict = {}
        for layerIndex in range(inSource.GetLayerCount()):
            ############################### LAYER #######################################
            inLayer = inSource.GetLayer(layerIndex)
            inLayer.ResetReading()          # not neccessary, ensures iterating from begining

            ############################### FEATURE #####################################
            for featureIndex in range(inLayer.GetFeatureCount()):
                feature = inLayer.GetNextFeature()

            ############################### GEOMETRY #####################################
                geometry = feature.GetGeometryRef()
                coord_system = geometry.GetSpatialReference()

                targetReference = SpatialReference()
                targetReference.ImportFromEPSG(4326) # WGS84
                geometry.TransformTo(targetReference)

                points = self.getPoints(geometry)
                # print(points)

                entryName = "Layer-" + str(layerIndex) + " Feature-" + str(featureIndex)
                self.points_dict[entryName] = points
                if self.points_dict.has_key('coordinates'):
                    self.points_dict['coordinates'] = self.points_dict['coordinates'] + points
                else:
                    self.points_dict['coordinates'] = points


    def exportToJSON(self):
        with open('WGS84_coordinates_from_GML.json', 'w') as file:
            json.dump(self.points_dict, file, indent=4)


if __name__ == '__main__':
    inSource = "/home/ivan/Downloads/katastarski_plan_CESTICE.gml"
    # inSource = /home/ivan/Downloads/Building_9620123VK0192B.gml"
    # inSource = "/home/ivan/Downloads/Building_9531109VK0193B.gml"
    # inSource = "/home/ivan/Downloads/Building_9642901VK3794B.gml"

    parser = GMLParser()
    parser.parse(inSource)
    print(parser.getCoordinatesDictionary())
