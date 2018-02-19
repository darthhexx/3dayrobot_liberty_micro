import math

def coord_bearing_degrees(lat2, long2, lat1, long1):
    """
    Calculates the bearing between two points. Returns in degrees.

    Formula:

    theta = atan2(sin(DELTAlong).cos(lat2),
                  cos(lat1).sin(lat2) - sin(lat1).cos(lat2).cos(DELTAlong))
    """
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    diffLong = math.radians(long1 - long2)

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
                                           * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    # We now have the initial beating but math.atan2 returns
    # values from -180 to 180 which isn't what we want for a compass
    # beating, solution is to normalize it
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = 180-initial_bearing

    return compass_bearing


def coord_bearing_radians(lat1, long1, lat2, long2):
    """
    Calculates the bearing between two points. Returns in radians.

    Formula:

    theta = atan2(sin(DELTAlong).cos(lat2),
                  cos(lat1).sin(lat2) - sin(lat1).cos(lat2).cos(DELTAlong))
    """
    deg = coord_bearing_degrees(lat1, long1, lat2, long2)
    return math.radians(deg)


def coord_dist_meters(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * \
        math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371000  # Radius of earth in meters. Use 3956 for miles
    return c * r


def subtract_angles(a, b):
    return a-b
