from math import pi, radians

# general settings
Debug = False #mainly visualisation tools

loca_theta_offset = radians(45.12)# pi/4 #-2.18 : experimental value from evitement data of begining of may

#Check_obstacle settings
lidar_x_offset = 0.0 # !! Careful : offset are not implemented yet (value different from 0 may not work)
lidar_y_offset = 0.0 # !! Careful : offset are not implemented yet (value different from 0 may not work)
lidar_theta_offset = 0 #radians(45.12) #HOW TO DETERMINE BELOW :
# (0). It corresponds to trigonometric angle in radians from 0° of table to 0° of lidar when robot 0° is aligned with table 0°

# 10 cm offset from table edge
table_x_min = 0.1 # meters
table_x_max = 2.9 # meters
table_y_min = 0.1 # meters
table_y_max = 1.9 # meters
# CloudPoints settings

# obstacle cylinder settings
stop_cyl_width = 0.4 # meters
stop_cyl_dist = 0.40 #meFbters

stop_circular_dist = 0.3
robot_radius = 0.15 # meters #from robot/lidar center for avoidance purposes

warning_cyl_width = 0.5 # meters
warning_cyl_dist = 0.7 # meters


min_lidar_dist = 0.15 # Meters (radius of the robot perimeter)
max_lidar_dist = 3.5 # Meters (diagonal of the table is sqrt(13), rounded it) 

    #amalgames settings
# maximum squared absolute distance between two points within the cloud points to be considerered as one same amalgame
amalgame_squared_dist_max = 0.025 # meters

pylone_diam = 0.11 # diameter in meters - adding 0.01 as margin due to lidar imprecision
pylone_radius = 0.05
amalg_min_size = 0.05 # meters # min size of detected pylones of 0.1m
amalg_max_size = 0.41 # meters  # max calculable size for centrale pylone
# max size for pylones is 0.1m
amalg_max_nb_pts = 50

odometry_tolerance = 0.1 # meters tolerance of odometry from robot motor, used to filter the possible position according to lidar

# Fixed Points / Beacons coordinates

unsymetrical_point_index = 3 # index of the point that can't be symetric with another point in known_points_in_mm
blue_points_in_mm = ( #(x,y) | Made from Eurobot2023_Rules_FR_FInale, Blue Side
    (-90, 1000), #A (bottom left)     | (22+22+45+5) Bordure mur + Bordure Mur + Moitié + Moitié trou
    (3090, 50), #B (middle top)
    (3090, 1950), #C (bottom right)
    (1275,2090)  #D (Center of Support de Balise | Top middle)
    # (225, 3100) #E (Center of Experience | top middle left)
    ) 

green_points_in_mm = ( #(x,y) | Made from Eurobot2023_Rules_FR_FInale, Green Side
    (50, 2094), #A (top left)  
    (1500, -94), #B (middle bottom)
    (2950, 2094), #C (top right)
    (-22, 1000), #D (Center of Support de Balise | Top middle)
)

""" # deprecated (wrong xy origin)
known_points_in_mm = ( #(x,y) | Made from Eurobot2023_Rules_FR_FInale, Blue Side
    (-94, 50), #A (bottom left)     | (22+22+45+5) Bordure mur + Bordure Mur + Moitié + Moitié trou
    (2094, 1500), #B (middle right)
    (-94, 2950), #C (top left)
    (1000, 3022), #D (Center of Support de Balise | Top middle)
    # (225, 3100) #E (Center of Experience | top middle left)
    ) 
"""