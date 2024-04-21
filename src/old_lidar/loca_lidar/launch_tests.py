import pytest
from typing import Tuple
import numpy as np
import loca_lidar.PatternFinder as pf
import loca_lidar.CloudPoints as cp
from loca_lidar.ObstacleCalc import ObstacleCalc
from  loca_lidar.PointsDataStruct import PolarPts, AmalgamePolar
import loca_lidar.config as config

import time
import cProfile


######### Test amalgame discovery from cloud points ############

def test_one_amalgame():
    # Amalgame between 10° & 20°
    # Check if amalgame is detected if only the amalgame is detected by lidar and nothing else (except dummy points)
    # + test if the algorithm isn't looping
    pts_test = np.array([
        (0.01, 0),
        (0.01, 1),
        (0.01, 2),
        (0.01, 3),
        (0.01, 4),
        (0.01, 5),
        (0.01, 6),
        (0.01, 7),
        (0.01, 8),
        (0.01, 9),
        (1.0, 10), #expected size : 0.18
        (1.0, 11), #expected center : (1, 15)
        (1.0, 12),
        (1.0, 13),
        (1.0, 14),
        (1.0, 15),
        (1.0, 16),
        (1.0, 17),
        (1.0, 18),
        (1.0, 19),
        (1.0, 20),
        (0.01, 21),
        (0.01, 22),
        (0.01, 23),
        (0.01, 24),
        (0.01, 25),
        (0.01, 26),
        (0.01, 27),
    ], dtype=PolarPts)

    pts_filtered = np.array([
        (1.0, 10), #expected size : 0.18
        (1.0, 11), #expected center : (1, 15)
        (1.0, 12),
        (1.0, 13),
        (1.0, 14),
        (1.0, 15),
        (1.0, 16),
        (1.0, 17),
        (1.0, 18),
        (1.0, 19),
        (1.0, 20),
    ], dtype=PolarPts)
    basic_filtered = cp.basic_filter_pts(pts_test)
    assert np.array_equal(basic_filtered, pts_filtered)
    amalgames = cp.amalgames_from_cloud(basic_filtered)
    assert np.isclose(amalgames[0]['size'], 0.17431148)
    assert np.isclose(amalgames[0]['center_polar']['angle'], 15.0)
    assert np.isclose(amalgames[0]['center_polar']['distance'], 1.0)
    filtered_amalgames = cp.filter_amalgame_size(amalgames)
    assert filtered_amalgames.size == 1 #size is 0.18, we should find amalgame

def test_continuous_amalgame(): 
    #test one amalgame present at beggining and end of scan
    #test two amalgames that are different but are next to each other in the scan
    # test one amalgame present randomly
    pts_test = np.array([
        (1.0, 0),
        (1.0, 1),
        (0.01, 2),
        (0.01, 3),
        (0.01, 4),
        (0.01, 5),
        (0.01, 6),
        (2.5, 7),
        (2.5, 8),
        (2.5, 9),
        (2.5, 10), #two close (when sorting by polar angle) amalgames
        (1.5, 11), 
        (1.5, 12),
        (1.5, 13),
        (1.5, 14),
        (3.50, 15),
        (3.50, 16),
        (3.50, 17),
        (3.50, 18),
        (0.7, 19), #random amalgame
        (0.7, 20),
        (0.7, 21),
        (0.7, 22),
        (0.01, 23),
        (0.01, 24),
        (0.01, 25),
        (1.0, 358),
        (1.0, 359),
    ], dtype=PolarPts)

    basic_filtered = cp.basic_filter_pts(pts_test)
    amalgames = cp.amalgames_from_cloud(basic_filtered)
    filtered_amalgames = cp.filter_amalgame_size(amalgames)
    # Test filtration :
    expected_filter = np.array([(2.5, 8.5), (1.5, 12.5)], dtype=PolarPts)
    assert filtered_amalgames['center_polar'] == expected_filter
    # Testing calculation of relative center : 
    tuple_amalgames = cp.amalgame_numpy_to_tuple(amalgames)
    
    # checking first & last item
    assert tuple_amalgames[0][0] == 1.0
    assert tuple_amalgames[0][1] == 359.5
    assert tuple_amalgames[1][0] == 2.5
    assert tuple_amalgames[1][1] == 8.5
    assert tuple_amalgames[2][0] == 1.5
    assert tuple_amalgames[2][1] == 12.5
    assert tuple_amalgames[3][0] == 0.7
    assert tuple_amalgames[3][1] == 20.5

def test_continuous_amalgame_two():
    pts_test = np.array([
        (1.0, 0),
        (1.0, 1),
        (0.01, 2),
        (0.01, 3),
        (0.01, 4),
        (0.01, 5),
        (0.01, 6),
        (0.01, 7),
        (0.01, 8),
        (0.01, 9),
        (3.50, 10), 
        (3.50, 11), 
        (3.50, 12),
        (3.50, 13),
        (3.50, 14),
        (3.50, 15),
        (3.50, 16),
        (3.50, 17),
        (3.50, 18),
        (3.50, 19),
        (3.50, 20),
        (0.01, 21),
        (0.01, 22),
        (0.01, 23),
        (0.01, 24),
        (0.01, 25),
        (1.0, 26),
        (1.0, 27),
    ], dtype=PolarPts)

    pts_filtered = np.array([
        (1.0, 10), #expected size : 0.18
        (1.0, 11), #expected center : (1, 15)
        (1.0, 12),
        (1.0, 13),
        (1.0, 14),
        (1.0, 15),
        (1.0, 16),
        (1.0, 17),
        (1.0, 18),
        (1.0, 19),
        (1.0, 20),
    ], dtype=PolarPts)
    basic_filtered = cp.basic_filter_pts(pts_test)
    assert np.array_equal(basic_filtered, pts_filtered)
    amalgames = cp.amalgames_from_cloud(basic_filtered)
    empty_amalgame = np.zeros((20,), dtype=AmalgamePolar)
    assert np.array_equal(amalgames, empty_amalgame) #size is 0.18, above 0.15 so we shoudl'nt find amalgame

def test_cone_detection(): 
    pts_test = np.array([
        (0.01, 0),
        (0.01, 1),
        (0.01, 2),
        (0.01, 3),
        (0.01, 4),
        (0.01, 5),
        (0.01, 6),
        (0.01, 7),
        (0.01, 8),
        (0.01, 9),
        (1.0, 10),
        (1.0, 11),
        (1.0, 12),
        (1.0, 13),
        (0.2, 14),
        (1.0, 15),
        (1.0, 16),
        (1.0, 17),
        (1.0, 18),
        (1.0, 19),
        (1.0, 20),
        (0.01, 21),
        (0.01, 22),
        (0.01, 180),
        (0.01, 181),
        (0.01, 182),
        (0.01, 183),
        (0.01, 184),
    ], dtype=PolarPts)

    # TODO : refactor to avoid this test is dependent on config.py
    basic_filtered = cp.basic_filter_pts(pts_test)

######### Test finding pattern from amalgames
#table_example_geogeb.gbb
# Only the 3 beacons, experience, poteau
amalgame_sample_1 = ( 
    (1.57, 125.67),  
    (1.59, 237.94), 
    (1.68, 310.59), #CAREFUL : USE DEGREES ANGLE POSTIVE ONLY
    (1.62, 337.7),
    (1.57, 350.22)
)

#lidar_table_full_test.gbb
# amalgame_sample_1 + 3 adversarial beacons, 3 obstacles
amalgame_sample_2 = ( 
    (1.57, 125.67),  
    (1.59, 237.94), 
    (1.68, 310.59), #CAREFUL : USE DEGREES ANGLE POSTIVE ONLY
    (1.62, 337.7),
    (1.57, 350.22),
    (0.59, 57.94), # adversarial beacon
    (2.15, 195.58),
    (2.15, 280.31),
    (1.00, 147.94), # obstacle
    (1.56, 187.75),
    (1.12, 264.51),
)

amalgame_1 = pf.GroupAmalgame(amalgame_sample_1, False)

blue_beacons = pf.GroupAmalgame(tuple((x / 1000, y / 1000) for x,y in config.blue_points_in_mm), True)

# maximum error tolerance set for unit tests is 2mm for 2D lidar pose estimation 
pos_abs_tol = 0.002 #meters 
angle_abs_tol = 0.02 #degrees

finder = pf.LinkFinder(blue_beacons, 0.02)

def temporary_test():
    
    sample = (
        (2.4845, 3.08016),
        (2.979, 22.4655),
        (2.66488, 26.1395),
        (1.65737, 42.755),
        (0.186, 52.6309),
        (1.26955, 70.8109),
        (1.0995, 96.4532),
        (0.528837, 130.17),
        (0.685167, 155.066),
        (2.707, 178.538),
        (1.963, 251.56),
        (2.0394, 305.016),
        (1.5682, 344.372),

    )
    """
    sample = (
        (1.65737, 42.755),
        (0.186, 52.6309),
        (0.528837, 130.17),
        (1.963, 251.56),
        (1.5682, 344.372),
    ) """
    t = time.time()
    amalg = pf.GroupAmalgame(sample, False)
    lidar2table = finder.find_pattern(amalg)
    lidar_pos = pf.lidar_pos_wrt_table(
        lidar2table, amalg.points, blue_beacons.points)

    # verify angle
    lidar_angle = pf.lidar_angle_wrt_table(
        lidar_pos, lidar2table, amalg.points, blue_beacons.points)
    print(time.time()-t)

def test_all_fixed_no_obstacle_1():
    # Robot origin is at x~0.5, y~1.5, theta ~32.06° left
    # no other obstacles, approximated lidar reading from geogebra
    lidar2table = finder.find_pattern(amalgame_1)

    # verify position
    lidar_pos = pf.lidar_pos_wrt_table(
        lidar2table, amalgame_sample_1, blue_beacons.points)
    expected_lidar_pos = np.array([0.5, 1.5]).reshape(2, 1)
    assert np.allclose(lidar_pos, expected_lidar_pos, atol=pos_abs_tol)

    # verify angle
    lidar_angle = pf.lidar_angle_wrt_table(
        lidar_pos, lidar2table, amalgame_sample_1, blue_beacons.points)
    expected_lidar_angle = 32.06
    assert np.isclose(lidar_angle, expected_lidar_angle, angle_abs_tol)

def test_partial_fixed_no_obstacle_1(): 
    #expected values :
    expected_lidar_pos = np.array([0.5, 1.5]).reshape(2, 1)
    expected_lidar_angle = 32.06

    #test amalgame_sample_1 with 4 points :
    first_four_amalgames = pf.GroupAmalgame(amalgame_sample_1[:4], False)
    lidar2table = finder.find_pattern(first_four_amalgames)

    # verify position
    lidar_pos = pf.lidar_pos_wrt_table(
        lidar2table, first_four_amalgames.points, blue_beacons.points)
    assert np.allclose(lidar_pos, expected_lidar_pos, atol=pos_abs_tol)

    # verify angle
    lidar_angle = pf.lidar_angle_wrt_table(
        lidar_pos, lidar2table, first_four_amalgames.points, blue_beacons.points)
    assert np.isclose(lidar_angle, expected_lidar_angle, angle_abs_tol)

    #test amalgame_sample_1 with 3 points:
    first_three_amalgames = pf.GroupAmalgame(amalgame_sample_1[:3], False)
    lidar2table = finder.find_pattern(first_three_amalgames)

    # verify position
    lidar_pos = pf.lidar_pos_wrt_table(
        lidar2table, first_three_amalgames.points, blue_beacons.points)
    assert np.allclose(lidar_pos, expected_lidar_pos, atol=pos_abs_tol)

    # verify angle
    lidar_angle = pf.lidar_angle_wrt_table(
        lidar_pos, lidar2table, first_three_amalgames.points, blue_beacons.points)
    assert np.isclose(lidar_angle, expected_lidar_angle, angle_abs_tol)

    #test amalgame_sample_1 with 2 points:
    first_two_amalgames = pf.GroupAmalgame(amalgame_sample_1[:2], False)
    with pytest.raises(ValueError):
        lidar2table = finder.find_pattern(first_two_amalgames)

def test_obstacle_calc():
    obs_calc = ObstacleCalc(0.0, 0.0, 0.0)
    robot_pose = (0.5, 1.5, 32.06)
    filtered_obstacles = obs_calc.calc_obstacles_wrt_table(robot_pose, amalgame_sample_2)
    assert np.isclose(filtered_obstacles[0][0], 0.5, atol=0.01)
    assert np.isclose(filtered_obstacles[0][1], 0.5, atol=0.01)
    assert np.isclose(filtered_obstacles[1][0], 1.5, atol=0.01)
    assert np.isclose(filtered_obstacles[1][1], 0.3, atol=0.01)
    assert np.isclose(filtered_obstacles[2][0], 1.5, atol=0.01)
    assert np.isclose(filtered_obstacles[2][1], 2.0, atol=0.01)

def test_obstacle_offset():
    obs_calc = ObstacleCalc(0.1, 0.2, 10)
    robot_pose = (0.5, 1.5, 32.06)
    filtered_obstacles = obs_calc.calc_obstacles_wrt_table(robot_pose, amalgame_sample_2)
    assert True #breakpoint
    raise NotImplementedError('calculation are not correct yet')



#TODO : check function lidar_angle_wrt_table with origin of robot aligned x or y axis to the beacon

#Test case : 2 perfect with geogebra (bottom right, top right pointing bottom left)
# take one of these and add false obstacles inside & outside 
# robot is outside 
#obstacles that make a valid positionning
#cloud points that are not valid


#def test_cloud_points():
    #assert all distances
if __name__ == "__main__":
    print(cp.make_enclosing_circle(
         [(0, 0.2), (0,0), (-0.1, 0.2), (-0.2, 0.2), (-0.15, 0.2), (0, 0.14), (-0.1, 0.09)]
        ))

    # Test Amalgames detection
    temporary_test()
    test_one_amalgame()
    test_continuous_amalgame()
    print("test_continuous_amalgame_two is broken")
    # test_continuous_amalgame_two()

    # Test obstacle avoidance
    test_cone_detection()


    # Test Obstacle Calc through request
    test_obstacle_calc()
    # TODO : test_obstacle_offset()


    # Test triangulation / Simultaneous Corresponance and Pose Estimation
    test_all_fixed_no_obstacle_1()
    test_partial_fixed_no_obstacle_1()
    # pytest.main(["-x", ".\\loca_lidar\\launch_tests.py"])

