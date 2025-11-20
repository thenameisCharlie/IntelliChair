import unittest
import json
from unittest.mock import patch

import sys
from pathlib import Path

# Add the parent directory (IntelliChair) to the system path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Now import the module which is located in the parent directory
from where_am_i_tool import coordinates_to_room, get_pose_with_fallback, POSE_PATH

# Set a temporary path for testing the JSON fallback
TEST_POSE_PATH = Path("/tmp/ic_test_pose.json")


class TestRoomMapping(unittest.TestCase):
    """Tests the logic for mapping coordinates to room names (AABB)."""

    def test_known_rooms(self):
        # Living Room: (-1 < x < 2 and -1 < y < 3)
        self.assertEqual(coordinates_to_room(0.5, 1.0), "Living Room")
        self.assertEqual(coordinates_to_room(1.9, 2.9), "Living Room")

        # Kitchen: (3 < x < 6 and -1 < y < 2)
        self.assertEqual(coordinates_to_room(4.0, 0.0), "Kitchen")
        self.assertEqual(coordinates_to_room(5.9, 1.9), "Kitchen")

        # Bedroom: (-2 < x < 1 and 4 < y < 7)
        self.assertEqual(coordinates_to_room(0.0, 5.0), "Bedroom")

    def test_unknown_areas_and_boundaries(self):
        # Gap between rooms
        self.assertEqual(coordinates_to_room(2.5, 0.0), "Unknown area")
        
        # Far outside
        self.assertEqual(coordinates_to_room(10.0, 10.0), "Unknown area")

        # Boundary checks (should be 'Unknown area' due to non-inclusive ranges)
        self.assertEqual(coordinates_to_room(2.0, 1.0), "Unknown area")
        self.assertEqual(coordinates_to_room(3.0, 0.0), "Unknown area")


class TestPoseFallback(unittest.TestCase):
    """Tests the priority and reliability of the get_pose_with_fallback function."""

    def setUp(self):
        # Ensure the test file does not exist before each test
        if TEST_POSE_PATH.exists():
            TEST_POSE_PATH.unlink()

    def tearDown(self):
        # Clean up the test file after each test
        if TEST_POSE_PATH.exists():
            TEST_POSE_PATH.unlink()

    # We patch the live SLAM function AND the POSE_PATH constant to use our test path
    @patch('where_am_i_tool._get_current_pose_from_lidar')
    @patch('where_am_i_tool.POSE_PATH', new=TEST_POSE_PATH)
    def test_1_live_slam_priority(self, mock_slam):
        """Test 1: SLAM is available and takes priority."""
        mock_slam.return_value = (5.5, 6.6, 7.7)

        # Create the file fallback data, but it should be ignored
        TEST_POSE_PATH.write_text(json.dumps({"x": 1.0, "y": 2.0, "theta": 3.0}))

        x, y, theta = get_pose_with_fallback()
        self.assertEqual((x, y, theta), (5.5, 6.6, 7.7))
        mock_slam.assert_called_once()


    @patch('where_am_i_tool._get_current_pose_from_lidar', new=None) # Explicitly remove SLAM
    @patch('where_am_i_tool.POSE_PATH', new=TEST_POSE_PATH)
    def test_2_file_fallback(self):
        """Test 2: SLAM is unavailable, file provides the pose."""
        # Create the file fallback data
        TEST_POSE_PATH.write_text(json.dumps({"x": -4.0, "y": 9.0, "theta": 1.5}))

        x, y, theta = get_pose_with_fallback()
        self.assertEqual(x, -4.0)
        self.assertEqual(y, 9.0)
        self.assertEqual(theta, 1.5)


    @patch('where_am_i_tool._get_current_pose_from_lidar', new=None) # Explicitly remove SLAM
    @patch('where_am_i_tool.POSE_PATH', new=TEST_POSE_PATH)
    def test_3_final_default_fallback(self):
        """Test 3: SLAM and file are unavailable, should return (0.0, 0.0, 0.0)."""
        # File is not created, SLAM is None 
        x, y, theta = get_pose_with_fallback()
        self.assertEqual((x, y, theta), (0.0, 0.0, 0.0))

