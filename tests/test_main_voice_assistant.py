import unittest
from unittest.mock import patch, MagicMock

# Import the functions we want to test
from main_voice_assistant import get_location_status, main


# Mock all ROS 2-related functions that will be called inside get_location_status
@patch('main_voice_assistant.rclpy')
class TestROS2Client(unittest.TestCase):
    """Tests the logic for calling the /where_am_i ROS2 service."""

    def test_service_success(self, mock_rclpy):
        """Test case where the ROS2 service call succeeds."""
        
        # 1. Mock the future result with a successful response
        mock_result = MagicMock()
        mock_result.message = "Test message: You are in the Living Room."
        
        mock_future = MagicMock()
        mock_future.result.return_value = mock_result
        
        # 2. Configure rclpy to return the mock future
        mock_client = MagicMock()
        mock_client.call_async.return_value = mock_future
        mock_client.wait_for_service.return_value = True

        mock_node = MagicMock()
        mock_node.create_client.return_value = mock_client
        
        mock_rclpy.create_node.return_value = mock_node
        
        # 3. Call the function
        result = get_location_status()

        # 4. Assert the result structure and content
        self.assertEqual(result, {"message": "Test message: You are in the Living Room."})
        mock_rclpy.spin_until_future_complete.assert_called_once()

    def test_service_unavailable_failure(self, mock_rclpy):
        """Test case where the ROS2 service is not available (timeout)."""
        
        # 1. Configure rclpy to report the service is unavailable
        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = False

        mock_node = MagicMock()
        mock_node.create_client.return_value = mock_client
        
        mock_rclpy.create_node.return_value = mock_node
        
        # 2. Call the function
        result = get_location_status()

        # 3. Assert the error message is returned
        self.assertEqual(result, {"error": "service not available"})
        # Assert that spin_until_future_complete was NOT called
        self.assertFalse(mock_rclpy.spin_until_future_complete.called)


# Patch the voice I/O, LLM, and ROS client for full isolation
@patch('main_voice_assistant.speak')
@patch('main_voice_assistant.listen')
@patch('main_voice_assistant.parse_command')
@patch('main_voice_assistant.get_location_status')
class TestMainLoop(unittest.TestCase):
    """Tests the main command interpretation and execution loop."""

    def test_status_command(self, mock_get_location, mock_parse, mock_listen, mock_speak):
        """Test case for 'Where am I?' command."""
        
        # 1. Setup Input/Output control
        mock_listen.side_effect = ["Where am I?", "exit"]  # First command, then exit
        mock_parse.return_value = {"action": "status"} # LLM output for "Where am I?"
        mock_get_location.return_value = {"message": "You are in the Bedroom."}

        # 2. Run the main loop
        main()

        # 3. Assert that the correct functions were called
        mock_parse.assert_called_with("Where am I?", ["Living Room", "Kitchen", "Bedroom"])
        mock_get_location.assert_called_once()
        mock_speak.assert_any_call("You are in the Bedroom.")


    def test_navigation_command(self, mock_get_location, mock_parse, mock_listen, mock_speak):
        """Test case for 'Go to the kitchen' command."""

        mock_listen.side_effect = ["Go to the kitchen.", "exit"]
        mock_parse.return_value = {"action": "go", "target": "Kitchen"}

        main()
        
        mock_speak.assert_any_call("Okay, going to the Kitchen.")
        mock_get_location.assert_not_called()


    def test_exit_command(self, mock_get_location, mock_parse, mock_listen, mock_speak):
        """Test case for "exit" command, which should break the loop."""

        mock_listen.side_effect = ["exit"] 
        
        main()

        # Assert that the loop breaks and calls the goodbye message
        mock_listen.assert_called_once()
        mock_speak.assert_any_call("Goodbye!")
        mock_parse.assert_not_called()
