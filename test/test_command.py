import pytest
from bugbase_node.odrive_interface import *
import logging
import time

class TestODriveInterface:
    
    default_logger = None
    drive = None
    def setup_method(self):
        default_logger = logging.getLogger(__name__)
        default_logger.setLevel(logging.DEBUG)

        self.driver = ODriveInterface(logger=default_logger)

    # def test_Init_no_logger(self):
    #     with pytest.raises(Exception) as e_info:
    #         driver = ODriveInterface()

    def test_Init_w_logger(self):
        assert self.driver.driver    == None
        assert self.driver.axes      == None
        assert self.driver.odrive_enable_watchdog == True
        assert self.driver.odrive_watchdog_timeout == 5.0
        assert self.driver.wheel_direction == False
        assert self.driver.turn_direction == False
        assert self.driver.base_width     == 0.5
        assert self.driver.rounds_per_meter == 1.0

    def test_Connect(self):
        assert self.driver.connect() == True

    def test_Connect_reconnect(self):
        assert self.driver.connect() == True
        assert self.driver.reboot() == True
        assert self.driver.connect() == True

    def test_Del(self):

        assert self.driver.connect() == True

        self.driver.configure_timeout(30)
        time.sleep(29)
        assert self.driver.driver.axis0.error == AXIS_ERROR_NONE
        time.sleep(30)
        assert self.driver.driver.axis0.error == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED
        

        del self.driver

        dev = odrive.find_any(timeout=30)
        assert dev.axis0.error == AXIS_ERROR_NONE
        assert dev.axis1.requested_state == AXIS_STATE_UNDEFINED