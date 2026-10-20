from lib.opensurgbot_pipeline import UserInterface3DViz, KinematicsManager, DriverInterface
import logging

if __name__ == "__main__":
    logger = logging.Logger("COM3", logging.DEBUG)
    logger.addHandler(logging.StreamHandler())

    driver1 = DriverInterface(serial_port="COM3",logger=logger)
    manager = KinematicsManager([driver1], logger=logger)
    ui = UserInterface3DViz(manager, logger=logger)
    try:
        ui.run()
    finally:
        driver1.return_home()
