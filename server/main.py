from server_ import SerialHandler, LOG
from kinematics import Kinematics
from visual import ObjectDetector, Camera
import sys

def main(args):
    sh = SerialHandler(port=args.port, baud=args.baud)
    ok = sh.find_and_connect()
    if not ok:
        LOG.error("Failed to connect to Arduino. Exiting.")
        sys.exit(1)

    
