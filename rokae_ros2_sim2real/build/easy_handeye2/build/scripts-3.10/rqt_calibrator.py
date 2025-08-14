#!/usr/bin/python3

import sys

from rqt_gui.main import Main

main = Main()
sys.exit(main.main(sys.argv, standalone='easy_handeye2.handeye_rqt_calibrator.RqtHandeyeCalibrator'))
