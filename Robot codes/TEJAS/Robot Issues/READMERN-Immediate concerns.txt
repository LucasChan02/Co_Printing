1.Any file required for reference of any syntax for the ampi_pro will be in the file named armpi_pro_demo in the src folder of armpi_pro folder.
2.For inverse kinematics to compute the joint angles or to find the reference positions for the end effector- armpi_pro---->src---->armpi_pro_kinematics---->kinematics---->ik_transform.py.
3.For all locomotion related codes for the armpi_pro- armpi_pro---->src---->armpi_pro_demo---->chassis_control_demo.
4.For all arm motions related to 3D printing related codes- armpi_pro---->testing---->Printing
5.Replace codes which use pandas as pd with other library functions used in python.
here, the python file mainasyncio.py is the executable python file for gcode printing.
5.Start the printing process by using "python3" to execute python files on the terminal.(EX: python3 mainasyncio.py)
6.Select directory(home/ubuntu/Desktop/armpi_pro/testing/Printing/) and follow the instructions stated in mainasyncio.py and eventually enter the name of the gcode file and then print.(gcode files have the extension .g)
7.Click on Ctrl+C in case you want to teminate a given program, click multiple times if necessary.