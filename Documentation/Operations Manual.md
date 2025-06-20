### Main Printing program

1. **Open a terminal in Ubuntu Linux.**

2. **Change directory**
```
cd /armpi_pro/testing/
```

3. **Execute the synced printing program**
```
python3 arm_print_sync_main.py
```

4. After a few lines of output, it will show you a ranger file selection interface.   
   **Select the gcode file** you want, usually placed in `gcodes/` subfolder  
	press 'q' to quit selection if anything's wrong

5. **Press Enter to select the gcode file.**  

### Ultimaker Cura slicer

1. **Open model file**  
Click the “Open File” icon or go to File > Open File, then select the file you want to print. Some common model formats are STL, OBJ. You can also open .3mf as a project file which has models and slicing options saved internally

2. **Position and adjust the model**  
Use the tools on the left to move, rotate, scale or mirror the model to optimize printing orientation
- Red axis is the X axis, DO NOT let anything go into $X\leqslant0$ region, otherwise it will cause a collision or the solves won't be valid. 

3. **Tune print settings**  
In the right panel, choose print profile <For_Robot_Printing>, settings like layer height, infill, and printing speed are customizable if needed. 

4. **Slice the Model**  
Click the blue “Slice” button at the bottom right. Cura will convert  3D model into layers and generate G-code your printer can understand. 

5. **Preview the Sliced Model**  
Click “Preview” to see the layer-by-layer view of the print and check for any issues

6. **Save the G-code File**  
After slicing, the “Slice” button changes to “Save to Disk.” Save the G-code file to your computer and transfer it to Raspberry Pi later.

### File Transfer

1. **Download and install LocalSend from** [https://localsend.org/download]

2. **Connect to the same network as the Raspberry Pi**
	SSID: EIB151
	Password: smarture

3. **In Ubuntu, start the program form Applications Menu, Accessories - LocalSend**

4. **Start the program on your computer and select [Send] tab**

5. **Select the GCode file. Drag-and -drop is also available**

6. **Choose the target receiver device, ArmPi_pro_XX**

7. **The file(s) will automatically be saved in `/home/ubuntu/Downloads/` 