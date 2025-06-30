;FLAVOR:Marlin
;TIME:165
;Filament used: 0.147176m
;Layer height: 0.6
;MINX:-2.54
;MINY:-29.5
;MINZ:0.6
;MAXX:0
;MAXY:29.5
;MAXZ:6
;TARGET_MACHINE.NAME:Unknown
;Generated with Cura_SteamEngine 5.10.1
M104 S210
M105
M109 S210
G92 E0
G28 ;Home
;Prime the extruder
G0 F1200 X20 Y60 Z50
G1 F200 E3
G92 E0
;Set feedrate percentage
M220 S59.5
M82 ;absolute extrusion mode
G1 F900 E-3
;LAYER_COUNT:10
;LAYER:0
M107
;MESH:Test_wall_L60_H6.obj
G0 F600 X0.00 Y-29.5 Z0.6
;TYPE:WALL-OUTER
G1 F900 E0
G1 F300 X0.00 Y29.5 E14.71761
G1 F900 E11.71761
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z1.2
G0 F600 X-2.54 Y29.5
G0 X-2.54 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:23.864165
;LAYER:1
M106 S255
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E14.71761
G1 F300 X0.00 Y29.5 E29.43522
G1 F900 E26.43522
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z1.8
G0 F600 X-2.54 Y29.5
G0 X-2.54 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:42.630010
;LAYER:2
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E29.43522
G1 F300 X0.00 Y29.5 E44.15283
G1 F900 E41.15283
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z2.4
G0 F600 X-2.54 Y29.5
G0 X-2.54 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:61.395856
;LAYER:3
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E44.15283
G1 F300 X0.00 Y29.5 E58.87044
;TIME_ELAPSED:73.399472
;LAYER:4
;MESH:Test_wall_L60_H6.obj
G0 F600 X0.00 Y29.5 Z3
;TYPE:WALL-OUTER
G1 F300 X0.00 Y-29.5 E73.58805
;TIME_ELAPSED:85.351631
;LAYER:5
;MESH:Test_wall_L60_H6.obj
G0 F600 X0.00 Y-29.5 Z3.6
;TYPE:WALL-OUTER
G1 F300 X0.00 Y29.5 E88.30566
G1 F900 E85.30566
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z4.2
G0 F600 X-2.54 Y29.5
G0 X-2.54 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:104.066020
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E88.30566
G1 F300 X0.00 Y29.5 E103.02327
G1 F900 E100.02327
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z4.8
G0 F600 X-2.54 Y29.5
G0 X-2.54 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:122.831865
;LAYER:7
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E103.02327
G1 F300 X0.00 Y29.5 E117.74088
;TIME_ELAPSED:134.835481
;LAYER:8
;MESH:Test_wall_L60_H6.obj
G0 F600 X0.00 Y29.5 Z5.4
;TYPE:WALL-OUTER
G1 F300 X0.00 Y-29.5 E132.45849
G1 F900 E129.45849
;MESH:NONMESH
G0 F360 X0.00 Y-29.5 Z6
G0 F600 X-2.54 Y-29.5
G0 X-2.54 Y29.5
G0 X0.00 Y29.5
;TIME_ELAPSED:153.549870
;LAYER:9
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E132.45849
G1 F300 X0.00 Y-29.5 E147.1761
;TIME_ELAPSED:165.553486
G1 F900 E144.1761
M107
M104 S0
;Retract the filament
G92 E1
G1 E-4 F300
G28 X0 Y0
M84
M82 ;absolute extrusion mode
M104 S0
;End of Gcode
;SETTING_3 {"global_quality": "[general]\\nversion = 4\\nname = For_Robot\\ndefi
;SETTING_3 nition = custom\\n\\n[metadata]\\ntype = quality_changes\\nquality_ty
;SETTING_3 pe = draft\\nsetting_version = 25\\n\\n[values]\\nacceleration_enable
;SETTING_3 d = False\\nadhesion_type = none\\nlayer_height = 0.6\\nlayer_height_
;SETTING_3 0 = 0.6\\nspeed_slowdown_layers = 0\\n\\n", "extruder_quality": ["[ge
;SETTING_3 neral]\\nversion = 4\\nname = For_Robot\\ndefinition = custom\\n\\n[m
;SETTING_3 etadata]\\ntype = quality_changes\\nquality_type = draft\\nintent_cat
;SETTING_3 egory = default\\nposition = 0\\nsetting_version = 25\\n\\n[values]\\
;SETTING_3 nmaterial_print_temperature = 210\\nretraction_amount = 3\\nretractio
;SETTING_3 n_enable = True\\nretraction_speed = 15\\nspeed_layer_0 = 6\\nspeed_p
;SETTING_3 rint = 10\\nspeed_topbottom = =speed_print / 2\\nspeed_travel = 10\\n
;SETTING_3 speed_wall = =speed_print / 2\\nspeed_wall_x = =speed_wall * 2\\nspee
;SETTING_3 d_z_hop = 6\\ntravel_avoid_distance = 2\\n\\n"]}
