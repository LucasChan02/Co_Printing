;FLAVOR:Marlin
;TIME:108
;Filament used: 0.147176m
;Layer height: 0.6
;MINX:-1.165
;MINY:-29.5
;MINZ:0.6
;MAXX:0
;MAXY:29.5
;MAXZ:6
;TARGET_MACHINE.NAME:Unknown
;Generated with Cura_SteamEngine 5.10.1
M104 S20
M105
M109 S20
G28 ;Home
M82 ;absolute extrusion mode
G92 E0
;Set feedrate percentage
M220 S70
;LAYER_COUNT:10
;LAYER:0
M106 S108.8
;MESH:Test_wall_L60_H6.obj
G0 F600 X0.00 Y-29.5 Z0.6
;TYPE:WALL-OUTER
G1 F720 X0.00 Y29.5 E14.71761
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z1.2
G0 F600 X-1.165 Y29.5
G0 X-1.165 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:14.155084
;LAYER:1
M106 S255
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F720 X0.00 Y29.5 E29.43522
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z1.8
G0 F600 X-1.165 Y29.5
G0 X-1.165 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:25.359256
;LAYER:2
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F720 X0.00 Y29.5 E44.15283
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z2.4
G0 F600 X-1.165 Y29.5
G0 X-1.165 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:36.563428
;LAYER:3
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F720 X0.00 Y29.5 E58.87044
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z3
G0 F600 X-1.165 Y29.5
G0 X-1.165 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:47.767600
;LAYER:4
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F720 X0.00 Y29.5 E73.58805
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z3.6
G0 F600 X-1.165 Y29.5
G0 X-1.165 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:58.971772
;LAYER:5
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F720 X0.00 Y29.5 E88.30566
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z4.2
G0 F600 X-1.165 Y29.5
G0 X-1.165 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:70.175944
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F720 X0.00 Y29.5 E103.02327
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z4.8
G0 F600 X-1.165 Y29.5
G0 X-1.165 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:81.380115
;LAYER:7
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F720 X0.00 Y29.5 E117.74088
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z5.4
G0 F600 X-1.165 Y29.5
G0 X-1.165 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:92.584287
;LAYER:8
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F720 X0.00 Y29.5 E132.45849
;MESH:NONMESH
G0 F360 X0.00 Y29.5 Z6
G0 F600 X-1.165 Y29.5
G0 X-1.165 Y-29.5
G0 X0.00 Y-29.5
;TIME_ELAPSED:103.788459
;LAYER:9
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F720 X0.00 Y29.5 E147.1761
;TIME_ELAPSED:108.707554
M107
M104 S0
;Retract the filament
G92 E1
G1 E-1 F300
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
;SETTING_3 nretraction_enable = False\\nspeed_layer_0 = 6\\nspeed_print = 24\\ns
;SETTING_3 peed_topbottom = =speed_print / 2\\nspeed_travel = 10\\nspeed_wall = 
;SETTING_3 =speed_print / 2\\nspeed_wall_x = =speed_wall * 2\\nspeed_z_hop = 6\\
;SETTING_3 n\\n"]}
