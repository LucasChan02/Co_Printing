;FLAVOR:Marlin
;TIME:181
;Filament used: 0.147176m
;Layer height: 0.6
;MINX:30
;MINY:-59.5
;MINZ:0.6
;MAXX:30
;MAXY:-0.5
;MAXZ:6
;TARGET_MACHINE.NAME:Unknown
;Generated with Cura_SteamEngine 5.10.1
M104 S210
M105
M109 S210
G28 ;Home
;Set feedrate percentage
M220 S35.2
;M83 ;E Relative
;Prime the extruder
G0 F1200 X20 Y-60 Z50
G1 F200 E10
G92 E0
M400
M300 S440 P100
M118 done
G0 X0 Y-80 Z0
G1 X40 Y-80 E20 F300
G0 Y-79
M400
G1 X0 Y-79 E20 F300
M400
; prime the nozzle
G0 F1200 X20 Y-60 Z20;
M82 ;absolute extrusion mode
G92 E0
;LAYER_COUNT:10
;LAYER:0
M107
M204 S3000
;MESH:Test_wall_L60_H6.obj
G0 F600 X30 Y-.5 Z0.6
;TYPE:WALL-OUTER
G1 F900 E0
G1 F300 X30 Y-59.5 E14.71761
G1 F900 E11.71761
;MESH:NONMESH
G0 F360 X30 Y-59.5 Z1.2
G0 F600 X30 Y-.5
;TIME_ELAPSED:23.401036
;LAYER:1
M106 S255
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E14.71761
G1 F300 X30 Y-59.5 E29.43522
G1 F900 E26.43522
;MESH:NONMESH
G0 F360 X30 Y-59.5 Z1.8
G0 F600 X30 Y-.5
;TIME_ELAPSED:41.661551
;LAYER:2
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E29.43522
G1 F300 X30 Y-59.5 E44.15283
G1 F900 E41.15283
;MESH:NONMESH
G0 F360 X30 Y-59.5 Z2.4
G0 F600 X30 Y-.5
;TIME_ELAPSED:59.922065
;LAYER:3
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E44.15283
G1 F300 X30 Y-59.5 E58.87044
G1 F900 E55.87044
;MESH:NONMESH
G0 F360 X30 Y-59.5 Z3
G0 F600 X30 Y-.5
;TIME_ELAPSED:78.182580
;LAYER:4
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E58.87044
G1 F300 X30 Y-59.5 E73.58805
G1 F900 E70.58805
;MESH:NONMESH
G0 F360 X30 Y-59.5 Z3.6
G0 F600 X30 Y-.5
;TIME_ELAPSED:96.443095
;LAYER:5
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E73.58805
G1 F300 X30 Y-59.5 E88.30566
G1 F900 E85.30566
;MESH:NONMESH
G0 F360 X30 Y-59.5 Z4.2
G0 F600 X30 Y-.5
;TIME_ELAPSED:114.703610
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E88.30566
G1 F300 X30 Y-59.5 E103.02327
G1 F900 E100.02327
;MESH:NONMESH
G0 F360 X30 Y-59.5 Z4.8
G0 F600 X30 Y-.5
;TIME_ELAPSED:132.964124
;LAYER:7
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E103.02327
G1 F300 X30 Y-59.5 E117.74088
G1 F900 E114.74088
;MESH:NONMESH
G0 F360 X30 Y-59.5 Z5.4
G0 F600 X30 Y-.5
;TIME_ELAPSED:151.224639
;LAYER:8
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E117.74088
G1 F300 X30 Y-59.5 E132.45849
G1 F900 E129.45849
;MESH:NONMESH
G0 F360 X30 Y-59.5 Z6
G0 F600 X30 Y-.5
;TIME_ELAPSED:169.485154
;LAYER:9
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E132.45849
G1 F300 X30 Y-59.5 E147.1761
;TIME_ELAPSED:181.489975
G1 F900 E144.1761
M204 S4000
M107
M104 S0
;Retract the filament
G92 E1
G1 E-10 F300
G28 X0 Y0
M84
M82 ;absolute extrusion mode
M104 S0
;End of Gcode
;SETTING_3 {"global_quality": "[general]\\nversion = 4\\nname = For_Robot_Printi
;SETTING_3 ng\\ndefinition = custom\\n\\n[metadata]\\ntype = quality_changes\\nq
;SETTING_3 uality_type = draft\\nsetting_version = 25\\n\\n[values]\\naccelerati
;SETTING_3 on_enabled = True\\nacceleration_travel_enabled = False\\nadhesion_ty
;SETTING_3 pe = none\\nlayer_height = 0.6\\nlayer_height_0 = 0.6\\nretraction_co
;SETTING_3 mbing = off\\nspeed_slowdown_layers = 0\\n\\n", "extruder_quality": [
;SETTING_3 "[general]\\nversion = 4\\nname = For_Robot_Printing\\ndefinition = c
;SETTING_3 ustom\\n\\n[metadata]\\ntype = quality_changes\\nquality_type = draft
;SETTING_3 \\nintent_category = default\\nposition = 0\\nsetting_version = 25\\n
;SETTING_3 \\n[values]\\ninset_direction = outside_in\\nmagic_fuzzy_skin_enabled
;SETTING_3  = False\\nmagic_fuzzy_skin_point_density = 2\\nmagic_fuzzy_skin_thic
;SETTING_3 kness = 0.02\\nmaterial_print_temperature = 210\\nretract_at_layer_ch
;SETTING_3 ange = True\\nretraction_amount = 3\\nretraction_enable = True\\nretr
;SETTING_3 action_speed = 15\\nspeed_layer_0 = 6\\nspeed_print = 10\\nspeed_topb
;SETTING_3 ottom = =speed_print / 2\\nspeed_travel = 10\\nspeed_wall = =speed_pr
;SETTING_3 int / 2\\nspeed_wall_x = =speed_wall * 2\\nspeed_z_hop = 6\\ntop_bott
;SETTING_3 om_pattern_0 = =top_bottom_pattern\\ntravel_avoid_distance = 2\\n\\n"
;SETTING_3 ]}
