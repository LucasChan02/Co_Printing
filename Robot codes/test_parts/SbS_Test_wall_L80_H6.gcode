;FLAVOR:Marlin
;TIME:263
;Filament used: 0.197166m
;Layer height: 0.6
;MINX:30
;MINY:-79.52
;MINZ:0.6
;MAXX:30
;MAXY:-0.48
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
M118 done
G0 X0 Y-80 Z0
G1 X30 Y-80 E15 F300
G0 Y-79
G1 X0 Y-79 E15 F300
M82 ;absolute extrusion mode
G92 E0
G92 E0
G1 F900 E-3
;LAYER_COUNT:10
;LAYER:0
M107
M204 S3000
;MESH:Test_wall_L60_H6.obj
G0 F360 X30 Y-.48 Z0.6
;TYPE:WALL-OUTER
G1 F900 E0
G1 F360 X30 Y-79.52 E19.71661
G1 F900 E16.71661
;MESH:NONMESH
G0 F360 X30 Y-79.52 Z1.2
G0 X30 Y-.48
;TIME_ELAPSED:34.386769
;LAYER:1
M106 S255
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E19.71661
G1 F360 X30 Y-79.52 E39.43322
G1 F900 E36.43322
;MESH:NONMESH
G0 F360 X30 Y-79.52 Z1.8
G0 X30 Y-.48
;TIME_ELAPSED:61.295146
;LAYER:2
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E39.43322
G1 F360 X30 Y-79.52 E59.14982
G1 F900 E56.14982
;MESH:NONMESH
G0 F360 X30 Y-79.52 Z2.4
G0 X30 Y-.48
;TIME_ELAPSED:88.203522
;LAYER:3
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E59.14982
G1 F360 X30 Y-79.52 E78.86643
G1 F900 E75.86643
;MESH:NONMESH
G0 F360 X30 Y-79.52 Z3
G0 X30 Y-.48
;TIME_ELAPSED:115.111899
;LAYER:4
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E78.86643
G1 F360 X30 Y-79.52 E98.58304
G1 F900 E95.58304
;MESH:NONMESH
G0 F360 X30 Y-79.52 Z3.6
G0 X30 Y-.48
;TIME_ELAPSED:142.020275
;LAYER:5
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E98.58304
G1 F360 X30 Y-79.52 E118.29965
G1 F900 E115.29965
;MESH:NONMESH
G0 F360 X30 Y-79.52 Z4.2
G0 X30 Y-.48
;TIME_ELAPSED:168.928652
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E118.29965
G1 F360 X30 Y-79.52 E138.01626
G1 F900 E135.01626
;MESH:NONMESH
G0 F360 X30 Y-79.52 Z4.8
G0 X30 Y-.48
;TIME_ELAPSED:195.837028
;LAYER:7
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E138.01626
G1 F360 X30 Y-79.52 E157.73286
G1 F900 E154.73286
;MESH:NONMESH
G0 F360 X30 Y-79.52 Z5.4
G0 X30 Y-.48
;TIME_ELAPSED:222.745405
;LAYER:8
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E157.73286
G1 F360 X30 Y-79.52 E177.44947
G1 F900 E174.44947
;MESH:NONMESH
G0 F360 X30 Y-79.52 Z6
G0 X30 Y-.48
;TIME_ELAPSED:249.653781
;LAYER:9
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E177.44947
G1 F360 X30 Y-79.52 E197.16608
;TIME_ELAPSED:263.032045
G1 F900 E194.16608
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
;SETTING_3 action_speed = 15\\nspeed_layer_0 = 6\\nspeed_print = 6\\nspeed_topbo
;SETTING_3 ttom = 6\\nspeed_travel = 6\\nspeed_wall = 6\\nspeed_wall_x = 6\\nspe
;SETTING_3 ed_z_hop = 6\\ntop_bottom_pattern_0 = =top_bottom_pattern\\ntravel_av
;SETTING_3 oid_distance = 2\\n\\n"]}
