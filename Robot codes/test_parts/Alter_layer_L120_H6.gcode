;FLAVOR:Marlin
;TIME:426
;Filament used: 0.321792m
;Layer height: 0.6
;MINX:0
;MINY:-64.5
;MINZ:0.6
;MAXX:0
;MAXY:64.5
;MAXZ:6
;TARGET_MACHINE.NAME:Unknown
;Generated with Cura_SteamEngine 5.10.1
M104 S210
M105
M109 S210
G28 ;Home
;Set feedrate percentage
M220 S35.2
;Prime the extruder
G0 F1200 X20 Y-60 Z50
G1 F1200 E10
G92 E0
M400
M118 done
G0 X0 Y-80 Z0
G1 X30 Y-80 E15 F300
G0 Y-79
G1 X0 Y-79 E15 F300
G0 Z20
M82 ;absolute extrusion mode
G92 E0
G92 E0
G1 F900 E-3
;LAYER_COUNT:10
;LAYER:0
M107
M204 S3000
;MESH:Test_wall_L60_H6.obj
G0 F360 X0.00 Y64.5 Z0.6
;TYPE:WALL-OUTER
G1 F900 E0
G1 F360 X0.00 Y-64.5 E32.17918
G1 F900 E29.17918
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z1.2
G0 X0.00 Y64.5
;TIME_ELAPSED:56.301120
;LAYER:1
M106 S255
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E32.17918
G1 F360 X0.00 Y-64.5 E64.35836
G1 F900 E61.35836
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z1.8
G0 X0.00 Y64.5
;TIME_ELAPSED:99.862830
;LAYER:2
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E64.35836
G1 F360 X0.00 Y-64.5 E96.53754
G1 F900 E93.53754
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z2.4
G0 X0.00 Y64.5
;TIME_ELAPSED:143.424539
;LAYER:3
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E96.53754
G1 F360 X0.00 Y-64.5 E128.71672
G1 F900 E125.71672
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z3
G0 X0.00 Y64.5
;TIME_ELAPSED:186.986249
;LAYER:4
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E128.71672
G1 F360 X0.00 Y-64.5 E160.8959
G1 F900 E157.8959
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z3.6
G0 X0.00 Y64.5
;TIME_ELAPSED:230.547958
;LAYER:5
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E160.8959
G1 F360 X0.00 Y-64.5 E193.07508
G1 F900 E190.07508
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z4.2
G0 X0.00 Y64.5
;TIME_ELAPSED:274.109668
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E193.07508
G1 F360 X0.00 Y-64.5 E225.25426
G1 F900 E222.25426
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z4.8
G0 X0.00 Y64.5
;TIME_ELAPSED:317.671377
;LAYER:7
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E225.25426
G1 F360 X0.00 Y-64.5 E257.43345
G1 F900 E254.43345
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z5.4
G0 X0.00 Y64.5
;TIME_ELAPSED:361.233087
;LAYER:8
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E257.43345
G1 F360 X0.00 Y-64.5 E289.61263
G1 F900 E286.61263
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z6
G0 X0.00 Y64.5
;TIME_ELAPSED:404.794796
;LAYER:9
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E289.61263
G1 F360 X0.00 Y-64.5 E321.79181
;TIME_ELAPSED:426.499726
G1 F900 E318.79181
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
