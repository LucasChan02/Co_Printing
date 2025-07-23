;FLAVOR:Marlin
;TIME:195
;Filament used: 0.24669m
;Layer height: 0.6
;MINX:0
;MINY:-49.5
;MINZ:0.6
;MAXX:0
;MAXY:49.5
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
G0 F1200 X0 Y-80 Z50
G1 F1200 E10
G1 E-2 F2400
G0 X0 Y-80 Z0
G0 Z20
M82 ;absolute extrusion mode
M83 ;relative extrusion mode
G1 F900 E-3
;LAYER_COUNT:10
;LAYER:0
M107
;MESH:Test_wall_L60_H6.obj
G0 F2400 X0.00 Y49.5 Z0.6
;TYPE:WALL-OUTER
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
G1 F900 E-3
;MESH:NONMESH
G0 F2400 X0.00 Y-49.5 Z1.2
G0 X0.00 Y49.5
;TIME_ELAPSED:22.733877
;LAYER:1
M106 S255
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
G1 F900 E-3
;MESH:NONMESH
G0 F2400 X0.00 Y-49.5 Z1.8
G0 X0.00 Y49.5
;TIME_ELAPSED:42.271481
;LAYER:2
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
G1 F900 E-3
;MESH:NONMESH
G0 F2400 X0.00 Y-49.5 Z2.4
G0 X0.00 Y49.5
;TIME_ELAPSED:61.809085
;LAYER:3
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
G1 F900 E-3
;MESH:NONMESH
G0 F2400 X0.00 Y-49.5 Z3
G0 X0.00 Y49.5
;TIME_ELAPSED:81.346690
;LAYER:4
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
G1 F900 E-3
;MESH:NONMESH
G0 F2400 X0.00 Y-49.5 Z3.6
G0 X0.00 Y49.5
;TIME_ELAPSED:100.884294
;LAYER:5
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
G1 F900 E-3
;MESH:NONMESH
G0 F2400 X0.00 Y-49.5 Z4.2
G0 X0.00 Y49.5
;TIME_ELAPSED:120.421898
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
G1 F900 E-3
;MESH:NONMESH
G0 F2400 X0.00 Y-49.5 Z4.8
G0 X0.00 Y49.5
;TIME_ELAPSED:139.959502
;LAYER:7
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
G1 F900 E-3
;MESH:NONMESH
G0 F2400 X0.00 Y-49.5 Z5.4
G0 X0.00 Y49.5
;TIME_ELAPSED:159.497106
;LAYER:8
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
G1 F900 E-3
;MESH:NONMESH
G0 F2400 X0.00 Y-49.5 Z6
G0 X0.00 Y49.5
;TIME_ELAPSED:179.034710
;LAYER:9
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-49.393 E24.66896
G0 X0.00 Y-49.5
;TIME_ELAPSED:195.738407
G1 F900 E-3
M82 ;absolute extrusion mode
M107
M104 S0
;Retract the filament
G1 E-5 F2400
G28 X0 Y0
M84
M82 ;absolute extrusion mode
M104 S0
;End of Gcode
;SETTING_3 {"global_quality": "[general]\\nversion = 4\\nname = For_Robot_Printi
;SETTING_3 ng\\ndefinition = custom\\n\\n[metadata]\\ntype = quality_changes\\nq
;SETTING_3 uality_type = draft\\nsetting_version = 25\\n\\n[values]\\naccelerati
;SETTING_3 on_enabled = False\\nacceleration_travel_enabled = False\\nadhesion_t
;SETTING_3 ype = none\\nlayer_height = 0.6\\nlayer_height_0 = 0.6\\nrelative_ext
;SETTING_3 rusion = True\\nretraction_combing = off\\nspeed_slowdown_layers = 0\
;SETTING_3 \n\\n", "extruder_quality": ["[general]\\nversion = 4\\nname = For_Ro
;SETTING_3 bot_Printing\\ndefinition = custom\\n\\n[metadata]\\ntype = quality_c
;SETTING_3 hanges\\nquality_type = draft\\nintent_category = default\\nposition 
;SETTING_3 = 0\\nsetting_version = 25\\n\\n[values]\\ncoasting_enable = True\\nc
;SETTING_3 oasting_speed = 100\\ninfill_wipe_dist = 0\\ninset_direction = outsid
;SETTING_3 e_in\\nmagic_fuzzy_skin_enabled = False\\nmagic_fuzzy_skin_point_dens
;SETTING_3 ity = 2\\nmagic_fuzzy_skin_thickness = 0.02\\nmaterial_print_temperat
;SETTING_3 ure = 210\\nretract_at_layer_change = True\\nretraction_amount = 3\\n
;SETTING_3 retraction_enable = True\\nretraction_speed = 15\\nspeed_layer_0 = 6\
;SETTING_3 \nspeed_print = 6\\nspeed_topbottom = 6\\nspeed_travel = 40\\nspeed_w
;SETTING_3 all = 6\\nspeed_wall_x = 6\\nspeed_z_hop = 40\\ntop_bottom_pattern_0 
;SETTING_3 = =top_bottom_pattern\\ntravel_avoid_distance = 2\\nwall_0_wipe_dist 
;SETTING_3 = 0\\n\\n"]}
