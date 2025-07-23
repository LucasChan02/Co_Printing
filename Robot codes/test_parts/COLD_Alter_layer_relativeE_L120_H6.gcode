;FLAVOR:Marlin
;TIME:318
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
M104 S20
M105
M109 S20
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
G1 X-20 Y-80 E10 F300
G0 Y-79
G1 X0 Y-79 E10 F300
G0 Z20
M82 ;absolute extrusion mode
M83 ;relative extrusion mode
G1 F900 E-3
;LAYER_COUNT:10
;LAYER:0
M107
M204 S3000
;MESH:Test_wall_L60_H6.obj
G0 F360 X0.00 Y-64.5 Z0.6
;TYPE:WALL-OUTER
G1 F900 E3
G1 F360 X0.00 Y64.5 E32.17918
;TIME_ELAPSED:34.444341
;LAYER:1
G1 F900 E-3
M106 S255
;MESH:Test_wall_L60_H6.obj
G0 F360 X0.00 Y64.5 Z1.2
;TYPE:WALL-OUTER
G1 F900 E3
G1 F360 X0.00 Y-64.5 E32.17918
;TIME_ELAPSED:56.500784
;LAYER:2
G1 F900 E-3
;MESH:Test_wall_L60_H6.obj
G0 F360 X0.00 Y-64.5 Z1.8
;TYPE:WALL-OUTER
G1 F900 E3
G1 F360 X0.00 Y64.5 E32.17918
;TIME_ELAPSED:78.557228
;LAYER:3
G1 F900 E-3
;MESH:Test_wall_L60_H6.obj
G0 F360 X0.00 Y64.5 Z2.4
;TYPE:WALL-OUTER
G1 F900 E3
G1 F360 X0.00 Y-64.5 E32.17918
;TIME_ELAPSED:100.613671
;LAYER:4
G1 F900 E-3
;MESH:Test_wall_L60_H6.obj
G0 F360 X0.00 Y-64.5 Z3
;TYPE:WALL-OUTER
G1 F900 E3
G1 F360 X0.00 Y64.5 E32.17918
;TIME_ELAPSED:122.670114
;LAYER:5
G1 F900 E-3
;MESH:Test_wall_L60_H6.obj
G0 F360 X0.00 Y64.5 Z3.6
;TYPE:WALL-OUTER
G1 F900 E3
G1 F360 X0.00 Y-64.5 E32.17918
G1 F900 E-3
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z4.2
G0 X0.00 Y64.5
;TIME_ELAPSED:166.583336
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-64.5 E32.17918
G1 F900 E-3
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z4.8
G0 X0.00 Y64.5
;TIME_ELAPSED:210.145046
;LAYER:7
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-64.5 E32.17918
G1 F900 E-3
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z5.4
G0 X0.00 Y64.5
;TIME_ELAPSED:253.706755
;LAYER:8
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-64.5 E32.17918
G1 F900 E-3
;MESH:NONMESH
G0 F360 X0.00 Y-64.5 Z6
G0 X0.00 Y64.5
;TIME_ELAPSED:297.268465
;LAYER:9
;TYPE:WALL-OUTER
;MESH:Test_wall_L60_H6.obj
G1 F900 E3
G1 F360 X0.00 Y-64.5 E32.17918
;TIME_ELAPSED:318.973395
G1 F900 E-3
M204 S4000
M82 ;absolute extrusion mode
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
;SETTING_3 pe = none\\nlayer_height = 0.6\\nlayer_height_0 = 0.6\\nrelative_extr
;SETTING_3 usion = True\\nretraction_combing = off\\nspeed_slowdown_layers = 0\\
;SETTING_3 n\\n", "extruder_quality": ["[general]\\nversion = 4\\nname = For_Rob
;SETTING_3 ot_Printing\\ndefinition = custom\\n\\n[metadata]\\ntype = quality_ch
;SETTING_3 anges\\nquality_type = draft\\nintent_category = default\\nposition =
;SETTING_3  0\\nsetting_version = 25\\n\\n[values]\\ninset_direction = outside_i
;SETTING_3 n\\nmagic_fuzzy_skin_enabled = False\\nmagic_fuzzy_skin_point_density
;SETTING_3  = 2\\nmagic_fuzzy_skin_thickness = 0.02\\nmaterial_print_temperature
;SETTING_3  = 210\\nretract_at_layer_change = True\\nretraction_amount = 3\\nret
;SETTING_3 raction_enable = True\\nretraction_speed = 15\\nspeed_layer_0 = 6\\ns
;SETTING_3 peed_print = 6\\nspeed_topbottom = 6\\nspeed_travel = 6\\nspeed_wall 
;SETTING_3 = 6\\nspeed_wall_x = 6\\nspeed_z_hop = 6\\ntop_bottom_pattern_0 = =to
;SETTING_3 p_bottom_pattern\\ntravel_avoid_distance = 2\\n\\n"]}
