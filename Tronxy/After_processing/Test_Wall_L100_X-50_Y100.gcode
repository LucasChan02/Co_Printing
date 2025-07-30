;FLAVOR:Marlin
;TIME:390
;Filament used: 0.154776m
;Layer height: 0.3
;MINX:85.2
;MINY:40.76
;MINZ:0.3
;MAXX:184.8
;MAXY:45
;MAXZ:9
;TARGET_MACHINE.NAME:Tronxy XY-3
;Generated with Cura_SteamEngine 5.10.1
M104 S200
M105
M109 S200
; XY-3 Start Code
G21 ; Set units to millimeters
G90 ; Set all axis to Absolute
M83 ; Set extrusion to Relative
M107 ; Disable all fans
M220 S100 ; Set feedrate percentage
M190 S60 ; Set bed temperature and wait
G28 ; Home all axis
; Uncomment the line below to enable ABL Mesh probing
;G29 ; Probe bed mesh for ABL
; For best results do not run nozzle heater while performing ABL
;G1 Z5.0 ; Raise nozzle to prevent scratching of heat bed
;G1 X0 Y60 ; Move nozzle to Home before heating
;M109 S200 T0 ; Set nozzle temp and wait
;G92 E0 ; Set Extruder position to zero
; Uncomment the following lines to enable nozzle purge line along left edge of bed
G1 Z2.0 F3000 ; Raise Z axis
G1 X1.1 Y60 Z0.2 F3600.0 ; Move to purge line start position
G1 Y280 F1500.0 E15 ; Draw first purge line
G1 X1.4 F3600.0 ; Move to side
G1 Y60 F1500.0 E10 ; Draw second purge line
G92 E0 ; Reset Extruder
G1 Z2.0 F3000 ; Move Z Axis up little to prevent scratching of Heat Bed
G1 X5 Y60 Z0.2 F3600.0 ; Move over to finish nozzle wipe
G92 E0
M82 ;absolute extrusion mode
M83 ;relative extrusion mode
G1 F2700 E-5
;LAYER_COUNT:30
;LAYER:0
M107
M204 S200
;MESH:Body1
G0 F3600 X184.8 Y45 Z0.3
;TYPE:WALL-OUTER
G1 F2700 E5
G1 F600 X85.2 Y45 E4.90944
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z0.9
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:30.245656
;LAYER:2
M106 S20.4
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z1.5
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:56.135000
;LAYER:4
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z2.1
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:82.024344
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z2.7
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:107.913688
;LAYER:8
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z3.3
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:133.803032
;LAYER:10
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z3.9
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:159.692376
;LAYER:12
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z4.5
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:185.581720
;LAYER:14
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z5.1
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:211.471064
;LAYER:16
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z5.7
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:237.360408
;LAYER:18
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z6.3
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:263.249752
;LAYER:20
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z6.9
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:289.139096
;LAYER:22
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z7.5
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:315.028440
;LAYER:24
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z8.1
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:340.917784
;LAYER:26
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X85.2 Y45 Z8.7
G0 F3600 X85.2 Y40.76
G0 X184.8 Y40.76
G0 X184.8 Y45
;TIME_ELAPSED:366.807128
;LAYER:28
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F600 X85.2 Y45 E5.16783
G1 F2700 E-5
; XY-3 End Code
M83 ; Set extrder to Relative
G1 E-5 F3000 ; Retract 5mm of filament at 50mm/s
G90 ; Set all axis to Absolute
G1 X0 Y280 ; Park print head
G1 Z10 ; Move up 10mm
M106 S0 ; Set fan speed to 0
M104 S0 ; Set Nozzle temp to 0
M140 S0 ; Set Bed temp to 0
M84 ; Disable all stepper motors
M82 ;absolute extrusion mode
M104 S0
;End of Gcode
;SETTING_3 {"global_quality": "[general]\\nversion = 4\\nname = Alter_layer_0.3\
;SETTING_3 \ndefinition = tronxy_xy3\\n\\n[metadata]\\ntype = quality_changes\\n
;SETTING_3 quality_type = normal\\nsetting_version = 25\\n\\n[values]\\naccelera
;SETTING_3 tion_enabled = True\\nacceleration_travel_enabled = False\\nadhesion_
;SETTING_3 type = none\\njerk_enabled = False\\nlayer_height = 0.3\\nlayer_heigh
;SETTING_3 t_0 = 0.3\\nmaterial_bed_temperature = 60\\nrelative_extrusion = True
;SETTING_3 \\nretraction_combing = no_outer_surfaces\\nspeed_slowdown_layers = 0
;SETTING_3 \\nsupport_enable = False\\nsupport_structure = tree\\nsupport_type =
;SETTING_3  buildplate\\ntravel_retract_before_outer_wall = False\\n\\n", "extru
;SETTING_3 der_quality": ["[general]\\nversion = 4\\nname = Alter_layer_0.3\\nde
;SETTING_3 finition = tronxy_xy3\\n\\n[metadata]\\ntype = quality_changes\\nqual
;SETTING_3 ity_type = normal\\nintent_category = default\\nposition = 0\\nsettin
;SETTING_3 g_version = 25\\n\\n[values]\\nacceleration_print = 200\\nacceleratio
;SETTING_3 n_travel = 1000\\nbrim_gap = 0.2\\nbrim_width = 4\\ncool_fan_speed = 
;SETTING_3 8\\ninfill_material_flow = 108\\ninfill_pattern = cubicsubdiv\\ninfil
;SETTING_3 l_wipe_dist = 0.04\\nmaterial_flow = 104.0\\nmaterial_flow_layer_0 = 
;SETTING_3 95.0\\nmaterial_initial_print_temperature = =material_print_temperatu
;SETTING_3 re\\nmaterial_print_temperature = 200\\nmaterial_print_temperature_la
;SETTING_3 yer_0 = =material_print_temperature\\nretract_at_layer_change = False
;SETTING_3 \\nretraction_amount = 5\\nretraction_hop = 0.6\\nretraction_hop_enab
;SETTING_3 led = False\\nskin_material_flow = 100.0\\nskirt_brim_material_flow =
;SETTING_3  92\\nskirt_brim_speed = 10\\nspeed_infill = =speed_print\\nspeed_pri
;SETTING_3 nt = 10\\nspeed_topbottom = 10\\nspeed_travel = 60\\nspeed_wall = 10\
;SETTING_3 \nspeed_wall_x = =speed_wall\\nsupport_angle = 60\\nsupport_material_
;SETTING_3 flow = 92.0\\nsupport_xy_distance = 0.6\\ntop_bottom_thickness = 1.4\
;SETTING_3 \ntop_layers = =0 if infill_sparse_density == 100 else math.ceil(roun
;SETTING_3 d(top_thickness / resolveOrValue('layer_height'), 4))\\ntop_thickness
;SETTING_3  = =top_bottom_thickness\\ntravel_avoid_distance = 4\\ntravel_avoid_o
;SETTING_3 ther_parts = True\\nwall_0_wipe_dist = 0.05\\nwall_line_count = 5\\nw
;SETTING_3 all_x_material_flow = 105.0\\n\\n"]}
