;FLAVOR:Marlin
;TIME:650
;Filament used: 0.552045m
;Layer height: 0.3
;MINX:1.2
;MINY:43.76
;MINZ:0.3
;MAXX:184.8
;MAXY:45
;MAXZ:17.4
;TARGET_MACHINE.NAME:Tronxy XY-3
;Generated with Cura_SteamEngine 5.10.1
M104 S201
M105
M109 S201
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
;M109 S201.0 T0 ; Set nozzle temp and wait
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
G1 F3000 E-6
;LAYER_COUNT:58
;LAYER:0
;note -- min layer time used
M107
M204 S200
;MESH:Body1
G0 F3600 X184.8 Y45 Z0.3
M104 S200
;TYPE:WALL-OUTER
G1 F3000 E6
G1 F1612.9 X1.2 Y45 E9.04992
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z0.9
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:26.660193
;LAYER:2
;note -- min layer time used
M106 S12.8
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z1.5
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:49.074794
;LAYER:4
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z2.1
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:71.489395
;LAYER:6
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z2.7
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:93.903995
;LAYER:8
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z3.3
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:116.318596
;LAYER:10
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z3.9
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:138.733197
;LAYER:12
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z4.5
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:161.147798
;LAYER:14
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z5.1
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:183.562399
;LAYER:16
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z5.7
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:205.976999
;LAYER:18
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z6.3
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:228.391600
;LAYER:20
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z6.9
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:250.806201
;LAYER:22
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z7.5
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:273.220802
;LAYER:24
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z8.1
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:295.635403
;LAYER:26
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z8.7
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:318.050003
;LAYER:28
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z9.3
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:340.464604
;LAYER:30
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z9.9
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:362.879205
;LAYER:32
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z10.5
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:385.293806
;LAYER:34
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z11.1
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:407.708407
;LAYER:36
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z11.7
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:430.123007
;LAYER:38
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z12.3
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:452.537608
;LAYER:40
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z12.9
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:474.952209
;LAYER:42
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z13.5
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:497.366810
;LAYER:44
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z14.1
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:519.781410
;LAYER:46
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z14.7
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:542.196011
;LAYER:48
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z15.3
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:564.610612
;LAYER:50
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z15.9
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:587.025213
;LAYER:52
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z16.5
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:609.439814
;LAYER:54
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
;MESH:NONMESH
G0 F300 X1.2 Y45 Z17.1
G0 F3600 X1.2 Y43.76
G0 X184.8 Y43.76
G0 X184.8 Y45
;TIME_ELAPSED:631.854414
;LAYER:56
;note -- min layer time used
;TYPE:WALL-OUTER
;MESH:Body1
G1 F3000 E6
G1 F1587.3 X1.2 Y45 E9.52623
G1 F3000 E-6
; XY-3 End Code
M83 ; Set extrder to Relative
G1 E-5 F3000 ; Retract 5mm of filament at 50mm/s
G90 ; Set all axis to Absolute
G1 X0 Y300 ; Park print head
G1 Z10 ; Move up 10mm
M106 S0 ; Set fan speed to 0
M104 S200 ; Set Final Nozzle temp to 0
M109 S200 ; Wait for Nozzle temp
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
;SETTING_3 n_travel = 1000\\nbrim_gap = 0.2\\nbrim_width = 4\\ncool_fan_enabled 
;SETTING_3 = True\\ncool_fan_speed = 5\\ninfill_material_flow = 108\\ninfill_pat
;SETTING_3 tern = cubicsubdiv\\ninfill_wipe_dist = 0.04\\nmaterial_final_print_t
;SETTING_3 emperature = 201\\nmaterial_flow = 104.0\\nmaterial_flow_layer_0 = 95
;SETTING_3 .0\\nmaterial_initial_print_temperature = =material_print_temperature
;SETTING_3 \\nmaterial_print_temperature = =default_material_print_temperature\\
;SETTING_3 nmaterial_print_temperature_layer_0 = 201.0\\nretract_at_layer_change
;SETTING_3  = False\\nretraction_amount = 6\\nretraction_hop = 0.6\\nretraction_
;SETTING_3 hop_enabled = False\\nretraction_speed = 50\\nskin_material_flow = 10
;SETTING_3 0.0\\nskirt_brim_material_flow = 92\\nskirt_brim_speed = 10\\nspeed_i
;SETTING_3 nfill = =speed_print\\nspeed_print = 60\\nspeed_topbottom = 60\\nspee
;SETTING_3 d_travel = 60\\nspeed_wall = 60\\nspeed_wall_x = =speed_wall\\nsuppor
;SETTING_3 t_angle = 60\\nsupport_material_flow = 92.0\\nsupport_xy_distance = 0
;SETTING_3 .6\\ntop_bottom_thickness = 1.4\\ntop_layers = =0 if infill_sparse_de
;SETTING_3 nsity == 100 else math.ceil(round(top_thickness / resolveOrValue('lay
;SETTING_3 er_height'), 4))\\ntop_thickness = =top_bottom_thickness\\ntravel_avo
;SETTING_3 id_distance = 1.0\\ntravel_avoid_other_parts = True\\nwall_0_wipe_dis
;SETTING_3 t = 0.05\\nwall_line_count = 5\\nwall_x_material_flow = 105.0\\n\\n"]
;SETTING_3 }
