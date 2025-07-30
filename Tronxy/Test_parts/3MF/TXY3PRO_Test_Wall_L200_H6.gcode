;FLAVOR:Marlin
;TIME:637
;Filament used: 0.282846m
;Layer height: 0.3
;MINX:55.274
;MINY:53.085
;MINZ:0.3
;MAXX:254.726
;MAXY:54.998
;MAXZ:6
;TARGET_MACHINE.NAME:Tronxy XY-3
;Generated with Cura_SteamEngine 5.10.1
M104 S200
M105
M109 S200
; XY-3 Start Code
G21 ; Set units to millimeters
G90 ; Set all axis to Absolute
M82 ; Set extrusion to Absolute
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
G1 X1.1 Y20 Z0.2 F3600.0 ; Move to purge line start position
G1 Y220 F1500.0 E15 ; Draw first purge line
G1 X1.4 F3600.0 ; Move to side
G1 Y20 F1500.0 E10 ; Draw second purge line
G92 E0 ; Reset Extruder
G1 Z2.0 F3000 ; Move Z Axis up little to prevent scratching of Heat Bed
G1 X5 Y20 Z0.2 F3600.0 ; Move over to finish nozzle wipe
G92 E0
M82 ;absolute extrusion mode
M83 ;relative extrusion mode
G1 F2700 E-5
;LAYER_COUNT:20
;LAYER:0
M107
M204 S200
;MESH:Body1
G0 F3600 X254.726 Y54.998 Z0.3
;TYPE:WALL-OUTER
G1 F2700 E5
G1 F438 X55.274 Y54.998 E13.46887
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z0.6
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:37.304752
;LAYER:1
M106 S51
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z0.9
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:69.094521
;LAYER:2
M106 S102
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z1.2
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:100.884289
;LAYER:3
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z1.5
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:132.674057
;LAYER:4
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z1.8
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:164.463826
;LAYER:5
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z2.1
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:196.253594
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z2.4
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:228.043363
;LAYER:7
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z2.7
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:259.833131
;LAYER:8
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z3
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:291.622899
;LAYER:9
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z3.3
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:323.412668
;LAYER:10
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z3.6
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:355.202436
;LAYER:11
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z3.9
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:386.992204
;LAYER:12
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z4.2
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:418.781973
;LAYER:13
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z4.5
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:450.571741
;LAYER:14
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z4.8
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:482.361510
;LAYER:15
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z5.1
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:514.151278
;LAYER:16
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z5.4
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:545.941046
;LAYER:17
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z5.7
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:577.730815
;LAYER:18
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X55.274 Y54.998 Z6
G0 F3600 X55.274 Y53.085
G0 X254.726 Y53.085
G0 X254.726 Y54.998
;TIME_ELAPSED:609.520583
;LAYER:19
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F438 X55.274 Y54.998 E14.17776
;TIME_ELAPSED:637.178054
G1 F2700 E-5
M204 S150
M82 ;absolute extrusion mode
M107
; XY-3 End Code
M83 ; Set extrder to Relative
G1 E-5 F3000 ; Retract 5mm of filament at 50mm/s
G90 ; Set all axis to Absolute
G1 X0 Y300 ; Park print head
G1 Z10 ; Move up 10mm
M106 S0 ; Set fan speed to 0
M104 S0 ; Set Nozzle temp to 0
M140 S0 ; Set Bed temp to 0
M84 ; Disable all stepper motors
M82 ;absolute extrusion mode
M104 S0
;End of Gcode
;SETTING_3 {"global_quality": "[general]\\nversion = 4\\nname = Alter_layer_prin
;SETTING_3 ting\\ndefinition = tronxy_xy3\\n\\n[metadata]\\ntype = quality_chang
;SETTING_3 es\\nquality_type = normal\\nsetting_version = 25\\n\\n[values]\\nacc
;SETTING_3 eleration_enabled = True\\nacceleration_travel_enabled = False\\nadhe
;SETTING_3 sion_type = none\\njerk_enabled = False\\nlayer_height = 0.3\\nlayer_
;SETTING_3 height_0 = 0.3\\nmaterial_bed_temperature = 60\\nrelative_extrusion =
;SETTING_3  True\\nretraction_combing = no_outer_surfaces\\nspeed_slowdown_layer
;SETTING_3 s = 0\\nsupport_enable = False\\nsupport_structure = tree\\nsupport_t
;SETTING_3 ype = buildplate\\ntravel_retract_before_outer_wall = False\\n\\n", "
;SETTING_3 extruder_quality": ["[general]\\nversion = 4\\nname = Alter_layer_pri
;SETTING_3 nting\\ndefinition = tronxy_xy3\\n\\n[metadata]\\ntype = quality_chan
;SETTING_3 ges\\nquality_type = normal\\nintent_category = default\\nposition = 
;SETTING_3 0\\nsetting_version = 25\\n\\n[values]\\nacceleration_print = 200\\na
;SETTING_3 cceleration_travel = 1000\\nbrim_gap = 0.2\\nbrim_width = 4\\ncool_fa
;SETTING_3 n_speed = 40\\ninfill_material_flow = 108\\ninfill_pattern = cubicsub
;SETTING_3 div\\ninfill_wipe_dist = 0.04\\nmaterial_flow = 104.0\\nmaterial_flow
;SETTING_3 _layer_0 = 95.0\\nmaterial_initial_print_temperature = =material_prin
;SETTING_3 t_temperature\\nmaterial_print_temperature = 200\\nmaterial_print_tem
;SETTING_3 perature_layer_0 = =material_print_temperature\\nretract_at_layer_cha
;SETTING_3 nge = False\\nretraction_amount = 5\\nretraction_hop_enabled = False\
;SETTING_3 \nskin_material_flow = 100.0\\nskirt_brim_material_flow = 92\\nskirt_
;SETTING_3 brim_speed = 10\\nspeed_infill = =speed_print\\nspeed_print = 10\\nsp
;SETTING_3 eed_topbottom = 10\\nspeed_travel = 60\\nspeed_wall = 10\\nspeed_wall
;SETTING_3 _x = =speed_wall\\nsupport_angle = 60\\nsupport_material_flow = 92.0\
;SETTING_3 \nsupport_xy_distance = 0.6\\ntop_bottom_thickness = 1.4\\ntop_layers
;SETTING_3  = =0 if infill_sparse_density == 100 else math.ceil(round(top_thickn
;SETTING_3 ess / resolveOrValue('layer_height'), 4))\\ntop_thickness = =top_bott
;SETTING_3 om_thickness\\ntravel_avoid_distance = 1.6\\ntravel_avoid_other_parts
;SETTING_3  = True\\nwall_0_wipe_dist = 0.05\\nwall_line_count = 5\\nwall_x_mate
;SETTING_3 rial_flow = 105.0\\n\\n"]}
