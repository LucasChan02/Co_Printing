;FLAVOR:Marlin
;TIME:233
;Filament used: 0.0580741m
;Layer height: 0.3
;MINX:105.226
;MINY:54.11
;MINZ:0.3
;MAXX:204.774
;MAXY:54.998
;MAXZ:3
;TARGET_MACHINE.NAME:Tronxy XY-3
;Generated with Cura_SteamEngine 5.10.1
M104 S200
M105
M109 S200
; XY-3 Start Code
G21 ; Set units to millimeters
G90 ; Set all axis to Absolute
M82 ; Set extrusion to Absolute
;M107 ; Disable all fans
M220 S100 ; Set feedrate percentage
;M190 S60 ; Set bed temperature and wait
;G28 ; Home all axis
; Uncomment the line below to enable ABL Mesh probing
;G29 ; Probe bed mesh for ABL
; For best results do not run nozzle heater while performing ABL
;G1 Z5.0 ; Raise nozzle to prevent scratching of heat bed
;G1 X0 Y0 ; Move nozzle to Home before heating
;M109 S200 T0 ; Set nozzle temp and wait
;G92 E0 ; Set Extruder position to zero
; Uncomment the following lines to enable nozzle purge line along left edge of bed
;G1 Z2.0 F3000 ; Raise Z axis
;G1 X1.1 Y20 Z0.2 F3600.0 ; Move to purge line start position
;G1 Y220 F1500.0 E10 ; Draw first purge line
;G1 X1.4 F3600.0 ; Move to side
;G1 Y20 F1500.0 E20 ; Draw second purge line
;G92 E0 ; Reset Extruder
;G1 Z2.0 F3000 ; Move Z Axis up little to prevent scratching of Heat Bed
;G1 X5 Y20 Z0.2 F3600.0 ; Move over to finish nozzle wipe
;G92 E0

M82 ;absolute extrusion mode
M83 ;relative extrusion mode
G1 F2700 E-5
;LAYER_COUNT:10
;LAYER:0
M107
M204 S200
;MESH:Body1
G0 F600 X204.774 Y54.998 Z0.3
;TYPE:WALL-OUTER
G1 F2700 E5
G1 F531 X105.226 Y54.998 E5.54477
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X105.226 Y54.998 Z0.9
G0 F600 X105.226 Y54.11
G0 X204.774 Y54.11
G0 X204.774 Y54.998
;TIME_ELAPSED:66.482907
;LAYER:2
M106 S102
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F531 X105.226 Y54.998 E5.8366
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X105.226 Y54.998 Z1.5
G0 F600 X105.226 Y54.11
G0 X204.774 Y54.11
G0 X204.774 Y54.998
;TIME_ELAPSED:110.760014
;LAYER:4
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F531 X105.226 Y54.998 E5.8366
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X105.226 Y54.998 Z2.1
G0 F600 X105.226 Y54.11
G0 X204.774 Y54.11
G0 X204.774 Y54.998
;TIME_ELAPSED:155.037120
;LAYER:6
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F531 X105.226 Y54.998 E5.8366
G1 F2700 E-5
;MESH:NONMESH
G0 F300 X105.226 Y54.998 Z2.7
G0 F600 X105.226 Y54.11
G0 X204.774 Y54.11
G0 X204.774 Y54.998
;TIME_ELAPSED:199.314227
;LAYER:8
;TYPE:WALL-OUTER
;MESH:Body1
G1 F2700 E5
G1 F531 X105.226 Y54.998 E5.8366
G1 F2700 E-5
;MESH:NONMESH
M204 S150
M82 ;absolute extrusion mode
M107
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
;SETTING_3 ype = buildplate\\n\\n", "extruder_quality": ["[general]\\nversion = 
;SETTING_3 4\\nname = Alter_layer_printing\\ndefinition = tronxy_xy3\\n\\n[metad
;SETTING_3 ata]\\ntype = quality_changes\\nquality_type = normal\\nintent_catego
;SETTING_3 ry = default\\nposition = 0\\nsetting_version = 25\\n\\n[values]\\nac
;SETTING_3 celeration_print = 200\\nacceleration_travel = 1000\\nbrim_gap = 0.2\
;SETTING_3 \nbrim_width = 4\\ncool_fan_speed = 40\\ninfill_material_flow = 108\\
;SETTING_3 ninfill_pattern = cubicsubdiv\\ninfill_wipe_dist = 0.04\\nmaterial_fl
;SETTING_3 ow = 104.0\\nmaterial_flow_layer_0 = 95.0\\nmaterial_initial_print_te
;SETTING_3 mperature = =material_print_temperature\\nmaterial_print_temperature 
;SETTING_3 = 200\\nmaterial_print_temperature_layer_0 = =material_print_temperat
;SETTING_3 ure\\nretraction_amount = 5\\nretraction_hop_enabled = False\\nskin_m
;SETTING_3 aterial_flow = 100.0\\nskirt_brim_material_flow = 92\\nskirt_brim_spe
;SETTING_3 ed = 10\\nspeed_infill = =speed_print\\nspeed_print = 10\\nspeed_topb
;SETTING_3 ottom = 10\\nspeed_travel = 10\\nspeed_wall = 10\\nspeed_wall_x = =sp
;SETTING_3 eed_wall\\nsupport_angle = 60\\nsupport_material_flow = 92.0\\nsuppor
;SETTING_3 t_xy_distance = 0.6\\ntop_bottom_thickness = 1.4\\ntop_layers = =0 if
;SETTING_3  infill_sparse_density == 100 else math.ceil(round(top_thickness / re
;SETTING_3 solveOrValue('layer_height'), 4))\\ntop_thickness = =top_bottom_thick
;SETTING_3 ness\\ntravel_avoid_other_parts = True\\nwall_0_wipe_dist = 0.05\\nwa
;SETTING_3 ll_line_count = 5\\nwall_x_material_flow = 105.0\\n\\n"]}
