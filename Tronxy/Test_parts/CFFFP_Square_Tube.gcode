;FLAVOR:Marlin
;TIME:66
;Filament used: 0.062864m
;Layer height: 0.4
;MINX:6.868
;MINY:-6.748
;MINZ:0.4
;MAXX:16.368
;MAXY:2.748
;MAXZ:4
;TARGET_MACHINE.NAME:Unknown
;Generated with Cura_SteamEngine 5.10.1
M104 S200
M105
M109 S200
G28 ;Home
G1 Z15.0 F6000 ;Move the platform down 15mm
;Prime the extruder
G92 E0
G1 F200 E3
G92 E0
M83 relative extrusion mode
G92 E0
G92 E0
;LAYER_COUNT:10
;LAYER:0
M106 S54.7
;MESH:Body1
G0 F360 X15.955 Y2.745 Z0.4
;TYPE:WALL-OUTER

;LAYER:2
;MESH:Body1
G0 F360 X15.955 Y2.745 Z1.2
;TYPE:WALL-OUTER
G1 F360 X50.0 Y50.0 E20.000
G1 F360 X-50.0 Y50.0 E20.000
G1 F360 X-50.0 Y-50.0 E20.000
G1 F360 X50.0 Y-50.0 E20.000

G1 F360 X50.0 Y50.0 E20.000
G1 F360 X-50.0 Y50.0 E20.000
G1 F360 X-50.0 Y-50.0 E20.000
G1 F360 X50.0 Y-50.0 E20.000

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
;SETTING_3 d = False\\nadhesion_type = none\\nlayer_height = 0.4\\nlayer_height_
;SETTING_3 0 = 0.4\\nspeed_slowdown_layers = 0\\n\\n", "extruder_quality": ["[ge
;SETTING_3 neral]\\nversion = 4\\nname = For_Robot\\ndefinition = custom\\n\\n[m
;SETTING_3 etadata]\\ntype = quality_changes\\nquality_type = draft\\nintent_cat
;SETTING_3 egory = default\\nposition = 0\\nsetting_version = 25\\n\\n[values]\\
;SETTING_3 nretraction_enable = False\\nspeed_layer_0 = 6\\nspeed_print = 6\\nsp
;SETTING_3 eed_topbottom = 6\\nspeed_travel = 6\\nspeed_wall = 6\\nspeed_wall_x 
;SETTING_3 = 6\\nspeed_z_hop = 6\\n\\n"]}
