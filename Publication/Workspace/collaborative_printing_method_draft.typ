#import "@preview/charged-ieee:0.1.4": ieee

#show: ieee.with(
  title: [Title],
  //bibliography: bibliography("refs.bib"),
)

= Methodology

== Materials and Equipment

The filament used in this study was Bambu PLA Basic, a modified polymer composite consisting of 99% polylactic acid and less than 1% of additives such as Polycaprolactone (PCL), this is a common blend formula, oriented for high-speed printing and can achieve printing speeds up to 250-300 mm/s in certain machines.

Being the most common material in FDM printing, PLA is widely adopted as it's easy to print and inexpensive. Meanwhile, its stiffness and strength can meet most printing needs. It is also worth mentioning that it can biodegrade in some artificial composting conditions. According to the data provided by the supplier, typical values of material properties are listed in Table:


// #table(
//   columns: 3,
//   align: (left, left, left),
//   // Main table header
//   table.header(
//     [Subjects], [Testing Methods], [Data],
//   ),
//   table.hline(),

//   // Physical Properties Group
//   table.cell(colspan: 3, align: center)[*Physical Properties*],
//   [Density], [ISO 1183], [1.24 g/cm^3],
//   [Melt Index], [210 $degree$C, 2.16 kg], [23.2 $plus.minus$ 3.5 g/10 min],
//   [Melting Temperature], [DSC, 10 $degree$C/min], [160 $degree$C],
//   [Glass Transition Temperature], [DSC, 10 $degree$C/min], [60 $degree$C],
//   [Vicar Softening Temperature], [ISO 306], [57 $degree$C],
//   [Heat Deflection Temperature], [ISO 75 1.8 MPa], [54 $degree$C],
//   [Heat Deflection Temperature], [ISO 75 0.45 MPa], [57 $degree$C],
//   [Saturated Water Absorption Rate], [25 $degree$C, 55% RH], [0.43 %],
//   table.hline(),
//   // Mechanical Properties Group
//   table.cell(colspan: 3, align: center)[*Mechanical Properties*],
//   [Young's Modulus (X-Y)], [ISO 527], [2580 $plus.minus$ 220 MPa],
//   [Young's Modulus (Z)], [ISO 527], [2060 $plus.minus$ 170 MPa],
//   [Tensile Strength (X-Y)], [ISO 527], [35 $plus.minus$ 4 MPa],
//   [Tensile Strength (Z)], [ISO 527], [31 $plus.minus$ 3 MPa],
//   [Breaking Elongation Rate (X-Y)], [ISO 527], [12.2 $plus.minus$ 1.8 %],
//   [Breaking Elongation Rate (Z)], [ISO 527], [7.5 $plus.minus$ 1.3 %],
//   [Bending Modulus (X-Y)], [ISO 178], [2750 $plus.minus$ 160 MPa],
// )


Filament spools underwent a dehumidification process in a laboratory oven at 56 $degree$C for a minimum duration of 4 hours. Printing operations were conducted in an ambient environment maintained at 21 $degree$C and 50% relative humidity. Each experimental batch was completed within a 3-hour timeframe.

All samples were printed on a modified Independent Dual Extruder (IDEX) machine based on Tronxy Gemini S. The FDM machine offers a maximum build volume of 300 x 300 x 390 mm for single-extruder operation and 180 x 300 x 390 mm when utilizing both extruders. With a maximum extrusion temperature of 275°C, it is compatible with a range of thermoplastic filaments. The printer is based on core-xy overall configuration, equipped with an OSG dual-axis metal guide rail on its X-axis and reports a positioning precision of 0.012 mm (XY) and 0.004 mm (Z). Additional integrated features include filament run-out detection and a power-loss print resumption function.

SolidWorks was used for 3D modeling, then Ultimaker Cura 5.11 acted as slicing software, converting STL model to toolpath in Gcode format, then the gcode was handled by a python programme for afterprocessing, where the inter-layer behaviour was modified to more controlled layer change movement and the split the task for two printheads. 

Tensile testing was performed using a TSA750 test stand with a M5-750 force gauge, the combination provides capacity upto 3750 N (750 lbF / 375 kgF), Accuracy of the force gauge is ±0.1% of full scale, and resolution is 1/5000. Live data stream was captured on a PC connected through RS232 interface which transmits at 500 Hz output rate. 



== Specimen Design

Ideally, printed plastic specimens should conform to the ISO 527-1 Tensile Test on Plastics standard. However, the inherent design limitations of the core-xz structure, specifically the shared linear guide rail along the x-axis for both printhead carriages, leads to their synchronized movement along the y-axis. This coupling of printhead travel patterns consequently precluded the controlled deposition of multiple lines along the y-axis.

For the experimental testing conducted in this study, ASTM D882-18 (similar to ISO 527-3 but not considered fully equivalent) was adopted. This standard specifies the determination of tensile properties for plastics in the form of thin sheets less than 1.0 mm in thickness, a condition compatible with the capabilities of the experimental setup. The nominal width of the specimens shall be in the range from 5.0 mm to 25.4 mm, and the width to thickness ratio should be greater than 8. And the test specimens should be selected so that thickness is uniform to within 5% of the average thickness over the length of the specimens between the grips.

Specimens fabricated for this study maintained a consistent width of 18 mm, corresponding to 60 deposited layers, and a thickness of 0.4 mm. The effective lengths investigated included 50 mm, 100 mm, and 150 mm.

== Experimental Design and Parameter Matrix


The objective of this experiment was to evaluate the effect of printing speed and thermal history (characterized by variations in bonding interval), on the tensile strength and inter-layer adhesion.

Variables:

Independent Variables: Parameters changed on purpose. Wall Print Speed: 5 mm/s to 60 mm/s with 5 mm/s step; Test region length (mm).

Dependent Variables: Ultimate Tensile Strength (MPa), Tear Strength (MPa), Joint Interval (s), Total Print Time (min)".

Constant Parameters: Layer Height: 0.3 mm, Travel Speed: 60 mm/s, Nozzle Temperature: 208°C, Bed Temperature: 60°C.

For each condition, a total of two replicate specimens were fabricated. Prior to mechanical testing, all specimens underwent visual inspection. Samples with noticeable defects, such as flowrate fluctuations, insufficient inter-layer adhesion, or visible void disconnected regions, were identified and excluded from further analysis. Valid specimen was then utilized for measuring the ultimate tensile strength. The other replicate specimen was used for tear strength analysis, assessing resistance to force applied along the specimen's width.

== Printing Procedure and Techniques

=== Slicing

The printing machine was loaded with Marlin 1.1 firmware, with configuration files where the duplicate mode was enabled to give synchronized instructions to both printheads with controlled XY offset. Y-axis's acceleration was capped at $500 "mm""/"s^2$ to reduce vibration of the build plate and avoid losing steps during fast travel. 

Slicing was done in Cura with setting profiles modified to comply with the target motion for carriage \#1, the path will eventually be duplicated in actual printing process and task distributed between two printheads.

It is also noteworthy that in the initialization sequence gcode, which includes heating and homing procedures, the coordinate system for extrusion commands was reconfigured to relative positioning. This adjustment allows better integration of post-processing operations, particularly given that extrusion amounts for each pass are calculated volumetrically, by removing the need to account for accumulated historical lengths simplifies modifications.

All specimens were oriented vertically (XZ) on the build plate, with one long edge in contact with the build surface along Y-axis. The specimen thickness was defined by a single extruded line of 0.4 mm. The center of each specimen was positioned at coordinates (75, 30) on the build plate. Printing was performed on a smooth glass build plate covered with paper-based masking tape, without the use of printed supplementary adhesion structures such as brim or raft. To ensure firm attachment and prevent warping, the printhead was calibrated to a first-layer height of 0.2 mm, thereby increasing the contact area with the masking tape.


// Dual-Head Technique Descriptions: 

=== Technique 1: Alternating Layer

#image("../Assets/alter_dual_printhead.png")

=== Technique 2: Bi-directional Joint

#image("../Assets/idex_printhead.png")

== Characterization (Testing)


=== Tensile Properties

Mechanical testing was performed following the ASTM D882-18 "Standard Test Method for Tensile Properties of Thin Plastic Sheeting," selected as the appropriate standard for specimens with a thickness less than 1.0 mm. Tests were conducted using a TSA750 test stand equipped with an M5-750 digital force gauge and G1061-2 wedge grips. 

Specimens were mounted in the grips with the longitudinal axis aligned with the direction of applied force. The initial grip separation (gauge length) was set to match the designated test region length for each experimental condition. Constant incremental force was applied gradually until specimen rupture. The crosshead speed was selected to provide a consistent strain rate according to the guidelines.

=== Specimen Measurement and Analysis

Prior to testing, specimen dimensions were verified using vernier calipers with an accuracy of $plus.minus 0.02$ mm. The width and thickness were measured at five distinct points along the gauge length to determine the average cross-sectional area $A_0$.

The Ultimate Tensile Strength $sigma_t$ was calculated by dividing the maximum force recorded during the test $F_{max}$ by the original average cross-sectional area:

$ sigma_t = F_{max} / A_0 $

The result is expressed in MPa. Specimens exhibiting obvious manufacturing defects or failures at the grip interface (jaw breaks) were excluded from the dataset.

=== Tear Strength Assessment

The second replicate from each experimental condition was subjected to tear resistance analysis. The specimen was oriented to apply load perpendicular to the deposited layers to assess the resistance to force applied along the specimen's width. The maximum force required to initiate and propagate separation was recorded as the Tear Strength.