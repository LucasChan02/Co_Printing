#import "@preview/charged-ieee:0.1.4": ieee

#show: ieee.with(
  title: [ A Strength-Aware Decomposition Framework for Multi-Head Additive 
    Manufacturing via Programmed Interlocking Interfacial Topologies ],
  abstract: [
    As additive manufacturing (AM) is evolving, multi-head or multi-robotic additive manufacturing have gained attention, especially as it enhances printing capabilities by performing conformal printing, multi-material printing, and supportless printing. Although decomposing or assigning equal distribution of work for multiple heads enables better efficiency for manufacturing, there are many knowledge gaps such as how decomposition of a part into sub-parts affects interfacial properties at the boundary of sub-parts, how different printing strategies of these sub-parts influence the resulting microstructure and consequently affect overall strength, etc. We develop a parametric framework to generate families of surface patterns—straight, angled, sinusoidal, interlocking, and zig-zag—on vertical partition planes in single-track, multi-layer builds. For each pattern, geometric descriptors (e.g., amplitude, wavelength, phase offset) are coupled with extrusion parameters (print speed, bonding interval, overlapping distance). Specimens are fabricated under controlled continuous extrusion and tested in uniaxial tension to quantify effective joint strength; concurrent with thermal imager providing thermal histories used to infer interdiffusion windows and cooling rates at the interface. Results are mapped to process–structure relationships that reveal how pattern topology and deposition timings interact to modulate bonded area and mechanical interlocking. Building on these relationships, we propose a decision-making framework that selects decompositions of geometry and printing strategy to maximize strength subject to constraints such as collision avoidance and path feasibility. The study provides systematic steps for strength-aware decomposition in multi-head AM systems.
  ],
  authors: (
    (
      name: " ",
      department: [ ],
      organization: [ ],
      location: [ ],
      email: " "
    ),
    (
      name: " ",
      department: [ ],
      organization: [ ],
      location: [ ],
      email: " "
    ),
  ),
  index-terms: ("Additive Manufacturing", "Multi-Head 3D Printing", 
    "Fused Filament Fabrication (FFF)", "Interfacial Strength", 
    "Mechanical Interlocking", "Process-Structure-Property (PSP) Model", 
    "Part Decomposition", "In-situ Thermal Monitoring"),
  //bibliography: bibliography("refs.bib"),
)

#set heading(numbering: "1.")


// Content goes below.

= Intro

== Background and Motivation

  Brief review of existing strategies: process parameter optimization (angle of repose, raster angles, layer height, etc.), thermal management, and geometric design. 

== Problem Statement

  Lack of understanding of how decomposition strategies affect interfacial strength from geometric and process parameters.

  Discussion of the thermo-physical phenomena in polymer bonding and how "cold joints" at the boundary leads to poor interconnection.

  Introduce controlled mechanical interlocking as a robust method, setting the stage for the novel programmable, vertically-oriented interlocking proposed in this work.

== Objectives

  + A Process-Structure-Property (PSP) model for quantifying the relationship between pattern topology, process parameters, and mechanical strength. Leading to a novel method for programmable interlocking interfaces. 
  + A "Strength-Aware Decomposition Framework" using Generative/Searching Algorithm (GA or MCTS) to optimize part decomposition for maximum mechanical integrity

= Parametric Modeling

== Parameterization of Interface geometries

  Create a mathematical way to describe interface topologies. Define following geometry descriptors for comparative analysis:

  - Bonding Surface Coefficient $k_s$

  - Mechanical Interlocking Depth (or Volumetric Interlock Ratio)

== Thermal History at Bonding Interface

Use "Interdiffusion Time Window" ($t_("interdiff")$) to describe thermal history, the duration the interface remains above sufficient bonding temperature $T_s$. 

In printing process, this is critically influenced by the "Bonding Interval" ($Delta t_"bond"$), a controllable parameter representing the time between adjacent depositions at the seam.


= Methodology

== Specimen Fabrication

=== Material

Polymaker Polylite™ PLA

#table(
  columns: (auto, auto, auto),
  inset: 2pt,
  align: (col, row) => if col == 2 { right + horizon } else { left + horizon },
  fill: (col, row) => if row == 0 { luma(240) } else { none },
  table.header(
    [*Property*], [*Testing Method*], [*Typical Value*],
  ),
  [Density], [ISO 1183], [1.22 g/cm#super[3] at 21.5°C],
  [Melt index], [210°C, 2.16 kg], [6 g/10min],
  [Glass transition temperature], [DSC, 10°C/min], [62 °C],
  [Melting temperature], [DSC, 10°C/min], [150 °C],
  [Vicat softening temperature], [ISO 306], [62.7 °C],
  [Heat deflection temperature], [ISO 75 1.8MPa], [57.6 °C],
  [Heat deflection temperature], [ISO 75 0.45MPa], [59.3 °C],
  [Young’s modulus (X-Y)], [ISO 527], [2932.2 ± 55.3 MPa],
  [Young’s modulus (Z)], [ISO 527], [2633.0 ± 117.4 MPa],
  [Tensile strength (X-Y)], [ISO 527], [49.8 ± 0.4 MPa],
  [Tensile strength (Z)], [ISO 527], [36.5 ± 0.6 MPa],
  [Elongation at break (X-Y)], [ISO 527], [6.3 ± 0.9 %],
  [Elongation at break (Z)], [ISO 527], [2.4 ± 0.3 %],
  [Bending modulus (X-Y)], [ISO 178], [2933.8 ± 78.3 MPa],
  [Bending strength (X-Y)], [ISO 178], [77.9 ± 0.4 MPa],
  [Charpy impact strength (X-Y)], [ISO 179], [17.1 ± 1.2 kJ/m#super[2]],
)

#linebreak()

=== Specimen Geometry

Single layer wall, height 18 mm, length 100,150 mm, thickness 0.4 mm.  ...

=== Design of Experiments (DoE) Matrix

  The purpose of the experiments is to systematically explore the parameter space.

  -   Factors: Interface Pattern (Type, Amplitude, Wavelength), 
        Process ($Delta t_"bond"$, Print Speed, Extrusion Multiplier).

  -   Responses: Mechanical (UTS, Modulus), Thermal 
        ($t_"interdiff"$), Microstructural (Void Fraction, Bonding Surface Coefficient etc.).

  For contact pattern families (sine wave, square wave, triangular wave, slope, interlocking as curve functions), create CAD models within the predefined range of parameters, Prepare samples using controlled continuous extrusion method.
  
== Microscopic image and Automated Feature Segmentation

  Capture microscopic pictures for indexing multi-section contact interface. For each speciment, combine multiple captured images and stitch them to form a scanning image which covers the total length of seam. 

// #grid(
//   columns: (1fr, 1fr),
//   align(center)[
//     #image("Assets/1103_0001_segments.png")
//   ],
//   align(center)[
//     #image("Assets/1103_0001_approximated.png")
//   ]
// )

#image("Assets/1103_0001_approximated.png", height: 30%)

#image("Assets/1103_0001_segments.png", height: 30%)

  To overcome manual analysis bias, a U-Net CNN could be implemented for objective segmentation of microscopic images. Divided segments should be categorized in several classes shown below:


  -   *Segmentation Classes:*
      1.  Adhesion_Interface_Closing
      2.  Adhesion_Interface_Opening_Upper
      3.  Adhesion_Interface_Opening_Lower
      4.  Horizontal_Interlock_Surface
      5.  Void/Porosity

    #image("Assets/segmentation_classes.png")

=== Semantic Segmentation of Materials (Image $->$ Mask)

  First of all, it is necessary to let the algorithm excels at classifying material regions before acquiring geometric properties at the boundary. The U-Net CNN's primary task should be to segment the image into its constituent regions: Material A, Material B, Void or Porosity. The classification of the interface is then performed in a precise, deterministic computational step (in section (b)) rather than a probabilistic pixel-level classification. 

=== Contour Extraction (Mask $->$ Vector)

Given the binary segmentaiton mask, the 2D boundary between them could be vectorized by converting the contact line of masks into polyline. Functions like `cv2.findContours()` can trace the boundary of regions and output a raw list of vertex coordinates which defines the interface.

=== Polyline Simplification

The canonical algorithm for this task is the Ramer-Douglas-Peucker (RDP) algorithm. The RDP algorithm decimates a polyline by recursively removing vertices that are within a specified perpendicular distance, $epsilon$ (epsilon), from a line segment connecting its endpoints.

The output of the RDP algorithm is the final, simplified interfacial polyline: $L = {P_0, P_1, dots, P_n}$, where each $P_i = (x_i, y_i)$ is a vertex. A segment $S_i$ is formally defined as the straight line connecting the vertices $P_i$ and $P_(i+1)$.

== Parametric Modeling of Interfacial Segments

=== Local Segment Properties
For each segment $S_i$, defined by its start and end vertices $P_i (x_i, y_i)$ and $P_(i+1) (x_(i+1), y_(i+1))$, its fundamental geometric properties are calculated. The coordinate system is defined such that the x-axis is parallel to the build plate ("Horizontal") and the y-axis is parallel to the build direction (vertical).

Segment Length ($L(S(i))$): The Euclidean distance of the segment, used for aggregating the total length of each segment class.
$ L(S(i)) = sqrt((x_(i+1) - x_i)^2 + (y_(i+1) - y_i)^2) $

Segment Angle ($theta_i$): The angle of the segment relative to the horizontal build plate is the primary feature for classification.
$ theta_i = op("atan2")(y_(i+1) - y_i, x_(i+1) - x_i) $


=== Classification Ruleset
The classification of each segment $S_i$ is based on physically-grounded angular thresholds derived from the FFF process. Here we recognise two key thresholds:
- $theta_H$ (Horizontal Tolerance): A small angle ($5^degree$ to $10^degree$) to account for process variability and deviation from a perfect horizontal.
- $theta_C$ (Critical Overhang Angle): The physical threshold for self-support repose angle formed in process, ( $45^degree$). Angles shallower than this are "overhangs," while angles steeper are "self-supporting".

#linebreak()
Using the calculated angle $theta_i$ (in degrees) for each segment, we apply the following ruleset to directly classify each segment according to the classes specified in the research outline:

1. Adhesion_Interface_Closing
$|theta_i| > theta_C$ ($|theta_i| > 30^degree$)

Steep side-wall of the interlocking tooth structure, formed where the extruded line closes the seam. This surface provides lateral constraint and contributes primarily to interfacial shear strength in the seam region.

2. Adhesion_Interface_Opening_Upper
$theta_H < theta_i <= theta_C$ ($5^degree < theta_i <= 30^degree$)

Steep upper surface of overhanging traces that start at the seam, defined by the angle of repose and extra extrusion (or overflow) amount. It is a primary surface for mechanical interlocking against vertical pull-out when horizontal bonding does not form.

3. Adhesion_Interface_Opening_Lower
$-theta_C <= theta_i < -theta_H$ ($-30^degree <= theta_i < -5^degree$)

Low-angle lower overhang. This segment rests on the opposing material from the previous layer and slightly drops to the sides, forming the upper "hook" of the mechanical interlock, also contributes to mechanical interlocking when formed on (2) of another material.

4. Horizontal_Interlock_Surface
$|theta_i| <= theta_H$ ($|theta_i| <= 5^degree$)

Near horizontal segments, representing the flat top or bottom of an interlockingly weaved section. This is the primary site for thermal diffusion bonding as interlocking length increases and contributes to both vertical tear strength and lateral pulling strength.

5. Void_Volume

Regions of un-filled space or porosity within the material, supposedly be identified during the material masking step. Void formation is typically attributed to insufficient extrusion volume or local bonding temperature at the starting points of "opening" traces, which can lead to incomplete seam closure, resulting in defected parts.



#linebreak()

=== Derivation of Segmental Parameters
Key parameters are the aggregate lengths of each segment class are used to count the total length for each types of segments:

- Total Horizontal Length ($L_("H")$):

  $ L_("H") = sum L(S_4(i)) $
  $forall S_4(i) in "Horizontal_Interlock_Surface" $
#linebreak()

- Total Inclined Segment Length ($L_("I")$)

  $ L_("I") = [sum L(S_1(i)) + sum L(S_2(i))] $

  $forall S_1(i) in "Adhesion_Interface_Closing" $

  $forall S_2(i) in "Adhesion_Interface_Opening_Upper" $
#linebreak()

- Total Transition Length ($L_("T")$)

  $ L_("T") = sum L(S_3(i)) $

  $forall S_3(i) in "Adhesion_Interface_Opening_Lower"$

These three parameters quantify the geometry of the microstructure based on the specified classes. $L_("H")$ represents the total length of horizontal bonding patches, which are integral to the designed interlocking patterns within the input geometry. $L_("I")$ denotes the sum of inclined segment lengths, encompassing both the side-walls of "closing" trace and upper "opening" traces connected in ideal situation. $L_"T"$ records the total length of transition segments, specifically the low-angle lower overhangs where starting lines partially embed into the opposing material from the previous layer, gradually developing to horizontal bonding.

#linebreak()

== Derivation of Aggregate Geometric Descriptors for PSP Modeling
With the segmental parameters and the full polyline from image processing, we can compute the high-level aggregate geometric descriptors specified in the research outline.

=== Descriptor 1: Bonding Surface Coefficient ($k_s$)

Quantifies the total increase in interfacial contact area created by the topology, relative to a perfectly flat, projected interface. This is a common metric in surface topography analysis.

Total Interface Arc Length: $L_("total") = L_("H") + L_("I") + L_("T")$

Projected Vertical height: $L_("projected") = |x_n - x_0|$ 

(the vertical distance between the start and end of the interface polyline $L$).

$ k_s = L_("total") / L_("projected") $

A perfectly flat, horizontal interface has $k_s = 1.0$. The "toothy" interface topology in Image will have $k_s > 1.15$, based on ideal triangular shape.

Hypothesis: $k_s$ serves as a proxy for the total potential for diffusion bonding. This variable will be a strong predictor for interfacial strength, particularly in pull and shear.

#linebreak()


=== Descriptor 2: Average Mechanical Interlocking Depth ($d_"inter"$)

Measures the average peak-to-valley horizontal amplitude of the interlocking topology at microscale.

Identify all vertex groups $F_i$ corresponding to a single interlocking feature (i.., one "peak" or one "valley"). Let there be $N_"inter"$ such features. For each feature $F_i$, collect its set of x-coordinates: $X_i = [x_j, dots, x_k]$ for all vertices $P$ in $F_i$. 

Calculate the horizontal depth $d_i$ for each feature: $d_i = max(X_i) - min(X_i)$. The descriptor is the average of these individual feature depths:

$ d_"inter" = (1 / N_"inter") sum_(i=1)^(N_"inter") d_i $

Hypothesis: $d_"inter"$ indicates the *average* depth of the mechanical interlock. It is a primary driver of Z-axis pull-out resistance. A deeper average interlock depth ($d_"inter"$) should provide greater and more consistent mechanical arresting of crack propagation. 

#linebreak()

=== Descriptor 3: Volumetric Interlock Ratio ($R_"VI"$)

Volumetric interlock ratio is designed to quantify the "geometric overlap" and "interlocking distance". It measures the 2D cross-sectional area of material that is geometrically "locked" within the convex hull of the opposing material.

Formulation:
+ Establish a vertical baseline $B$ that represents the "average" interface plane. A robust definition for $B$ is the line $x = x_("base")$, where $x_("base") = 1/(n+1) sum_(i=0)^(n) x_i$ (the mean x-coordinate of all vertices).
+ The interface polyline $L$ will weave across this baseline $B$. The regions enclosed by $L$ and $B$ represent the interlocking "teeth" and "valleys".
+ The Shoelace Formula (Gauss's area formula) is the standard and most efficient algorithm for calculating the area of these enclosed polygons from their vertices.
  For a closed polygon with $k$ vertices $(x_1, y_1), dots, (x_k, y_k)$ ordered counter-clockwise, the area $A$ is:
  $ A = 1/2 | sum_(i=1)^(k) (x_i y_(i+1) - x_(i+1) y_i) | $
  (where $(x_(k+1), y_(k+1)) = (x_1, y_1)$).


Find all intersection points between the polyline $L$ and the baseline $B$. Use these intersection points and the original vertices of $L$ to define the set of $k$ closed, simple polygons $"Poly"_1, dots, "Poly"_k$ that represent the interlocking regions.

Calculate the area $A_j = "Area"("Poly"_j)$ for each polygon using the Shoelace formula. Total Interlock Area: $A_("interlock") = sum_(j=1)^(k) A_j$. Bounding Box Area: $A_("bbox") = L_("projected") times d_"inter"$.

$ R_"IV" = A_("interlock") / A_("bbox") $

For instance, a deep, thin spike (high $d_"inter"$, low $R_"IV"$) would be mechanically weak and fracture easily. A deep, wide spike (high $d_"inter"$, high $R_"IV"$) would be far stronger. $d_"inter"$ captures the amplitude of the interlock, while $R_"IV"$ captures its shape, volume, and efficiency.

Hypothesis: $R_"IV"$ is a more complete descriptor of mechanical interlocking than $d_"inter"$ alone. It will be one of the strongest predictors of tensile strength, as it directly models the theoretical interlocking distance and volume of material engaged in the connection.

#pagebreak()

= Predictive Model for Interfacial Strength (WIP)

Synthesize all previous components into the final Process-Structure-Property (PSP) model, which will predict the Tensile_Strength using both the controllable Process parameters and the derived Structure descriptors.

== PSP Model Variable Definitions

A predictive model requires a well-defined set of independent (predictor) and dependent (response) variables. Table below serves as the data dictionary for experiments, mapping the inputs and outputs and model training.

#figure(
  table(
    columns: 4,
    align: (left, left, center, left),
    table.header(
      [*Category*], [*Variable*], [*Symbol*], [*Definition / Formulation*]
    ),
    // Process Group
    table.cell(rowspan: 5)[*Process*\ (Predictor)],
    [Print Speed], [$v_p$], [Nozzle travel speed (mm/s).],
    [Bonding Interval], [$t_b$], [Time between adjacent material depositions (s).],
    [Overlap Distance], [$d_o$], [Programmed horizontal overlap (mm).],
    [Extrusion Multiplier], [$E_m$], [Volumetric extrusion factor (%).],
    [Cooling Rate], [$d T / d t$], [Measured cooling rate at the interface (K/s).],

    // Structure (Aggregate) Group
    table.cell(rowspan: 3)[*Structure*\ (Aggregate Predictor)],
    [Surface Area Enhance. Factor], [$k_s$], [$L_("total") / L_("projected")$],
    [Mechanical Interlocking Depth], [$d_"inter"$], [$max(Y) - min(Y)$],
    [Volumetric (Areal) Interlock Ratio], [$R_"IV"$], [$A_("interlock") / A_("bbox")$],

    // Structure (Segmental) Group
    table.cell(rowspan: 4)[*Structure*\ (Segmental Predictor)],
    [Total Horizontal Length], [$L_("Horizontal")$], [$sum L(S(i))$ for Horizontal class.],
    [Total Closing Length], [$L_("Closing")$], [$sum L(S(i))$ for Closing class.],
    [Total Upper Opening Length], [$L_("Opening, U")$], [$sum L(S(i))$ for Opening\_Upper class.],
    [Total Lower Opening Length], [$L_("Opening, L")$], [$sum L(S(i))$ for Opening\_Lower class.],

    // Property Group
    table.cell(rowspan: 1)[*Property*\ (Response)],
    [Tensile Strength], [$sigma_T$], [Max tensile stress at failure (MPa).],
  ),
  caption: [Table 1. Process-Structure-Property (PSP) Model Variable Definitions]
)


//---


=== ML Algorithm

Standard Multivariate Linear Regression (MLR) is a valid starting point, but the physical relationships in FFF are known to be highly complex and non-linear. For example, the effect of Print_Speed heavily interacts with thermal parameters and is non-linear to bond quality.

A more powerful and appropriate approach is to use tree-based ensemble models, such as Random Forest (RF) or Gradient Boosting (XGBoost).

== Parametric Generation of Interface Patterns





= Results

== Mechanical Testing Outcomes

  Strength comparisons among optimized pattern families.

  Statistical analysis (regression) to assess process parameters' influence.

== Thermal Behavior

  Cooling rates and diffusion windows correlated to bond strength.

== Process–Structure–Strength Mapping and Result Comparison

  Visualization of relationships between pattern geometry, timing, and strength.



= Discussion

== Key Trends Interpretation 

  How pattern topology affects bonded area and mechanical interlocking.
  Trade-offs between deposition timing, overlap, and cooling.
  
== Decomposition Strategy

  Insights into geometry assignment and printer path planning for maximizing joint integrity.

#pagebreak()

= Strength-Aware Decomposition Framework (Preserved for journal work)

== Optimization Problem Formulation

Using the validated PSP model as the engine for generative design. 

Objective Function: Maximize the predicted strength along the entire seam, eliminate the weakest link.

Decision Variables: Parameters defining the decomposition seam path and the vertical interlocking geometry.

Constraints: Manufacturing limitations, including minimum bonding interval ($Delta t_"bond", min$, depend on layer printing time and printhead avoidance distance); maximum horizontal interlocking length ($(d_"inter")_max$ for neighbouring layers, depend on part geometry); minimum boundary incline angle in input geometry (to ensure sufficient bonding at the interface while providing clearance if multiple heads working on different layer).

== Algorithm Implementation

  Criteria for selecting decomposition planes based on predicted strength and print feasibility.

  nonlinear programming?

== Validation and Case Study

...

---
