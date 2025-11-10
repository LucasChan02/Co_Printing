#import "@preview/charged-ieee:0.1.4": ieee

#let Category = none

#show: ieee.with(
  title: [
  ],
  abstract: [
  ],
  authors: (
    (
      name: "Author One",
      department: [Dept. of Mechanical Engineering],
      organization: [University of Research],
      location: [City, Country],
      email: "author.one@university.edu"
    ),
  ),
  index-terms: (
    "Additive Manufacturing", "Multi-Head 3D Printing", 
    "Fused Filament Fabrication (FFF)", "Interfacial Strength", 
    "Mechanical Interlocking", "Computer Vision", 
    "Process-Structure-Property (PSP) Model", "Part Decomposition", 
    "Genetic Algorithm", "In-situ Thermal Monitoring"
  ),
  //bibliography: bibliography("refs.bib"),
)

// --- Content Goes Below ---

== Introduction

=== The Productivity Imperative and Multi-Head AM
Overview of Additive Manufacturing (AM) transitioning to serial 
production and the need for higher throughput. Introduction of 
multi-head systems, specifically those with independent XY-plane 
motion, as a key enabler for parallel processing by decomposing 
large parts.

=== The Interfacial Strength Bottleneck
While decomposition boosts productivity, it creates seams that 
are structural weak points. Discussion of the thermo-physical 
phenomena of polymer bonding (reptation, $T_g$) and how "cold 
joints" form due to insufficient thermal energy at the interface, 
leading to poor interdiffusion.

=== State-of-the-Art in Interface Engineering
Brief review of existing strategies: process parameter optimization (raster angles, layer hight, etc.), thermal management, and geometric design. Introduction of controlled mechanical interlocking as a robust method, setting the stage for the novel programmable, vertically-oriented interlocking proposed in this work.

=== Research Gap and Proposed Contribution
Current decomposition strategies optimize for time or material, 
not mechanical strength, treating interfaces as a liability. 
This research reframes the interface as a designable feature.
Contributions are:
1.  A novel method for programmable interlocking interfaces.
2.  A quantitative Process-Structure-Property (PSP) model 
    linking parameters to strength, built on thermal monitoring 
    and CV-based microstructural analysis.
3.  A "Strength-Aware Decomposition Framework" using a 
    Genetic Algorithm (GA) to optimize part decomposition 
    for maximum mechanical integrity.

== Parametric Modeling and Thermal Dynamics

=== Mathematical Parameterization of Interface Geometries
Establish a formal mathematical framework to describe interface topologies (e.g., Sinusoidal, Triangular, Square, Interlocking Features).
Define derived, geometry-agnostic descriptors for comparative analysis:
-   Surface Area Enhancement Factor (SAEF)
-   Mechanical Interlocking Depth (MID)
-   Volumetric Interlock Ratio (VIR)

=== Modeling of Thermal Dynamics at the Bonding Interface
Bond strength is governed by thermal history, specifically the 
"Interdiffusion Time Window" ($t_("interdiff")$), the duration the 
interface remains above $T_g$. This is critically influenced 
by the "Bonding Interval" ($Delta t_"bond"$)—a controllable 
parameter in multi-head systems representing the time between 
adjacent depositions at the seam.

== Experimental Methodology

=== Specimen Fabrication and Design of Experiments (DoE)
-   *Material Selection:* Polyethylene Terephthalate Glycol (PETG).
-   *Specimen Geometry:* ASTM D638 Type I ("dog-bone") with 
    the engineered interface at the gauge length center.
-   *DoE Matrix:* A full factorial experiment to systematically 
    explore the parameter space.
    -   *Factors:* Interface Pattern (Type, Amplitude, Wavelength), 
        Process ($Delta t_"bond"$, Print Speed, Extrusion Multiplier).
    -   *Responses:* Mechanical (UTS, Modulus), Thermal 
        ($t_"interdiff"$), Microstructural (Void Fraction, etc.).

=== In-situ Process Monitoring and Post-Process Mechanical Testing
-   *Thermal Imaging:* Use of a high-resolution IR camera to 
    record thermal video of interface formation, allowing for 
    empirical calculation of $t_"interdiff"$.
-   *Mechanical Testing:* Uniaxial tensile testing (ASTM D638) 
    on a universal testing machine to acquire stress-strain 
    curves and determine UTS, Modulus, and Elongation at Break.
-   *Microstructural Analysis:* Sectioning, polishing, and 
    microscopic imaging (Optical/SEM) of interfacial zones 
    to provide visual data for segmentation.

=== Automated Interfacial Feature Segmentation via Deep Learning
To overcome manual analysis bias, a U-Net convolutional 
neural network (CNN) is developed for objective, high-throughput 
semantic segmentation of microscopic images.
-   *Segmentation Classes:*
    1.  Adhesion_Interface_Closing
    2.  Adhesion_Interface_Opening_Upper
    3.  Adhesion_Interface_Opening_Lower
    4.  Horizontal_Interlock_Surface
    5.  Void/Porosity
-   *Workflow:* Data Annotation (ground truth), Model Training 
    (with augmentation), Validation (using mean 
    Intersection-over-Union, mIoU), and Deployment.

== Development of a Predictive Process-Structure-Property (PSP) Model
The analytical core synthesizing all experimental data into a 
quantitative, causal chain.

=== Correlating Process Parameters to Interfacial Structure (Process -> Structure)
A multivariate regression model mapping controllable DoE inputs 
(e.g., Amplitude, $Delta t_"bond"$) to quantitative structural 
metrics from the U-Net (e.g., Void_Area_Fraction, 
Area_Horizontal_Interlock_Surface).

=== Mapping Interfacial Structure to Mechanical Performance (Structure -> Property)
A second regression model linking the measured structural 
features and thermal data (e.g., Void Fraction, $t_"interdiff"$) 
to the final experimentally measured Ultimate Tensile Strength (UTS). 
This model reveals the "strength contribution" of each 
microstructural feature.

=== The Integrated PSP Framework and Model Validation
Combining the two stages into a single framework that predicts 
UTS directly from the initial process parameters. The model is 
validated using a hold-out test set, with performance evaluated 
by $R^2$ and Mean Absolute Percentage Error (MAPE).

== A Strength-Aware Decomposition Framework for Multi-Head AM

=== Optimization Problem Formulation
Using the validated PSP model as a generative design engine.
-   *Objective Function:* Maximize the minimum predicted 
    strength along the entire seam (a "max-min" approach) 
    to eliminate the weakest link.
-   *Decision Variables:* Parameters defining the decomposition 
    seam path and the local interlocking geometry (Amplitude, 
    Wavelength) along that path.
-   *Constraints:* Manufacturing limitations (e.g., 
    $Delta t_"bond", min$, $("MID")_max$, minimum part incline 
    angle, assemblability).

=== Algorithmic Implementation using a Genetic Algorithm (GA)
A GA is used to solve the complex, non-linear optimization 
problem.
-   *Chromosome Encoding:* A numerical string representing 
    a complete decomposition solution (seam path + interface 
    geometry).
-   *Fitness Function:* The PSP model is used to evaluate 
    the fitness of each "chromosome" based on its 
    predicted minimum seam strength.
-   *Genetic Operators:* Selection, Crossover, and Mutation 
    evolve the population toward an optimal solution.

=== Case Study and Validation
-   *Benchmark Geometry:* A complex, representative 3D 
    model (e.g., topologically optimized bracket) is selected.
-   *Computational Demonstration:* The GA framework is 
    run to generate an optimized decomposition path and 
    variable interface geometry.
-   *Experimental Validation:*
    -   *Version A (Baseline):* Fabricated with a simple, 
        non-optimized decomposition.
    -   *Version B (Optimized):* Fabricated using the 
        GA-generated solution.
    -   Both versions are mechanically tested to failure. 
        The hypothesis is that Version B will show a 
        significantly higher failure load and fail in 
        the bulk material, not at the interface.

== Conclusion and Future Work

=== Summary of Contributions
Recap of the novel programmable interlocking interface, 
the automated U-Net microstructural analysis pipeline, 
the validated PSP model, and the generative, 
GA-based "Strength-Aware Decomposition Framework" that 
engineers interfaces to be stronger than the parent material.

=== Discussion and Implications
This framework overcomes a critical barrier for multi-head AM, 
enabling the fabrication of large, functional, load-bearing 
parts. It represents a paradigm shift from treating interfaces 
as a problem to an opportunity for localized material 
engineering.

=== Limitations and Future Work
-   *Limitations:* Study confined to one material (PETG) and 
    one loading condition (quasi-static tensile).
-   *Future Work:*
    -   *Material Expansion:* Extend to high-performance 
        polymers, composites, and dissimilar materials.
    -   *Multi-Objective Optimization:* Use NSGA-II to 
        simultaneously optimize for strength, print time, 
        and surface finish.
    -   *Advanced Loading Conditions:* Investigate fatigue, 
        shear, and impact strength.
    -   *Closed-Loop Process Control:* Use in-situ thermal 
        data for real-time, dynamic adjustment of print 
        parameters to ensure optimal bond quality.




From samples

acquire the images (side view of connections) of multiple speciments printed under a grid of parameters

研究的创新点在于, 着眼于一种仅在 多打印头独立XY驱动, 但共用Z桁架的gantry形式打印机中 便于实现的层间结合方式, 允许多个打印头同时work on same workpiece 并在打印层切换时, 通过控制打印头在结合界面的停留时机, 制造出垂直方向的啮合结构. 

首先制备多种不同几何形状接合面的cad模型, 用不同接合参数打印. 在显微镜下放大到层的尺度, 拍摄锯齿状的啮合结构. 获取的图像将被用于训练计算机视觉模型CV, 从而可以自动化分段标注层和层,材料和材料间的啮合界面, 并分类统计分段的长度. 

分段考虑有以下几种: 1. 方向朝向接缝的打印线(一层末端)和另一材料打印线的结合斜面, 2. 方向离开接缝的打印线(一层起始)和另一材料的结合边界上半部斜面, 3. 方向离开接缝的打印线(一层起始)和另一材料的结合边界下半部斜面, 4. 随着结合界面斜率增加或者是interlocking互锁长度增加, 产生的水平结合面 5. 因一层起始处材料堆积不足造成的空穴

这个分段模型将会被用数学方法予以描述, 

可变的参数有接合面曲线的局部斜率, 互锁结构的分段数(同样意义上是水平互锁结合面的数量), 以及互锁长度. zigzag的互锁结构, 从层的尺度来看可以视作一种使得结合表面面积增大\\结合更紧密的programmable扰动/噪声. 这些参数可以被用作打印路径规划的输出结果.

需要控制的变量有, 接合的时间间隔(需要一定的残余温度才能保证高强度的结合), 一层结束前瞬间的提前挤出滑行量, 换层时的回抽量, 新一层起始时的加压额外挤出量prime pressure. 这些变量会影响每个个体结合斜面的倾角. 当然, 如果要建立更精确的模型, 喷嘴处材料的热动力学, 已沉积材料的冷却环境等因素都需要被纳入考量

我们获得这个分段接合面模型之后, 便可以利用现存的其他研究成果, 通过控制打印时双材料的结合时间间隔微调熔融材料的结合强度, 通过调整参数化的3d模型改变分段接合面的各组长度和角度等参数. 最终制备多批不同参数组合的模型, 在测试台上测量拉伸强度, 验证模型能够准确描述围观结构.

// ---

=== techniques to consider

GANN 
one-shot Learning


// ---

// Set up the document structure.
#set document(title: "A Computational Framework for Interfacial Topology in Multi-Material Additive Manufacturing")
#set heading(numbering: "I.")
#show heading.where(level: 2): set heading(numbering: "1.1")
#show heading.where(level: 3): set heading(numbering: "1.1.1")

= A Computational Framework for Interfacial Topology in Multi-Material Additive Manufacturing

== Context: The Process-Structure-Property (PSP) Paradigm in Additive Manufacturing

[cite_start]This report is situated within the Process-Structure-Property (PSP) paradigm, a foundational concept in materials science and engineering.[cite: 1] The core objective of the associated research is to establish a quantitative model linking manufacturing *Process* parameters (e.g., print speed, extrusion multiplier, overlap distance) to the resultant mechanical *Property* (e.g., interfacial tensile strength).

This document provides the critical missing component: a robust mathematical formulation for the intervening *Structure*.

[cite_start]In the context of multi-material Fused Filament Fabrication (FFF), the interface (as depicted in Image 2) is not a simple 2D boundary but a complex, 3D structure engineered with a specific topology.[cite: 4] The objective of this report is to develop the computational framework required to parameterize and quantify this interfacial structure.

== The Research Problem: Quantifying the "Strength-Aware" Interface

[cite_start]The research outline [cite: 6] identifies a "Lack of understanding of how decomposition strategies affect interfacial strength from geometric and process parameters."

[cite_start]This is a well-known challenge; parts fabricated via multi-material FFF are often weakest at the interface between dissimilar materials, a result of poor chemical adhesion (thermal bonding) and residual stresses.[cite: 7]

[cite_start]The "Programmed Interlocking Interfacial Topologies" proposed in the research [cite: 6] represent a strategy to mitigate this weakness.

[cite_start]The "teeth-like" structures, visible in the micrograph (Image 2), are designed to supplement (or replace) weak chemical bonding with robust mechanical keying.[cite: 9]

[cite_start]The core research gap, therefore, is the transition from a qualitative design ("triangular wave" vs. "sine wave" [cite: 6]) to a quantitative structural model.

The strength of the interface is not monolithic; it is a complex function of its constituent segments.

For example, a horizontal segment's contribution to strength is governed by thermal diffusion, while an angled, "overhanging" segment's contribution is governed by mechanical interlocking.

A model is required to de-couple and quantify these distinct contributions, providing the feature set for the final PSP model.

This report provides the mathematical basis for this de-coupling and feature extraction.

= Data Acquisition and Formulation of the Interfacial Polyline

== From Micrograph to Mask: Semantic Segmentation of the Interface

The analysis pipeline originates with the physical cross-section, captured via optical microscopy (Image 2).

[cite_start]The research outline [cite: 6] correctly specifies a "U-Net CNN... for objective, high-throughput semantic segmentation" to automate the analysis and eliminate manual bias.

The primary role of this U-Net should be to perform a robust, region-based segmentation rather than a direct, pixel-level classification of the final segment types.

The proposed segmentation classes should be simplified to:
- Material A (e.g., the beige material)
- Material B (e.g., the green material)
- Void/Porosity

[cite_start]The geometric classifications sought by the user (e.g., Adhesion_Interface_Opening_Upper, Horizontal_Interlock_Surface [cite: 6]) are not intrinsic pixel-level properties.

They are geometric properties of the boundary vector between the segmented Material A and Material B regions.

This workflow simplifies the deep learning task—making it more robust and accurate—and grounds the final classification in precise computational geometry rather than probabilistic pixel classification.

The U-Net finds the interface; the model in Section III analyzes it.

== From Mask to Vector: Contour Extraction

Given the binary (or multi-class) segmentation mask from the U-Net, the next step is to vectorize the 2D boundary between Material A and Material B. This process converts the raster mask into a vector polyline (as conceptualized in Image 1).

[cite_start]This is a standard operation in computer vision.[cite: 11] [cite_start]The recommended algorithm is `cv2.findContours()` (or an equivalent), widely available in image processing libraries such as OpenCV.[cite: 12] This function traces the boundary of the segmented regions and outputs a raw, ordered list of vertex coordinates, $L_("raw")= {(x_0, y_0), (x_1, y_1), ... }$, that defines the interface.

== Polyline Simplification and Segment Definition

The raw contour $L_"raw"$ is over-specified, noisy, and biased by the pixel grid of the sensor.

It must be simplified to recover the intended geometric segments of the programmed topology.

[cite_start]The canonical algorithm for this task is the Ramer-Douglas-Peucker (RDP) algorithm.[cite: 15] [cite_start]The RDP algorithm decimates a polyline by recursively removing vertices that are within a specified perpendicular distance, $epsilon$, from a line segment connecting its endpoints.[cite: 15]

The output of the RDP algorithm is the final, simplified interfacial polyline: $L = {P_0, P_1, ..., P_n}$, where each $P_i = (x_i, y_i)$ is a vertex.

A segment $S_i$ is formally defined as the straight line connecting the vertices $P_i$ and $P_(i+1)$.

The $epsilon$ tolerance for the RDP algorithm is a critical model hyperparameter.

It must be tuned to match the physical scale of the investigation.

A small $epsilon$ will be sensitive to microscopic surface roughness and printing artifacts, generating many short, noisy segments.

A large $epsilon$ will smooth over these features and capture only the primary topology (e.g., reducing the path in Image 1 to a perfect, simple triangular wave).

For this application, $epsilon$ should be selected to be large enough to filter pixel-level noise but small enough to preserve the distinct vertices of the programmed interlocking "teeth."

= A Mathematical Model for the Classification of Interfacial Boundary Segments

This section provides the core mathematical model for classifying each segment $S_i$ of the simplified polyline $L$, directly addressing the query to "differentiate different types of segments."

== Local Segment Property Calculation

For each segment $S_i$, defined by its start and end vertices $P_i(x_i, y_i)$ and $P_(i+1)(x_(i+1), y_(i+1))$, its fundamental geometric properties are calculated.

The coordinate system is defined such that the x-axis is parallel to the build plate (horizontal) and the y-axis is parallel to the build direction (vertical).

*Segment Length ($L(S_i)$):* The Euclidean distance of the segment.
$L(S_i) = sqrt((x_(i+1) - x_i)^2 + (y_(i+1) - y_i)^2)$
[cite_start]This value is used for aggregating the total length of each segment class.[cite: 18]

*Segment Angle ($theta_i$):* The angle of the segment relative to the horizontal build plate.
$theta_i = arctan(y_(i+1) - y_i, x_(i+1) - x_i)$
The use of the two-argument arctangent ($"atan2"$) is critical.

It correctly computes the angle in all four quadrants, producing an unambiguous result in radians from $-pi$ to $pi$ ($-180^degree$ to $180^degree$).

This angle $theta_i$ is the primary feature for classification.

== A Physically-Grounded Threshold Model

[cite_start]The segment classifications defined in the research outline [cite: 6] (Horizontal_Interlock_Surface, Adhesion_Interface_Opening, Adhesion_Interface_Closing) are not arbitrary.

They map directly to distinct physical phenomena in the FFF process.

[cite_start]*Horizontal Segments:* These represent surfaces parallel to the build plate, where bonding is dominated by thermal diffusion between layers (inter-layer adhesion).[cite: 20]

*Angled Segments:* These represent overhangs and side-walls. The physical behavior of a deposited bead on an angled surface is dictated by the self-supporting angle.

[cite_start]In FFF, it is widely cited that overhang angles up to $45^degree$ (measured from the horizontal) can be printed without significant drooping or failure.[cite: 22] [cite_start]Angles steeper than $45^degree$ are "self-supporting," while angles shallower than $45^degree$ are "overhangs" that require support.[cite: 24]

This $45^degree$ rule provides the physical basis for differentiating the segments.

The Adhesion_Interface_Opening segments are the "hooks" of the interlock—these are the low-angle surfaces that rest on the opposing material and would fail if printed alone.

The Adhesion_Interface_Closing segments are the steep, self-supporting side-walls of the "teeth."

This allows the definition of a formal classification ruleset based on two key angular thresholds:
- $theta_H$ (Horizontal Tolerance): A small angle (e.g., $5^degree$ or $10^degree$) to account for process variability.
- Segments with an angle below this are considered "horizontal."
- $theta_C$ (Critical Overhang Angle): The physical threshold for self-support, (e.g., $45^degree$).

== Formal Classification Ruleset

For any given segment $S_i$, its absolute angle in degrees, $theta."i.deg" = |"degrees"(theta_i)|$, is used for classification.

=== Class 1: Horizontal_Interlock_Surface
*Mathematical Rule:* $theta."i.deg" <= theta_H$ (e.g., $"<= 10"^degree$)
*Physical Interpretation:* A (near) horizontal segment, representing the flat top or bottom of an interlocking "tooth" (see Image 2).
*Hypothesized Strength Contribution:* This surface area is the primary site for thermal diffusion bonding (chemical adhesion) between the two materials.

=== Class 2: Adhesion_Interface_Opening (Overhang Surface)
*Mathematical Rule:* $theta_H < theta."i.deg" <= theta_C$ (e.g., $10^degree < theta."i.deg" <= 45^degree$)
*Physical Interpretation:* A low-angle overhang. [cite_start]This segment rests on the previously deposited material, forming the "hook" or "dovetail" [cite: 5] of the mechanical interlock.
*Hypothesized Strength Contribution:* This is the primary surface responsible for mechanical interlocking against Z-axis (vertical) tensile pull-out.
[cite_start]The Upper and Lower sub-classes [cite: 6] can be differentiated by the sign of the original $theta_i$.

=== Class 3: Adhesion_Interface_Closing (Steep/Side Contact)
*Mathematical Rule:* $theta."i.deg" > theta_C$ (e.g., $"> 45"^degree$)
*Physical Interpretation:* A steep, self-supporting side-wall of the interlocking "tooth."
*Hypothesized Strength Contribution:* This surface provides lateral constraint and contributes primarily to interfacial shear strength.

This classification model is summarized in Table 1.

#figure(
  table(
    columns: 4,
    [Segment Classification],
    [Mathematical Rule (based on $|theta_i|$)],
    [Physical Interpretation (FFF Process)],
    [Hypothesized Primary Contribution to Strength],
    [Horizontal_Interlock_Surface],
    [$|theta_i| <= theta_H$],
    [Horizontal deposition, flat top/bottom of a feature.],
    [Thermal/Diffusion Bonding: Chemical adhesion between materials.],
    [Adhesion_Interface_Opening],
    [$theta_H < |theta_i| <= theta_C$],
    [Low-angle (non-self-supporting) overhang; rests on other material.],
    [Mechanical Interlocking (Tensile): Resists Z-axis (vertical) pull-out.],
    [Adhesion_Interface_Closing],
    [$|theta_i| > theta_C$],
    [Steep-angle (self-supporting) side-wall.],
    [Mechanical Interlocking (Shear): Resists lateral (X/Y-axis) shear forces.],
  ),
  caption: [Table 1. Segment Classification Ruleset and Physical Significance]
)

= Derivation of Aggregate Geometric Descriptors for PSP Modeling

[cite_start]With each segment $S_i$ classified and its length $L(S_i)$ known, we can compute the high-level aggregate geometric descriptors specified in the research outline.[cite: 6] These descriptors distill the complex polyline $L$ into a concise set of features for the final PSP model.

== Descriptor 1: Surface Area Enhancement Factor (SAEF)
*Definition:* Quantifies the total increase in interfacial contact area created by the topology, relative to a perfectly flat, projected interface.
[cite_start]This is a common metric in surface topography analysis.[cite: 25]

*Formulation:*
- Calculate Total Interface Arc Length: $L_"total" = sum_i L(S_i)$
- Calculate Projected Horizontal Length: $L_"projected" = |x_n - x_0|$ (the horizontal distance between the start and end of the interface).
- $S A E F = frac(L_"total", L_"projected")$

*Interpretation:* A perfectly flat, horizontal interface has $S A E F = 1.0$.
The "toothy" topology in Image 2 will have $S A E F > 1.0$.
*PSP Hypothesis:* $S A E F$ serves as a proxy for the total potential for chemical/diffusion bonding.
[cite_start]It is hypothesized that this variable will be a strong predictor for interfacial strength, particularly in shear.[cite: 27]

== Descriptor 2: Mechanical Interlocking Depth (MID)
*Definition:* Measures the peak-to-valley vertical amplitude of the interlocking topology.

*Formulation:*
- Collect all y-coordinates from the vertices of $L$: $Y = {y_0, y_1, ..., y_n}$.
- $M I D = max(Y) - min(Y)$

[cite_start]*Interpretation:* This is the "amplitude" of the interface wave.[cite: 29]
[cite_start]*PSP Hypothesis:* $M I D$ directly quantifies the depth of the mechanical interlock.[cite: 31] It is hypothesized to be a primary driver of Z-axis pull-out resistance.
[cite_start]A deeper interlock should provide greater mechanical arresting of crack propagation and delamination.[cite: 9]

== Descriptor 3: Volumetric (Areal) Interlock Ratio (VIR)
[cite_start]*Definition:* This is the most sophisticated descriptor, designed to quantify the "geometric overlap" and "interlocking distance".[cite: 6] It measures the 2D cross-sectional area of material that is geometrically "locked" within the convex hull of the opposing material.

*Formulation (Proposed Algorithm):*
- *Define Baseline:* Establish a vertical baseline $B$ that represents the "average" interface plane.
A robust definition for $B$ is the line $x = x_"base"$, where $x_"base" = frac(1, n+1) sum_(i=0)^n x_i$ (the mean x-coordinate of all vertices).
- *Identify Interlocking Polygons:* The interface polyline $L$ will weave across this baseline $B$.
[cite_start]The regions enclosed by $L$ and $B$ represent the interlocking "teeth" and "valleys".[cite: 33]
- [cite_start]*Calculate Polygon Areas:* The Shoelace Formula (also known as Gauss's area formula or the Surveyor's formula) is the standard and most efficient algorithm for calculating the area of these (potentially complex) enclosed polygons from their vertices.[cite: 35]
For a closed polygon with $k$ vertices $(x_1, y_1), ..., (x_k, y_k)$ ordered counter-clockwise, the area $A$ is:
$A = frac(1, 2) abs( sum_(i=1)^k (x_i y_(i+1) - x_(i+1) y_i) )$
(where $(x_(k+1), y_(k+1)) = (x_1, y_1)$).

*VIR Calculation:*
#set enum(numbering: "a.")
+ Algorithmically find all intersection points between the polyline $L$ and the baseline $B$.
+ Use these intersection points and the original vertices of $L$ to define the set of $k$ closed, simple polygons $"Poly"_1, ..., "Poly"_k$ that represent the interlocking regions.
+ [cite_start]Calculate the area $A_j = "Area"("Poly"_j)$ for each polygon using the Shoelace formula.[cite: 35]
+ Calculate Total Interlock Area: $A_"interlock" = sum_(j=1)^k A_j$.
+ Calculate Bounding Box Area: $A_"bbox" = L_"projected" times M I D$.
+ $V I R = frac(A_"interlock", A_"bbox")$
#set enum(numbering: "1.")

[cite_start]*Interpretation:* The research outline [cite: 6] lists both MID and VIR as necessary descriptors.
This is appropriate: a deep, thin "spike" (high MID, low VIR) would be mechanically weak and fracture easily.
[cite_start]A deep, wide "dovetail" (high MID, high VIR) would be far stronger.[cite: 5] $M I D$ captures the amplitude of the interlock, while $V I R$ captures its shape, volume, and efficiency.
*PSP Hypothesis:* $V I R$ is a more complete descriptor of mechanical interlocking than $M I D$ alone.
[cite_start]It is hypothesized that $V I R$ will be one of the strongest predictors of tensile strength, as it directly models the "interlocking distance" and volume of material engaged in resisting fracture.[cite: 10]

= A Predictive Model for Interfacial Strength: Integrating Structure and Process

[cite_start]This section synthesizes all previous components into the final Process-Structure-Property (PSP) model, as specified in the research objective.[cite: 6] This model will predict the Tensile_Strength (Property) using both the controllable Process parameters and the derived Structure descriptors.

== PSP Model Variable Definitions

A robust predictive model requires a well-defined set of independent (predictor) and dependent (response) variables.
Table 2 serves as the "data dictionary" for the complete experiment, mapping the inputs and outputs of the PSP framework.

#figure(
  table(
    columns: 5,
    [Category], [Variable], [Symbol], [Definition / Formulation], [Source / Method],
    // Process
    [Process \ (Predictor)], [Print Speed], [$v_p$], [Nozzle travel speed (mm/s).], [Machine Setting [cite: 6]],
    [], [Bonding Interval], [$t_b$], [Time between adjacent material depositions (s).], [Machine Setting [cite: 6]],
    [], [Overlap Distance], [$d_o$], [Programmed horizontal overlap (mm).], [Machine Setting [cite: 6]],
    [], [Extrusion Multiplier], [$E_m$], [Volumetric extrusion factor (%).], [Machine Setting [cite: 6]],
    [], [Cooling Rate], [$frac(d T, d t)$], [Measured cooling rate at the interface (K/s).], [Thermal Imaging [cite: 6]],
    // Structure (Aggregate)
    [Structure \ (Aggregate Predictor)], [Surface Area Enhance. Factor], [$S A E F$], [$L_"total" \/ L_"projected"$], [Sec 4.1 (from Polyline)],
    [], [Mechanical Interlocking Depth], [$M I D$], [$max(Y) - min(Y)$], [Sec 4.2 (from Polyline)],
    [], [Volumetric (Areal) Interlock Ratio], [$V I R$], [$A_"interlock" \/ A_"bbox"$], [Sec 4.3 (from Polyline)],
    // Structure (Segmental)
    [Structure \ (Segmental Predictor)], [Total Horizontal Length], [$L_H$], [$sum L(S_i)$ for all $S_i$ in Horizontal class.], [Sec 3.3 (from Polyline)],
    [], [Total Opening Length], [$L_O$], [$sum L(S_i)$ for all $S_i$ in Opening class.], [Sec 3.3 (from Polyline)],
    [], [Total Closing Length], [$L_C$], [$sum L(S_i)$ for all $S_i$ in Closing class.], [Sec 3.3 (from Polyline)],
    // Property
    [Property \ (Response)], [Tensile Strength], [$sigma_T$], [Max tensile stress at failure (MPa).], [Tensile Test [cite: 6]],
  ),
  caption: [Table 2. Process-Structure-Property (PSP) Model Variable Definitions]
)

This data structure, by including both aggregate (SAEF, MID, VIR) and segmental (L_H, L_O, L_C) descriptors, allows the model to answer highly specific research questions (e.g., "Is overall strength more correlated with total horizontal bonding area, $L_H$, or the mechanical hook shape, $V I R$?").

== Recommended Analytical Model: Beyond Linear Regression

[cite_start]While standard Multivariate Linear Regression (MLR) is a valid starting point [cite: 39][cite_start], the physical relationships in FFF are known to be highly complex and non-linear.[cite: 42] For example, the effect of Print_Speed on bond quality is non-linear and heavily interacts with thermal parameters.

A more powerful and appropriate approach is to use tree-based ensemble models, such as Random Forest (RF) or Gradient Boosting (e.g., XGBoost).

*Justification:*
- [cite_start]*Non-Linearity:* These models excel at capturing complex, non-linear relationships and thresholds without requiring a priori definition of polynomial terms.[cite: 43]
- *Interaction Handling:* They automatically model high-order interaction effects between variables (e.g., the combination of high $M I D$ and a high $"Cooling"_"Rate"$ may have a unique, synergistic effect on $sigma_T$).
- *Robustness:* They are robust to multicollinearity, which is expected in this dataset (e.t., $M I D$ and $V I R$ will likely be correlated).
- [cite_start]*Proven Efficacy:* These models (RF, XGBoost) have demonstrated high accuracy (e.g., $R^2 > 0.9$) in predicting the mechanical properties of 3D printed parts from process parameters.[cite: 44]

== The Key Output: Feature Importance Analysis

[cite_start]The most valuable output of this modeling approach—directly addressing the stated "lack of understanding" [cite: 6][cite_start]—is not just its predictive accuracy, but its ability to compute Feature Importance.[cite: 45]

After training the Random Forest model on the database (defined in Table 2), it can quantitatively rank all independent variables (both Process and Structure) by their contribution to the model's predictive accuracy.

This analysis will provide a direct, quantitative answer to the central research question by identifying which features matter most.

It will reveal, for example, whether interfacial strength is dominated by the $V I R$ (a structural feature), by the $L_H$ (a bonding feature), or by a $"Bonding"_"Interval"$ (a process parameter).

This analysis provides the scientific "understanding" that the research is seeking.

= Recommendations for Implementation and Experimental Validation

== Proposed Computational Toolkit (Python-based)

This entire analytical framework can be implemented using a stack of open-source Python libraries:
- *Image Segmentation (U-Net):* PyTorch or TensorFlow/Keras.
- [cite_start]*Image Processing & Contouring:* OpenCV [cite: 11] [cite_start]and scikit-image.[cite: 48]
- [cite_start]*Computational Geometry:* Shapely.[cite: 48] This library is ideal, providing built-in, optimized functions for RDP simplification (polygon.simplify(epsilon)) and all necessary polygon operations (intersections, area calculations).
- [cite_start]*PSP Modeling (RF/XGBoost):* scikit-learn.[cite: 49] [cite_start]This is the standard for machine learning, containing robust, well-documented implementations of Random Forest, regression metrics, and feature importance analysis.[cite: 44]
- [cite_start]*Advanced Shape Analysis:* For future work, scikit-shapes [cite: 50] could be explored for more advanced 3D shape registration and analysis.

== Experimental Validation Protocol

The following protocol details the research loop required to build and validate the proposed PSP model:
+ *Fabricate (Process):* Define a Design of Experiments (DoE). [cite_start]Fabricate tensile specimens [cite: 6] by systematically varying both the programmed topology (e.g., triangular, sine, square, varying amplitude and wavelength) and the key Process parameters (Overlap_Distance, Bonding_Interval, Print_Speed).
+ [cite_start]*Test (Property):* Conduct standard tensile tests (e.g., ASTM D638) on all samples to acquire the Tensile_Strength ($sigma_T$) response variable.[cite: 7]
+ [cite_start]*Image (Structure):* For each tested sample, perform cross-sectioning and microscopic imaging (as in Image 2) to capture the as-built interfacial topology.[cite: 52]
+ *Extract (Structure):* Apply the U-Net segmentation (Sec 2.1), `cv2.findContours` (Sec 2.2), and RDP simplification (Sec 2.3) to all micrographs to generate the simplified polylines ($L$).
+ *Calculate (Structure):* Batch-process all polylines using the classification model (Sec III) and aggregate descriptor calculations (Sec IV) to compute the full set of Structure features.
+ *Build Database:* Collate all Process, Structure, and Property data into a single master table (as defined in Table 2).
+ *Train & Analyze (PSP):* Train the Random Forest (or XGBoost) model (Sec V) on the database to predict $sigma_T$.
+ *Analyze Results:* Generate the Feature Importance plot (Sec 5.3) to identify the key drivers of interfacial strength.

== Final Conclusion: Enabling the "Strength-Aware Decomposition Framework"

[cite_start]The ultimate goal of the research is a "Strength-Aware Decomposition Framework" that utilizes a "Generative Algorithm (GA or PSO or MCTS)".[cite: 6] Any such optimization algorithm requires a quantitative objective function (or "fitness function") to optimize.

[cite_start]The "Problem Statement" [cite: 6] implies this function is currently unknown.

The predictive PSP model developed in this report *is* that objective function.

The Feature Importance analysis (Sec 5.3) will identify the most critical geometric parameters (e.g., $V I R$ and $L_H$) that govern strength.

The trained Random Forest model itself can serve as the fitness function.

The Generative Algorithm can then be programmed to search the vast design space of possible decomposition topologies.

For each candidate topology, it will:
- Generate the 2D interface polyline.
- Calculate the Structure descriptors (SAEF, MID, VIR, etc.).
- Feed these descriptors into the trained PSP model to predict its $sigma_T$.

[cite_start]The GA's fitness function is to maximize this predicted $sigma_T$, subject to the manufacturability constraints (e.g., "minimum bonding interval," "maximum horizontal interlocking length" [cite: 6]).

This framework provides the foundational quantitative language (the descriptors) and the evaluative logic (the PSP model) required for the "Strength-Aware Decomposition Framework" to function.

It moves the research from a pattern-based approach ("triangles are strong") to a feature-based, predictive, and optimized paradigm.

#heading(numbering: none, "Works cited")
- Process-Structure-Properties-Performance Modeling for Selective Laser Melting - MDPI, accessed on November 9, 2025, https://www.mdpi.com/2075-4701/9/11/1138
- Linking Process, Structure, and Property in Additive Manufacturing Applications through Advanced Materials Modelling - UPCommons, accessed on November 9, 2025, https://upcommons.upc.edu/bitstreams/d4075efd-352e-46ed-bcba-233174587837/download
- (PDF) Modeling process-structure-property relationships for additive manufacturing, accessed on November 9, 2025, https://www.researchgate.net/publication/323128111_Modeling_process-structure-property_relationships_for_additive_manufacturing
- Lattice Structures—Mechanical Description with Respect to Additive Manufacturing, accessed on November 9, 2025, https://www.researchgate.net/publication/385450672_Lattice_Structures-Mechanical_Description_with_Respect_to_Additive_Manufacturing
- Mechanical Behaviour of Macroscopic Interfaces for 3D Printed Multi-material Samples, accessed on November 9, 2025, https://www.matec-conferences.org/articles/matecconf/pdf/2022/15/matecconf_newtech22_01004.pdf
- Research_outline.txt
- Investigation of Interlayer Interface Strength and Print Morphology Effects in Fused Deposition Modeling 3D-Printed PLA - PMC - NIH, accessed on November 9, 2025, https://pmc.ncbi.nlm.nih.gov/articles/PMC9828590/
- Effect of Process Parameters on the Performance of Drop-On-Demand 3D Inkjet Printing: Geometrical-Based Modeling and Experimental Validation - MDPI, accessed on November 9, 2025, https://www.mdpi.com/2073-4360/14/13/2557
- Cooperative enhancement of multi-material interface strength by mechanical interlocking structures and FDM path planning |
- Request PDF - ResearchGate, accessed on November 9, 2025, https://www.researchgate.net/publication/383877988_Cooperative_enhancement_of_multi-material_interface_strength_by_mechanical_interlocking_structures_and_FDM_path_planning
- Printing 3D objects with interlocking parts - ResearchGate, accessed on November 9, 2025, https://www.researchgate.net/publication/275058983_Printing_3D_objects_with_interlocking_parts
- Find Co-ordinates of Contours using OpenCV | Python - GeeksforGeeks, accessed on November 9, 2025, https://www.geeksforgeeks.org/python/find-co-ordinates-of-contours-using-opencv-python/
- Position coordinates on geometric image - Python - OpenCV Forum, accessed on November 9, 2025, https://forum.opencv.org/t/position-coordinates-on-geometric-image/7488
- Python OpenCV - get coordinates of borders of elements inside an image - Stack Overflow, accessed on November 9, 2025, https://stackoverflow.com/questions/52193545/python-opencv-get-coordinates-of-borders-of-elements-inside-an-image
- Image segmentation mask to polygon for coco json - Stack Overflow, accessed on November 9, 2025, https://stackoverflow.com/questions/68663512/image-segmentation-mask-to-polygon-for-coco-json
- Ramer–Douglas–Peucker algorithm - Wikipedia, accessed on November 9, 2025, https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
- Simplify Line (Cartography)—ArcGIS Pro | Documentation, accessed on November 9, 2025, https://pro.arcgis.com/en/pro-app/latest/tool-reference/cartography/simplify-line.htm
- Polyline Simplification - Matthew Deutsch, accessed on November 9, 2025, http://matthewdeutsch.com/projects/polyline-simplification/
- Evaluating segment, part/path, and polyline lengths - ArcMap Resources for ArcGIS Desktop, accessed on November 9, 2025, https://desktop.arcgis.com/en/arcmap/latest/extensions/data-reviewer/evaluating-segment-part-path-and-polyline-lengths.htm
- Calculating angle between polyline segments and assign this value to point?, accessed on November 9, 2025, https://gis.stackexchange.com/questions/184190/calculating-angle-between-polyline-segments-and-assign-this-value-to-point
- Why Part Orientation Matters in 3D Printing - AZoM, accessed on November 9, 2025, https://www.azom.com/article.aspx?ArticleID=24438
- Model orientation best practices for SLA printing - Support | Formlabs, accessed on November 9, 2025, https://support.formlabs.com/s/article/Model-Orientation
- Understanding 3D Printing Overhangs: Support, Angles, and Material Effects, accessed on November 9, 2025, https://www.3dmag.com/3d-wikipedia/3d-printing-overhang-support-angles-materials/
- How to 3D Print Overhangs - All3DP, accessed on November 9, 2025, https://all3dp.com/2/3d-printing-overhang-how-to-master-overhangs-exceeding-45/
- Practical overhang limits - 3D Printing Stack Exchange, accessed on November 9, 2025, https://3dprinting.stackexchange.com/questions/19225/practical-overhang-limits
- Surface topography measurement: 2D profile measurements - Wahyudin Syam, accessed on November 9, 2025, https://www.wasyresearch.com/surface-topography-measurement-2d-profile-measurements/
- How to estimate the increase in surface area from average roughness value and projected area? |
- ResearchGate, accessed on November 9, 2025, https://www.researchgate.net/post/How_to_estimate_the_increase_in_surface_area_from_average_roughness_value_and_projected_area
- Investigating the Bond Strength of FRP Rebars in Concrete under High Temperature Using Gene-Expression Programming Model - MDPI, accessed on November 9, 2025, https://www.mdpi.com/2073-4360/14/15/2992
- A Hybrid SVR-Based Prediction Model for the Interfacial Bond Strength of Externally Bonded FRP Laminates on Grooves with Concrete Prisms - PMC - NIH, accessed on November 9, 2025, https://pmc.ncbi.nlm.nih.gov/articles/PMC9370787/
- Triangle wave - Wikipedia, accessed on November 9, 2025, https://en.wikipedia.org/wiki/Triangle_wave
- Formula for a Triangle Wave : r/askmath - Reddit, accessed on November 9, 2025, https://www.reddit.com/r/askmath/comments/ojhsm5/formula_for_a_triangle_wave/
- Full article: Enhanced geometrical control in cold spray additive manufacturing through deep neural network predictive models - Taylor & Francis Online, accessed on November 9, 2025, https://www.tandfonline.com/doi/full/10.1080/17452759.2025.2472388?af=R
- Hybrid additive and subtractive manufacturing of large-scale and multi-material parts by Eric Weflen A dissertation submitted to - Department of Industrial and Manufacturing Systems Engineering - Iowa State University, accessed on November 9, 2025, https://www.imse.iastate.edu/files/2024/07/Weflen_iastate_0097E_21303.pdf
- AutoCAD 2024 Help | About Finding Area and Mass Properties Information | Autodesk, accessed on November 9, 2025, https://help.autodesk.com/view/ACD/2024/ENU/?guid=GUID-0BCD5F58-D932-4503-9F0E-C8FDCDE6A3E3
- Calculating the total area from two intersected polylines using ArcGIS or Python, accessed on November 9, 2025, https://gis.stackexchange.com/questions/316149/calculating-the-total-area-from-two-intersected-polylines-using-arcgis-or-python
- The Shoelace Algorithm - 101 Computing, accessed on November 9, 2025, https://www.101computing.net/the-shoelace-algorithm/
- Shoelace formula - Wikipedia, accessed on November 9, 2025, https://en.wikipedia.org/wiki/Shoelace_formula
- The Shoelace Algorithm For Calculating Polygon's Area In Excel - My Engineering World, accessed on November 9, 2025, https://myengineeringworld.net/2014/06/shoelace-polygon-area-excel.html
- Limited-damage 3D-printed interlocking connection for timber volumetric structures: Experimental validation and computational modelling - ResearchGate, accessed on November 9, 2025, https://www.researchgate.net/publication/364502915_Limited-damage_3D-printed_interlocking_connection_for_timber_volumetric_structures_Experimental_validation_and_computational_modelling
- MODELING OF TENSILE AND BENDING STRENGTH FOR PLA PARTS PRODUCED BY FDM - DergiPark, accessed on November 9, 2025, https://dergipark.org.tr/tr/download/article-file/910659
- Getting started with Multivariate Multiple Regression - UVA Library, accessed on November 9, 2025, https://library.virginia.edu/data/articles/getting-started-with-multivariate-multiple-regression
- Development of Prediction Method for Dimensional Stability of 3D-Printed Objects - MDPI, accessed on November 9, 2025, https://www.mdpi.com/2076-3417/13/19/11027
- Feature Engineering for Surrogate Models of Consolidation Degree in Additive Manufacturing - MDPI, accessed on November 9, 2025, https://www.mdpi.com/1996-1944/14/9/2239
- Accurate Estimation of Tensile Strength of 3D Printed Parts Using Machine Learning Algorithms - MDPI, accessed on November 9, 2025, https://www.mdpi.com/2227-9717/10/6/1158
- Machine learning models to predict the relationship between printing parameters and tensile strength of 3D Poly (lactic acid) scaffolds for tissue engineering applications - PubMed, accessed on November 9, 2025, https://pubmed.ncbi.nlm.nih.gov/37651988/
- Predicting the Tensile Properties of Automotive Steels at Intermediate Strain Rates via Interpretable Ensemble Machine Learning - MDPI, accessed on November 9, 2025, https://www.mdpi.com/2032-6653/16/3/123
- Splitting tensile strength prediction of Metakaolin concrete using machine learning techniques - PMC - NIH, accessed on November 9, 2025, https://pmc.ncbi.nlm.nih.gov/articles/PMC10654708/
- A tree-based machine learning surrogate model for predicting off-axis tensile mechanical properties of 2.5D woven composites at high temperatures |
- Request PDF - ResearchGate, accessed on November 9, 2025, https://www.researchgate.net/publication/389566329_A_tree-based_machine_learning_surrogate_model_for_predicting_off-axis_tensile_mechanical_properties_of_25D_woven_composites_at_high_temperatures
- PySLM: A Python Library for 3D Printing and Additive Manufacturing - GitHub, accessed on November 9, 2025, https://github.com/drlukeparry/pyslm
- scikit-learn: machine learning in Python - GitHub, accessed on November 9, 2025, https://github.com/scikit-learn/scikit-learn
- scikit-shapes/scikit-shapes: Shape processing in Python. - GitHub, accessed on November 9, 2025, https://github.com/scikit-shapes/scikit-shapes
- The Effect of Size on the Mechanical Properties of 3D-Printed Polymers - MDPI, accessed on November 9, 2025, https://www.mdpi.com/2071-1050/16/1/356
- Experimental study on interface failure behavior of 3D printed continuous fiber reinforced composites |
- Request PDF - ResearchGate, accessed on November 9, 2025, https://www.researchgate.net/publication/362626384_Experimental_study_on_interface_failure_behavior_of_3D_printed_continuous_fiber_reinforced_composites
- Experimental Characterization and Modeling of 3D Printed Continuous Carbon Fibers Composites with Different Fiber Orientation Produced by FFF Process - PMC - NIH, accessed on November 9, 2025, https://pmc.ncbi.nlm.nih.gov/articles/PMC8838912/
- 3D printed, bio-inspired prototypes and analytical models for structured suture interfaces with geometrically-tuned deformation and failure behavior |
- Request PDF - ResearchGate, accessed on November 9, 2025, https://www.researchgate.net/publication/266914282_3D_printed_bio-inspired_prototypes_and_analytical_models_for_structured_suture_interfaces_with_geometrically-tuned_deformation_and_failure_behavior