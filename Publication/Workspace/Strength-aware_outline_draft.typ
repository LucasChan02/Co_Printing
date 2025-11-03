#import "@preview/charged-ieee:0.1.4": ieee

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
Brief review of existing strategies: process parameter optimization 
(raster angles, etc.), thermal management, and geometric design. 
Introduction of mechanical interlocking as a robust method, 
setting the stage for the novel programmable, vertically-oriented 
interlocking proposed in this work.

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
Establish a formal mathematical framework to describe interface 
topologies (e.g., Sinusoidal, Triangular, Square, Interlocking 
Features).
Define derived, geometry-agnostic descriptors for comparative 
analysis:
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
