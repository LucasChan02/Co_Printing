#import "@preview/charged-ieee:0.1.4": ieee

#show: ieee.with(
  title: [ A Strength-Aware Decomposition Framework for Multi-Head Additive 
    Manufacturing via Programmed Interlocking Interfacial Topologies ],
  abstract: [
    
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

// Content goes below.

== Intro

=== Background and Motivation

  Brief review of existing strategies: process parameter optimization (raster angles, layer hight, etc.), thermal management, and geometric design. 

=== Problem Statement

  Lack of understanding of how decomposition strategies affect interfacial strength from geometric and process parameters.

  Discussion of the thermo-physical phenomena in polymer bonding and how "cold joints" at the boundary leads to poor interconnection.

  Introduce controlled mechanical interlocking as a robust method, setting the stage for the novel programmable, vertically-oriented interlocking proposed in this work.

=== Objectives

  + A Process-Structure-Property (PSP) model for quantifying the relationship between pattern topology, process parameters, and mechanical strength.
  + A novel method for programmable interlocking interfaces. 
  + A "Strength-Aware Decomposition Framework" using Generative/Searching Algorithm (GA or PSO or MCTS) to optimize part decomposition for maximum mechanical integrity

== Parametric Modeling

=== Parameterization of Interface geometries

  Create a mathematical way to describe interface topologies. Define following geometry descriptors for comparative analysis:
  -   Surface Area Enhancement Factor
  -   Mechanical Interlocking Depth
  -   Volumetric Interlock Ratio

=== Thermal History at Bonding Interface

Use "Interdiffusion Time Window" ($t_("interdiff")$) to describe thermal history, the duration the interface remains above sufficient bonding temperature $T_g$. 

In printing process, this is critically influenced by the "Bonding Interval" ($Delta t_"bond"$), a controllable parameter representing the time between adjacent depositions at the seam.


== Methodology

=== Sample Fabrication

  From the definition of contact surface pattern families (sine wave, square wave, triangular wave, slope, interlocking as curve functions) create CAD models within a predefined range of parameters for the curve, Prepare samples using controlled continuous extrusion method.

  collect temperature decendt along deposited material during printing using hermal imager.
  
=== Microscopic image processing

    Capture microscopic pictures for indexing multi-section contact interface. Train a CV model that can automate the trace contace edge marking process and count the total length for each types of segments. 

=== Parametric Generation of Interface Patterns

  Mathematical descriptors: amplitude, wavelength, phase offset.
  Computation of joint filling rate and geometric overlap/void.

=== Process Parameterization

  Controlle d variables: print speed, bonding interval, overlap distance, extrusion multiplier.
  Derived quantities: cooling rate, int erdiffusion window (from thermal imaging).

=== Specimen Fabrication and Testing

  Sample design and print conditions. produce speciments 
  Tensile test setup and data acquisition.
  Use of thermal imaging for process monitoring.

=== Automated Feature Segmentation via Computer Vision
To overcome manual analysis bias, a U-Net CNN could be implemented for objective, high-throughput semantic segmentation of microscopic images.
-   *Segmentation Classes:*
    1.  Adhesion_Interface_Closing
    2.  Adhesion_Interface_Opening_Upper
    3.  Adhesion_Interface_Opening_Lower
    4.  Horizontal_Interlock_Surface
    5.  Void/Porosity

-   *Workflow:* Data Annotation (ground truth), Model Training (with augmentation), Validation (using mean Intersection-over-Union, mIoU), and Deployment.

== Results

=== Mechanical Testing Outcomes

  Strength comparisons among pattern families.
  Statistical analysis (regression) to assess parameter influence.

=== Thermal Behavior

  Cooling rates and diffusion windows correlated to bond strength.

=== Microstructural Observations

  SEM or optical imaging (if available) showing bonding zones.

=== Process–Structure–Strength Mapping

  Visualization of relationships between pattern geometry, timing, and strength.

== Discussion

=== Interpretation of Key Trends

  How pattern topology affects bonded area and mechanical interlocking.
  Trade-offs between deposition timing, overlap, and cooling.
  
=== Implications for Decomposition Strategy

  Insights into geometry assignment for maximizing joint integrity.

== Strength-Aware Decomposition Framework

=== Framework Overview

  Integration of material's mechanical data and geometric descriptors into decision logic.
  
=== Algorithmic Implementation

  Criteria for selecting decomposition planes based on predicted strength and print feasibility.

  Constraints: minimum bonding interval (depend on layer printing time), maximum horizontal interlocking length (for neibouring layers, depend on part geometry), minimum allowed incline angle in input geometry (to ensure sufficient teeth shaped bonding at the interface without introducing flow adjustment or changes)

  nonlinear programming

=== Case Study or Demonstration

  Application of the framework to a complex geometry with multi-head planning.

---
