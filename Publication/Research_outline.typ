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
    "Mechanical Interlocking", "Computer Vision", 
    "Process-Structure-Property (PSP) Model", "Part Decomposition", 
    "Genetic Algorithm", "In-situ Thermal Monitoring"),
  //bibliography: bibliography("refs.bib"),
)

// Content goes below.

== Intro

=== Background and Motivation

=== Problem Statement

  Lack of understanding of how decomposition strategies affect interfacial strength from geometric and process parameters.

=== Objectives

  + Develop a Process-Structure-Property (PSP) model for quantifying the relationship between pattern topology, process parameters, and mechanical strength.
  + A novel method for programmable interlocking interfaces. 
  + A "Strength-Aware Decomposition Framework" using Generative Algorithm (GA or PSO or MCTS) to optimize part decomposition for maximum mechanical integrity

== Parametric Modeling

Create a mathematical way to describe interface topologies. Define derived geometry descriptors for comparative analysis:
-   Surface Area Enhancement Factor (SAEF)
-   Mechanical Interlocking Depth (MID)
-   Volumetric Interlock Ratio (VIR)

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

== Results

=== Mechanical Testing Outcomes

  Strength comparisons among pattern families.
  Statistical analysis (ANOVA, regression) to assess parameter influence.

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

From samples

acquire the images (side view of connections) of multiple speciments printed under a grid of parameters

研究的创新点在于, 着眼于一种仅在 多打印头独立XY驱动, 但共用Z桁架的gantry形式打印机中 便于实现的层间结合方式, 允许多个打印头同时work on same workpiece 并在打印层切换时, 通过控制打印头在结合界面的停留时机, 制造出垂直方向的啮合结构. 

首先制备多种不同几何形状接合面的cad模型, 用不同接合参数打印. 在显微镜下放大到层的尺度, 拍摄锯齿状的啮合结构. 获取的图像将被用于训练计算机视觉模型CV, 从而可以自动化分段标注层和层,材料和材料间的啮合界面, 并分类统计分段的长度. 

分段考虑有以下几种: 1. 方向朝向接缝的打印线(一层末端)和另一材料打印线的结合斜面, 2. 方向离开接缝的打印线(一层起始)和另一材料的结合边界上半部斜面, 3. 方向离开接缝的打印线(一层起始)和另一材料的结合边界下半部斜面, 4. 随着结合界面斜率增加或者是interlocking互锁长度增加, 产生的水平结合面 5. 因一层起始处材料堆积不足造成的空穴

这个分段模型将会被用数学方法予以描述, 

可变的参数有接合面曲线的局部斜率, 互锁结构的分段数(同样意义上是水平互锁结合面的数量), 以及互锁长度. zigzag的互锁结构, 从层的尺度来看可以视作一种使得结合表面面积增大\\结合更紧密的programmable扰动/噪声. 这些参数可以被用作打印路径规划的输出结果.

需要控制的变量有, 接合的时间间隔(需要一定的残余温度才能保证高强度的结合), 一层结束前瞬间的提前挤出滑行量, 换层时的回抽量, 新一层起始时的加压额外挤出量prime pressure. 这些变量会影响每个个体结合斜面的倾角. 当然, 如果要建立更精确的模型, 喷嘴处材料的热动力学, 已沉积材料的冷却环境等因素都需要被纳入考量

我们获得这个分段接合面模型之后, 便可以利用现存的其他研究成果, 通过控制打印时双材料的结合时间间隔微调熔融材料的结合强度, 通过调整参数化的3d模型改变分段接合面的各组长度和角度等参数. 最终制备多批不同参数组合的模型, 在测试台上测量拉伸强度, 验证模型能够准确描述围观结构.