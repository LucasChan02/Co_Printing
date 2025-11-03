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




From samples

acquire the images (side view of connections) of multiple speciments printed under a grid of parameters

研究的创新点在于, 着眼于一种仅在 多打印头独立XY驱动, 但共用Z桁架的gantry形式打印机中 便于实现的层间结合方式, 允许多个打印头同时work on same workpiece 并在打印层切换时, 通过控制打印头在结合界面的停留时机, 制造出垂直方向的啮合结构. 

首先制备多种不同几何形状接合面的cad模型, 用不同接合参数打印. 在显微镜下放大到层的尺度, 拍摄锯齿状的啮合结构. 获取的图像将被用于训练计算机视觉模型CV, 从而可以自动化分段标注层和层,材料和材料间的啮合界面, 并分类统计分段的长度. 

分段考虑有以下几种: 1. 方向朝向接缝的打印线(一层末端)和另一材料打印线的结合斜面, 2. 方向离开接缝的打印线(一层起始)和另一材料的结合边界上半部斜面, 3. 方向离开接缝的打印线(一层起始)和另一材料的结合边界下半部斜面, 4. 随着结合界面斜率增加或者是interlocking互锁长度增加, 产生的水平结合面 5. 因一层起始处材料堆积不足造成的空穴

这个分段模型将会被用数学方法予以描述, 

可变的参数有接合面曲线的局部斜率, 互锁结构的分段数(同样意义上是水平互锁结合面的数量), 以及互锁长度. zigzag的互锁结构, 从层的尺度来看可以视作一种使得结合表面面积增大\\结合更紧密的programmable扰动/噪声. 这些参数可以被用作打印路径规划的输出结果.

需要控制的变量有, 接合的时间间隔(需要一定的残余温度才能保证高强度的结合), 一层结束前瞬间的提前挤出滑行量, 换层时的回抽量, 新一层起始时的加压额外挤出量prime pressure. 这些变量会影响每个个体结合斜面的倾角. 当然, 如果要建立更精确的模型, 喷嘴处材料的热动力学, 已沉积材料的冷却环境等因素都需要被纳入考量

我们获得这个分段接合面模型之后, 便可以利用现存的其他研究成果, 通过控制打印时双材料的结合时间间隔微调熔融材料的结合强度, 通过调整参数化的3d模型改变分段接合面的各组长度和角度等参数. 最终制备多批不同参数组合的模型, 在测试台上测量拉伸强度, 验证模型能够准确描述围观结构.