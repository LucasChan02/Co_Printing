## FEA Setup
For static analysis, The CAD model of [Steel Workbench Top, 72" Wide x 30" Deep](https://www.mcmaster.com/9054T115/) from McMaster-Carr has been used for the sheet metal tabletop geometry.

![|400](https://www.mcmaster.com/mvD/Library/CAD1/20250427/D581752B/9054T115_Steel%20Workbench%20TopM.GIF)
Technical Drawing
![|300](resources/Pasted%20image%2020251003105534.png)
View from top
![|300](resources/{DDFD9637-1546-4038-9F27-FEE44C6E20A6}.png)
View from below, the support brace can be moved to different locations according to need.

![|400](resources/{6B396360-3148-4A7E-A126-9F296DCDB266}.png)
At `pos0`, the IRB1200-5-90's Centre of mass is 473mm above the base with a 35mm horizontal distance to the centre line. We calculate the robot weight as 54.3kg taking arm load into consideration, the payload is not included at this point.

Since the steel grade is not specified but only noted as stainless steel or powder coated steel, the properties of AISI 316 were used to represent the general case. The support braces and the flanged edge are connected through threaded holes and short slots which compensates the error caused by folding, as a result the bolt connection could not be setup and we opted to the firm component interactions at the contacting surfaces. The tabletop's edges were fixed just as how they're connected to the table frame, the latter was assumed to be rigid to simplify the model.

![|450](resources/{360B6DA0-9851-457D-A59F-4046D82956C2}.png)

The loads on the tabletop were set as two static forces distributed on the footprint of the robot from remote points (COMs). Mesh control has been applied to the mounting holes, the outer edge of robot footprint as well as all the flanged edges where singularities are likely to occur.

![|350](resources/{E3036B83-4B0D-4432-9F8D-FE489EB10410}.png)

## Results
### Static Study 1

#### Von Mises Stress Plot

![](resources/{2F50CD6A-8BD4-4094-98ED-5B3AE36ED7E0}.png)

The stress concentration can be observed on the front edge of the robot base where the table is mostly pushed down. The maximum stressed point occurd at the inner side of the flange on the shorter edge of the table. 

![](resources/{7DBE73F1-467A-4656-9DD7-23472B24A4C1}.png)

#### Displacement

![](resources/{755B4562-C6DA-4450-BADB-C869C447FABA}.png)


The deformation in the robot contact patch ranged from 3.4 mm to 0.6 mm (front to rear), which translates to$0.729\degree$incline at the robot base and 11.47 mm displacement at the tool tip. 

According to the CAD model, the support braces have a distance of 3.4 mm to the table surface, thus only starts supporting the top only when the deformation of sheet metal reaches that amount and touches the support brace, as shown in the viewpoint from below.

![](resources/{E7722D11-DF57-4025-A937-FA0E97303F0A}.png)

#### Strain

![](resources/{7BC77F1B-1A9E-4399-9696-83D2F7E67BFB}.png)


#### Factor of Safety

In regions near the front bolt connection holes and the flanges on the shorter edge of the table, factor of safety can partially drop below 2.0, which indicates the tabletop can handle the static load with some margin.

![](resources/{5D595E0F-4F37-45A5-ABD6-8F74DD20E1E8}.png)








---
# 1) Bending stiffness depends on profile

Two key facts:

- Bending stress (and deflection) depend on the **second moment of area** (also called moment of inertia) I of the cross-section. For rectangular sections:  
   $I_x=\dfrac{b h^3}{12}$. For a rectangular _hollow_ section the inner rectangle is subtracted: Ix=BH3−bh312I_x=\dfrac{B H^3 - b h^3}{12}. The larger I is, the stiffer the member in bending. ([SkyCiv](https://skyciv.com/docs/tutorials/section-tutorials/moment-of-inertia-of-a-rectangle/?utm_source=chatgpt.com "Moment of Inertia of a Rectangle"))
    
- Bending (flexural) stress: σ=MyI\sigma = \dfrac{M y}{I} where M = applied bending moment, y = distance from neutral axis to extreme fiber. Deflection is proportional to 1/EI1/EI (E = Young’s modulus). So if I goes down, stress and deflection for the same load go up. ([Xometry](https://www.xometry.com/resources/materials/bending-stress/?utm_source=chatgpt.com "Bending Stress: Understand This Concept and How It Arises"))
    

In plain language: hollow sections can be _very_ efficient (high I for low weight) because material placed away from the neutral axis contributes more to I. But whether a _specific_ hollow extrusion is stiffer or weaker than a _specific_ solid extrusion depends on the exact outer size, wall thickness and slot geometry.

# 2) Quick numerical comparison (best-effort, with stated assumptions)

You gave two product specs:

- **A — Hollow**: Double 6-Slot rail, **60 mm high × 30 mm wide**, hollow extrusion (aluminum, T6, anodized). (T-slot dims shown in product text.)
    
- **B — Solid**: Double 6-Slot rail, **50 mm high × 25 mm wide**, **solid** extrusion (same alloy/temper noted).
    

Those cross-sections are complex (T-slots, internal webbing), so I approximate each shape as a plain outer rectangle. For the hollow profile I model it as a rectangular tube (outer 60×30 mm) with a uniform wall thickness. Because the manufacturer doesn’t list wall thickness in your snippet, I run a representative case and show sensitivity.

Assumptions used below (explicit):

- Material E is same for both (aluminum alloy ≈ same temper listed).
    
- Solid section: 50 mm (h) × 25 mm (b).
    
- Hollow section: outer 60 mm (h) × 30 mm (b). I assume wall thickness t = **3.0 mm** for the hollow tube in the first example (a plausible extrusion wall thickness for light rails).  
    (I then show how results change if t increases.)
    

Using the rectangular and hollow-rectangle formulas:

- Solid 50×25 mm:  
    Isolid=25⋅50312=260,417 mm4I_{solid} = \dfrac{25 \cdot 50^3}{12} = 260{,}417\ \text{mm}^4.
    
- Hollow 60×30 mm with t = 3 mm (inner dims = 54 × 24 mm):  
    Ihollow=30⋅603−24⋅54312≈225,072 mm4I_{hollow} = \dfrac{30\cdot60^3 - 24\cdot54^3}{12} \approx 225{,}072\ \text{mm}^4.
    

**Result (t = 3 mm):** hollow I ≈ **225,072 mm⁴**, solid I ≈ **260,417 mm⁴** → the hollow profile has **~86%** of the solid profile’s second moment of area (i.e. ~14% _less_ bending stiffness about the strong axis) under those assumptions.

But the hollow section is **much lighter** in cross-sectional area for t = 3 mm:

- Solid area = 50×25 = 1,250 mm²
    
- Hollow (t=3 mm) area ≈ 504 mm² → ~40% of the solid’s area (big weight saving).
    

**Sensitivity:** increasing the hollow wall thickness increases I. With the same outer 60×30 mm, the hollow tube reaches _and surpasses_ the solid’s I when t ≈ **4.0 mm** (i.e. with thicker walls the hollow 60×30 tube becomes stiffer than the 50×25 solid). So wall thickness is a controlling parameter.

(Equations and calculation approach referenced from standard section formulas; see links on rectangular and hollow rectangular I formulas and bending formulas). ([SkyCiv](https://skyciv.com/docs/tutorials/section-tutorials/moment-of-inertia-of-a-rectangle/?utm_source=chatgpt.com "Moment of Inertia of a Rectangle"))

# 3) Interpretation — what the numbers mean for your two product choices

- **Hollow profile (60×30)** is _not automatically_ stiffer than the smaller solid (50×25). With a thin wall (example 3 mm), the hollow profile **can be less stiff** in bending even though its outer size is larger. The hollow construction reduces area near the centroid which lowers I unless wall thickness and/or outer dimensions compensate. (Matches general structural guidance — hollow sections can be very efficient but depend on how much material is at the outside). ([Engineering Stack Exchange](https://engineering.stackexchange.com/questions/40736/hollow-rectangular-sections-versus-solid-sections?utm_source=chatgpt.com "Hollow (rectangular) sections versus solid sections"))
    
- **Weight vs stiffness tradeoff**: the hollow extrusion is likely much lighter per unit length (good if weight matters). The solid profile gives higher I per cross-section area in our example (so better stiffness if you want minimal deflection under transverse loads).
    
- **T-slot geometry matters**: actual T-slot shapes and internal webs change I significantly — the T-slots and internal ribs can put material away from the neutral axis (helping stiffness) or remove material where it matters. A simplified rectangular tube is an approximation; real extrusions should use the manufacturer’s cross-sectional properties or an FEA model for accuracy. See vendor design notes for extrusions and bending checks. ([docs.vention.io](https://docs.vention.io/docs/designing-with-t-slot-aluminum-extrusions?utm_source=chatgpt.com "Designing with T-Slot Aluminum Extrusions"))
    
| quantity                                                         | Solid rail (50 × 25 nominal) | Hollow rail (60 × 30 nominal) |
| ---------------------------------------------------------------- | ---------------------------- | ----------------------------- |
| Cross-section area (mm²)                                         | **449 mm²**                  | **410 mm²**                   |
| Ixx (mm⁴) — bending about strong axis (horizontal centroid axis) | **65,301 mm⁴**               | **89,021 mm⁴**                |
| Iyy (mm⁴) — bending about weak axis (vertical centroid axis)     | **11,680 mm⁴**               | **8,066 mm⁴**                 |
| Section modulus Sx = Ixx / y_max (mm³)                           | **2,499 mm³**                | **2,818 mm³**                 |


---

# Steel Beam Reinforcement 

* **Beam Type:** Two end fixed beam, rectangular cross section
* **Load (P):** 543 N (at the centre)
* **Length (L):** 30 inches = 0.762 m
* **Material:** 4140 Alloy Steel
	Modulus of Elasticity (E):
	    $E = 205 \text{ GPa} = 205 \times 10^9 \text{ Pa}$
	Yield Strength ($\sigma_y$):
	    $\sigma_y = 655 \text{ MPa} = 655 \times 10^6 \text{ Pa}$
* **Constraint 1 (Strength):** The maximum bending stress must not exceed the allowable stress.
* **Constraint 2 (Deflection):** Maximum deflection ($\delta_{max}$) at the centre must not exceed 0.2 mm.

### Safety Factor and Allowable Stress

* **Factor of Safety:** 2.0
* **Allowable Stress ($\sigma_{allow}$):**
    $\sigma_{allow} = \frac{\sigma_y}{FS} = \frac{655 \times 10^6 \text{ Pa}}{2.0} = 327.5 \times 10^6 \text{ Pa}$

### Beam Strength Equations

For a rectangular cross-section with width **W** and height **H**:
* **Moment of Inertia (I):**
    $I = \frac{WH^3}{12}$
* **Section Modulus (S):**
    $S = \frac{WH^2}{6}$
* **Maximum Bending Moment ($M_{max}$):** Occurs at the fixed ends.
    $M_{max} = \frac{PL}{8} = \frac{543 \text{ N} \times 0.762 \text{ m}}{8} = 51.72 \text{ N} \cdot \text{m}$
* **Maximum Deflection ($\delta_{max}$):** Occurs at the centre.
    $\delta_{max} = \frac{PL^3}{192EI}$

#### A. Strength Constraint Calculation

The bending stress must be less than or equal to the allowable stress.
$\sigma = \frac{M_{max}}{S} \le \sigma_{allow}$
$\frac{51.72 \text{ N} \cdot \text{m}}{WH^2 / 6} \le 327.5 \times 10^6 \text{ Pa}$
$WH^2 \ge \frac{6 \times 51.72 \text{ N} \cdot \text{m}}{327.5 \times 10^6 \text{ Pa}}$
$WH^2 \ge 9.48 \times 10^{-7} \text{ m}^3$

#### B. Deflection Constraint Calculation

The maximum deflection must be less than or equal to the 0.2 mm limit.
$\delta_{max} = \frac{PL^3}{192EI} \le \delta_{limit}$
$\frac{543 \text{ N} \times (0.762 \text{ m})^3}{192 \times (205 \times 10^9 \text{ Pa}) \times (\frac{WH^3}{12})} \le 0.0002 \text{ m}$
$WH^3 \ge \frac{543 \times (0.762)^3 \times 12}{192 \times (205 \times 10^9) \times 0.0002}$
$WH^3 \ge 3.65 \times 10^{-7} \text{ m}^4$

To find a specific solution, we assume a relationship **H = 2W** for a support beam.

1.  From the **strength** inequality:
    $W(2W)^2 \ge 9.48 \times 10^{-7} \implies 4W^3 \ge 9.48 \times 10^{-7}$
    $W^3 \ge 2.37 \times 10^{-7} \implies W \ge 0.00619 \text{ m}$
    $W \ge 6.19 \text{ mm}$

2.  From the **deflection** inequality:
    $W(2W)^3 \ge 3.65 \times 10^{-7} \implies 8W^4 \ge 3.65 \times 10^{-7}$
    $W^4 \ge 4.5625 \times 10^{-8} \implies W \ge 0.01462 \text{ m}$
    $W \ge 14.62 \text{ mm}$

To satisfy both conditions, we must choose the larger value for W, i.e.

* **Minimum Width (W):**
    $W = 14.62 \text{ mm}$
* **Minimum Height (H):**
    $H = 2W = 2 \times 14.62 = 29.24 \text{ mm}$

The required cross-sectional area ($A = W \times H$) is $14.62 \text{ mm} \times 29.24 \text{ mm} = 427.47 \text{ mm}^2$, meets the given condition. 

### Steel Bar Stock

Since the installation holes will be drilled to connect the robot base and table frame, the actual width should be the sum of the calculated width and largest hole diameter. Round up to 1-1/4 inch for convenience.

![Technical Drawing](https://www.mcmaster.com/mvD/Library/CAD1/20250511/03D07CC5/6552K353_Multipurpose%204140%20Alloy%20Steel%20BarM.GIF)

From McMaster, [1-1/4" Thick, 1-1/4" Wide, 36" Long Multipurpose 4140 Alloy Steel Bar](https://www.mcmaster.com/8910K68/) costs $96.48 Each, an equivalent low carbon steel bar costs $83.17 Each, which indicates that sheet metal tabletop + reinforcement can not be considered as an economical alternative plan as the total cost would be over 660 USD.