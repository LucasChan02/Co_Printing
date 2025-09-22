## Standards

### AWS A5.10

The primary standard of bare aluminium filler metals in North America is AWS A5.10, *Specification for Bare Aluminum and Aluminum-Alloy Welding Electrodes and Rods*. 

This specification prescribes the required chemical composition, testing procedures and packaging for a wide range  of aluminium alloys. Wires classified under this standard typically carry the prefix "ER".

### ISO 18273

The corresponding international standard is EN ISO 18273, *Welding consumables - Wire electrodes, wires and rods for welding of aluminium and aluminium alloys - Classification*. 

This standard also classifies solid wires and rods based on their chemical composition. For the most common alloys, these two standards are largely harmonized, meaning a wire certified to AWS classification has a direct and chemically equivalent counterpart under ISO.

## Candidate Filler Wires

### ER4043 (Al-Si Alloy)

- **Classification:** AWS A5.10: ER4043; ISO 18273: S Al 4043 (AlSi5).25
  
- **Primary Alloying Element:** ER4043 is an aluminium-silicon alloy containing approximately 5% silicon by weight.

- **Characteristics:** The addition of silicon is the defining feature of this alloy and is responsible for its primary welding characteristics. Silicon significantly lowers the melting point and narrows the solidification range of the alloy, which imparts great fluidity to the molten weld pool. This high fluidity and narrower freezing range make ER4043 significantly less sensitive to solidification cracking when welding on 60XX series base alloy. The crack resistance is a major advantage for the high-restraint conditions often found in multi-layer WAAM builds. 

### ER5356 (Al-Mg Alloy)

- **Classification:** AWS A5.10: ER5356; ISO 18273: S Al 5356 (AlMg5Cr(A)).7
  
- **Primary Alloying Element:** ER5356 is an aluminum-magnesium alloy containing approximately 5% magnesium, with smaller additions of manganese and chromium for grain structure control. 

- **Characteristics:** The high magnesium content makes ER5356 a significantly stronger, harder and more ductile filler metal than ER404\ in the welding condition. This translates to higher tensile and shear strength in the final deposit. Mechanically, the wire itself is stiffer and has a higher columnar strength than ER4043. This property vastly improves its feedability in the long conduits of automated and robotic GMAW systems, reducing the likelihood of wire jamming, a common cause of production downtime. It also has a better aesthetic response to anodizing. The major limitation is a susceptibility to stress corrosion cracking (SCC) when expose to temperature for prolonged periods or after undergoing a heat treatment cycle.

---

#### Comparison of the chemical composition

| Element                                                                                   | ER4043 (Al-Si) Requirements (%) | ER5356 (Al-Mg) Requirements (%) |
| ----------------------------------------------------------------------------------------- | ------------------------------- | ------------------------------- |
| Silicon (Si)                                                                              | 4.5 - 6.0                       | 0.25 max                        |
| Magnesium (Mg)                                                                            | 0.05 max                        | 4.5 - 5.5                       |
| Iron (Fe)                                                                                 | 0.80 max                        | 0.40 max                        |
| Copper (Cu)                                                                               | 0.30 max                        | 0.10 max                        |
| Manganese (Mn)                                                                            | 0.05 max                        | 0.05 - 0.20                     |
| Chromium (Cr)                                                                             | -                               | 0.05 - 0.20                     |
| Zinc (Zn)                                                                                 | 0.10 max                        | 0.10 max                        |
| Titanium (Ti)                                                                             | 0.20 max                        | 0.06 - 0.20                     |
| Beryllium (Be)                                                                            | 0.0003 max                      | 0.0003 max                      |
| Others (each)                                                                             | 0.05 max                        | 0.05 max                        |
| Others (total)                                                                            | 0.15 max                        | 0.15 max                        |
| Aluminum (Al)                                                                             | Remainder                       | Remainder                       |

#### As-Deposited Mechanical Properties

| Property                         | ER4043 (Al-Si)          | ER5356 (Al-Mg)          |
| -------------------------------- | ----------------------- | ----------------------- |
| **Typical Tensile Strength**     | 185-200 MPa (27-29 ksi) | 204-290 MPa (30-42 ksi) |
| **Typical Shear Strength**       | ~76 MPa (11 ksi)        | ~124 MPa (18 ksi)       |
| **Typical Hardness (Weld Zone)** | 41-75 HV                | 60-77 HV                |
| **Ductility/Formability**        | Lower                   | Higher                  |

- **Microstructure:** The rapid solidification inherent in WAAM will result in a fine, cast dendritic microstructure for both alloys. Some studies reported that ER5356 deposits on 6061 can get a finer, more equiaxed grain structure than ER4043. However, some other research has found the opposite as variations in heat input and interpass cooling can easily overwhelm the subtle effects of chemistry on grain size. 
- **Tensile, Yield and Shear Strength:** ER5356 consistently produces a stronger weld deposit. its higher intrinsic strength provides a significant advantage especially in shear strength. 
- **Ductility:** ER5356 is generally considered to have higher ductility and toughness and can undergo more plastic deformation before fracturing. 

## WAAM Process Environment

**Heat Accumulation:** Aluminium possesses high thermal conductivity, approximately three to four times that of steel. In the WAAM process, this property leads to rapid and widespread heat distribution throughout the part with each deposited layer. As the build height increases, the path for heat to dissipate into the substrate or build plate becomes longer and less effective, leading to a progressive accumulation of heat within the component.

This sustained high temperature has negative effects: it can lead to a loss of dimensional accuracy as the part distorts, and it can alter the solidification conditions for subsequent layers, affecting bead geometry and defect formation. Common thermal management strategies include implementing forced interpass cooling (e.g., with compressed air or CO2), programming dwell times between layers to allow for natural cooling, and using actively cooled substrates or fixtures.

**Porosity:** Aluminium alloys are highly susceptible to hydrogen porosity. The solubility of hydrogen in liquid aluminium is many times greater than its solubility in solid aluminium. During welding, hydrogen from various sources can be readily absorbed into the molten weld pool. As the pool solidifies, the excess hydrogen is rejected from the solidifying metal, forming gas pores that become trapped in the final deposit. The primary sources of hydrogen are moisture and hydrocarbon contaminants present on the surface of the filler wire, the base plate, or entrained in the shielding gas. 

**Oxide Layer:** Aluminium surfaces are naturally covered by a thin, tenacious, and refractory layer of aluminium oxide (Al2​O3​). This oxide layer has a melting point of over 2000°C, far higher than the ~660°C melting point of aluminium itself. If not adequately removed prior to and during welding, this oxide can become entrapped in the weld pool, leading to a lack of fusion and other defects.

## Summary 

**ER4043 (Al-Si):** This alloy is characterized by its exceptional fluidity, superior resistance to hot cracking, and smoother weld bead profile. ese attributes make it highly suitable for fabricating components with complex geometries where thermal stresses are a primary concern. Furthermore, ER4043 is the mandatory choice for applications that will undergo a post-weld heat treatment (PWHT) to recover the strength lost in the heat-affected zone (HAZ) of the 60xx base material, or for components intended for service at elevated temperatures (above 66°C or 150°F). Its primary drawbacks are lower as-deposited strength and ductility compared to ER5356.

**ER5356 (Al-Mg):** This alloy offers significantly higher as-deposited tensile strength, shear strength, and toughness. Its greater wire stiffness enhances feedability in robotic and automated systems, a crucial factor for process reliability in a production WAAM environment. ER5356 provides an excellent colour match after anodizing, making it the default choice where a uniform, high-quality finish is required. However, it is highly susceptible to stress corrosion cracking (SCC) if subjected to post-weld heat treatment or prolonged service at elevated temperatures, precluding its use in such applications. 
