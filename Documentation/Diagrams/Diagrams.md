
### Full Program Flowchart

```mermaid
flowchart TD
  A["Start (__main__)"] --> B["main()"]
  B --> C["rospy.init_node()"]
  B --> D["ranger_file_selection()"]
  D -->|No file| E[Exit Program]
  D -->|File chosen| F[launch log viewer subprocess]
  F --> G[connect to serial port]
  G --> H[initialize arm to last_position]
  H --> I["stream_gcode_and_move_robot()"]
  I --> J[read next G-code line]
  J --> K["parse_gcode_for_move()"]
  
  K -->|not G0/G1| L1[send to printer]
  L1 --> R["wait for "ok""]
  R -->|ok| I
  R -->|error| T[raise Exception]

  K -->|G0/G1| L2[compute move_data]
  L2 --> M1[interpolate positions]
  M1 --> M2["call move_arm_to_target()"]
  M2 --> N[setPitchRanges → inverse_kinematics → transformAngelAdaptArm]
  M2 --> Q["bus_servo_control.set_servos()"]
  L2 --> R2["wait for "done""]
  R2 -->|done| I
  R2 -->|error| T

  B --> V["rospy.on_shutdown(stop_arm_movement)"]
  V --> S1["turn off heater (M104 S0)"]
  V --> S2["move_arm_to_target(parking)"]
  V --> S3[terminate log viewer]
  S1 & S2 & S3 --> Z[cleanup & exit]
```


### Separate IK Algorithm Diagram


```mermaid
flowchart TD
  A["input: (x,y,z), pitch"] --> B[apply offsets/clearances]
  B --> C["compute theta6 = atan2(y/x)"]
  B --> D[compute l_x, l_z]
  D --> E[compute end-effector angle gamma]
  E --> F[compute l_cg, l_eg, beta]
  F --> G["compute triangle lengths & angles (theta4, theta5, theta3)"]
  G --> H[check joint limits]
  H -->|valid| I[transform angles → servo PWM]
  H -->|invalid| J[return False]
  I --> K[return servo values]
```
[Main flowchart](https://mermaid.live/view#pako:eNptk9tu4jAQhl9l5IsVK1GWBLIULlZqS0_bUnVL1YuGKnITA1YTT2Q70AO93QfYR9wn2bFToK32AiSPP_8z82fmhaWYCTZg0xyX6ZxrC9fDiQLYiydsbN25kSQFlypJvk7YHezs_IB9unOhhos4eN-HDyis0ZRPLamkTRQJfyKGjuBqJnQylblIjMhFaiVulYaOW10guPsVHMaHj9LCpcaZ5sU74oiuIZ2jEWoFR3HOK5XOIccZLKRYCg2mui81psIY_-rI5z-OU1SKUoJFMEJLnkOJ2nrk2CMnsSueLuSzAK4LR-bc2KREI12pnj3x7Cm1Y6wWvEhmzsaEqywpcCESjfdoN02devpnTGQGSjxaON5xPORSCU_89MQZ6ZVcG_EmN0Xt5TZC7nfm-1dIIu1vx8EKzoOYXMhcoaWWygrt2fPAa16R5pKThyQGE4YPE_YmduWF8GEFp9uj0Br1Cq5jzaURcPiYivKt623ydeKQ7CzKygrwTWfc8jp16FOPgtiXU2LOiVkbWH-QUV3eKKT6Up7ntQQZnlhMaO5mYuvfqNa7oD7tpbTp_MrNkIG_v_-AVAvhHHsgKwtuZVqHLY2ZoZaLPSLzvYyXdk8X79V-UeL7ytAM6gUmNBdWY96iDHXEbLK_dXMVfnQyQyW2Xnpm5WJrO8MPftb21Vtws9kTVImZVzbDpWoYi6Xv3xlRCGXX-W_8o3FAr2ylFeB0CnNBjmpojIJ2F8btT6ir9D920miRS7NPcCcmpUIq94m2--OJcQBfSM39dTx8G6e54KoqKSRoMe9Yk820zNjA6ko0WUFK3B3Zix9XZufUyoQNnF9iyqvckmXqlZ6VXN0iFuuXGqvZnA2mPDd0qkqaJTGU3G39BqEhF_oAK2XZoO8V2OCFPbJBJwxbnV7ve9iNgqAX7PaiJnuicL_VafeibrfX3g2joB-9Ntmzz9lu9cOo321H_SDYjYIoil7_AYFzoC8)

[IK flowchart](https://mermaid.live/view#pako:eNpNkd1um0AQhV9lNBeVI2EHzF-M1EpNHCdpVSkXlSoVomoLw08Lu2hZEv_Et32APGKfpMsSGt8sLOc7Z46GA6YiI4wwr8VTWjKp4Os64QAf4wQr3vYqgtnW2ln7MwvaSqVlgg8wn3-Ay5i1bb0Dkecdqe48rYlJxlPqHgb_pYGudEoqGh1DoEpSLID3wBTjy9nufHums97YdTyR9Y-tpY-9EddGvP4vEs_mlOeUKiGB8aImKFjTMANfG3hzkpQWQxTp86ceb6CNgW5Oq8lqTKqJF6rs4N2Y3MHMtPassb3_-nSn5jcm6jZOS0p_wy9RcQV11VRqXMLtID8_srrKnuEuVno_XS5kM6X__fMCHclHAfffvpw4Kv7q-RRLUr3ksGF1R4a4MyM_T8Jo13ivF48WFrLKMFKyJwsbkg0brngYnAnq8g0lGOnXjHLW1yrBhB-1rWX8uxDN5JSiL0qM8mGqhX2bMUXrihWSvSH6R5C8Ej1XGDlLE4HRAbcYuU64cMMwWHq-44TORehbuNOfg4Vrh_7qwnE823GD8Gjh3gy1F6ulv_Js33U93w7sYHX8BwlN1RE)