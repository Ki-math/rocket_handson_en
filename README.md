# Rocket Flight Analysis Hands-On Workshop

An English edition of the [Japanese rocket workshop](https://github.com/Ki-math/rocket_handson), using MATLAB and Simulink to model a hybrid rocket, analyze its flight, and estimate landing dispersion.

## Get started

The workshop targets **MATLAB R2026a on Windows**, with Simulink, Aerospace Toolbox, and Aerospace Blockset. The supplementary transfer-function and stability-margin exercise also uses Control System Toolbox.

The presentation includes embedded videos and is stored with **Git LFS**. Install [Git LFS](https://git-lfs.com/), then clone the repository:

```sh
git lfs install
git clone https://github.com/Ki-math/rocket_handson_en.git
cd rocket_handson_en
git lfs pull
```

1. Open `Rocket_HandsOn.prj` in MATLAB. Opening the project sets up the helper-function paths.
2. Open `rocket_workshop_src.mlx` in the Live Editor.
3. Run the sections in order, starting with the parameter definitions.
4. Open [MATLAB_Rocket_Hands_On_Workshop.pptx](MATLAB_Rocket_Hands_On_Workshop.pptx) for the accompanying 46-slide presentation.

Run the single-flight sections first. The Monte Carlo section performs 50 simulations and takes longer.

## Workshop contents

1. Understand vehicle, aerodynamic, propulsion, and parachute models.
2. Analyze a flight using the six-degree-of-freedom (6DOF) model.
3. Explore the effects of launch tilt, launch azimuth, wind speed, and wind direction.
4. Predict landing dispersion using Monte Carlo simulation.
5. Explore roll control using the supplementary 1DOF model and presentation appendix.

| File or folder | Purpose |
| --- | --- |
| `rocket_workshop_src.mlx` | Main interactive workshop, equations, and exercises |
| `rocket_6dof_simulation.slx` | 6DOF rocket flight model |
| `roll_1dof_model.slx` | Supplementary roll-control model |
| `common/` | Setup, launch/wind conventions, signal logging, and simulation helpers |
| `evaluation/` | Flight summaries, plots, dispersion, and supplementary control analysis |
| `animation/` | 3D flight animation and coordinate-frame visualization |
| `MATLAB_Rocket_Hands_On_Workshop.pptx` | English workshop slides, illustrations, and embedded media |

## Conventions

- Calculations use SI units and radians; plots convert angles to degrees.
- The inertial frame is **UEN**: x = Up, y = East, z = North. The animation reorders coordinates to ENU for display.
- Launch tilt is measured from vertical. Azimuth is 0° at north and 90° at east.
- Wind direction means **where the wind comes from**. A 270° wind comes from the west and moves east.
- The default Monte Carlo exercise uses seed 7 and independent standard normal samples, scaled by the specified 3-sigma uncertainties. Negative tilt and wind-speed samples are clipped at zero; directions wrap at 360°.

The English edition includes the local workshop updates available on 19 November 2026. Model equations, parameters, and numerical algorithms are preserved. Explanations, comments, messages, and figure labels are translated; outdated saved Live Script outputs are cleared. Reference book covers, company logos, and original source video footage retain their original appearance. English captions identify Japanese-language references.
