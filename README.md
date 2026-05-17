# RealTime-Buoyancy-Unity

**Real-Time Surface-Pressure Buoyancy with Adaptive Curved Clipping**
A Unity + C++ implementation of Hirae et al. (2025)'s closed-form hydrostatic-pressure integrator, with a custom *Adaptive Curved Clipping* refinement for coarse meshes on Gerstner waves.

> Final project for **KTH DH2323 Computer Graphics and Interaction**, by Felix Stenberg.


https://github.com/user-attachments/assets/0cdf0944-02fa-4a78-9991-8b227398369c



---

## Highlights

- **Closed-form per-triangle buoyancy**, exact force and torque, no quadrature error, even on coarse meshes (Hirae et al. 2025, Eqs. 5–9).
- **Wave-agnostic C++ DLL** geometry-only; the wave model lives in C# so the plugin is reusable across wave systems.
- **Adaptive Curved Clipping** refines per-vertex water heights along the clipping chord without geometric surface intersection. This was the project's main research contribution.
- **Burst-compiled wave sampler** `IJobParallelFor` + `Unity.Mathematics`, ~6–10× speedup over plain C#.
- **Garland–Heckbert runtime mesh simplification** for buoyancy meshes (`UnityMeshSimplifier`).
- **Bug found in the source paper.** A typographical error in Hirae et al.'s printed torque equations (Eqs. 7–9) was identified, re-derived from scratch, and corrected. The derivation notes and report draft live locally under the intentionally ignored `resources/` folder. Authors have been contacted.

---

## Gallery

### Cube + Bunny + dragon side-by-side

https://github.com/user-attachments/assets/1b3ab453-ee31-407c-a144-26f57436fcab

### Cube bobbing, damping in action

https://github.com/user-attachments/assets/482b7696-d519-409c-b86e-d4ab08798bbc


### Angular damping coefficient sweep

https://github.com/user-attachments/assets/6adece10-8944-466a-9a49-3b4d31057bae

### Bug demos (the project's two most instructive failures)

| Sub-triangle winding-order bug | Paper Eq. 7–9 typo |
|---|---|
| <video src="https://github.com/user-attachments/assets/9a35cc60-b630-49f2-92fa-6fcc3e815b20" controls="controls"></video> | <video src="https://github.com/user-attachments/assets/49bce1f3-21c5-4b03-97ac-f12052b57b00" controls="controls"></video> |
### Linear vs Adaptive clipping (debug gizmos)
| Linear clipping | Adaptive curved clipping |
|---|---|
| ![Linear clipping gizmo: green = dry, blue = submerged, orange = intersecting, white = straight chord.](report-assets/NewVideosAndPicture/GizmoNoClipping.png) | ![Adaptive clipping gizmo: magenta line + yellow dots = chord refinement samples; green vertical lines = depth correction at each sample.](report-assets/NewVideosAndPicture/GizmoClipping.png) |

### Mesh simplification (Garland–Heckbert)
| 30,000-triangle ground truth | Roughly 7k-triangle buoyancy mesh |
|---|---|
| ![Full bunny mesh used as ground truth in convergence tests.](report-assets/VideosAndPicturesFromBlog/FullmeshBunny.png) | ![Decimated bunny used for runtime buoyancy.](report-assets/VideosAndPicturesFromBlog/LowmeshBunny.png) |

### Test environments
| Force Sweep | Adaptive Clipping Comparison |
|---|---|
| ![Force sweep test running in Unity.](report-assets/NewVideosAndPicture/A1Testing.png) | ![Adaptive clipping comparison scene, captured late in the run after the objects had drifted from their initial spacing.](report-assets/NewVideosAndPicture/A6Testing.png) |

| Equilibrium Tilt (Corrected) | Equilibrium Tilt (Paper formula with typo) |
|---|---|
| ![Corrected formula settling at 26.565°.](report-assets/NewVideosAndPicture/A3Testing.png) | ![Printed formula settling at 45°.](report-assets/NewVideosAndPicture/A3TestingWrongFormula.png) |

### Concept diagrams, Per-triangle force/torque 
 ![Force and torque from hydrostatic pressure acting through the center of pressure.](report-assets/VideosAndPicturesFromBlog/Torque-Force5.PNG)

### `BuoyancyController` inspector
![Unity inspector showing density, angular momentum damping factor, adaptive toggle, and mesh simplification target.](report-assets/NewVideosAndPicture/BuoyancyController.png)

---

## More videos

All public clips live under [`report-assets/`](report-assets/). Click to download/play:

---

## Results at a glance

| Metric | Value |
|---|---|
| Cube force-sweep error vs analytical $F_b = \rho g V$ | < 10⁻⁶ % (FP noise) |
| Equilibrium tilt (corrected formula) vs Igarashi & Nakamura analytical 26.565° | **26.565°**, residual torque 2.4×10⁻⁵ N·m |
| Equilibrium tilt (paper formula, with typo) | 45.000°, residual torque 215.9 N·m |
| Linear path tilt error (~7k tris) vs 30k ground truth (rough waves) | ~6° (mesh/dynamics dominated) |
| Adaptive N=4 tilt error vs 30k ground truth | ~5.5° (Adaptive vs Linear gap < 0.5°) |
| Adaptive N=4 vs N=16 difference | 0° (fully converged at N=4) |
| 100-sphere stress test, Linear C++ DLL | **363 FPS** average |
| 100-sphere stress test, Adaptive N=1 (managed C#) | 53 FPS avg, degrades to 3 FPS at equilibrium |

Plots live in [`report-assets/plots/`](report-assets/plots/) and are regenerated with `python resources/plot_tests.py` and `python resources/plot_performance.py`.

| Accuracy validation | Convergence study | Stress test |
|---|---|---|
| ![Cube submersion force sweep, computed vs analytical, machine precision.](report-assets/plots/a1_cube_force_sweep.png) | ![Convergence: Linear, Adaptive N=4, Adaptive N=16 vs 30k ground truth (rough waves).](report-assets/plots/a6_rough_error.png) | ![100-sphere FPS over time: Linear C++ vs Adaptive C# paths.](report-assets/plots/stress_test_fps.png) |

| Equilibrium tilt, corrected formula vs paper formula |
|---|
| ![Corrected formula converges to 26.565°; paper formula sticks at 45°. Residual torque decays 6 orders of magnitude with the fix.](report-assets/plots/a3_a4_equilibrium_tilt.png) |

---


## Building

### Native plugin (C++ DLL)

The plugin uses GLM as a git submodule. Initialize submodules first:

```sh
git submodule update --init --recursive
```

Then configure and build with CMake (Windows / Visual Studio shown; any CMake-supported toolchain works):

```sh
cd NativePlugin
cmake -S . -B build
cmake --build build --config Release
```

The resulting DLL is consumed by Unity via `[DllImport]` in the buoyancy controllers.

### Unity project

Open [`BaseProject/`](BaseProject/) in **Unity 6.x** (URP). Press Play. Test scenes contain prefabs for the cube/bunny/dragon/sphere stress scenarios used in the report.


## Documentation

Blog can be found at: https://crispigt.com/dh2323

---

## References

- **Hirae, H., Morishima, S., & Ando, R.** (2025). *An Analytical Integrator for Solid-Fluid Coupled Buoyancy Forces.* SIGGRAPH Asia 2025 Technical Communications.
- **Hori, T.** (2021). *Proof that the Center of Buoyancy is Equal to the Center of Pressure by means of the Surface Integral of Hydrostatic Pressure Acting on the Inclined Ship.* Nagasaki Institute of Applied Science.
- **Fábián, G.** (2025). *Approximate and exact buoyancy calculation for real-time floating simulation of meshes.* Eurographics 2025 Short Paper.
- **Flick, J.** (2024). *Waves, Catlike Coding.* https://catlikecoding.com/unity/tutorials/flow/waves/ (MIT)
- **Garland, M. & Heckbert, P.** (1997). *Surface simplification using quadric error metrics.* SIGGRAPH '97.
- **Igarashi, T. & Nakamura, R.** (2007). *The equilibrium angles of floating cubes.*
- **Hwang, W. & Salvendy, G.** (2010). *Number of people required for usability evaluation: the `10 ± 2` rule.* CACM 53(5), 130–133.

---

## Acknowledgments

Christopher Peters and the DH2323 course team at KTH. Jasper Flick (Catlike Coding) for the MIT-licensed Gerstner wave implementation. Hirae et al. for the integrator that this work builds on.

## License

Code in this repository is released under the MIT License; see [`LICENSE`](LICENSE).

Original documentation, screenshots, videos, and report media by Felix Stenberg are licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) unless noted otherwise.

Third-party assets and dependencies (Catlike Coding wave shader, GLM, UnityMeshSimplifier, Stanford bunny/dragon meshes) retain their respective licenses.
