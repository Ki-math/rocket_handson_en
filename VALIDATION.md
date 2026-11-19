# English Edition Validation

Validated on 19 November 2026 using MATLAB R2026a Update 4 and Microsoft PowerPoint on Windows.

- Opened both MATLAB projects and resolved helper functions from their respective project folders.
- Ran the default flight in both editions. Logged position samples, sample times, and flight-summary tables were exactly identical.
- Ran two additional scenarios through `workshop_run_scenarios`; flight summaries and landing-dispersion plots completed successfully.
- Generated English single-flight plots successfully. The complete 50-run Monte Carlo exercise was not rerun for this localization check.
- Checked that the Simulink archives have identical contents except for translated comments in three MATLAB Function charts. Executable chart code is unchanged.
- Verified all ZIP packages and XML parts, and checked editable project/presentation text for remaining Japanese characters.
- Opened the English presentation in PowerPoint and rendered all 46 slides. Checked the translated layouts and illustrations.
- Verified that the seven embedded videos, two 3D models, and animated GIF are byte-for-byte unchanged.

The default flight summary was approximately 51.906 s flight time, 306.57 m apogee, and 2,776.3 m landing range. These values confirm consistency with the supplied Japanese project; this check does not independently validate the physical model.
