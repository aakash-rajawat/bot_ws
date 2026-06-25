# Third-Party Notices

Unless otherwise noted, original source code in this repository is licensed
under the Apache License, Version 2.0. Third-party software, generated-code
templates, model weights, and research publications retain their own licenses
and attribution requirements.

See `CITATION.cff` for machine-readable citation metadata.

## Software

### GTSAM

- Project: https://github.com/borglab/gtsam
- License: simplified BSD license, with additional third-party license notes in
  the upstream repository.
- Use in this workspace: pose-graph SLAM and relative-pose factor fusion.

### XFeat / accelerated_features

- Project: https://github.com/verlab/accelerated_features
- License: Apache-2.0.
- Pinned by: `third_party.repos`.
- Use in this workspace: lightweight image feature extraction and matching in
  the XFeat/LightGlue correspondence service.

### LightGlue

- Project: https://github.com/cvg/LightGlue
- License: Apache-2.0.
- Use in this workspace: local feature matching method referenced by the
  XFeat/LightGlue vision pipeline.

### CasADi Generated C Templates

- Project: https://github.com/casadi/casadi
- License note: checked-in generated C files state that CasADi template code is
  permissively licensed under MIT-0, generated runtime content is not
  copyrighted, and user code is owned by the user.
- Use in this workspace: generated covariance and residual code under
  `src/bot_vision/generated` and `src/bot_multisensor_odometry/generated`.

### DUGMA

- Paper/code: https://github.com/Canpu999/DUGMA
- Licensing status: no upstream software license was verified in the GitHub
  repository during the local audit.
- Use in this workspace: algorithmic reference for Dynamic Uncertainty-Based
  Gaussian Mixture Alignment.
- Distribution note: treat DUGMA as a research-paper reference unless explicit
  upstream source-code permission is obtained. Do not copy or redistribute
  upstream DUGMA source code without permission.

### ROPTLIB

- Project: https://github.com/whuang08/ROPTLIB
- Pinned by: `third_party.repos`.
- Licensing status: no root upstream software license was verified in the
  GitHub repository during the local audit.
- Use in this workspace: Riemannian optimization support for the DUGMA M-step.
- Distribution note: keep ROPTLIB as an external user-fetched dependency unless
  the upstream license is clarified or explicit permission is obtained.

## Research References

The workspace implements or builds on the following publications:

- Henry, S., & Christian, J. A. (2024). LOSTU: Fast, Scalable, and
  Uncertainty-Aware Triangulation. https://doi.org/10.48550/arXiv.2311.11171
- Pu, C., Li, N., Tylecek, R., & Fisher, R. B. (2018). DUGMA: Dynamic
  Uncertainty-Based Gaussian Mixture Alignment.
  https://doi.org/10.48550/arXiv.1803.07426
- Carvalho Filho, J. G. N. D., Carvalho, E. A. N., Molina, L., & Freire,
  E. O. (2019). The Impact of Parametric Uncertainties on Mobile Robots
  Velocities and Pose Estimation. IEEE Access, 7, 69070-69086.
  https://doi.org/10.1109/ACCESS.2019.2919335
