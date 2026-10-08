^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cyclo_duck_mjlab
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-10-08)
------------------
* Established ``cyclo_duck_mjlab`` as an independent package based on upstream 0.0.3, retaining the existing K1 training tasks.
* Renamed the Python distribution from ``cyclo_mjlab`` to ``cyclo_duck_mjlab`` and moved the container workspace to ``/workspace/cyclo_duck_mjlab``. Updated installation instructions and Git-state export paths accordingly.
* Isolated the ``cyclo_duck`` container, image and cache volumes from the original ``cyclo_mjlab`` environment.
* Added the ``cyclo_duck`` submodule and updated it to ``d1151ac`` with the articulated Cyclo Duck CAD assets.
* Added Cyclo Duck robot and scene MJCF assets with the MD walking model's 14-joint interface, fixed jaw, floating base, joint limits, initial poses, sensors and reference contact geometry.
* Preserved Cyclo CAD visuals, mass and inertia, including the fixed jaw; re-encoded oversized visual meshes without reducing geometry. Documented source revisions and asset licenses.
* The Cyclo Duck learning task and BAM actuator integration are not yet included.
* Contributors: Insu Park

0.0.3 (2026-09-18)
------------------
* Updated the ``ai_sapiens`` submodule from 0.1.1 to 0.2.2.
* Updated K1 masses, centers of mass, inertias, and meshes from the upstream model.
* Fixed duplicate IMU site errors by reusing the upstream ``imu`` site.
* Contributors: Insu Park

0.0.2 (2026-09-02)
------------------
* Updated the K1 MJCF source to use the ``ai_sapiens`` submodule.
* Simplified the Mimic ONNX interface to ``obs`` input and ``actions`` output.
* Added license and attribution notices to the modified files.
* Contributors: Insu Park

0.0.1 (2026-08-28)
------------------
* Developed as an external package for MJLab and MuJoCo.
* Verified compatibility with MJLab 1.2.0.
* Verified compatibility with MuJoCo and MuJoCo Warp 3.5.0.
* Verified compatibility with Python 3.11.
* Added reinforcement learning environments for the ROBOTIS K1 Rev.1 humanoid robot.
* Added velocity-tracking locomotion support.
* Added Mimic Dance1 and Dance2 reference-motion tracking support.
* Added motion conversion and kinematics-only replay tools.
* Added a Docker development environment for training and playback.
* Contributors: Insu Park
