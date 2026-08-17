# MuJoCo 3.10.0 (vendored)

- `lib/libmujoco.so.3.10.0` (SONAME `libmujoco.so.3.10.0`), symlinks
  `libmujoco.so.3` and `libmujoco.so`.
- `include/mujoco/` — full public C headers.
- `licenses/MUJOCO_LICENSE` (Apache-2.0) and
  `licenses/MUJOCO_LICENSES_THIRD_PARTY.md`.

Copied 2026-08-17 from the official `mujoco==3.10.0` PyPI wheel installed at
`/home/christian/msc_project/.venv/lib/python3.12/site-packages/mujoco/`
(dist-info `mujoco-3.10.0`). Not hand-edited. To upgrade, replace all of the
above from a newer wheel in one change.

SHA-256 of the shared library:
`872b2954e9760b1df122e3c807a907ff806e1980099b00e15789ae67e08b75d5`

## GLFW (for the optional viewer)

The wheel ships no GLFW C library and `libglfw3-dev` is not installed
system-wide, so GLFW is vendored too: `lib/libglfw3.a` and `include/GLFW/`
copied 2026-08-17 from the FetchContent build in
`/home/christian/msc_project/cpp/build-gpmp2/_deps/glfw-{src,build}`
(zlib/libpng licence at `licenses/GLFW_LICENSE.md`). Linking the static
library additionally needs the system X11/GL libraries the viewer target
should list explicitly.
