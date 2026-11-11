// Nodalis User Guide. Build with:
//   typst compile --root . --ignore-system-fonts docs/user-guide/main.typ nodalis-user-guide.pdf
// from the repository root. The version is read from the workspace Cargo.toml, which is why
// the root must be the repository and not this folder.
// Each topic lives in its own file under chapters/, so that changes to different features do
// not touch the same file. See README.md in this folder.

#import "template.typ": *

#show: guide.with(version: toml("../../Cargo.toml").workspace.package.version)

#include "chapters/introduction.typ"
#include "chapters/first-analysis.typ"

#include "chapters/interface/workspaces.typ"
#include "chapters/interface/project.typ"
#include "chapters/interface/links.typ"
#include "chapters/interface/sources.typ"
#include "chapters/interface/study.typ"
#include "chapters/interface/main-window.typ"
#include "chapters/interface/files.typ"
#include "chapters/interface/undo.typ"
#include "chapters/interface/view.typ"
#include "chapters/interface/selection.typ"
#include "chapters/interface/input.typ"
#include "chapters/interface/settings.typ"

#include "chapters/geometry-mesh/geometry.typ"
#include "chapters/geometry-mesh/mesh.typ"
#include "chapters/geometry-mesh/mesh-tools.typ"
#include "chapters/geometry-mesh/mesh-sets.typ"

#include "chapters/model/model-properties.typ"
#include "chapters/model/materials.typ"
#include "chapters/model/plastic.typ"
#include "chapters/model/hyperelastic.typ"
#include "chapters/model/sections.typ"
#include "chapters/model/shell-section.typ"
#include "chapters/model/membrane-section.typ"
#include "chapters/model/features.typ"
#include "chapters/model/interactions.typ"
#include "chapters/model/rigid-body.typ"
#include "chapters/model/amplitudes.typ"
#include "chapters/model/distributions.typ"
#include "chapters/model/initial-conditions.typ"
#include "chapters/model/steps.typ"
#include "chapters/model/dynamic.typ"
#include "chapters/model/complex-frequency.typ"
#include "chapters/model/cyclic-symmetry.typ"
#include "chapters/model/modal-dynamics.typ"
#include "chapters/model/sensitivity.typ"
#include "chapters/model/creep.typ"
#include "chapters/model/bcs-loads.typ"
#include "chapters/model/symmetry.typ"
#include "chapters/model/moving-load.typ"
#include "chapters/model/excitation.typ"
#include "chapters/model/defined-fields.typ"

#include "chapters/analysis/run.typ"

#include "chapters/results/viewing.typ"
#include "chapters/results/animation.typ"
#include "chapters/results/evaluation.typ"
#include "chapters/results/campbell.typ"
#include "chapters/results/paraview.typ"
