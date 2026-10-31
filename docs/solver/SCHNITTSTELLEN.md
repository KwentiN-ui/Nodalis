# Schnittstellen des PrePoLix-Solvers (`plxs`)

Stand M0 (2026-10-31). Dieses Dokument legt fest, worauf sich die Threads der Meilensteine
M1 bis M4 verlassen können: den Crate-Schnitt unter `solver/`, die IR in `plxs-core`, die
Trait-Signaturen der Fachcrates und die Regeln für Änderungen daran. Das Konzept dazu steht in
`KONZEPT_EIGENER_SOLVER.md` (Projektordner, Abschnitt 3).

## Grundregeln

- **Kein CalculiX-Code.** CalculiX steht unter GPL-2.0-only, PrePoLix unter GPL-3.0-or-later;
  die beiden sind nicht kompatibel. Es wird keine Zeile aus den Fortran- oder C-Quellen
  übernommen, übersetzt oder abgeschrieben, auch keine Tabellen. Erlaubt sind das
  CalculiX-Handbuch (Verfahrensbeschreibung), Lehrbücher und ccx als Black-Box-Orakel für
  Referenzergebnisse. Jede Crate nennt ihre Quellen im `//!`-Kommentar.
- **Ein Numerik-Ökosystem.** Lineare Algebra ausschließlich über `faer` (dicht und sparse,
  Ordering, dichte Eigenlöser), Parallelität über `rayon`, `num-complex` nur als Zahltyp.
  Kein nalgebra, ndarray, sprs, russell oder Ähnliches daneben. Eigenlöser (Lanczos/Arnoldi)
  werden auf `faer` aufgesetzt, nicht aus einer weiteren Crate bezogen. Die Versionen stehen in
  `[workspace.dependencies]`.
- **Workspace-Lints gelten**: `unsafe_code = "deny"`, `clippy::all`; `cargo fmt`.
- **Alles cross-platform** (Linux und Windows), reines Rust, keine C/Fortran-Abhängigkeiten.
- **Jeder PR bringt Fixtures mit** (siehe `FIXTURES.md`): Die Fixtures seiner Stufe müssen
  mit `plxs-fixtures run` grün sein, bevor er gemerged wird.

## Crate-Schnitt

Alle Crates liegen unter `solver/plxs-*` (Workspace-Member `solver/plxs-*`), Präfix `plxs-`.
Abhängigkeiten zeigen nur nach unten:

```
plxs-cli            Binary `plxs`: ccx-kompatible Kommandozeile, stdout-Protokoll, Exit-Codes
  └ plxs-session    Session-API: Modell laden, Steps rechnen, Zustand, Ergebnisse, Restart
      ├ plxs-input      .inp -> IR (M0, fertig)
      ├ plxs-output     .frd (ASCII/binär), .dat, .sta, .cvg; Expansion der Ergebnisse (M1)
      └ plxs-procedures Static, Nlgeom/Newton, Visco, Dynamic, Heat, Coupled, Frequency,
            │           Buckle, Complex Frequency, Modal/Steady State Dynamics
            ├ plxs-assembly   DOF-Nummerierung, MPC-Elimination, CSR-Muster, parallele Assemblierung
            │   ├ plxs-elements   Formfunktionen, Integration, K/M/f_int je Elementtyp, Expansion
            │   ├ plxs-materials  Elastisch, Plastisch, Kriechen, thermisch, Zustandsvariablen
            │   ├ plxs-constraints SPC, Equation, Rigid Body, Tie, Transform, Cyclic, Submodel
            │   └ plxs-contact    Suche, N2S/S2S/Mortar, Surface Behavior, Reibung, Gap Conductance
            └ plxs-linalg     Sparse-Strukturen, LDLᵀ/LU (faer), CG, Eigenlöser
plxs-core           IR: Modell, Steps, State, Results, Progress, Fehler. Keine Abhängigkeit nach oben.
plxs-verify         Fixture-Korpus, Golden Files, Vergleich, Binary `plxs-fixtures`
```

`plxs-core`, `plxs-input`, `plxs-verify` und `plxs-cli` sind in M0 gefüllt; die übrigen Crates
existieren leer mit einem `//!`-Kommentar und werden von den Threads der Stufen gefüllt.
`plxs-core` hängt bewusst nicht von `plx-model`/`plx-mesh` ab, damit der Solver ohne PrePoLix
gebaut und getestet werden kann; `plxs-verify` nutzt `plx-io` nur zum Lesen von `.frd`/`.dat`.

## Die IR (`plxs-core`) ist der Vertrag

`plxs_core::Analysis { model: Model, steps: Vec<Step> }` ist das vollständige Datenmodell
(Dokumentation in den Modulen `mesh`, `element`, `material`, `section`, `constraints`,
`contact`, `amplitude`, `step`, `model`, `state`, `results`, `progress`, `error`). Regeln:

1. **Nutzer-Ids bleiben** (Knoten- und Elementnummern der `.inp`); intern wird über dichte
   Indizes gerechnet (`Nodes::index`, `Model::element_index`), die Ausgabe schreibt Ids.
2. **Namen in Großbuchstaben** (Sets, Surfaces, Materialien, Interaktionen, Amplituden), wie
   CalculiX sie liest (`normalized_name`).
3. **Steps sind vollständig.** `Step` enthält, was während des Steps gilt; die
   Carry-over-Regeln von CalculiX (`OP=NEW`/`OP=MOD` für Randbedingungen, Lasten,
   Temperaturen; Ausgabeanforderungen werden übernommen, Lasten eines Perturbation-Steps
   nicht) löst `plxs-input` auf. Prozeduren schauen nie in frühere Steps.
4. **Alles ist `Clone + Serialize + Deserialize`** (serde), auch `State` und `Results`, damit
   Phase 2 Zustände zwischen Nodes reichen und Restarts speichern kann. Nach dem
   Deserialisieren `Model::rebuild_indices()` aufrufen.
5. **Keine Rechenlogik in `plxs-core`**: nur Daten, `validate` (Referenzen, Sections
   zuweisen) und Fehlertypen.
6. **Fehler tragen den Item-Namen** (`Error::Input { keyword, message, location }`,
   `UndefinedReference { keyword, kind, name }`) und formatieren sich im ccx-Wortlaut
   (`*ERROR reading *BOUNDARY: ...`, `*ERROR in calinput: keyword *X cannot be interpreted`),
   weil `solver_check` und der Job-Monitor den Text lesen.
7. **Änderungen an der IR sind ein eigener, kleiner PR** mit Begründung, den der Koordinator
   reviewt; Threads hängen Felder an (`Option<..>` mit `Default`), statt Bestehendes
   umzubauen. Ein PR, der nebenbei die IR ändert, wird nicht angenommen.

Was die IR heute schon abdeckt, das aber erst später gerechnet wird: alle Elementtypen des
Inventars (`ElementType`), alle Prozeduren (`Procedure`), Kontakt, zyklische Symmetrie,
Submodelle, `*SENSITIVITY` (nur geparst; rechnet weiter mit ccx), `*RESTART`, `*CONTROLS`.

## Session-API (`plxs-session`, M1 T3)

```rust
pub struct SessionOptions { pub threads: usize, pub work_dir: PathBuf }

impl Session {
    pub fn new(model: Model, options: SessionOptions) -> Result<Session>;
    /// Rechnet einen Step auf dem aktuellen Zustand; Ereignisse gehen an `progress`.
    pub fn run_step(&mut self, step: &Step, progress: &mut dyn Progress) -> Result<StepOutcome>;
    pub fn state(&self) -> &State;
    pub fn results(&self) -> &Results;
    /// Zustand plus Modell-Fingerprint für `*RESTART, WRITE` (postcard-serialisiert).
    pub fn snapshot(&self) -> Snapshot;
    pub fn resume(model: Model, snapshot: Snapshot, options: SessionOptions) -> Result<Session>;
}

pub enum StepOutcome { Converged, Diverged { increment: u32, message: String }, Partial { increments: u32 } }
```

Ein `rayon`-Pool je Session mit `threads` Threads (Default: `OMP_NUM_THREADS`, sonst alle
Kerne). Kein globaler Zustand: mehrere Sessions pro Prozess müssen möglich sein.

## Traits der Fachcrates

Die Signaturen sind die Vorgabe für M1/M2; Abweichungen im Detail (Lebensdauern, Puffer)
entscheidet der Thread, der die Crate füllt, und dokumentiert sie hier im selben PR.

### `plxs-linalg` (M1 T1)

```rust
/// Symmetrisches CSR-Muster (obere Hälfte) mit Werten; Indizes u32, Werte f64.
pub struct SparseMatrix { .. }
pub struct Pattern { .. }           // einmal je Step berechnet, über Iterationen wiederverwendet

pub trait LinearSolver {
    /// Symbolische Analyse (Ordering, Supernodes) für ein Muster; wird wiederverwendet.
    fn analyze(&mut self, pattern: &Pattern) -> Result<()>;
    /// Numerische Faktorisierung der Werte zum zuletzt analysierten Muster.
    fn factorize(&mut self, matrix: &SparseMatrix) -> Result<()>;
    /// Löst K x = b für eine oder mehrere rechte Seiten (in place).
    fn solve(&self, rhs: &mut [f64]) -> Result<()>;
    /// Trägheit (Anzahl negativer Pivots) nach LDLᵀ, für den Sturm-Check der Eigenlöser.
    fn inertia(&self) -> Option<usize>;
}
// Implementierungen: LdltSolver (faer, symmetrisch), LuSolver (faer, unsymmetrisch:
// Reibung, Coriolis), CgSolver (Jacobi-/ILU-vorkonditioniert) für die GUI-Optionen
// "Iterative Scaling/Cholesky".

pub trait EigenSolver {
    /// Die `count` kleinsten Eigenpaare von K x = λ M x (Shift-Invert-Lanczos auf LDLᵀ),
    /// mit Sturm-Sequenz-Prüfung, dass keiner verpasst wurde.
    fn solve(&mut self, k: &SparseMatrix, m: &SparseMatrix, count: usize, shift: f64) -> Result<EigenPairs>;
}
```

### `plxs-elements` (M1 T2, M2 T4/T5)

```rust
pub trait ElementFormulation {
    fn kind(&self) -> ElementType;
    /// Freiheitsgrade je Knoten (3, 6, 2 oder 1 für Wärme), in der Reihenfolge der IR.
    fn dofs_per_node(&self) -> usize;
    fn integration(&self) -> &IntegrationRule;
    /// Tangentensteifigkeit und innere Kräfte am aktuellen Zustand (linear: K, f_int = K u).
    fn stiffness(&self, ctx: &ElementContext, out: &mut ElementMatrix) -> Result<()>;
    fn internal_force(&self, ctx: &ElementContext, out: &mut [f64]) -> Result<()>;
    fn mass(&self, ctx: &ElementContext, lumped: bool, out: &mut ElementMatrix) -> Result<()>;
    /// Spannungen/Dehnungen an den Integrationspunkten in den Zustand schreiben.
    fn update_state(&self, ctx: &ElementContext, state: &mut ElementState) -> Result<()>;
    /// Extrapolation von Integrationspunkt-Werten an die Knoten (Matrizen wie im Handbuch).
    fn extrapolate(&self, values: &[f64], out: &mut [f64]);
    /// Flächenlast auf Face n -> äquivalente Knotenkräfte.
    fn face_load(&self, ctx: &ElementContext, face: u8, pressure: f64, out: &mut [f64]) -> Result<()>;
    /// Expansion in Volumenelemente für die `.frd` (Schalen, Balken, 2D), sonst `None`.
    fn expansion(&self, ctx: &ElementContext) -> Option<Expansion>;
}
// `shape`-Modul: Formfunktionen und Integrationsregeln aller Familien an einem Ort; auch
// Flächenlasten, Kontaktsuche und Extrapolation greifen darauf zu.
```

### `plxs-materials` (M2 T8/T9)

```rust
pub trait MaterialLaw {
    /// Zahl der Zustandsvariablen je Integrationspunkt.
    fn state_size(&self) -> usize;
    /// Spannung und konsistente Tangente für eine Dehnung (bzw. ein Dehnungsinkrement),
    /// Temperatur und Zeitschritt; aktualisiert die Zustandsvariablen.
    fn stress(&self, input: &MaterialInput, state: &mut [f64]) -> Result<MaterialOutput>;
    /// Wärmeleitung und spezifische Wärme bei Temperatur T (für thermische Elemente).
    fn thermal(&self, temperature: f64) -> ThermalProperties;
}
// Implementierungen: LinearElastic, J2Plastic (isotrop/kinematisch/kombiniert, Radial Return),
// NortonCreep (implizit), jeweils mit Temperaturinterpolation aus den IR-Tabellen.
```

### `plxs-constraints` (M2 T6)

```rust
/// Alle Zwangsbedingungen eines Steps als Transformationsmatrix T (u = T u_red + u_0):
/// SPCs, Equations, Rigid Bodies, Ties, Transforms, Cyclic, Submodel-Werte.
pub struct ConstraintSet { .. }
impl ConstraintSet {
    /// Baut die Eliminationsmatrix; löst Überschneidungen (SPC auf abhängigem DOF, zwei MPCs
    /// auf demselben DOF) durch Re-Pivotierung auf, statt abzubrechen.
    pub fn build(model: &Model, step: &Step, state: &State, dofs: &DofMap) -> Result<ConstraintSet>;
    pub fn reduce(&self, k: &SparseMatrix) -> SparseMatrix;        // Tᵀ K T
    pub fn reduce_rhs(&self, f: &[f64], out: &mut [f64]);
    pub fn expand(&self, u_red: &[f64], out: &mut [f64]);
    /// Nichtlineare MPCs (Rigid Body mit großen Rotationen) je Newton-Iteration neu.
    pub fn update(&mut self, state: &State) -> Result<()>;
}
```

### `plxs-contact` (M3 T11/T12)

```rust
pub trait ContactFormulation {
    /// Kontaktsuche: aktive Slave-Knoten/-Integrationspunkte und ihre Master-Faces.
    fn search(&mut self, geometry: &ContactGeometry, state: &State) -> Result<ContactSet>;
    /// Superset des Matrixmusters für diesen Step (Kontaktsuche-Ergebnis + Reserve).
    fn pattern(&self, set: &ContactSet) -> PatternContribution;
    /// Kontaktsteifigkeit und -kräfte für die aktiven Paare (Penalty: Federn; Mortar: Lagrange).
    fn assemble(&self, set: &ContactSet, state: &State, k: &mut SparseAssembler, f: &mut [f64]) -> Result<()>;
    /// Kontaktstatus, Druck, Spalt je Slave-Punkt für CDIS/CSTR/CELS und `*CONTACT PRINT`.
    fn outputs(&self, set: &ContactSet, state: &State) -> ContactOutputs;
}
// Implementierungen in dieser Reihenfolge: NodeToSurfacePenalty, SurfaceToSurfacePenalty, Mortar.
```

### `plxs-procedures` (M1 T3, M2 T7/T8/T9, M3)

```rust
pub trait Procedure {
    /// Rechnet den Step auf dem Zustand; schreibt Inkremente in `results`, meldet Ereignisse.
    fn run(&self, ctx: &mut ProcedureContext, step: &Step, progress: &mut dyn Progress) -> Result<StepOutcome>;
}
// ProcedureContext: Modell, DofMap, Assembler, Solver, State, Results, Thread-Pool.
// Static (linear), Nonlinear (Newton mit automatischer Inkrementierung nach *CONTROLS-Defaults),
// Frequency, Buckle, Dynamic (HHT-α / explizit), HeatTransfer, Coupled/Uncoupled,
// ModalDynamics, SteadyStateDynamics, ComplexFrequency, Visco.
```

### `plxs-output` (M1 T3)

```rust
pub fn write_frd(path: &Path, model: &Model, results: &Results, binary: bool) -> Result<()>;
pub fn write_dat(path: &Path, results: &Results) -> Result<()>;
pub fn write_sta(path: &Path, rows: &[StaRow]) -> Result<()>;
pub fn write_cvg(path: &Path, rows: &[CvgRow]) -> Result<()>;
```

Abnahme: `plx_io::frd::read_frd` liest die eigene und die ccx-Datei, die geparsten Strukturen
sind gleich (Klasse D); `plx_io::dat::parse_dat` entsprechend. Blocküberschriften der `.dat`
wörtlich wie in `crates/plx-io/src/dat.rs` dokumentiert.

## Progress und stdout-Protokoll

`plxs_core::Progress` empfängt `ProgressEvent` (Step, Increment, Iteration, IncrementDone,
Cutback, Eigenvalue, StepDone, Warning, Info, JobDone). `plxs-cli` druckt daraus CalculiX'
Zeilen, die `plx-job` (`live.rs`) und `solver_check` lesen:

```
This is Version <x>            (plxs -v; die Startzeile enthält das Wort CalculiX)
 STEP n
 increment n attempt m
 increment size= <f>
 actual step time= <f>
 iteration n
 Job finished
*ERROR ...                     (Abbruch, Exit-Code 1)
*ERROR in calinput: keyword *X cannot be interpreted
```

stdout wird nach jeder Zeile geflusht. Exit-Code 0 nur nach `Job finished`.

## Stand nach M1 (lineare Statik, Volumenelemente)

Was M1 gebaut hat und wo es von den Signaturen oben abweicht (Entscheidungen des M1-Threads):

- `plxs-linalg`: `Pattern` ist ein CSC-Muster der **oberen Dreieckshälfte** (faer erwartet CSC;
  für eine symmetrische Matrix sind CSC und CSR der oberen Hälfte dieselben Daten).
  `Pattern::from_blocks(n, blöcke)` baut es aus den DOF-Listen der Elemente, `SparseMatrix::add_block`
  streut eine dichte Elementmatrix ein, `restrict(keep)` schneidet die freien DOFs heraus
  (SPC-Behandlung in M1; `plxs-constraints` mit `T` kommt in M2). `LinearSolver` wie oben,
  Implementierung `LdltSolver` (faer, supernodales LDLᵀ, AMD-Ordering, `inertia()` aus D);
  `LuSolver`/`CgSolver`/`EigenSolver` folgen in M2/M3.
- `plxs-elements`: statt des Traits `ElementFormulation` vorerst ein konkreter `SolidElement`
  (`solid(kind)`) mit `stiffness`, `strains`, `internal_force`, `face_load`, `body_force`,
  `volume`, `integration_point_coordinates`, `extrapolation()`; `mass` und `expansion` kommen
  mit M2 (Eigenfrequenzen, Strukturelemente). Das `shape`-Modul hält Formfunktionen,
  Flächen und Integrationsregeln aller sechs Familien.
- `plxs-materials`: `LinearElastic::of(material, temperatur)` (Temperaturinterpolation der
  IR-Tabelle), `isotropic_matrix`, `density`; der Trait `MaterialLaw` kommt mit der Plastizität.
- `plxs-assembly`: `DofMap` (3 DOF je Knoten), `Spc::of_step`, `Reduction` (freie DOFs,
  vorgeschriebene Werte), `ElementTable` (Formulierung, Materialindex, Dichte, IP-Layout),
  `assemble_stiffness` (rayon, Batches von ~64 MB, sequentielles Einstreuen), `load_vector`
  (CLOAD, DLOAD P/GRAV/CENTRIF, Amplituden).
- `plxs-procedures`: `run_step` verteilt auf `StaticLinear` (feste Inkremente aus
  `initial`/`period`, K und Faktorisierung einmal je Step, Reaktionen = K·u); `fields` und
  `history` bauen die Felder der `.frd` und die Blöcke der `.dat` in CalculiX-Reihenfolge.
- `plxs-output`: `write_frd(path, model, results, version)` schreibt ASCII (binär folgt),
  `write_dat`, `write_sta`, `write_cvg`.
- `plxs-session`: wie oben; `run_step` führt die Prozedur auf dem Pool in einem Scoped Thread
  aus und reicht `ProgressEvent`s über einen Kanal an das (nicht `Send`-pflichtige) `progress`.
- Abnahmetest: `solver/plxs-cli/tests/static_corpus.rs` löst alle `static/*`-Fixtures mit dem
  gebauten `plxs` und vergleicht mit den Golden Files (läuft im CI ohne ccx). Der Selbsttest
  der App läuft mit `PLXS=target/debug/plxs cargo test -p plx-app plxs_passes`.

### Was von CalculiX nur durch Beobachtung bekannt ist

Kein CalculiX-Quelltext wurde gelesen. Die folgenden Konventionen wurden aus ccx-Ausgaben
(Einzelelement-Orakel mit zufälligen Lastfällen, `scripts/`-freie Skripte im Thread) abgeleitet
und sind in `plxs-elements/src/solid.rs` bzw. `plxs-procedures/src/fields.rs` dokumentiert:

- **Knotenextrapolation** (`extrapolation_matrix`): C3D8/C3D20R mit den Tabellenwerten
  2.549/−0.683/0.183/−0.04904 (Ecken) und 0.933/−0.25/0.06698 (Mittelknoten), d. h. trilineare
  Lagrange-Werte auf ~4 Stellen gerundet; C3D20 (27 Punkte) die Least-Squares-Anpassung der
  Serendipity-Formfunktionen, auf 5 Dezimalen gerundet (die triquadratische Lagrange-Lösung
  weicht bei verzerrten Elementen um Prozente ab); Tetraeder exakt linear; C3D15 linear im
  Dreieck × Least-Squares-Gerade durch die drei ζ-Ebenen; **C3D6**: Gewichte 1.1455/−0.1455,
  wobei ccx den bevorzugten Punkt nach der Parität der lokalen Knotennummer wählt statt nach
  der ζ-Lage des Knotens (vermutlich ein ccx-Fehler; wird nachgebildet, damit Klasse A hält).
- **FORC** in der `.frd` ist K·u an allen Knoten (innere Kräfte), nicht nur die Reaktionen.
- **Knotenspannungen** sind der ungewichtete Mittelwert der je Element extrapolierten Werte.
- **C3D8R**: ccx = Uniform-Strain-Element (mittlere B-Matrix, exaktes Volumen) plus
  Flanagan-Belytschko-Hourglass-Steifigkeit mit `ε = 0.004·E·V·Σ_j b_j·b_j` je Mode (unabhängig
  von ν; für Quader auf 1e-6 bestätigt, Fixture `elementtyp-c3d8r` grün). Bei verzerrten
  Elementen nutzt ccx leicht andere Hourglass-Vektoren (~0,1 % der Steifigkeit, nicht
  identifiziert); plxs nimmt die orthogonalisierten γ-Vektoren nach Flanagan-Belytschko.
- **ERROR (STR(%))**: ccx schreibt das Feld immer, wenn `S` angefordert ist und `NOE` fehlt.
  Der Wert ist je Element konstant und an den Knoten das Maximum der Nachbarelemente, folgt
  aber **nicht** der Handbuchbeschreibung (Spannweite der ersten Hauptspannung an den
  Integrationspunkten relativ zu ihrem Betragsmaximum); keine der getesteten Formeln
  (Hauptspannungen, von Mises, Komponenten, Spannweite/Standardabweichung, IP- oder
  Knotenwerte) reproduziert die Zahlen. plxs schreibt die Handbuchformel; `plxs-verify`
  vergleicht beim Feld `ERROR` nur die Präsenz. Die App zeigt das Feld nur an.

### Benchmark (M1-Abnahme)

Block 38×38×38 C3D8 (59 319 Knoten, 177 957 DOF, 173 394 Gleichungen), Fuß fest, Druck
oben, 4 Threads, im Cloud-Container (4 Kerne; ccx 2.21 aus Debian mit SPOOLES, kein PaStiX):

| Löser | Wand-Zeit | davon Faktorisierung |
|---|---|---|
| ccx 2.21, SPOOLES, 4 Threads | 91 s | – |
| plxs 0.3.15, faer LDLᵀ (AMD), 4 Threads | 62 s | 59 s (Assemblierung 0,8 s, Lösen 0,5 s) |

Verschiebungen stimmen auf 2e-6 überein. Die Faktorisierung dominiert: AMD erzeugt für den
3D-Block 444 Mio. Einträge in L (natürliche Ordnung 864 Mio.); ein Nested-Dissection-Ordering
(Minimum-Fill für 3D-Gitter typisch 3–5× weniger) ist der Hebel für M4, nicht die
Assemblierung. Die Messlatte des Konzepts (ccx/PaStiX 8,7 s) stammt von anderer Hardware.

## Stand nach M2 T7 (Eigenwerte und Buckling)

Was T7 gebaut hat (`*FREQUENCY`, `*BUCKLE`, beide auch mit `PERTURBATION`):

- `plxs-linalg`: `ShiftInvertLanczos` (`eigen.rs`) implementiert `EigenSolver` auf faer:
  Shift-Invert-Lanczos mit voller Reorthogonalisierung (DGKS), dickem Restart und **Locking**
  konvergierter Vektoren mit frischem Startvektor (so werden exakt entartete Eigenwerte wie die
  beiden Biegemoden eines quadratischen Stabs alle gefunden); Sturm-Check über die Trägheit
  der LDLᵀ-Faktorisierung (`LdltSolver::inertia`, negative Pivots). `LdltSolver` kennt jetzt
  eine dynamische Regularisierung winziger Pivots (`set_regularization`), weil faer ohne
  Bunch-Kaufman-Pivotisierung bei indefinitem `K − σM` an exakten Nullpivots scheitert;
  Shifts dicht an einem Eigenwert bleiben zu vermeiden. `SparseMatrix::add_scaled` und
  `linear_combination` für `K − σM` auf gleichem Muster.
- `plxs-elements`: `SolidElement::mass` (konsistent, mit der **Integrationsregel des
  Elements selbst**: C3D20R mit 2×2×2, nicht voll – nur so stimmen die ccx-Eigenwerte auf
  7 Stellen), `geometric_stiffness` (Anfangsspannungsmatrix aus IP-Spannungen),
  `initial_displacement_stiffness` und `linearized_strains` (total-lagrangesch, Bathe 6.3.4,
  `B_L = B_0 + B_1(u)`); neue Regeln `tet14`, `wedge18` für Massenmatrizen von C3D10/C3D15.
- `plxs-assembly`: `assemble_mass` (Fehler ohne `*DENSITY`), `assemble_geometric_stiffness`,
  `assemble_tangent_stiffness` (`K + K_u(u) + K_σ(σ)`).
- `plxs-procedures`: `eigen::FrequencyProcedure` (Shift 0 bzw. `(2π·untere Grenze)²`,
  negative Ersatzshifts bei singulärem `K`, obere Grenze als Filter, Massennormierung,
  Partizipationsfaktoren `Γ = φᵀ M r` für sechs Starrkörperbewegungen um den globalen
  Ursprung auf den **freien** DOFs, effektive Modalmasse `Γ²`, STORAGE → `state.stored_modes`)
  und `eigen::BuckleProcedure` (statische Referenzlösung als Inkrement 0, `K_σ` der
  Beullast, `(K + λK_σ)φ = 0` über den Operator `K⁻¹K_σ` im `K`-Skalarprodukt, kleinstes
  positives λ). `fields::build_fields` und `history::write_table` sind herausgelöst, damit
  Prozeduren ihre Felder und `.dat`-Tabellen selbst zusammensetzen.
- `plxs-output`: Eigenwerttabellen der `.dat` (`EigenTable`, Überschriften und Spalten wie
  ccx, TOTAL-Zeile) als `HistoryBlock`s mit leerem `set`; `.frd` mit Lastfallnummern,
  Modalkopfzeilen (`1PGM`/`1PGK`/`1PHID`/`1PSUBC`/`1PMODE`) und Analysetext `MODAL`.
- `plxs-cli` schreibt `<job>.eig` (postcard von `StoredModes`), wenn ein Step STORAGE=YES
  verlangt; `plxs-verify` vergleicht Moden als Unterräume (Vorzeichen, entartete Paare,
  Partizipationsfaktoren invariant, siehe `compare.rs`).
- Abnahmetest `solver/plxs-cli/tests/eigen_corpus.rs`: alle `eigen/*`-Fixtures grün.

### Was von CalculiX nur durch Beobachtung bekannt ist (T7)

Aus ccx-2.21-Läufen (Fixtures und Orakel-Decks mit variierter Vorlast und rechteckigem
Querschnitt) abgelesen, kein Quelltext:

- **Lastfallnummern** der `.frd`: 100 + eine je Step (statische Inkremente teilen sich die
  Nummer ihres Steps); Frequenzmode k → Step-Nummer + k − 1; Beulen: Referenzlösung →
  Step-Nummer, Mode k → Step-Nummer + k. Wertspalte mit 10 signifikanten Stellen.
- **Massenmatrix** mit der Integrationsregel des Elements (reduziert für C3D20R).
- **Beulmoden** sind auf Einheits-Formänderungsenergie normiert: `φᵀ K φ = 1` mit der
  Referenzsteifigkeit (Euklid-Norm der ccx-Moden exakt `1/√(φᵀKφ)` der unnormierten).
- **Perturbation**: `K_ref = K + K_u(u_pre) + K_σ(σ_pre)` (beide Terme nötig: die
  Eigenfrequenzen von `eigenfrequenz` stimmen nur so auf 7 Stellen). Beim Beulen mit Vorlast
  wird die Beullast statisch mit `K_ref` gelöst (Verschiebungen auf 6 Stellen gleich), die
  Spannung der Beullast ist das **linearisierte Inkrement** `σ_b = D B_L(u_pre) u_b`
  (mit `B_0 u_b` liegt der Beulwert bei Vorlast 0,8 % daneben, so auf 7 Stellen: fünf
  Orakel-Fälle mit axialer, seitlicher und Zug-Vorlast). Die Spannungen und Dehnungen der
  Referenzlösung und der Moden eines perturbierten `*FREQUENCY` sind ebenfalls
  `D B_L(u_pre) φ`; die **Beulformen** dagegen bekommen die linearen Dehnungen `B_0 φ`
  (mit `B_L` wären Schubspannungen um ein Viertel falsch). Ein Restterm bis 1e-3 an den
  Lastknoten des freien Endes ist nicht identifiziert (Fixture `vorspannung-beulen` deshalb
  Klasse B; Eigenwerte, Referenzlösung und Modenformen stimmen auf 1e-5).
- **FORC** von Referenzlösung und Moden ist die innere Kraft `∫ B_0ᵀ σ dV` (bei Vorlast nicht
  der Lastvektor: die Anfangsspannungsmatrix der Vorlast fehlt darin, ccx-Werte an freien
  Knoten entsprechend ungleich null).
- **Lanczos-Genauigkeit**: der zweite Parameter von `*BUCKLE` (Writer: 1e-4) stoppt ccx bei
  dicht liegenden Beulwerten bis 2e-5 vom Eigenwert entfernt; die Eigenvektoren von ccx sind
  auf etwa 1e-5 konvergiert (Modenformen bekommen in `plxs-verify` zehnfache Toleranz). Der
  Writer-Test `vorspannung-beulen` verlangt deshalb Genauigkeit 1e-10 und beide Moden des
  fast entarteten Paars.
- **Partizipationsfaktoren** werden mit den Starrkörperbewegungen um den globalen Ursprung auf
  den freien DOFs gebildet (feste DOFs tragen nicht bei).
## Stand nach M2 T10 (Lasten und Amplituden)

Lastvektor (`plxs-assembly::load_vector`: CLOAD, DLOAD P/GRAV/CENTRIF, Amplituden mit
`TIME=TOTAL TIME`, `SHIFTX`, `SHIFTY`) stammt aus M1; T10 hat die Regeln dahinter gegen ccx 2.21
geprüft und die Lücken geschlossen. Alles unten wurde durch ccx-Läufe kleiner Mehr-Step-Modelle
beobachtet, nicht aus dem Quelltext:

- **Lineare `*STATIC`-Steps werden einmal am Step-Ende gelöst**, auch mit kleinerem
  Anfangsinkrement oder `DIRECT`: ein Inkrement der Größe der Periode in `.frd`, `.dat` und
  `.sta`. Inkremente gibt es erst mit NLGEOM bzw. anderen Nichtlinearitäten (T8).
- **Zeitwerte**: Die `.frd` (`100CL`) und die `.dat`-Blocküberschriften (`... and time`) tragen
  die **Gesamtzeit**, nicht die Step-Zeit (`ResultsIncrement::value` = Gesamtzeit bei
  `IncrementKind::Time`).
- **Amplituden**: Wert bei Zeit t = Tabelle(t − SHIFTX) + SHIFTY, linear zwischen den Punkten,
  vor dem ersten und nach dem letzten Punkt konstant; t ist die Step-Zeit oder mit
  `TIME=TOTAL TIME` die Gesamtzeit.
- **Übernahme in den nächsten Step** (`plxs-input/src/carry.rs`, aufgelöst im Leser, die IR
  bleibt unverändert):
  - Ein `*CLOAD` ohne `OP=NEW` ersetzt die übernommene Last an demselben Knoten und DOF;
    andere Knoten eines übernommenen Sets behalten ihre (das Set wird dafür in Einzelknoten
    aufgeteilt). Innerhalb eines Steps addieren sich Karten auf denselben Knoten/DOF.
  - `*DLOAD`: `P<n>` ersetzt den Druck derselben Elementfläche, `GRAV` die Schwerkraft
    derselben Richtung (eine andere Richtung kommt hinzu), `CENTRIF` die Zentrifugallast mit
    demselben Punkt und derselben Achse. Eine `CENTRIF` mit anderer Achse ignoriert ccx (die
    übernommene bleibt); plxs ebenso, mit Warnung. Innerhalb eines Steps addiert sich alles.
  - Randbedingungen und Lasten mit einer **Step-Zeit-Amplitude werden mit ihrem Wert am
    Step-Ende eingefroren** (Amplitude entfällt); Total-Time-Amplituden wirken weiter.
    Geprüft für `*BOUNDARY`, `*CLOAD`, `*DLOAD` P/GRAV/CENTRIF; für `*CFLUX`, `*DFLUX`,
    `*FILM`, `*RADIATE` nicht geprüft und nicht umgesetzt (T9).
  - `OP=NEW` auf einer zweiten Karte derselben Art im Step lässt die erste Karte stehen (wie
    ccx).
- PrePoLix selbst schreibt in jedem Step `OP=NEW` und alle Lasten neu; die Übernahmeregeln
  greifen nur für Eingaben von Hand. Wandernde Lasten sind gewöhnliche Step-Folgen.

Offen für andere Threads:

- **T8 (nichtlinear)**: `DefaultAmplitude::Ramp` rampt in `amplitude_factor` von 0 auf den
  Wert. ccx rampt in einem Folgestep von dem Wert am Ende des vorigen Steps auf den neuen;
  übernommene, unveränderte Lasten bleiben also konstant. Für lineare Steps egal, für NLGEOM
  nötig (Vorwert je Last im Zustand halten).
- **T5/T6**: `*CLOAD` auf DOF 4-6 (Momente) läuft durch `load_vector`, sobald `DofMap` den
  Knoten Rotations-DOFs gibt; bis dahin Warnung. Momente am Referenzpunkt schreibt PrePoLix
  als DOF 1-3 des Rotationsknotens eines `*RIGID BODY` (T6).
- Der Zähler im `100CL`-Kopf (ccx: 101, 102, ... je Ausgabezeitpunkt des Jobs) ist in plxs
  fest 101; `plx-io` liest für Zeitinkremente nur den `1PSTEP`-Kopf.
- `*DESIGN VARIABLES` wird nur gelesen (`Model::design_variables`), gerechnet wird nichts.

Abnahme: `solver/plxs-cli/tests/loads_corpus.rs` (alle `loads/*`-Fixtures, läuft im CI ohne
ccx), dazu die Übernahmetests in `plxs-input/src/tests.rs`.

## Was M1 zu liefern hat (Abnahme)

- `solver_check::run` aus `plx-app` besteht mit `plxs` als Executable (Kragbalken mit Kraft
  und Druck, Selbsttest-Keywords).
- Alle Fixtures `static/*` (Stufe M1 in `FIXTURES.md`) sind grün:
  `cargo run -p plxs-verify --bin plxs-fixtures -- run --solver target/debug/plxs --area static`.
- Benchmark 176k-DOF-Block (Build-Doku) gegen ccx-PaStiX, 4 Threads, als Zahl im PR.
