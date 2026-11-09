# Schnittstellen des Nodalis-Solvers (`plxs`)

Stand M0 (2026-10-31). Dieses Dokument legt fest, worauf sich die Threads der Meilensteine
M1 bis M4 verlassen können: den Crate-Schnitt unter `solver/`, die IR in `plxs-core`, die
Trait-Signaturen der Fachcrates und die Regeln für Änderungen daran. Das Konzept dazu steht in
`KONZEPT_EIGENER_SOLVER.md` (Projektordner, Abschnitt 3).

## Grundregeln

- **Kein CalculiX-Code.** CalculiX steht unter GPL-2.0-only, Nodalis unter GPL-3.0-or-later;
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
`plxs-core` hängt bewusst nicht von `plx-model`/`plx-mesh` ab, damit der Solver ohne Nodalis
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
Submodelle, `*RESTART`, `*CONTROLS`. `*SENSITIVITY` rechnet plxs seit T17 (Abschnitt „Stand nach T17“).

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

`Snapshot { model_fingerprint, state, average_force }` (M3 T16): `average_force` ist der
zeitgemittelte Kraftmaßstab der Residuenprüfung, den die Session von Step zu Step trägt;
ohne ihn konvergiert ein fortgesetzter Lauf nicht genau wie der Lauf von vorn. Der
Fingerprint deckt das Modell ohne Sets, Surfaces und Amplituden ab (ein fortsetzendes Modell
darf solche ergänzen, siehe `KONZEPT_CCX_ABLOESUNG.md` 3.5).

### Restart und Zustand (M3 T16)

- `Session::snapshot()` liefert nach jedem Step den `Snapshot` (Kopie des `State`, Fingerprint,
  `average_force`); `Session::resume(model, snapshot, options)` setzt mit dem nächsten Step
  fort. Die Ergebnisse früherer Steps gehören nicht zur Session, sie bleiben beim Aufrufer
  (Nodalis hält sie gebunden, KONZEPT_CCX_ABLOESUNG 3.5).
- `restart::model_fingerprint` ist FNV-1a über die postcard-Bytes von Knoten, Elementen,
  Materialien, Sections, Constraints, Kontakt, Anfangsbedingungen und physikalischen
  Konstanten: stabil über Prozesse und Rust-Versionen. Sets, Surfaces, Amplituden und
  `*HEADING` zählen nicht, ein fortsetzendes Modell darf sie ergänzen. Der Fingerprint wird in
  `Session::new` vom Modell genommen, wie es übergeben wird (bevor Ties Knoten verschieben).
  `resume` prüft zusätzlich, dass die Felder des Zustands zur Knoten- und Elementzahl passen.
- `restart::encode_snapshot`/`decode_snapshot`: Binärform (`PLXSSNAP`, Formatnummer,
  postcard) für `<job>.s<n>.state` und gebundene Ergebnisse. Eine andere Formatnummer wird
  abgelehnt, nicht umgewandelt (keine Migration).
- Kommandozeile: `*RESTART, WRITE` in einem Step schreibt `<job>.rout` (`PLXSROUT`):
  `RestartFile { analysis, snapshots }` mit dem Modell und den Steps bis zum letzten Snapshot,
  wie gelesen (vor der Expansion von Schalen und Balken), weil eine Restart-Eingabe wie bei
  CalculiX keine Modelldefinition hat. Gespeichert wird der Zustand am Step-Ende (`FREQUENCY`
  > 0), `OVERLAY` behält nur den letzten. `*RESTART, READ, STEP=n` liest `<job>.rin`, liest die
  Eingabe mit `plxs_input::read_inp_continuing` weiter (Steps ab n+1 nummeriert, Carry-over
  vom Step n, neue Sets erlaubt), rechnet nur die neuen Steps und schreibt eine
  eigenständige `.frd` mit absoluten Step-Nummern, wie ccx. Das Format ist plxs-eigen, eine
  ccx-`.rout` liest plxs nicht.
- Abnahme: `solver/plxs-cli/tests/restart.rs` setzt Kriechen, wandernde Lasten, Amplituden
  über die Gesamtzeit und gelockerte/verriegelte Constraints nach jedem Step fort; die
  Ergebnisse stimmen mit dem Lauf von vorn auf 1e-6 überein (praktisch bitgleich).

### Profiling und Muster-Wiederverwendung (M3 T16)

- `plxs_procedures::profile::Profile` misst die Wandzeit je Phase (Assemblierung,
  Constraint-Reduktion, symbolische Analyse, Faktorisierung, Lösen, Ergebnisse) über alle
  Steps einer Session; `ProcedureContext::profile`, `Session::profile()`. `plxs --profile`
  oder `PLXS_PROFILE=1` druckt die Tabelle am Ende des Jobs.
- `LdltSolver::analyze` überspringt die symbolische Analyse, wenn das Muster dasselbe ist
  wie beim letzten Aufruf (Newton-Iterationen eines Steps, Steps desselben Modells);
  `analyses_reused()` zählt das. Das Muster-Superset bei Kontakt (Kontaktelemente ändern das
  Muster je Iteration) kommt mit T11.
- Messung (Container, 4 Kerne, 24³ C3D8, 45 000 Gleichungen, NLGEOM, 4 Inkremente, 8
  Iterationen). 2026-10-31 mit AMD: Faktorisierung 16,2 s von 20,3 s, symbolische Analyse
  0,2 s. 2026-11-01 mit Nested Dissection und baumparallelem LDLᵀ (master): Faktorisierung
  9,0 s von 15,3 s, die Analyse kostet einmal 2,2 s (ND ist teurer als AMD), die anderen 7
  werden wiederverwendet; ohne Wiederverwendung wären es rund 15 s mehr. Assemblierung 2,2 s,
  Reduktion 0,6 s, Lösen 0,6 s. Der Hebel bleibt die Faktorisierung.

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
// ModalDynamics, SteadyStateDynamics, ComplexFrequency, Visco, Sensitivity (T17).
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
Zeilen, die `plx-solve` (`live.rs`) und `solver_check` lesen:

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
- `plxs-output`: `write_frd(path, model, results, version)` schreibt ASCII, `write_frd_as(.., FrdFormat::Binary)`
  binär (Koordinaten f64 Flag 3, Ergebnisse f32 Flag 2; CLI `plxs -o bin`, M3 T16),
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
  von ν; für Quader auf 1e-6 bestätigt, Fixture `elementtyp-c3d8r` grün). Die
  Hourglass-Vektoren bei verzerrten Elementen hat M2 T4 identifiziert (siehe dort).
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

- **Lastfallnummern** der `.frd`: 100 + laufende Nummer des geschriebenen Inkrements über
  alle Steps (jeder Frequenzmode, die Beul-Referenzlösung und jeder Beulmode zählen wie ein
  statisches Inkrement); die fünfte Spalte der `100CL`-Zeile trägt dieselbe laufende Nummer.
  Wertspalte mit 10 signifikanten Stellen.
- **`*BUCKLE` mit MPCs**: Referenzlösung, `K`, `K_σ` und Moden laufen über
  `ModelConstraints::step_set` (homogene SPCs plus alle MPCs, `homogeneous_constraints`).
  Fixture `eigen/beulen-starr`: Stab mit verdrehgesichertem Starrkörperende. ccx findet
  mit der Vorgabegenauigkeit 1e-4 nur einen Mode des entarteten Paares, der Writer-Test
  fordert daher 1e-10 und vier Faktoren.
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
## Stand nach M2 T4 (2D-Elemente, restliche Volumenelemente)

- **2D-Elemente** (`plxs-elements/src/plane.rs`): CPS/CPE/CAX 3, 4, 4R, 6, 8, 8R werden wie
  in ccx in eine Lage Volumenelemente expandiert (3→C3D6, 4→C3D8, 4R→C3D8R, 6→C3D15,
  8→C3D20, 8R→C3D20R). Eben: Knoten bei z = ±t/2 (t aus `*SOLID SECTION`, Standard 1), bei
  quadratischen Elementen zusätzlich Mittelebenenknoten auf den senkrechten Kanten.
  CAX: Drehung um ±1° um die y-Achse (x = r cos φ, z = r sin φ), Steifigkeit, Lasten und
  Volumen ×180 (voller Umfang; CLOAD gilt für den vollen Umfang). Der expandierte
  Freiheitsgrad wird über `Expansion::dof_map` auf die 2D-Knoten kondensiert (Tᵀ K T);
  Scheibe: der dritte DOF des 2D-Knotens trägt die Dickenänderung (w oben = −w unten),
  Platte (CPE) und CAX: w = 0. `SolidElement` mit Expansion hat dieselbe öffentliche API wie
  ein Volumenelement; `plane(kind, dicke)` baut es.
- **Ausgabe**: die `.frd` schreibt die 2D-Elemente unexpandiert (frd-Typen 7/8/9/10).
  Berichtete Knotenwerte (Orakel-Anpassung, exakt bis zur Rundung): Verschiebungen,
  Spannungen und Dehnungen = Dickenmittel der rohen kartesischen Werte der expandierten
  Knoten, Simpson (unten + 4·Mitte + oben)/6 wo ein Mittelknoten existiert, sonst
  Mittelwert; FORC = Summe der rohen kartesischen Kräfte der expandierten Knoten (×180 bei
  CAX). `ElementTable::reported(model, u, f)` rechnet die Vektoren für `fields`/`history` um.
- **Unbenutzte DOFs** (`DofMap::unused`): Knoten ohne Element und der dritte DOF von CPE-
  und CAX-Knoten werden auf null gehalten, auch wenn eine Randbedingung sie nennt.
- **C3D8R**: Spannung/Dehnung aus der mittleren B-Matrix (wie die Steifigkeit), nicht aus
  B(0). Hourglass `K = ε/8 Σ γγᵀ ⊗ I` mit `γ = h − (h·z)·b_z`: ccx korrigiert die
  Hourglass-Basis nur mit der globalen z-Komponente (auf verzerrten und gedrehten Elementen
  auf 1e-6 bestätigt). Ein verzerrtes C3D8R ist deshalb in x/y nicht rotationsfrei; der
  Unit-Test prüft dort nur Translationen.
- **C3D8I**: Inkompatible Moden mit der Zentrums-Jacobimatrix, aber **ohne** Taylors
  Skalierung det J0 / det J (Steifigkeit eines verzerrten Elements trifft ccx damit auf 1e-7,
  mit Skalierung lag sie 2e-3 daneben). Das Element besteht den Patch-Test auf verzerrten
  Netzen deshalb nicht exakt, genau wie ccx. Betrifft auch die Schalenexpansion (S4 → C3D8I).
- **Flächenlasten**: Integrationsregel der Vierecksflächen wie ccx: 1 Punkt für C3D6 und
  C3D8R, 2×2 für C3D8/C3D8I, C3D20R und C3D15, 3×3 für C3D20.
- **Knotenkoordinaten** in der `.frd` laufen durch f32 (ccx schreibt einfache Genauigkeit).
- **Einzelelement-Orakel** (verzerrte Elemente, Steifigkeit über FORC = K·u, Spannungen bei
  Zufallsverschiebung): C3D4, C3D6, C3D8, C3D8R, C3D8I, C3D20, C3D20R und alle 2D-Typen
  ≤ 2e-6, C3D10 1e-6. **C3D10** extrapoliert in ccx mit auf fünf Dezimalen gerundeten
  Eckgewichten (1.92705/−0.30902, Zeilensumme 0.99999, eine gleichförmige Spannung kommt
  also 1e-5 zu niedrig heraus); plxs bildet das nach, Mittelknoten = Mittel der Ecken.
  **C3D15**: die Eckknotenwerte von ccx sind linear in den IP-Werten, aber mit einer Matrix,
  die bis ~1e-5 von der exakten (und von gerundeten Varianten) abweicht und nicht
  identifiziert ist; Feldabweichung ≤ 5e-6. Kleine Komponenten verlieren damit relativ zur
  eigenen Größe 1e-5 und mehr, deshalb vergleicht `plxs-verify` seit T4 relativ zur
  Feldnorm (`FIXTURES.md`).
- Fixtures `plane/ebene-{cps,cpe,cax}{3,4,4r,6,8,8r}` (Writer-Test
  `calculix_runs_every_plane_element_type`, verzerrtes Netz, Druck und Einzellast),
  Abnahmetest `solver/plxs-cli/tests/plane_corpus.rs`.

## Stand nach M2 T9 (Wärme, Wärmedehnung, Temperaturfelder)

Gebaut: `*HEAT TRANSFER` stationär und transient (automatische Inkremente, `DELTMX`, `DIRECT`),
`*CFLUX`, `*DFLUX` (Flächen und `BF`), `*FILM`, `*RADIATE` (ohne Hohlraumstrahlung),
`*INITIAL CONDITIONS, TYPE=TEMPERATURE`, `*TEMPERATURE` als Werte und mit `FILE=`/`BSTEP=`,
Wärmedehnung in `*STATIC`, `*COUPLED TEMPERATURE-DISPLACEMENT` und
`*UNCOUPLED TEMPERATURE-DISPLACEMENT` ohne NLGEOM (mit NLGEOM folgt nach T8), Ausgaben NT,
HFL und RFL in `.frd` und `.dat`.

Wo es liegt: `plxs-elements/src/heat.rs` (Leitungs- und Kapazitätsmatrix, Gradienten,
Flächenpunkte), `plxs-assembly/src/heat.rs` (Assemblierung, Lasten mit Tangente,
Temperatur-SPCs auf DOF 11), `plxs-procedures/src/heat.rs` (Inkrementschleife, Newton,
gekoppelte Mechanik) und `plxs-procedures/src/thermal.rs` (Anfangs- und Feldtemperaturen,
frd-Leser für `FILE=`, thermische Dehnungen). `plxs-core` bekam nur
`IncrementKind::Dynamic` (frd-Analysetyp 1 für transiente Inkremente).

### Was von CalculiX durch Beobachtung bekannt ist (Wärme)

Kein CalculiX-Quelltext wurde gelesen; alles aus Einzelelement-Orakeln mit ccx 2.21:

- **RFL** ist K·T (nur der Leitungsanteil) an allen Knoten, wie FORC = K·u.
- **Filme** mit konsistenter Matrix h∫NNᵀdA; die Umgebungstemperatur wirkt sofort voll, der
  Koeffizient wird in stationären Steps mit der Zeit hochgefahren. **Strahlung**:
  εσ((Ts−T0)⁴−(T−T0)⁴) mit an den Flächenpunkt interpolierter Temperatur.
- **Kapazität** konsistent, Rückwärts-Euler; sie wirkt nur auf die Raten freier Knoten, ein
  vorgeschriebener Temperatursprung erzeugt keine Kapazitätsreaktion.
- **Rampen**: transiente Steps setzen Randbedingungen und Lasten sprunghaft (STEP),
  stationäre Steps fahren vorgeschriebene Temperaturen vom Startwert und Flüsse/Filme von 0 an.
- **Inkremente**: Wachstum ×1,5, wenn im Step mindestens 2 Inkremente fertig sind, die letzten
  beiden je ≤ 4 Iterationen brauchten und das letzte nicht verkleinert wurde. Mit `DELTMX`
  wird ein Inkrement, dessen größte Änderung an freien Knoten `DELTMX` übersteigt, mit
  dt·0,85·DELTMX/ΔTmax wiederholt; Nichtkonvergenz viertelt. ccx zählt ein lineares Problem
  als 2 Iterationen.
- **Materialtemperatur**: lineare Hexaeder (C3D8, C3D8I, C3D8R) werten Leitfähigkeit,
  Kapazität und Ausdehnung bei der Element-Mitteltemperatur aus, alle anderen Elemente an den
  Integrationspunkten interpoliert.
- **Wärmedehnung** α(T)(T−Z) − α(T0)(T0−Z); C3D20 integriert die thermische Last mit 2×2×2
  Punkten wie C3D20R; FORC = K·u − f_th.
- **Flächenintegration** für Flüsse, Filme, Strahlung: lineare Dreiecke und die Vierecke von
  C3D6 ein Punkt, quadratische Dreiecke drei Punkte, Vierecke von C3D20 3×3, alle anderen
  Vierecke (C3D8*, C3D20R, C3D15) 2×2.
- **frd**: der Wert in `100CL` ist die Gesamtzeit (10 signifikante Stellen), Spalte 1 ist
  100 + laufende Nummer, die Inkrementspalte ebenfalls die laufende Nummer.
- **`FILE=`** nimmt den Namen wörtlich (mit `.frd`) und liest das letzte Inkrement von `BSTEP`.
- `.dat`-Überschriften: „temperatures“, „heat generation“, „total heat generation“,
  „heat flux (elem, integ.pnt.,qx,qy,qz)“.

### Bekannte Abweichungen

- **C3D8R** in Wärme: ccx rechnet einpunktig ohne Hourglass-Kontrolle (singulär, Ergebnisse
  unbrauchbar); plxs integriert die Leitung mit 2×2×2 und gibt den Fluss am einen Punkt aus.
- **C3D20 mit temperaturabhängiger Leitfähigkeit**: ccx' effektive Leitfähigkeit an den 27
  Punkten folgt keiner der getesteten Temperaturinterpolationen (Formfunktionen, Mittelwert,
  Ecken linear, Punktpermutationen); plxs interpoliert mit den Formfunktionen. Abweichung
  einige Prozent; C3D20R und alle anderen Typen stimmen auf ~1e-5.
- **C3D20-Spannungen mit Temperaturfeld** weichen ab, die Verschiebungen stimmen.
- **Lasten aus dem Vorstep** werden in stationären Steps mit mehreren Inkrementen von 0 an
  hochgefahren statt vom Vorwert (wie bei den mechanischen Lasten, siehe T10: Vorwert je Last
  im Zustand halten, gehört zu T8).
- Materialien ohne `*ELASTIC` sind für reine Wärmeleitung erlaubt (wie ccx); mechanische
  Steps melden den Fehler erst beim Assemblieren der Steifigkeit.
- `*TEMPERATURE, FILE=` liest nur ASCII-`.frd`; C3D8I-Inkompatible-Moden fehlen in der
  thermischen Last.
- **CPS8, CPE8, CAX8 mit Temperaturfeld**: die Verschiebungen weichen um etwa 1 % ab (die
  thermische Last nimmt wie bei C3D20 die reduzierte Integration; mit voller Integration ist
  es nicht besser). Alle anderen 2D-Typen stimmen auf Rundung.
- **CAX8R-Wärmefluss**: ccx schreibt eine dritte Komponente von etwa 1 % des Radialflusses
  (Kartesisches im 2°-Segment); plxs schreibt 0. Temperaturen und RFL stimmen.
- Abnahmetest: `solver/plxs-cli/tests/heat_corpus.rs` (alle `heat/*`, auch `film-2d` mit
  CPS8).

### 2D-Elemente in der Wärme (mit T4)

Ebene und axialsymmetrische Elemente rechnen auf ihrer Expansion: Leitungs- und
Kapazitätsmatrix, Körperwärme und Flächenpunkte im 3D-Element, die Temperatur ist auf allen
Lagen eines 2D-Knotens gleich (`Expansion::scalar_map`), Matrizen und Lasten werden auf die
2D-Knoten summiert (×180 bei CAX). Kanten `F<n>` sind die Flächen `n+2` der Expansion, mit
denselben Integrationsregeln wie die Volumenelemente (CPS8 3×3 wie C3D20). ccx schreibt für
Knoten von 2D-Elementen **RFL = 0** in die `.frd` (geprüft CPS4, CPS8, CPE4, CAX4); in der
`.dat` steht der echte Wert. plxs macht beides so. Einzelelement-Orakel aller CPS/CPE/CAX-
Typen (transient, Film, Körperwärme) stimmen auf ~1e-5, außer den 4R-Typen (C3D8R, in ccx
singulär, s. o.).

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
- **Signal-Amplituden** (Nodalis, `*AMPLITUDE, SHAPE=...`): plxs wertet die Form exakt zu jeder
  Zeit aus, statt einer Tabelle; außerhalb von Start..Ende ist der Wert 0 (plus SHIFTY).
  `SHAPE=PULSE` mit `Start, Breite` (Halbsinus, Ersatz für einen Dirac-Stoß), `SHAPE=SINE` mit
  `Start, Ende, Frequenz, Phase in Grad`, `SHAPE=CHIRP` mit `Start, Ende, Startfrequenz,
  Endfrequenz` und `SWEEP=LINEAR|LOG`. Wo eine Prozedur zwischen Knickpunkten linear
  interpoliert (Inkrementgrenzen der nichtlinearen Statik, Abschnitte der expliziten
  GPU-Integration), liefert `Amplitude::corner_times` 48 Stützstellen je Periode.
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
    Geprüft für `*BOUNDARY`, `*CLOAD`, `*DLOAD` P/GRAV/CENTRIF, seit T9 auch für `*CFLUX`
    und `*FILM`.
  - Wärmelasten (T9): `*CFLUX` ersetzt den übernommenen Fluss am selben Knoten, `*DFLUX` den
    mit demselben Label (`S<n>`, `BF`) am selben Element, `*FILM` und `*RADIATE` den auf
    derselben Fläche; innerhalb eines Steps addiert sich alles.
  - `OP=NEW` auf einer zweiten Karte derselben Art im Step lässt die erste Karte stehen (wie
    ccx).
- Nodalis selbst schreibt in jedem Step `OP=NEW` und alle Lasten neu; die Übernahmeregeln
  greifen nur für Eingaben von Hand. Wandernde Lasten sind gewöhnliche Step-Folgen.

Offen für andere Threads:

- **T8 (nichtlinear)**: `DefaultAmplitude::Ramp` rampt in `amplitude_factor` von 0 auf den
  Wert. ccx rampt in einem Folgestep von dem Wert am Ende des vorigen Steps auf den neuen;
  übernommene, unveränderte Lasten bleiben also konstant. Für lineare Steps egal, für NLGEOM
  nötig (Vorwert je Last im Zustand halten).
- **T5/T6**: `*CLOAD` auf DOF 4-6 (Momente) läuft durch `load_vector`, sobald `DofMap` den
  Knoten Rotations-DOFs gibt; bis dahin Warnung. Momente am Referenzpunkt schreibt Nodalis
  als DOF 1-3 des Rotationsknotens eines `*RIGID BODY` (T6).
- Der Zähler im `100CL`-Kopf (ccx: 101, 102, ... je Ausgabezeitpunkt des Jobs) wird seit T9
  wie bei ccx hochgezählt.
- `*DESIGN VARIABLES` wird nur gelesen (`Model::design_variables`), gerechnet wird nichts.

Abnahme: `solver/plxs-cli/tests/loads_corpus.rs` (alle `loads/*`-Fixtures, läuft im CI ohne
ccx), dazu die Übernahmetests in `plxs-input/src/tests.rs`.

## Stand nach M2 T5 (Schalen, Balken, Stäbe, U1)

- **Expansion** (`plxs-elements/src/expand/`): vor dem Rechnen wird die Analyse umgebaut,
  das Ergebnis ist ein Modell aus Volumenelementen plus Gleichungen; `OutputMap` bildet die
  Ergebnisse zurück auf das, was ccx schreibt. Die CLI ruft `expand::expand` nach dem Lesen
  und `OutputMap::apply` vor `write_frd`.
  - Schalen S3/S4/S4R/S6/S8/S8R → C3D6/C3D8I/C3D8R/C3D15/C3D20/C3D20R, drei Knoten je
    Normale (unten, Mitte, oben, Offset in Dicken). Balken B31/B32/B32R → C3D8I/C3D20/C3D20R,
    acht Knoten je Querschnitt (Ecken (−+, −−, +−, ++) in (1, 2), dann Kantenmitten);
    `*BEAM SECTION`-Vektor = 1-Richtung, 2-Richtung = t × n1. Profile: RECT, CIRC (die
    Eckknoten liegen bei 45° auf der Ellipse, Schlitzkoordinaten · 1/√2), PIPE, BOX. B32R
    integriert ccx 2.21 nicht reduziert, sondern je Profil eigen (Black-Box):
    RECT und CIRC 2 × 5 × 5 Gauß-Kronrod (0, ±1/√3, ±√(6/7), Gewichte 28/45, 27/55,
    98/495; exakt wie ccx); PIPE acht Punkte auf dem Kreis mit Radius √((r_a² + r_i²)/2),
    ab der 1-Achse, je ein Achtel der Ringfläche, axial die zwei Gauß-Punkte
    (Verschiebungen wie ccx); BOX Simpson mit fünf Punkten je Wand auf den Wandmittellinien
    (16 Punkte, Ecken geteilt), axial zwei Gauß-Punkte (2e-4 zu steif). Diese Elemente
    behalten ihre Balken-Section, `ElementTable` baut daraus die Regel
    (`expand::section_rule`, `SolidElement::with_rule`).
  - Knotennummern wie ccx: ab größter Id + 1 (+ eine Id je Knoten eines `*TRANSFORM`), Knoten
    aufsteigend, 3 bzw. 8 fortlaufende Ids je Expansion; nicht benutzte (Mitte von
    Schalen-Mittelknoten) bleiben Lücken. Die `.frd` enthält nur Knoten geschriebener
    Elemente.
  - Kopplung: ohne Knoten Mittelwert-MPC (Schale unten/oben, Balken die vier Ecken);
    Knoten (Kreuzung mehrerer Expansionen oder alle drei Rotationen einer Schale gehalten,
    nicht unter `*TRANSFORM`) als linearisierter dehnbarer Starrkörper mit Hilfsknoten für
    Rotation und Dehnung; Rotations-Randbedingungen und Momente über Mittelwert-Rotations-
    MPCs (Hilfsknoten, DOF 1 = Winkel). Voll gehaltene Balkenknoten sind keine Knoten; an
    gehaltenen B32-Endknoten folgen die Kantenmitten dem Knoten nur längs der Achse; in
    der Querschnittsebene bewegen sich gegenüberliegende Kantenmitten symmetrisch zum Knoten
    (Querkontraktion und Verzerrung frei; ccx-Beobachtung, balken_b32*_laengs).
  - Einzelkräfte auf Knoten ohne Knoten wirken direkt auf die expandierten Knoten:
    Schale (−1/2, 2, −1/2) an Ecken quadratischer Elemente, sonst 1/2, 1/2; Balken 1/4 je
    Ecke (FORC von ccx zeigt genau diese Werte).
  - `*TRANSFORM` (rechtwinklig) an Schalen-/Balkenknoten: Verschiebungs-Randbedingungen als
    Gleichung am Originalknoten, Rotationen als Mittelwert-Rotation um die lokale Achse
    (Handbuch), Lasten in globale Komponenten. Der Transform verlässt danach das Modell.
  - T3D2 = B31 mit quadratischem Querschnitt der Fläche (ccx 2.21 liefert identische
    Spannungen inklusive seines Faktors 0,99996), Verdrillung über eine Mittelwert-Rotation
    gehalten. Stäbe und Membranen schreibt ccx 2.21 auf den Originalknoten (be2, tr3, qu4):
    FORC = Summe, andere Felder (auch DISP) = Mittel über den Querschnitt.
  - `OUTPUT=2D` (an irgendeinem `*NODE FILE`/`*EL FILE`) schreibt alle Schalen und Balken
    ebenso auf den Originalelementen (S3/S6/S4/S8 als tr3/tr6/qu4/qu8, B31 als be2, B32 als
    be3 mit Knotenfolge Ende, Ende, Mitte). Querschnittsmittel wie ccx 2.21: Schale
    Simpson (1, 4, 1)/6 über unten, Mitte, oben, ohne Mittelknoten das Mittel beider
    Flächen; Balken das Mittel der vier Ecken; FORC die Summe aller expandierten Knoten.
    Wo mehrere Expansionen zusammentreffen, gilt das Mittel der Expansionen, DISP ist dort
    der eigene Wert des Knotens. Ohne Rotationen (wie ccx).
- **U1** (`plxs-elements/src/u1.rs`) nativ: Steifigkeit aus der Nachgiebigkeit des
  eingespannten Elements, wie ccx 2.21 sie rechnet (Black-Box, k von 0,003 bis 100 und
  zwei Elementlängen): Längs EA, Torsion G(I11 + I22) (ohne k), Biegung mit
  Endmomenten exakt nach Euler-Bernoulli, unter Endkraft L³/(3EI) + s(φ − 3)/(1 + φ) mit
  s = L/(kGA), φ = 12EI/(kGAL²) (nicht Timoshenkos L³/(3EI) + s). I11 steift die Biegung
  mit Durchbiegung in 1-Richtung. Der fünfte Wert von SECTION=GENERAL ist k (die IR nennt
  das Feld `torsion`). Rotationen der U1-Knoten liegen auf Hilfsknoten; `ElementTable`
  führt U1 als `LinearBlock` (nur K). Ausgabe wie ccx: keine Elemente, DISP und FORC mit
  sechs Komponenten.
- **MPCs**: die erzeugten Gleichungen der Expansion laufen über die Constraint-Elimination
  von T6 (`plxs-constraints`, `ModelConstraints::step_set`); das frühere `mpc.rs` von T5
  ist mit dem Merge von T6 entfallen. Knoten in Gleichungen oder Starrkörpern zählen nicht
  als unbenutzt (T4 hält sonst Knoten ohne Element fest). Originalknoten in Knotenflächen
  (Tie-Slaves) bleiben unabhängig: ihre Mittelwert-Gleichung macht einen expandierten
  Knoten abhängig, der Tie bindet den Originalknoten (ccx 2.21: die expandierten Knoten
  folgen nur im Mittel, die Schale bleibt drehbar). Jeder Knoten eines `*RIGID BODY`
  verschiebt die erste expandierte Knotennummer um eins (ccx-Beobachtung).
- **Dokumentierte Abweichungen** (Fixture-`deviations`, `FIXTURES.md`; seit Stufe 7 der
  CalculiX-Ablösung entfallen, die Referenzen sind plxs-Snapshots):
  - Balken mit Rotations-Randbedingung: ccx schreibt an den expandierten Knoten der
    gehaltenen Balkenknoten FORC, die nicht K·u sind (Summe nicht im Gleichgewicht mit den
    Lasten); plxs schreibt K·u.
  - U1: STRESS/TOSTRAIN/FORC von ccx sind elementinterne Werte; plxs schreibt K·u (mit
    Momenten) und keine Elementfelder.
- **M3D3** (`plxs-elements/src/membrane.rs`) nativ als CST-Scheibenelement in seiner
  Ebene (ccx 2.21 trifft das exakt): Steifigkeit als `LinearBlock`, Spannungen und
  Dehnungen konstant je Element, an den Knoten gemittelt (`ElementTable::membranes`,
  `fields::nodal_average_with`), Dickendehnung ε33 = −ν/(1 − ν)(ε11 + ε22). Quer zur Ebene
  keine Steifigkeit.
- **Offen**: die genaue
  BOX-Regel von ccx, die Knotenspannungen von PIPE (ccx extrapoliert anders als die lineare
  Querschnittsanpassung, bis 0,3 %),
  `*NODAL THICKNESS`, Druck auf Balken, U1 an Schalen-/Balkenknoten,
  zylindrischer `*TRANSFORM` an Schalenknoten, edge-tie-m3d4 (Mechanismus, siehe unten).
- Abnahmetest `solver/plxs-cli/tests/structural_corpus.rs` (27 von 33 grün, die offenen
  mit Grund gelistet).
## Stand nach M2 T6 (Constraints und MPCs)

`plxs-constraints` ist gefüllt; die Signatur oben (`ConstraintSet::build`) wurde aufgeteilt:

- `ModelConstraints::new(&mut Model)` bereitet einmal je Modell vor (in `Session::new`, vor der
  Elementtabelle): `*TIE` projiziert die Slave-Knoten auf die nächste Master-Fläche
  (Gauß-Newton auf den Flächenkoordinaten, Gitter-Suche) und verschiebt sie bei `ADJUST`
  (Default) auf die Fläche, wie ccx auch in der `.frd`; `*RIGID BODY` wird linearisiert
  (`u_i = u_ref + θ × (x_i − x_ref)`, θ = Verschiebungen des ROT NODE), `*EQUATION` übernommen,
  `*TRANSFORM` (R und C) als Rotation je Knoten. Alles wird zu linearen Bedingungen auf den
  **primären DOFs** `knoten * 3 + d` (an transformierten Knoten die lokalen Richtungen, wie
  ccx SPCs, Equation-Terme und CLOADs dort liest).
- `ModelConstraints::step_set(knoten, aktiv, spc_dofs) -> ConstraintSet` je Step: SPCs zuerst,
  dann Rigid Bodies, Equations, Ties. Ist der abhängige DOF schon gebunden, wird die
  Bedingung in den unabhängigen DOFs umgeschrieben und auf einen anderen Term pivotiert
  (bevorzugt derselbe Knoten; `ConstraintSet::notes`); widersprüchliche fallen mit Warnung
  heraus (`warnings`), redundante still. Ausnahme `*TIE`: gebundene Slave-DOFs bleiben wie bei
  ccx ungekoppelt. DOFs ohne Element und ohne Bedingung (z. B. reine Hilfsknoten) sind wie bei
  ccx inaktiv.
- `ConstraintSet`: `u = T v + g` mit `reduce(K) = TᵀKT` (neues Muster, `Pattern::from_columns`),
  `reduce_rhs`, `particular(spc_werte) = g`, `expand(v, g)`. Die Prozeduren nutzen das statt
  `Reduction` (die für SPC-only bleibt); M2-Threads mit eigenen Prozeduren (Eigenwerte,
  Nichtlinear, Wärme) sollten ebenfalls `step_set` nehmen, sonst fehlen MPCs.
- Mit 2D-Elementen (T4): Knoten außerhalb aller Elemente, die ein Constraint nennt
  (`ModelConstraints::referenced_nodes`, z. B. Ref-/Rot-Knoten eines Starrkörpers), werden in
  `Session::new` aus `DofMap::unused` genommen. Ein CPS/CPE-Knoten nimmt nur in der Ebene am
  Starrkörper teil (seine Dickenänderung bleibt frei, wie ccx); er steht für zwei Lagen
  z = ±t/2, damit Drehungen um x/y und die z-Verschiebung des Ref-Knotens bestimmt sind.
- `plxs-assembly`: `ElementTable::discrete` (SPRING1/2, GAPUNI; `entries` hält nur noch
  Kontinuumselemente, `entry_of` bildet Modell-Index auf Eintrag ab), `active_dofs()`,
  `rotate_concentrated_loads` (CLOAD lokal an transformierten Knoten),
  `Spc::apply_fixed` (`*BOUNDARY, FIXED` = Verschiebung am Ende des Vorsteps).
- `StaticLinear`: Gaps werden mit einer Statusiteration gelöst (offen/geschlossen bis
  stabil, ohne Reibung pfadunabhängig); ccx rechnet `GAPUNI` als nichtlineare Feder mit
  Inkrementen, das Ergebnis am Step-Ende ist dasselbe (Klasse B, die sechs
  `nonlinear/druck-nur-*` sind grün). Die Zugkraft σ∞ von `*GAP` wird ignoriert.

Beobachtete ccx-Konventionen (Orakel, kein Quelltext):

- Die `.frd` enthält nur Knoten, die zu Elementen gehören (Referenz-/Rotationsknoten von
  Rigid Bodies und reine Equation-Hilfsknoten fehlen); `SPRING2` und `GAPUNI` stehen als
  Linienelement Typ 11 im Netz, `SPRING1` nicht.
- Die Knotenmittelung von S/E zählt `SPRING2`/`GAPUNI` als Elemente mit Spannung null mit
  (an einem Knoten mit 2 Volumen- und 3 Federelementen ist der Wert 2/5 des Volumenmittels).
- `*GAP`: Feld 5 ist unbenutzt, Feld 6 die Steifigkeit (Default ≈ 1e12), Feld 7 σ∞; der
  Parser las vorher Feld 5 als Steifigkeit (= 0).
- Rigid Bodies verhalten sich in linearer Statik wie die linearisierte Bedingung (Fixtures
  auf 1e-6).

`plxs-verify`: Abweichungen sind jetzt relativ zur Norm des ganzen Feldes (alle Komponenten;
vorher je Komponente, wodurch reines Rundungsrauschen wie S12 bei einachsigem Zug als Fehler
zählte), mindestens `ABSOLUTE_FLOOR = 1e-7`. Neues Fixture-Feld `undetermined` für
Komponenten, die das Modell nicht bestimmt (Teil nur in z gehalten; ccx liefert dort
beliebige Starrkörperwerte), mit Begründung in `note`.

Abnahme: `solver/plxs-cli/tests/constraints_corpus.rs` (alle `constraints/*` plus
`nonlinear/druck-nur-*`; ausgenommen mit Grund: vier CPS8-Fixtures → T4,
`verteilte-temperatur` → T9/T10, `tet-schraeg`/`tet-vorspannung`: C3D10-Knotenspannungen
überall 1,7e-5 neben ccx bei Verschiebungen und Kräften auf 1e-6 → Elementthema) und
`held_dependent_dofs_are_repivoted` (Equations mit gehaltenem ersten Term ergeben dasselbe
wie die vom Writer umgestellten). Gegencheck 30³-C3D8-Block mit Rigid Body und Moment:
plxs 6,0 s, ccx 2.21 12,4 s (4 Threads), Verschiebungen auf 6,6e-6 gleich.

## Stand nach M2 T8 (Nichtlinearität und Plastizität)

Was T8 gebaut hat und wo es von den Signaturen oben abweicht:

- `plxs-materials`: `MaterialLaw { state_size, is_nonlinear, stress(&MaterialInput, &mut
  [f64]) -> MaterialOutput, elastic_matrix }` (kein `Result`, kein `thermal`: Wärme kommt mit
  T9). `MaterialInput` trägt die Gesamtdehnung (klein oder Green-Lagrange), Temperatur, `dt`,
  Gesamtzeit und `creep_active`; `MaterialOutput` Spannung und Tangente auf
  Ingenieurdehnungen. `law_of(material, temperatur, finite_strain)` liefert `LinearElastic`,
  `J2Plastic` oder `FiniteStrainJ2`. 14 Zustandsgrößen je Integrationspunkt: PEEQ, sechs
  plastische Dehnungen (bzw. unter NLGEOM `C_p⁻¹ − 1`), sechs Rückspannungen, Kriechrate.
- `J2Plastic` (Simo & Hughes, Box 3.2): Radial Return mit stückweise linearen Kurven,
  isotrop/kinematisch (Prager, `ΔX = Δκ·√(2/3)·n̂`)/kombiniert, konsistente algorithmische
  Tangente. Kombiniert: erste Zeile der Tabelle = Fließgrenze, Rest = kinematische Kurve
  (`*CYCLIC HARDENING` kommt bei Bedarf). Kriechen als Perzyna-Überspannung mit Norton
  `Δγ = Δt·A·t^m·(q − σ_y)^n`, implizit (geschachteltes Newton/Bisektion), nur unter `*VISCO`;
  `*STATIC` schaltet es ab (ccx-Regel). Ohne `*PLASTIC` null Fließgrenze.
- `FiniteStrainJ2`: multiplikative Zerlegung, logarithmische Dehnungen, Exponential-Return
  (Simo 1992): `C = 1 + 2E`, `U = √C`, `b_e^tr = U·C_p⁻¹·U`, Hencky-Dehnung aus dem Spektrum,
  kleine-Dehnungs-Kern darauf, `S = U⁻¹ τ U⁻¹`. Die Tangente `dS/dE` wird als zentrale
  Differenz gebildet (12 Kernauswertungen; analytisch wäre die Ableitung der Tensorfunktionen
  nötig). Die Fließkurve ist wahre Spannung über logarithmischer plastischer Dehnung; der
  Zugstab trifft diese Lösung auf 1e-6.
- `plxs-elements`: `SolidElement::nonlinear(coords, u, nlgeom, d_elastic, material, fe, ke)`:
  Total-Lagrange, `K = ∫B_LᵀDB_L + Lᵀ(S⊗I)L`, Material-Callback je Integrationspunkt
  (`PointKinematics`: F, E, Gewicht; `cauchy(&S)`). C3D8R mit mittleren Gradienten +
  Hourglass, C3D8I mit effektivem Gradienten der inkompatiblen Moden (exakt für den linearen
  Pfad; mit `nlgeom = false` reproduziert `nonlinear` genau `stiffness`).
- `plxs-assembly`: `assemble_nonlinear(table, model, input, state, k)` liefert `f_int`,
  Cauchy-Spannungen, Dehnungen, neue Zustandsgrößen, Flusssumme; der Zustand des
  Inkrementanfangs bleibt unverändert, bis die Prozedur akzeptiert.
- `plxs-procedures::nonlinear::NonlinearStatic` für `*STATIC` mit NLGEOM oder nichtlinearem
  Material und für `*VISCO`: Newton-Raphson mit den `*CONTROLS`-Defaults (`Controls::of_step`
  liest TIME INCREMENTATION und FIELD), automatische Inkrementierung (×1.5 nach zwei
  schnellen Inkrementen ≤ 4 Iterationen, ×0.75 nach > 10, Cutback 0.25 bei Divergenz, 0.5
  bei > 16 geschätzten Iterationen, maximal 5 Cutbacks, CETOL-Cutback für Kriechen), `.sta`
  und `.cvg` je Iteration, Progress-Events Increment/Iteration/IncrementDone. SPCs und
  MPCs (T6) werden wie in der linearen Prozedur eliminiert (`u = T v + g`, `Tᵀ K T Δv = Tᵀ r`);
  Federn und Gaps (T6) gehen in Tangente und innere Kraft ein, ein Gap-Statuswechsel
  erzwingt eine weitere Iteration. Vorgeschriebene Verschiebungen gehen als linearer
  Prädiktor `Tᵀ K T Δv = Tᵀ (r − K Δg)` mit der Tangente des konvergierten Zustands in die
  erste Iteration (sonst steht der erste Newton-Schritt auf einem stark verzerrten Zustand
  und konvergiert nur linear). Ebene und achsensymmetrische Elemente (T4) sind in
  nichtlinearen Steps noch nicht erlaubt (`not implemented yet`). Druck bleibt Totlast (keine
  Folgelast); die Unterschiede zu ccx liegen bei 2 % Dehnung unter 1e-3.
- Felder: `PE` (PEEQ, Knotenmittel), `ENER` (= ½σ:ε mit der Gesamtdehnung, wie ccx);
  `.dat`-Blöcke `equivalent plastic strain (elem, integ.pnt.,pe)` und `internal energy
  density (elem, integ.pnt.,energy)`, Elementblöcke vor Knotenblöcken (Reihenfolge des
  Nodalis-Writers; `compare_dat` vergleicht Sets nach Namen).
- `plxs-verify`: Klasse B vergleicht nur noch das Step-Ende (auch bei gleicher
  Inkrementanzahl: die Inkremente nach dem Fließbeginn liegen über der Toleranz), die
  Inkrementanzahl nur als Größenordnung (Faktor 3). Skalierung je Feld über alle
  Komponenten, nicht je Komponente.
- Abnahmetest `solver/plxs-cli/tests/nonlinear_corpus.rs`: alle `nonlinear/*`-Fixtures
  grün oder vom Solver mit `not implemented yet` abgelehnt; ein falsches Ergebnis fällt
  durch. Seit dem Merge von master 14/14 grün.
- Randbedingungen in nichtlinearen Folgeschritten laufen vom Wert am Ende des Vorschritts
  zum neuen Wert (`Spc::start_from_previous`, aus dem Kontakt-Thread übernommen); vorher
  wurde der Wert doppelt gezählt (`membran-vorgespannt`: U1 0,2 statt 0,1).

### Membranen nativ (M3D3, M3D4)

`membran-vorgespannt` (vorgespannter Streifen unter Druck, NLGEOM) lag mit der
C3D8I-Expansion bei U3 0,0212 statt 0,0271: der expandierte Block trägt durch Biegung.
ccx 2.21 als Blackbox (Version aus Debian, kein Quellcode gelesen):

- Ein M3D4-Streifen, gelenkig gelagert, Querlast, linear: Verschiebungen 1e10, also
  quer zur Ebene praktisch keine Steifigkeit. Die gleiche Schicht als C3D8 oder C3D8I mit
  Mittelwert-Gleichungen trägt endlich.
- In der Ebene ist M3D4 genau eine C3D8-Schicht, deren Dicke sich symmetrisch ändern darf
  (Gleichungen oben + unten = 2 × Mitte, getestet für Dicke 0,01 bis 10 an einem
  10 × 10-Element); für dünne Schichten also das Scheibenviereck mit 2 × 2 Punkten, die
  Abweichung wächst mit (Dicke/Elementgröße)².
- Unter NLGEOM trifft eine total-Lagrangesche Membran (Green-Lagrange, St. Venant-
  Kirchhoff im ebenen Spannungszustand) mit **mitgehendem Druck** ccx auf 1e-5
  (0,0270916 gegen 0,0270918); als Totlast 0,0270645.

Umsetzung: `plxs_elements::membrane::Membrane` (M3D3 ein Punkt, M3D4 2 × 2), lineare
Steifigkeit als `LinearBlock { membrane: true }`, in nichtlinearen Steps total Lagrange
in `assemble_nonlinear` (dort jetzt auch die übrigen `LinearBlock`s wie U1, die vorher
fehlten). Druck `P`/`P1` (+Normale) und `P2` (−Normale) über
`plxs_assembly::membrane_pressures`; unter NLGEOM als Folgelast (Kraft auf der
verformten Fläche, Laststeifigkeit nicht in der Tangente). Masse (konsistent) und
GRAV/CENTRIF auf Membranen ebenfalls; vorher wurden Lasten auf M3D3 still ignoriert.
Damit eine Membran ohne Vorspannung nicht singulär ist, bekommt sie eine winzige
Steifigkeit gegen die Neigung der Normalverschiebung (`OUT_OF_PLANE` = 1e-9 E t
∫∇N·∇N dA); ccx verhält sich genauso (in `edge-tie-m3d4` liegen die freien
Membranknoten auf der Geraden zwischen festem und gekoppeltem Rand, plxs trifft sie auf
1e-4). Der Block in `edge-tie-m3d4` hängt aber wie an einem Scharnier, seine Drehung
legen nur diese winzigen Steifigkeiten fest; die Fixture bleibt offen.

Nachtrag (Membranen II): In der Ebene ist M3D4 (M3D4R) in ccx exakt eine C3D8- (C3D8R-)
Schicht der Membrandicke, deren Ober- und Unterseite sich in der Ebene gleich und entlang
der Normalen symmetrisch bewegen (geprüft für Dicke 0,01 bis 10). plxs baut diese Schicht
je Viereck auf (auf die Ebene durch den Mittelpunkt projiziert), kondensiert die
Dickenänderung und nimmt ihre Steifigkeit für kleine Dehnungen; die Spannungen kommen aus
den Punkten der Schicht, mit den C3D8-Gewichten extrapoliert und über die Dicke gemittelt.
`membran-m3d4`, `membran-m3d4r` und `membran-s4r` sind damit exakt grün (3e-7). Unter
NLGEOM bleibt die total-Lagrangesche Membran; die Differenz Schicht minus eigene
Integration (Hourglass von M3D4R, (t/h)²-Term von M3D4) kommt als konstante Steifigkeit
dazu, vorher von linearen Feldern wegprojiziert (sonst kostet eine Starrkörperdrehung
eines verzerrten C3D8R Kraft).

Vorspannung: `*BUCKLE` nimmt die Anfangsspannungssteifigkeit der Membranen
(`add_membrane_geometric_stiffness`), `*FREQUENCY`/`*BUCKLE` mit PERTURBATION die
total-Lagrangesche Tangente der Membranen und die U1-Blöcke (fehlten vorher). Gegen ccx
2.21: vorgespannter Streifen (`membran-vorgespannt` + `*FREQUENCY, PERTURBATION`)
858,38 statt 858,24 Hz (2e-4), Mode 3 1,5e-3; C3D8I-Platte mit M3D4-Haut unter Druck,
Beulfaktor 1,16104 statt 1,16069 (3e-4; ohne Haut weicht plxs bei C3D8I schon um 5e-4
ab).

Offen: Druck auf Volumenelementen unter NLGEOM (bleibt Totlast, siehe oben).

### Beobachtungen an CalculiX 2.21 (nur aus Ausgaben)

- Mindestens zwei Iterationen je Inkrement, auch wenn die erste konvergiert.
- `.frd`-Inkremente tragen die **Gesamtzeit** (100CL), nicht die Step-Zeit.
- `ENER` ist ½σ:ε mit der Gesamtdehnung, nicht die elastische Energiedichte.
- Kinematische Verfestigung: bei Entlastung bricht ccx 2.21 mit „too many cutbacks“ ab,
  bei Lastumkehr liefert es je nach Rückkraft inkonsistente plastische Dehnungen; die
  Fixture `plastisch-kinematisch` ist deshalb monoton (Bauschinger im Unit-Test).
- Finite-Dehnungs-Plastizität: ccx’ Zugstab unter NLGEOM weicht von der exakten
  Lösung (wahre Spannung über logarithmischer plastischer Dehnung) in zweiter Ordnung ab:
  PE um +0,2 % bei 1 % Dehnung, +0,7 % bei 5 %; die Spannung auf 2e-4. plxs trifft die
  exakte Lösung. Die Fixture `plastisch-nlgeom` lässt PEEQ deshalb aus der Feldausgabe
  weg; S/E/U/RF stimmen auf 7e-4 (Cauchy aus Kirchhoff `τ/J`). Ein stark eingespannter
  Biegebalken mit 1 % plastischer Dehnung weicht an der Einspannecke um 0,6 % im
  hydrostatischen Anteil ab (Volumenänderung des plastischen Flusses zweiter Ordnung).
- `.dat`: Blöcke in der Reihenfolge der `*EL PRINT`/`*NODE PRINT`-Karten der Eingabe.

## Stand nach M3 T11 (Penalty-Kontakt Node-to-Face und Face-to-Face)

Was T11 gebaut hat und wo es von der Trait-Skizze oben abweicht (kein Trait, ein
konkretes `ContactModel`; Mortar siehe T12 unten):

- `plxs-contact::ContactModel::new(&mut Model)` löst die `*CONTACT PAIR`s auf: Slave-Punkte
  (Node-to-Face: die Slave-Knoten mit ihrer Knotenfläche; Face-to-Face: je Slave-Face vier
  Dreiecke um den Mittelpunkt mit der 7-Punkt-Regel, 28 Punkte je Viereck), Master-Faces,
  Gesetz (`law::Law`), Reibung (`law::FrictionLaw`), `*CLEARANCE`, `ADJUST` (verschiebt
  Slave-Knoten auf den Master, ändert also `model.nodes`). `begin_step` wendet
  `*MODEL CHANGE, TYPE=CONTACT PAIR` (über `State::inactive_pairs`) und `*CHANGE FRICTION`
  an; `prepare` sucht je Inkrement die Kandidaten-Faces (Gitter) und liefert die DOF-Blöcke
  für das Matrixmuster; `evaluate` liefert je Punkt eine Penalty-Feder (`ContactElement`:
  DOFs, symmetrische Tangente, innere Kraft, unsymmetrischer Rest) mit Return-Mapping für
  die Reibung; `output` die Knotenfelder und `*CONTACT PRINT`-Blöcke.
- Gleitreibung macht die Tangente unsymmetrisch (`dτ/dd = μ dp/dd` entlang der
  Gleitrichtung). `plxs-linalg::gmres` löst dann `(K + E) Δv = r` mit dem faktorisierten
  symmetrischen Teil als Vorkonditionierer (rechts, Restart 60); ohne Gleiten bleibt es
  beim direkten LDLᵀ-Solve. Ohne den Rest konvergiert μ = 10 nicht.
- Prädiktor: in der ersten Iteration eines Inkrements bekommen Punkte innerhalb des Cutoffs
  ohne Druck die Steifigkeit des Gesetzes und die Haftsteifigkeit, aber keine Kraft
  (Face-to-Face: offene Punkte; Node-to-Face: Knoten im Zug). Ohne sie treibt der erste
  Schritt einen nur über Kontakt gehaltenen Körper weg oder lässt alle Punkte gleiten.
- `plxs-procedures`: `*STATIC` mit Kontakt läuft immer nichtlinear (NonlinearStatic, auch
  ohne NLGEOM). Ein Wechsel des Kontaktstatus (offen/haftend/gleitend) erzwingt eine weitere
  Iteration wie ein Gap. CONT.EL. in `.cvg`/`.sta` zählt die Kontaktfedern mit Kraft.
  Kontakt in Wärme- und gekoppelten Steps sowie auf 2D-Elementen: `not implemented yet`.
- IR (additiv): `HistoryOutput.surfaces: Option<(slave, master)>` für
  `*CONTACT PRINT, SLAVE=, MASTER=`.
- Mehrschritt-Randbedingungen (auch für T8 relevant): `Spc::start_from_previous` lässt jede
  vorgeschriebene Verschiebung eines nichtlinearen Steps beim Wert des Vorschritts beginnen;
  RAMP geht linear zum neuen Wert, STEP/eigene Amplitude springt im ersten Inkrement. Vorher
  blieb ein im Folgestep neu gehaltener Freiheitsgrad auf seinem alten Wert stehen, und ein
  in beiden Steps gleich vorgegebener Wert wurde doppelt gezählt (Fixture
  `membran-vorgespannt`: U1 0,2 statt 0,1).
- Abnahmetest `solver/plxs-cli/tests/contact_corpus.rs`: mit T12 34 von 36 `contact/*`-Fixtures
  grün, der Rest (CPE8-Kontakt, Kontakt im gekoppelten Step) mit `not implemented yet`.
  Neu: neun handgeschriebene Varianten (`n2s-*` mit HARD/LINEAR/c0/EXPONENTIAL, Reibung,
  Haftung μ = 10, Clearance, Model Change, Contact Print; `s2s-contact-print`).
  `plxs-verify` kennt `noise` jetzt auch für `.dat`-Felder (`MOMENT_ABOUT_CG`).

### M3 T12: Mortar (`TYPE=MORTAR`, `LINMORTAR`)

- Duale Mortar-Formulierung (Wohlmuth): je Slave-Face duale Formfunktionen
  `ψ_k = Σ_j A_kj N_j`, `A = D M⁻¹` (`D = diag ∫N`, `M = ∫N Nᵀ`, über dieselben 28
  Integrationspunkte wie Face-to-Face; `geometry::dual_coefficients`). Je Slave-Knoten wird
  über seine Punkte mit `a = ψ_k dA` gemittelt: gewichteter Spalt `ḡ_k`, Normale,
  Relativverschiebung. Wegen `∫ψ_k N_j = δ_kj ∫N_k` ist die Slave-Seite der Knoten selbst, die
  Master-Seite die Mortar-Projektion. Die Feder des Knotens (Gesetz, Reibung, Element) ist
  dieselbe wie bei Face-to-Face (`Pair::respond`), mit der Fläche `∫N_k`.
- Ohne `*SURFACE BEHAVIOR` (oder HARD) erzwingt ccx den Kontakt exakt (Lagrange-Multiplikatoren,
  COPEN 0); plxs nimmt ein Penalty `K = 10⁴ E / h` (`law::MORTAR_STIFFNESS`, h typische
  Facegröße), die Restdurchdringung liegt weit unter der Toleranz. Mit LINEAR wirkt das Gesetz
  auf `ḡ_k`.
- Ausgabe: CPRESS ist der Knotenmultiplikator (= Knotenkraft / `∫N_k`), gelistet werden alle
  Slave-Knoten; CSLIP wird auch reibungsfrei geschrieben (ccx tut das; die ccx-Werte sind
  nicht identisch, aber gegen CPRESS vernachlässigbar). Getiedter Mortar-Kontakt läuft über die
  Integrationspunkte wie Face-to-Face.
- Fixtures: `kontakt-mortar`, `region-mortar` (Nodalis-Writer) und neu `mortar-frictional`,
  `mortar-linear`, `mortar-rough`, `mortar-switched-on`; alle stimmen inklusive `CONTACT`.

### Beobachtungen an CalculiX 2.21 (Kontakt, nur aus Ausgaben)

- Face-to-Face: Ein Slave-Punkt neben der Master-Fläche (Fußpunkt auf Rand oder Ecke der
  nächsten Face geklemmt, der Punkt selbst seitlich daneben) bekommt keine Feder
  (`BESIDE_TOLERANCE` = 1 % der typischen Kantenlänge seitlicher Abstand). Vorher galt der
  Normalabstand zum geklemmten Fußpunkt als Spalt: eine Slave-Fläche, die über das Ende der
  Master-Fläche hinausragt, bekam Scheinüberschneidungen in der Größe ihres Überstands
  (Smoke-Test 0.4.0, Lasche mit Bolzen, C3D10: bis 1,1 mm, ~10¹² Pa, Newton flatterte ohne
  Ende). Fixture `s2s-overhang`.
- Node-to-Face, LINEAR: geglättet `p = K d/2 + (K d/π)·atan(K d/(π σ∞))`, σ∞ ohne Angabe
  `E/70000` des Slave-Materials; Feder innerhalb `5 c0` (mit c0) bzw. `10⁻³ √A` (A
  Knotenfläche). EXPONENTIAL `p = p0·10^(2d/c0)` bis zum Cutoff c0. HARD = LINEAR mit
  K = 50 E. Reibung wirkt auch im Zug (Schub an Knoten mit negativem CPRESS), Grenze μ|p|.
  `*CLEARANCE` wird ignoriert. Ein TABULAR-Gesetz mit erstem Punkt (0, 0) ergab in ccx gar
  keinen Kontakt (Körper fliegt weg); keine Fixture.
- Face-to-Face: nur Überschneidung trägt (`p = K d`, σ∞ und c0 ohne Wirkung, EXPONENTIAL nur
  für d > 0, TABULAR ergab Druck null). Integration wie oben, exakt an den Contact-Print-Werten
  bestätigt (Reihenfolge der Punkte stimmt).
- `.frd` `CONTACT`: COPEN, CSLIP1, CSLIP2, CPRESS, CSHEAR1, CSHEAR2; COPEN negativ bei
  Durchdringung; CSLIP ist die gesamte tangentiale Relativverschiebung (bei Haftung = τ/λ),
  reibungsfrei null. Node-to-Face listet nur Knoten mit Feder, Face-to-Face alle Knoten.
  **Abweichung:** die Extrapolation der Face-to-Face-Werte auf die Knoten ist nicht
  identifiziert (manche Knoten tragen den exakten Knotenwert, andere einen
  Integrationspunktwert, CPRESS auch negativ); plxs passt je Face bilinear an und mittelt.
  Die S2S-Fixtures vergleichen `CONTACT` deshalb nicht (`deviations`), U/S/FORC und der
  Contact Print stimmen.
- `.dat` (`*CONTACT PRINT` braucht SLAVE/MASTER): Blöcke „… (slave element+face,…)“ bzw.
  „… (slave node,…)“ „for all contact elements and time“, „total number of contact elements
  for time“, „statistics for slave set …“ mit vier beschrifteten Zeilen (Kraft+Moment um den
  Ursprung, Schwerpunkt+mittlere Normale, Moment um den Schwerpunkt, Fläche+Normalkraft
  (+ = Zug)+Schub); die Statistik nur für Face-to-Face.
- `FORC` enthält die Kontaktkräfte.
- Node-to-Face mit Slave-Knoten, die über den Master-Rand hinausgleiten: ccx lässt im
  Folgestep Eckknoten fallen, Kantenmittenknoten mit gleichem Überstand nicht; die Regel ist
  nicht identifiziert (plxs: Projektion am Inkrementanfang mehr als 10⁻³ in natürlichen
  Koordinaten außerhalb → keine Feder). Deshalb keine N2S-Fixture zu CHANGE FRICTION; die
  S2S-Fixture `region-friction-change` deckt es ab.
## Stand nach M3 T13 (implizite Dynamik)

Was T13 gebaut hat (`*DYNAMIC` ohne `EXPLICIT`, linear elastisch, ohne `NLGEOM`):

- `plxs-procedures`: `dynamic::DynamicProcedure`, HHT-α in Verschiebungsform
  (α aus `ALPHA=`, Vorgabe −0,05, auf [−1/3, 0] begrenzt; β = (1−α)²/4, γ = ½−α),
  Systemmatrix `M/(βΔt²) + (1+α)γ/(βΔt) C + (1+α)K` mit Rayleigh-Dämpfung `C = αM + βK`
  aus `*DAMPING`, Randbedingungen über `ModelConstraints::step_set` (u = T v + g, wie die
  Statik). `DIRECT` und automatische Inkremente, Anfangsgeschwindigkeiten und -drehungen
  (`*INITIAL CONDITIONS, TYPE=VELOCITY`), Step-Folgen statisch → dynamisch → dynamisch
  (Verschiebungen, Geschwindigkeiten, Beschleunigungen und Energie in `State`).
  Ausgaben `U`, `V` (`VELO`), `S`, `E`, `RF` (innere Kräfte `K u`), `ENER`, `ERR`; `.dat`
  mit `U`, `V`, `RF`, `S`, `E` (`history::write_history_with_velocities`).
- `plxs-elements`: `SolidElement::dynamic_mass_rule` und `fully_integrated` (C3D20R → 3×3×3,
  C3D8R → C3D8), `mass_with` für eine gewählte Regel und für expandierte 2D-Elemente (Masse
  am 3D-Ersatz, dann kondensiert wie die Steifigkeit).
- `plxs-assembly`: `assemble_mass_with(.., MassRule::Full | Element, ..)`; die Eigenwerte
  behalten `MassRule::Element`.
- `plxs-procedures::fields`: `ENER` (½σ:ε, Knotenmittel) in `build_fields` für alle
  Prozeduren; `V` gilt nicht mehr als unbekannte Ausgabe.
- `plxs-input`: Steps ohne `AMPLITUDE=` bekommen in `*DYNAMIC` die Sprung-Amplitude.
- Abnahmetest `solver/plxs-cli/tests/dynamic_corpus.rs`: alle `dynamic/*`-Fixtures mit
  Referenz grün (Abweichung ≤ 7,5e-6 relativ zur Feldnorm). `dynamik` und
  `dynamik_gedaempft` (60 Inkremente, Goldens zu groß fürs Repo) stimmen lokal auf 6e-6 bzw.
  7,5e-6; neu sind die kleinen Writer-Fixtures `dynamik-kurz` (Rayleigh α und β),
  `dynamik-automatisch` und `statisch-dynamisch`.

### Was von CalculiX durch Beobachtung bekannt ist (T13)

Aus ccx-2.21-Läufen (Fixtures, Ein-Freiheitsgrad-Decks, Einzelelemente C3D8/C3D8R/C3D20/
C3D20R/C3D4/C3D10/C3D6/C3D15/CPS8/CAX8 mit variiertem Δt und α) abgelesen, kein Quelltext:

- **Vorgabe-Amplitude** eines `*DYNAMIC`-Steps ohne `AMPLITUDE=` ist der Sprung (STEP), nicht
  die Rampe der Statik.
- **Anfangsbeschleunigung**: ccx löst nicht `M a₀ = f₀ − C v₀ − K u₀`, sondern
  `(M + (1+α)β h² K) a₀ = f₀ − C v₀ − K u₀` mit `h = Δt/10` (dem Faktor von K auf sieben
  Stellen), **ohne** Dämpfungsterm in der Matrix. In dieser Matrix stehen Steifigkeit und
  Masse reduzierter Elemente **voll integriert** (C3D20R wie C3D20 mit 3×3×3, C3D8R wie
  C3D8), C3D4 nur mit einem Viertel seiner Steifigkeit; `K u₀` auf der rechten Seite ist
  ebenfalls voll integriert (ohne das Viertel). Das hält `a₀` auch dort endlich, wo die
  reduzierte Masse von C3D20R allein singulär ist.
- **Massenmatrix der Zeitschritte**: konsistent mit der vollen Regel (C3D20R 3×3×3) – anders
  als bei den Eigenwerten (Regel des Elements, T7).
- **Automatische Inkremente** eines linearen dynamischen Steps: die ersten beiden mit der
  Anfangsgröße, ab dem dritten jeweils ×1,5 bis zur Maximalgröße, das letzte bis zum
  Step-Ende gekürzt; jedes Inkrement zählt in der `.sta` als zwei Iterationen.
- **`.frd`**: Analysetyp 1 (dynamisch), Wert = Gesamtzeit, Feldreihenfolge DISP, VELO,
  STRESS, TOSTRAIN, FORC, ENER, ERROR; FORC ist `K u`.
- **ENER über Steps**: ein dynamischer Step integriert die Energie von null an auf die
  Energie des vorigen Steps drauf (nach statischer Vorlast: statische Energie plus ½σ:ε des
  dynamischen Zustands). `State::energy` trägt sie weiter. ccx verlangt `ENER` schon im
  ersten Step, wenn ein späterer Step dynamisch ist und `ENER` ausgibt.

### Bekannte Abweichungen (T13)

- `*DYNAMIC, EXPLICIT` wird abgelehnt (Unsupported): das zentrale-Differenzen-Schema von ccx
  ist nicht identifiziert (an einem Ein-Freiheitsgrad-Deck passt weder Standard-CD noch
  HHT mit α = 0). `NLGEOM` und nichtlineare Werkstoffe in der Dynamik kommen mit T8.
- **Vorlast**: nach einem statischen Step weichen VELO, Spannungen und Kräfte im ersten
  dynamischen Inkrement um etwa 2e-3 ab (DISP 2e-5); die rechte Seite der
  Anfangsbeschleunigung bei `u₀ ≠ 0` ist nicht vollständig identifiziert (volle Integration
  ist zehnmal näher als `K u₀` des Elements). Fixture `statisch-dynamisch` vergleicht diese
  Felder deshalb nicht (`deviations`).
- **Anfangsgeschwindigkeit zusammen mit Rayleigh-Dämpfung**: nicht identifiziert (die
  Zeitschritte weichen auch bei freiem `a₀` ab).
- **C3D8I** (inkompatible Moden tragen in ccx vermutlich Masse), **CPS4/CPE4/CAX4**
  (4e-2) sowie **Schalen und Balken** (S8R 2,5e-4, S4-Kantenkopplung 5,6e-4, B32R 5,9e-4,
  S8 4e-5 mit Rampe) weichen in den Zeitschritten ab: die Massenkonvention der expandierten
  Elemente ist offen. Fixtures dafür gibt es noch nicht.
## Stand nach T17 (Sensitivity)

`plxs-procedures::sensitivity` rechnet `*SENSITIVITY` mit `*DESIGN VARIABLES, TYPE=COORDINATE`
und den Design Responses, die Nodalis schreibt: `MASS`, `STRAINENERGY` (beide mit Element-Set)
und `EIGENFREQUENCY` (nach `*FREQUENCY, STORAGE=YES`, die Moden kommen aus
`State::stored_modes`), dazu `*FILTER, TYPE=LINEAR`. Andere Responses (`STRESS`, `DISPLACEMENT`
usw.), andere Filtertypen und `NLGEOM` melden `Unsupported`. `IncrementKind::Sensitivity` (IR,
angehängt) kennzeichnet die Datensätze; die `.frd` schreibt sie als Analyseart 3, die `.dat`
die Tabelle `D E S I G N   R E S P O N S E   I N F O R M A T I O N`
(`plxs_output::dat::DESIGN_RESPONSE_TABLE`, Set = Response-Namen mit Komma).

Ableitungen: je Designknoten zentrale Differenzen der Elementgrößen der umliegenden Elemente
(Schritt 1e-6 der Elementgröße), projiziert auf die Knotennormale. `MASS`: ρ dV.
`STRAINENERGY`: ½ uᵀ dK_S u + λᵀ (dF − dK u) mit K λ = K_S u (λ = u ohne Teilmenge), dF aus
Druck auf bewegten Flächen und Volumenlasten (GRAV, CENTRIF). `EIGENFREQUENCY`:
φᵀ (dK − λ dM) φ je gespeicherter Mode.

### Was von CalculiX durch Beobachtung bekannt ist (T17)

Abgeleitet aus Orakel-Läufen mit ccx 2.21 (Kragbalken C3D8, C3D20, C3D10, gekrümmte Platte mit
ungleichmäßigem Netz, Teilmengen, Teil-SPCs, Druck, Schwerkraft, Filter mit lokalisierter
Sensitivität):

- **Designknoten**: Knoten des Sets auf Außenflächen (Fläche keines zweiten Elements). Normale =
  normierte Summe der **Einheits**normalen aller angrenzenden Außenflächen (nicht flächen­
  gewichtet; Eckknoten und Mittenknoten gleich); Komponenten von DOFs mit SPC im Step werden
  vor dem Normieren gestrichen, ganz gehaltene Knoten haben die Normale 0 und keine
  Sensitivität.
- **DFDN** = Ableitung / Knotenfläche, Knotenfläche = Σ ∫ N_a dA über die Außenflächen (bei
  C3D8 ein Viertel jeder angrenzenden Fläche). `SENMASS` ist damit auf ebenen Flächen genau ρ.
  Eckknoten quadratischer Elemente (deren ∫ N_a dA verschwindet oder negativ ist) bekommen den
  Mittelwert der Mittenknoten auf den angrenzenden Außenkanten (C3D20 und C3D10 auf 1e-6).
- **LENGTH GRADIENT** in der `.dat` = 2-Norm von DFDN über alle Knoten.
- **DFDNFIL** = gefiltertes Feld, auf Länge 1 normiert; ohne Filter DFDN normiert. Linearer
  Filter: jeder Designknoten j verteilt seinen Wert auf die Designknoten i im Radius r mit
  Gewicht (1 − d/r) / Σ_i (1 − d/r), also **spaltenweise** normiert (auf 1e-7 getroffen;
  zeilenweise Normierung liegt 2 % daneben).
- **Datensätze**: NORM (Wert 0), dann je Response ein Satz `SENMASS`/`SENENER`, bei
  `EIGENFREQUENCY` je Mode ein Satz `SENFREQ` mit der Frequenz als Wert; alle mit Inkrement 1.
  Die `.dat` druckt je Mode eine eigene Tabelle, die übrigen Responses gemeinsam.
- **Eigenfrequenzen**: ccx differenziert jede Mode der gespeicherten Basis einzeln. Bei
  doppelten Eigenwerten (quadratischer Querschnitt) hängen die Einzelwerte und ihre Längen von
  der Basis ab, die der Eigenlöser wählt; nur die Summe über das Cluster (Spur) ist invariant.
  `plxs-verify` vergleicht dort die Summe von DFDN und lässt DFDNFIL und die Länge weg. Ohne
  Entartung (Rechteckquerschnitt) stimmt alles auf 1e-4 (ccx' Finite Differenzen und
  Eigenvektoren auf 1e-5), daher Toleranzklasse B für den Bereich.

### Bekannte Abweichung: STRAINENERGY

ccx' Strain-Energy-Sensitivität ist nicht die Ableitung seiner eigenen Response. In allen acht
Orakel-Modellen gilt auf 1e-5: ccx = exakt − Q mit Q = Σ_ip w_ip dV_ip/ds (Formänderungs­
energiedichte mal Ableitung des Integrationspunktvolumens), d. h. im expliziten Teil
½ uᵀ dK u fehlt die Volumenänderung. Finite Differenzen mit ccx selbst (Knoten verschieben,
neu rechnen) bestätigen den exakten Wert, nicht ccx' Ausgabe. Mit Element-Set verwendet ccx
außerdem λ = u statt der Adjungierten K λ = K_S u (Nodalis schreibt immer alle Elemente).
plxs rechnet die exakte Ableitung; die Fixtures führen `SENENER` und
`GRADIENT_LENGTH STRAINENERGY` als `deviations` (die `.dat`-Vergleiche beachten sie seit T17),
`plxs-cli/tests/sensitivity_corpus.rs` prüft die Ableitung gegen Finite Differenzen.

## Stand nach M3 T14 (Modalverfahren, komplexe Eigenwerte)

Was T14 gebaut hat, alles auf den Moden eines vorangehenden `*FREQUENCY, STORAGE=YES`
(`state.stored_modes`, massennormiert):

- `plxs-procedures::modal::ModalDynamicsProcedure` (`*MODAL DYNAMICS`): jede Mode
  `q̈ + 2ζω q̇ + ω² q = φᵀ f(t)`, die Last zwischen den Inkrementenden linear, jedes Inkrement
  **exakt** integriert (Matrixexponential des Zustands `(q, q̇, p, Δp)`, gilt auch für
  ω = 0 und überkritische Dämpfung). `U`, `V`, `S`, `E`, `RF` (`K u`), `ENER` wie in T13.
- `modal::SteadyStateProcedure` (`*STEADY STATE DYNAMICS`, harmonisch):
  `q = φᵀ f / (ω² − Ω² + 2iζωΩ)`, Real- und Imaginärteil als `DISP`…`FORC` und
  `DISPI`, `STRESSI`, `TOSTRAII`, `FORCI`; `PU` → `PDISP`, `PHS` → `PSTRESS` (Betrag und
  Phase). Amplituden sind Funktionen der Frequenz.
- `*MODAL DAMPING`: Bereiche (Moden außerhalb ungedämpft) und Rayleigh
  `ζ = α/(2ω) + βω/2`.
- `complex::ComplexFrequencyProcedure` (`*COMPLEX FREQUENCY, CORIOLIS`): Coriolis-Matrix
  `G = 2∫ρ Nᵀ Ω̃ N dV` der Elemente der `CENTRIF`-Last, `Γ = Φᵀ G Φ`, das gyroskopische
  Problem `q̈ + Γ q̇ + Λ q = 0` über die reell schiefsymmetrische Zustandsmatrix
  `[[0, Λ^½], [−Λ^½, −Γ]]` und deren symmetrische Einbettung doppelter Größe
  (`plxs_linalg::dense::symmetric_eigen`, faer).
- `eigen`: ein perturbierter Frequenzschritt bekommt zu `K + K_u + K_σ` das **Spin
  Softening** `−ω² ∫ρ Nᵀ(I − aaᵀ)N dV` jeder `CENTRIF`-Last (`complex::add_spin_softening`).
- `plxs-output`: `IncrementKind::SteadyState` (Analysetyp 1, Inkrement 0 im `1PSTEP`),
  komplexe Moden mit `1PGM`/`1PGK`/… Kopf und Inkrement 0, `PDISP`/`PSTRESS`-Komponenten
  (Typ 12 bzw. 14), `DISPI`/`FORCI` mit ALL-Zeile; `.dat`-Tabellen `COMPLEX EIGENVALUE
  OUTPUT` und `EIGENMODE TURNING DIRECTION` (`EigenTable`).
- `plxs-verify`: komplexe Moden werden als komplexe Vektoren aus `PDISP` nach dem besten
  Einheits-Phasenfaktor verglichen, `DISP` als Realteil des gedrehten Kandidaten; Spannungen
  und Dehnungen komplexer Moden nicht (Realteil einer Phase, die keiner ausschreibt).
  `PDISP`/`PSTRESS` werden überall als `MAG·e^{iPHA}` verglichen (die Phase einer
  Rundungsnull ist beliebig). `deviations` gilt jetzt auch für `.dat`-Komponenten.
- Abnahmetest `solver/plxs-cli/tests/modal_corpus.rs`: alle `modal/*`-Fixtures mit Referenz
  grün; neu ist die kleine Writer-Fixture `modal-kurz` (4 Moden, Modaldynamik mit Rayleigh,
  Sweep mit Bias, PU/PHS). `modal` (ohne Golden im Repo) stimmt lokal auf 5e-6.

### Was von CalculiX durch Beobachtung bekannt ist (T14)

- `*MODAL DYNAMICS` und `*STEADY STATE DYNAMICS` stimmen mit exakter Modalintegration bzw.
  der harmonischen Modalformel auf 5e-6; ccx schreibt keine `.sta`-Zeilen für sie.
  Ohne `AMPLITUDE=` wirken die Lasten der Modaldynamik sofort (wie `*DYNAMIC`).
- **Frequenzraster** des Steady State: der Bereich wird an den Eigenfrequenzen darin
  geteilt, jedes Intervall bekommt `points` Frequenzen (Enden geteilt),
  `f = (f₁+f₂)/2 + (f₂−f₁)/2 · sign(y)|y|^(1/bias)` mit `y` gleichmäßig in [−1, 1]
  (Bias 1 und 2,5 nachgeprüft).
- **Phase** in `PDISP`/`PSTRESS`: `atan(Im/Re)` in Grad, plus 180° bei negativem Realteil
  (Wertebereich −90° … 270°); Rundungsnullen haben Phase 0.
- **Komplexe Moden**: die „Partizipationsfaktoren“ der `.dat` sind die Modalkoordinaten mit
  der betragsgrößten auf 1 gesetzt; die `.frd`-Mode ist dieselbe Kombination,
  massennormiert (`‖c‖ = 1`). `1PGK` = (2π f)², Wert = Frequenz.
- Ein **perturbierter** `*FREQUENCY` nach einer statischen `CENTRIF`-Last enthält das Spin
  Softening (ohne liegen die Rotor-Frequenzen 1,3e-4 zu hoch); ohne `PERTURBATION` rechnet
  ccx mit der linearen Steifigkeit, die Coriolis-Kopplung nimmt `*COMPLEX FREQUENCY` aber
  in beiden Fällen aus der `CENTRIF`-Last des Steps.

### Bekannte Abweichungen (T14)

- **Drehrichtung** (`TURNING DIRECTION`): plxs nimmt den Umlaufsinn der Bahn
  `Re(x e^{iμt})` um die Achse (alle Knoten einig). ccx gibt für das zweite Biegepaar des
  Rotors bei beiden Moden B aus, obwohl die Knoten der ersten vorwärts laufen; das Kriterium
  von ccx ist nicht identifiziert. Die Rotor-Fixtures vergleichen die Komponente nicht
  (`deviations`).
- Nicht gebaut: `*MODAL DYNAMICS, STEADY STATE`, `*STEADY STATE DYNAMICS, HARMONIC=NO`,
  Fußpunkterregung (vorgeschriebene Verschiebungen ≠ 0), `*COMPLEX FREQUENCY, FLUID`,
  zyklisch symmetrische Moden in den Modalverfahren, `PNT`; Coriolis nur für
  Volumenelemente. Diese Fälle werden mit `Unsupported` abgelehnt.
## Stand nach M3 T15 (zyklische Symmetrie und Submodell)

Was T15 gebaut hat:

- `plxs-constraints::cyclic`: `CyclicSymmetry::new` projiziert die Slave-Seite des
  `*TIE, CYCLIC SYMMETRY`, um den Sektorwinkel `φ = 2π/N` zurückgedreht, mit der Tie-Projektion
  auf die Master-Flächen (nicht passende Netze gehen also). Beide Achsrichtungen werden
  versucht; die, bei der mehr Knoten (näher) treffen, gewinnt. `ModelConstraints::new`
  dreht danach die Achse im Modell (`constraints.cyclic_symmetry.axis`) so, dass die
  Slave-Seite bei `+φ` liegt; Ausgabe und spätere Steps müssen sie nicht neu orientieren.
  `ModelConstraints::cyclic_real` sind die Bedingungen des Knotendurchmessers 0
  (`u_s = R(φ) Σ wᵢ uᵢ`), die `step_set` jedem Step mitgibt (statisch, `*FREQUENCY` ohne
  `*SELECT CYCLIC SYMMETRY MODES`). `ModelConstraints::cyclic_step_set(nodes, active, spc, k)`
  baut den verdoppelten Satz für Knotendurchmesser `k`: Realteil auf den DOFs `0..3n`,
  Imaginärteil auf `3n..6n`, SPCs und MPCs auf beiden Kopien, gekoppelt durch
  `c_s = e^{−ikφ} R(φ) c_m`.
- `plxs-procedures::eigen`: `*FREQUENCY` eliminiert jetzt SPCs **und** MPCs über
  `ModelConstraints::step_set` (`homogeneous_constraints`); Starrkörpervektoren für
  Partizipationsfaktoren werden auf die unabhängigen DOFs eingeschränkt
  (`restrict_to_free`, lokal an transformierten Knoten). `*BUCKLE` nutzt noch nur SPCs.
- `plxs-procedures::cyclic`: Frequenzschritt je Knotendurchmesser `NMIN..NMAX` (Vorgabe
  0 bis N/2) auf `diag(K, K)`, `diag(M, M)`; Moden normiert auf `aᵀMa + bᵀMb = 1`. Jeder
  Eigenwert kommt doppelt; die `.frd` bekommt nur den ersten des Paars mit `DISP`/`DISPI`,
  `STRESS`/`STRESSI`, `TOSTRAIN`/`TOSTRAII` (`plxs_output::imaginary_field_name`).
  STORAGE speichert die verdoppelten Vektoren mit `nodal_diameters`.
- `plxs-procedures::submodel`: liest die globale ASCII-`.frd` (`INPUT=` relativ zum
  Jobverzeichnis; Volumenelemente inkl. expandierter Schalen/Balken), sucht jeden Knoten der
  `*BOUNDARY, SUBMODEL`-Sets per Newton im isoparametrischen Element (Raster der
  Elementboxen, sonst nächstes Element mit Warnung) und interpoliert `DISP` des letzten
  Inkrements von `STEP=n`; `StaticLinear` schreibt die Werte (mit Amplitude, lokal an
  transformierten Knoten) in die SPC-Werte. Nur DOF 1–3; `GLOBAL ELSET` wird mit Warnung
  ignoriert.
- `plxs-output`: `.dat` mit `EigenTable::CYCLIC_EIGENVALUES` (Spalte NODAL DIAMETER) und
  `TURNING_DIRECTION` (Achsreferenz + F/B); `.frd` mit `1PAX`, Modusnummern je
  Knotendurchmesser und **NGRAPH-Expansion** (`Sectors`): Sektor `j` ist das Modell um `−jφ`
  gedreht, Knoten-/Element-Ids um `j·max(Id)` verschoben; Vektoren und Tensoren werden
  gedreht, zyklische Moden zusätzlich phasenverschoben: `c_j = e^{ikjφ} R(−jφ) c`.
- `plxs-verify`: zyklische Moden werden bis auf einen komplexen Faktor vom Betrag 1
  verglichen (Phase per kleinste Quadrate aus `DISP`/`DISPI`, Betrag muss 1 sein);
  Partizipationsfaktoren und effektive Modalmassen einzelner zyklischer Moden hängen von
  dieser Phase ab und werden nicht verglichen; die Drehrichtung stehender Wellen
  (Knotendurchmesser 0 und N/2) ist bei ccx Rundungsrauschen und wird ausgelassen.
  Fixtures dürfen Zusatzdateien in `<name>.files/` mitbringen (globale `.frd`).
- Abnahmetest `solver/plxs-cli/tests/cyclic_corpus.rs`: alle `cyclic/*`-Fixtures grün
  (`cyclic_sector`, `zyklisch-sektor` mit NGRAPH=12 und Knotendurchmessern 0–2,
  `zyklisch-statisch` mit Fliehkraft, `submodel`, `submodel-innen` mit Knoten im Inneren
  globaler Elemente).

### Was von CalculiX nur durch Beobachtung bekannt ist (T15)

- Phasenkonvention: Die Slave-Seite eilt der Master-Seite um `kφ` nach (`e^{−ikφ}`), egal
  wie die Achse in der Eingabe zeigt; die `.dat` druckt als „Axis reference direction“ das
  Gegenteil der Achse, um die die Slave-Seite bei `+φ` liegt, und alle Moden mit
  `0 < k < N/2` als B (rückwärts). Getauschte Slave/Master-Flächen kehren die Referenzachse um.
- Ohne `*SELECT CYCLIC SYMMETRY MODES` rechnet ccx Knotendurchmesser 0 als reelles Problem
  (einfache Eigenwerte, keine Paare, normale Eigenwerttabelle); statische Steps ebenso.
- Gesamte effektive Masse, Summen der Modalmassen: auf den unabhängigen DOFs der reduzierten
  Massenmatrix `TᵀMT` (Abhängige gehen über `T` ein), Starrkörpervektor nur auf der
  Realteil-Kopie – so stimmen alle Summen auf 7 Stellen, und die Totalmasse hängt vom
  Knotendurchmesser ab.
- Feldnamen des Imaginärteils: `I` angehängt, bei acht Zeichen ersetzt es das letzte
  (`TOSTRAII`). Die `100CL`-Nummer zählt jeden geschriebenen zyklischen Modus doppelt
  (Partner mitgezählt), `1PMODE` ist die ungerade Nummer im Knotendurchmesser.
- NGRAPH: alle Knoten (auch die der Slave-Seite) werden kopiert, die Kopien liegen in
  Richtung der Master-Seite (`−jφ`).
- Submodell: Bei deckungsgleichen Knoten und bei Knoten im Inneren globaler Elemente
  stimmen ccx und plxs auf 1e-6 (lineare Interpolation in C3D8).

## Was M1 zu liefern hat (Abnahme)

- `solver_check::run` aus `plx-app` besteht mit `plxs` als Executable (Kragbalken mit Kraft
  und Druck, Selbsttest-Keywords).
- Alle Fixtures `static/*` (Stufe M1 in `FIXTURES.md`) sind grün:
  `cargo run -p plxs-verify --bin plxs-fixtures -- run --solver target/debug/plxs --area static`.
- Benchmark 176k-DOF-Block (Build-Doku) gegen ccx-PaStiX, 4 Threads, als Zahl im PR.

## Stand nach der Performance-Runde 1 (2026-10-31, Thread "CCX durch eigenen Solver ersetzen")

- `plxs-linalg::ordering`: Nested Dissection als Fill-reducing Ordering für Systeme ab 3000
  Unbekannten (`LdltSolver`, Konstante `DISSECTION_FROM`), darunter weiter faer's AMD.
  Bisektion geometrisch am Median der längsten Achse, wenn der Löser die Positionen der
  Unbekannten kennt (`LdltSolver::set_coordinates`, von `static_linear` und `heat` gesetzt),
  sonst über Level-Strukturen; Vertex-Separator = kleinere Randseite; Supervariablen (DOFs
  eines Knotens) bleiben zusammen; Blätter in Cuthill-McKee-Reihenfolge.
- Messung 39³-Hex-Block, 173k Gleichungen, 4 Threads: AMD 840 Mio. Nonzeros im Faktor, 37 s;
  Dissection 239 Mio., 10,6 s; CalculiX/PaStiX (Scotch) 213 Mio., 4,5 s Faktorisierung, 10,7 s
  Gesamtlauf gegen 12,5 s plxs. Die Ergebnisse (Fixture-Korpus) ändern sich nicht.
- `PLXS_ORDERING=amd|nd` erzwingt eine Ordnung (für Messungen).
- Offen (T20): faer parallelisiert nur innerhalb eines Supernodes; eine baumparallele
  supernodale LDLᵀ auf faer's dichten Kernen soll den Rest (Faktor ~2 in der numerischen
  Faktorisierung) holen. Benchmark-Eingaben: `/mnt/project-files/nodalis/workbench/benchmark/`
  (Projektordner, nicht im Repo), Zahlen in `KONZEPT_CCX_ABLOESUNG.md` Abschnitt 7.

## Stand nach T20 (2026-11-01, Thread "Solver schneller als PaStiX machen")

- `plxs-linalg::supernodal`: eigene numerische Faktorisierung für supernodale Strukturen,
  parallel über den Eliminationsbaum. faer liefert weiter die symbolische Analyse (Ordnung,
  Supernodes, Struktur und Speicherlayout des Faktors), deshalb bleiben `solve` und die
  Inertia-Zählung faer's. Left-looking wie faer: ein Supernode holt die Updates aller
  Nachfahren, deren Zeilen in seine Spalten reichen, direkt aus deren fertigen Spalten (ein
  dichtes Produkt je Nachfahre, dann Scatter); dann faer's dichte LDLᵀ des Diagonalblocks und
  die Dreieckslösung des Blocks darunter (transponiert in einem zusammenhängenden Puffer,
  faer's Kerne sind darauf deutlich schneller). Teilbäume laufen ab genug Arbeit parallel
  (rayon), kleine sequentiell ohne Rekursion; große Supernodes nahe der Wurzel teilen ihre
  Spalten in Blöcke, die ihre Updates parallel holen. Kein Speicher über den Faktor hinaus;
  Puffer der Produkte (`Workspace`) bleiben über Refaktorisierungen erhalten.
- Kleine Systeme, für die faer die simpliziale Struktur wählt, rechnet weiter faer.
  `LdltSolver::force_supernodal()` erzwingt den neuen Pfad (Tests); `PLXS_FACTOR=faer`
  schaltet für Messungen auf faer's supernodale Faktorisierung zurück.
- Der Faktor wird in `analyze` einmal sequentiell beschrieben: Page-Faults, die mehrere
  Threads gleichzeitig nehmen, kosten in der Cloud-VM ein Vielfaches.
- Verworfen: eine multifrontale Variante. Sie war mit warmem Speicher 20 % schneller (6,4 s
  gegen 8,0 s für den 173k-Block, 4 Threads), brauchte aber 1,7 GB Update-Matrizen zusätzlich
  zum 1,9-GB-Faktor; bei großen Modellen (1 Mio. Gleichungen) ein Speicherrisiko, und in der VM
  war die erste Faktorisierung dadurch langsamer.
- Ergebnisse unverändert (Fixture-Korpus grün; Block-Ergebnisse bis auf die letzte Stelle der
  `.frd` gleich). Zahlen gegen ccx/PaStiX in `KONZEPT_CCX_ABLOESUNG.md` Abschnitt 7.4.
- Offen: faer's dichte Kerne erreichen in der VM 40-50 GFlop/s je Kern (OpenBLAS in PaStiX
  ca. 55-60); Dreieckslösung und dichte LDLᵀ ca. 25-35. Multithreaded Vorwärts-/
  Rückwärtseinsetzen; Separator-Verfeinerung im Ordering (Scotch füllt 11 % weniger).

## Stand nach dem iterativen Solver mit Multigrid (2026-11-04, Thread "Speed up the iterative solver")

- `plxs-linalg::pcg`: PCG mit algebraischem Multigrid als Vorkonditionierer
  (`plxs-linalg::amg`, Smoothed Aggregation nach Vaněk/Mandel/Brezina): Aggregate aus
  Knoten über die Frobenius-Normen der Kopplungsblöcke, je Aggregat die orthonormierten
  Starrkörpermoden (6 bei Verschiebungen, die Konstante bei Temperatur) als tentativer
  Prolongator, ein gedämpfter Jacobi-Schritt glättet ihn, Galerkin-Produkt `PᵀAP`, gröbste
  Ebene (≤ 1500 Unbekannte) wird mit `LdltSolver` faktorisiert. Ein V-Zyklus mit
  Chebyshev-Glätter (Grad 2, oberes Spektrum von `D⁻¹A`, λ_max per Potenziteration) pro
  CG-Iteration; symmetrisch, also bleibt CG anwendbar. IC(0) bleibt als Rückfall
  (`PLXS_PRECONDITIONER=ic0`, oder wenn die Hierarchie nicht gebaut werden kann:
  Vergröberung stockt, Matrix indefinit).
- Speicherformat `plxs-linalg::bsr`: alle Matrizen der Hierarchie in dichten Blöcken (BSR,
  Blockgröße zur Laufzeit): 3×3 je Knotenpaar auf der feinsten Ebene (Unbekannte auf drei je
  Knoten aufgefüllt, gesperrte Komponenten als entkoppelte Einheitszeilen), 6×6 je
  Aggregatpaar darüber, 3×6 für den Prolongator. Die Knoten werden intern per Reverse
  Cuthill-McKee umnummeriert (Cache-Lokalität des Produkts; auf Quentins Tet-Modell
  Faktor 1,5 im Produkt und bessere Aggregate). `Unknowns` (Knoten, Komponente, Position je
  Unbekannte) kommt von den Prozeduren über `Equations::prepare`
  (`Unknowns::from_free`); ohne sie ist alles skalar (Eigenwert-Iteration).
- Messungen (Container, 4 Threads, Lösen ohne Aufbau; vorher = IC(0)-PCG):
  Hex-Block C3D8 173k: 171 → 27 Iterationen, 5,2 s → 0,5 s (Aufbau 1,0 s);
  dünne Platte C3D8 123k: 1872 → 36 It., 27 s → 0,4 s; C3D20R-Block 223k: 449 → 50 It.,
  44 s → 4,6 s; Quentins C3D10-Halter 591k: 1556 → 198 It., 230 s → 23 s (Aufbau 4,6 s);
  Kontaktmodell Lasche 73k, 14 Newton-Iterationen: Aufbau je Iteration 1,2 s, Lösen 1,3 s.
  Quadratische Elemente brauchen etwa doppelt so viele Iterationen wie lineare.
- Benchmark-Werkzeuge: `solver/benchmarks/gen_block.py` (Hex-/Tet-Blöcke beliebiger
  Größe, C3D8/C3D20R/C3D4/C3D10), Eingaben in `/mnt/project-files/nodalis/benchmark/`.
  Messknöpfe: `PLXS_PCG_TRACE=1` (Zeit in Produkt/Vorkonditionierer), `PLXS_AMG_TRACE=1`
  (Phasen des Aufbaus), `PLXS_AMG_STRENGTH/DEGREE/COARSE/LEVELS/MAX_AGG`.
- Zweite Runde (gleicher Thread): die ganze Hierarchie liegt in einfacher Genauigkeit
  (`Bsr<f32>`, auch eine Kopie der feinsten Ebene; nur die gröbste Ebene wird in f64
  faktorisiert, das CG außen bleibt f64). Halbiert den Speicherverkehr des V-Zyklus und
  den Speicher des Vorkonditionierers (Halter: 1,8 → 0,9 × Matrix), kostet aber
  Iterationen, weil das CG mit einem gerundeten Vorkonditionierer seine Richtungshistorie
  nicht mehr voll nutzt (Hex 30 → 39, C3D10-Halter 200 → 220, dünne C3D20R-Platte
  65 → 115); netto trotzdem schneller (Halter 23 → 14,6 s, C3D20R-Block 2,9 → 1,8 s).
  Versucht und verworfen: nur die feine Ebene oder nur die groben Ebenen in f32, Residuen
  der feinen Ebene in f64, Umschalten auf f64 unter 1e-7 — jede f32-Komponente kostet
  dieselben Iterationen. `beta` im CG in der Polak-Ribière-Form (flexibles CG, ein
  Skalarprodukt mehr), damit ein leicht veränderlicher Vorkonditionierer zulässig ist.
  Block-Kernel per const generics für 1×1, 3×3, 3×6, 6×3, 6×6; Parallelisierung der
  Produkte in Stücken nach Blockzeilenzahl (grobe Ebenen mit 500 Blockzeilen beschäftigten
  sonst nur 3 Threads). Hierarchie wird über Faktorisierungen gleichen Musters
  (Newton-Iterationen) behalten (`Amg::refresh`: Diagonale, λ_max und die f32-Kopie der
  feinsten Ebene folgen den neuen Werten, Prolongatoren und grobe Operatoren bleiben),
  solange ein Lösen nicht mehr als das 1,5-Fache der Iterationen des ersten Lösens nach
  dem Aufbau braucht und sich die Diagonale um weniger als 25 % (euklidisch) geändert hat
  (`PLXS_AMG_REBUILD=1` baut immer neu). Gelernt am Fixture dynamik-automatisch: die
  Dynamik löst erst die Beschleunigungsmatrix, dann `K + cM` mit demselben Muster, die
  behaltene Hierarchie passte nicht und das CG lief 1800 Iterationen ins Leere. Deshalb
  zusätzlich: mit behaltener Hierarchie bricht das CG nach dem 3-Fachen der Iterationen
  des ersten Lösens ab, `Equations::solve` baut neu (`PcgSolver::rebuild_preconditioner`)
  und wiederholt einmal, erst danach greift der Rückfall auf den direkten Solver. Kontaktmodell Lasche,
  14 Newton-Iterationen: Aufbau 10,9 s → 2,7 s, gesamt 44 s → 32,5 s (direkt: 73 s).
- Dritte Runde, direkter Solver (`plxs-linalg::ordering`): die Nested Dissection probiert
  je Teilgraph alle drei Achsen und fünf Schnittlagen (30–70 % des Gewichts) und nimmt den
  kleinsten Separator; der Separator ist eine minimale Knotenüberdeckung der Kanten
  zwischen den Seiten (König über ein maximales Matching, Kuhn) statt des kleineren Randes
  und wird danach gierig verfeinert (Separatorknoten auf eine Seite schieben, wenn weniger
  Knoten der anderen Seite nachrücken müssen); Blätter 60 statt 240 Freiheitsgrade.
  Quentins Halter: Faktor 18,8 GB → 5,2 GB (PaStiX/Scotch ≈ 3,1 GB), passt damit in den
  Speicher, Faktorisierung 12 s auf 4 Threads, Gesamtlauf 28 s (iterativ 19 s); Hex-Block
  173k: 239 → 233 Mio. Nichtnullen. Die Ordnung kostet jetzt ~5 s auf dem Halter.
- Automatische Wahl (Folge-PR, Quentin 2026-11-04): ab 50 000 Gleichungen iterativ, wenn
  die Unbekannten nur Verschiebungen (oder Temperaturen) sind, also Volumenmodelle in
  statischen, nichtlinearen und Wärmeschritten; der direkte Solver bleibt Rückfall (nach
  spätestens 2000 Iterationen, `AUTOMATIC_MAX_ITERATIONS`). Modelle mit Rotationen
  (Schalen, Balken), Eigenwertiteration und implizite Dynamik
  (`Equations::preferring_direct`, eine Faktorisierung für viele Systeme) sowie kleine
  Modelle bleiben bei der Speicherregel.
- Offen: p-Multigrid (erste Ebene = Eckknoten) für quadratische Netze; Blätter der
  Dissection per Minimum Degree statt Cuthill-McKee und ein mehrstufiger Partitionierer
  (Scotch-artig) wären die nächsten Schritte zum Füllgrad von PaStiX.

## Stand nach dem V-Zyklus auf der Grafikkarte

- `plxs-linalg::gpu` (Feature `gpu`, an in `plxs-cli` per Default und in `plx-app`): der
  AMG-V-Zyklus als wgpu-Compute-Shader (WGSL, f32; `gpu_product.wgsl`, `gpu_vector.wgsl`).
  Die Hierarchie wird pro `Amg::build` einmal hochgeladen, pro Anwendung geht das Residuum
  hoch und die Korrektur zurück; CG und das Produkt mit der feinen Matrix bleiben f64 auf
  der CPU. Die gröbste Ebene (höchstens 4096 Unbekannte) wird als dichte Inverse
  angewandt, damit der Zyklus ohne Rundreise in der Mitte in einem Command Buffer läuft.
- Speicherformat auf der Karte: Sliced ELLPACK aus Blöcken (32 Blockzeilen je Slice, in
  Fenstern von 256 Blockzeilen nach Länge sortiert), ein Thread je skalarer Zeile, damit
  benachbarte Threads benachbarten Speicher lesen. `Amg::refresh` schreibt die neuen
  Werte der feinen Ebene, die Diagonale und die Chebyshev-Koeffizienten neu.
- Auswahl (`AmgOptions::gpu`, `PLXS_GPU=off|on`): automatisch nur `DiscreteGpu` und ab
  100 000 Gleichungen; `on` nimmt jeden Adapter (lavapipe im Container). Jeder Fehler beim
  Hochladen oder im Zyklus (auch ein Panic in wgpu) fällt auf die CPU zurück; der Grund
  steht in der Vorkonditionierer-Zeile des Monitors.
- Hochladen in Stücken von 16 MB und ohne Speicherbudget-Schwelle: wgpu stagt einen ganzen
  Puffer auf einmal und prüfte das Budget gegen den Staging-Heap (256 MB ohne Resizable
  BAR), daran scheiterte die feine Matrix des Halters.
- Messung (Quentin, RTX 4070, 16 Threads, C3D10-Halter 591k): Vorkonditionierer
  11,4 → 1,5 s, Lösen 15,9 → 5,8 s, Iterationen 217 / 214, Hochladen 0,35 s. Jetzt
  dominiert das f64-Produkt auf der CPU (3,7 s). Nächster Hebel: CG mit Produkt ganz auf
  der Karte (braucht `SHADER_F64` oder ein f32/f64-Doppelwort-Produkt).

## Stand nach dem CG auf der Grafikkarte

- Mit `SHADER_F64` (NVIDIA/AMD unter Vulkan und DX12, lavapipe; nicht Metal) läuft das
  ganze flexible CG auf der Karte: f64-Kopie der feinen Matrix im selben Sliced-Layout wie
  die f32-Kopie (`GpuCycle::attach`, nach jedem Aufbau und jedem `refresh`), Vektoren
  b, x, r, z, p, q in f64, Skalarprodukte in zwei Stufen (256 Teilsummen je Produkt,
  aufsummiert im Ein-Thread-Kernel `gpu_cg_scalar.wgsl`, der auch alpha, beta,
  Abbruch und Indefinitheit führt). Der V-Zyklus wird inline aufgenommen
  (r → f32 → Zyklus → f64 → z), also keine Übertragung pro Iteration.
- Die CPU liest nur alle 4 Iterationen (`BATCH`) 16 Skalare zurück; nach dem Abbruch
  setzen die Kernel alpha = beta = 0, die übrigen Iterationen eines Blocks ändern x nicht.
  Iterationszahl und Residuum sind dieselben wie auf der CPU (lavapipe: identisch).
- Messung (Quentin, RTX 4070, C3D10-Halter 591k): Lösen 15,9 s (CPU) → 5,8 s (nur
  V-Zyklus auf der Karte) → 1,4 s (ganzes CG), 214 Iterationen. Jetzt dominiert der Aufbau
  der Hierarchie auf der CPU (3 s).
- Ohne f64, bei zu großer f64-Matrix (Bindungsgrenze, lavapipe 128 MB) oder bei einem
  Fehler bleibt das CG auf der CPU; der Grund steht in der Vorkonditionierer-Zeile.

## Stand nach dem Multigrid für kontaktgehaltene Körper (2026-11-09, Thread "Improve multigrid for contact-held bodies")

Anlass: Quentins Lasche/Seilrolle (Bolzen, Rolle, Laschen aus C3D10, 238k Gleichungen,
die Körper nur durch Penalty-Kontakt und schwache Federn 1e-6·E·L gehalten): 20 Lösungen
mit 90 bis 390 CG-Iterationen (Volumenkörper allein: 30 bis 50), über 80 % der Zeit im
V-Zyklus, behaltene Hierarchie oft zu lange behalten.

- Ursache 1, Aggregate über Kontaktflächen: die Stärke der Kopplung sah den Penalty-Block
  zwischen Bolzen und Bohrung als starke Nachbarschaft, Aggregate (und damit die
  Starrkörpermoden des tentativen Prolongators) reichten über zwei Körper hinweg. Jetzt
  liefert `Model::node_bodies` die Zusammenhangskomponenten des Netzes über seine
  Kontinuums- und Strukturelemente (Federn, Gaps und Kontakt verbinden nichts, ein Knoten
  ohne Element ist ein eigener Körper), `Equations::displacement_unknowns` gibt sie als
  `Unknowns::body` an den Solver, und `amg::aggregate` fasst nie Knoten zweier Körper
  zusammen (die Körper werden auf die groben Ebenen vererbt). Lasche/Seilrolle: 3325 →
  1855 Iterationen über die 20 Lösungen.
- Ursache 2, Eigenwertschätzer: λ_max(D⁻¹A) aus 12 Potenziterationen × 1,1 lag auf der
  Rolle 4 % zu tief (4,25 statt 4,42), der Chebyshev-Glätter verstärkte die obersten Moden
  und der Vorkonditionierer war nicht mehr positiv definit (negatives r·z im CG, 84 statt
  72 Iterationen). Jetzt 30 Lanczos-Schritte auf `D^-1/2 A D^-1/2` (größter Ritz-Wert per
  Sturm-Bisektion) × 1,02 (`EIGENVALUE_STEPS`, `EIGENVALUE_MARGIN`, zum Messen
  `PLXS_AMG_MARGIN`).
- Ursache 3, was bleibt: die Ritz-Werte von M⁻¹A aus den CG-Koeffizienten (Lanczos-
  Tridiagonalmatrix, `PLXS_PCG_TRACE=1` druckt die kleinsten und größten) zeigen auf der
  Rolle ein dichtes Bündel bei 0,02 bis 0,05, dessen Vektoren zu 83 % axial sind und an den
  Stirnflächen der Rolle bei Radius 40 bis 70 mm sitzen: die Biegung der dünnen Flansche
  der Seilrille, nicht die Starrkörperbewegung (deren Rayleigh-Quotienten liegen bei 0,55
  bis 1,0). Rolle allein mit festgehaltener Bohrung 84 Iterationen, Bolzen 39, Blöcke
  25 bis 37 (kleinster Ritz-Wert dort 0,17 bis 0,28).
- Abhilfe, Lernen aus dem ersten Lösen: das erste Lösen nach einem Aufbau hebt bis zu 96
  Lanczos-Vektoren (f32) auf; nach der Konvergenz liefern die Ritz-Vektoren mit Werten
  unter 0,15 × dem größten (höchstens 24, `DEFLATE_*`) zweierlei. (a) Deflation nach Frank
  und Vuik (projiziertes DPCG: `P A x̃ = P b` mit `P = I − A W E⁻¹ Wᵀ`, `E = WᵀAW`
  Cholesky; die Richtungskorrektur nach Saad lief in f64 auseinander, weil die Rundung in
  span(W) wächst) für die weiteren Lösungen mit derselben Hierarchie, `Deflation::refresh`
  zieht `AW` und `E` bei behaltener Hierarchie nach; nur im CG auf dem Prozessor. (b) Die
  drei langsamsten werden Kandidaten des Nahnullraums (`PLXS_AMG_CANDIDATES`, 0 schaltet
  ab): `Amg::build(a, unknowns, candidates, options)` nimmt sie zu den sechs
  Starrkörpermoden, die nächste Faktorisierung baut die Hierarchie damit neu
  (`candidates_pending`), grobe Blöcke 9×9 und 3×9 (feste Kernel in `bsr`; 12×12 für sechs
  Kandidaten). Die Kandidaten gelten für alle weiteren Hierarchien des Musters
  (`analyze` verwirft sie), jede davon lernt ihre Deflation beim ersten Lösen neu. Auf der Grafikkarte sammelt das CG keine Lanczos-Vektoren, deshalb läuft
  das lernende Lösen eines Musters auf dem Prozessor, alles Weitere auf der Karte.
  Seilrolle NLGEOM (4 Lösungen): 76 → 49/46/42 Iterationen (Deflation allein 63, sechs
  Kandidaten 38, aber ein 12×12-Zyklus kostet das 1,4-Fache; drei sind netto am
  schnellsten, −34 % Zeit je Lösen). Lasche/Seilrolle (gleiche Binary, 0 gegen 3
  Kandidaten): 1870 → 1485 Iterationen, Lösen 124 → 105 s, Lauf 3:39 → 3:16 min (sechs
  Kandidaten: 1622 Iterationen, 3:53 min); mit der Kostenregel unten 1348 Iterationen,
  Lösen 94 s, Lauf 3:11 min (Ausgangsstand 3325 Iterationen).
- Behalten oder neu bauen: statt der 1,5-fachen Iterationen wiegt `PcgSolver::factorize`
  die Kosten: die Hierarchie bleibt, solange die Mehr-Iterationen des letzten Lösens
  gegenüber dem ersten nach dem Aufbau, über zwei Lösungen (`STALE_HORIZON`) zum
  gemessenen Tempo je Iteration, weniger kosten als der gemessene Aufbau; die 25 %-Regel
  für die Diagonale und anstehende Kandidaten bleiben. `PLXS_PCG_TRACE=1` druckt die
  Entscheidung mit den Zahlen.
- Probiert und verworfen: ganze Hierarchie in f64 (gleiche Iterationen), Penalty 100-fach
  weicher (gleich), Stärke 0,001 (schlechter), Chebyshev Grad 4 (70 statt 90, zu teuer),
  adaptive Smoothed Aggregation (αSA nach Brezina u. a.: 30 Testzyklen auf `A x = 0` je
  Vektor, Rolle 72 → 63 → 57 → 52 mit 1 bis 3 Vektoren, aber ~4 s je Runde bei 238k;
  bleibt mit `PLXS_AMG_ADAPTIVE=n` zum Experimentieren, Default 0). Mit Federn 1e4-fach
  steifer 83 Iterationen, erst elastische Bettung (1e8-fach) bringt 25: die Federn sind
  nicht das Problem.
- Block-Jacobi statt skalarer Diagonale (Nachtrag, Lasche an GAPUNI-Federn): Quentins
  Lasche (C3D20/C3D15, 87k Gleichungen) hängt nur an 814 Druckfedern (`*GAP`, k = 1e12 N/m,
  etwa 6000-mal steifer als ein Element) in radialer Richtung. Die skalare Diagonale der
  Bohrungsknoten ist in allen drei Komponenten riesig, obwohl die Feder nur eine Richtung
  hält: der Chebyshev-Glätter bewegt die tangentialen Komponenten dieser Knoten praktisch
  nicht (Eigenwerte von D⁻¹A bei 1e-4), die Stärke-Regel hielt alle ihre Kopplungen für
  schwach (5 Ebenen, 2,08 × Matrix, Aufbau 8,5 s), und das CG brauchte 280 bis 690
  Iterationen (Ritz-Werte 4e-4). Mit Federn 1e9 statt 1e12 waren es 46: die Anisotropie
  war die Ursache. Jetzt skaliert das Multigrid überall mit den inversen Diagonalblöcken
  (`block_diagonal_inverses`: Cholesky `D_b = L_b L_bᵀ` je Knoten bzw. Aggregat, 3×3 fein,
  6×6/9×9 grob; nicht positiv definite Blöcke fallen auf die Skalare zurück): Stärke
  `‖L_b⁻¹ A_bc L_c⁻ᵀ‖_F ≥ θ`, Glätter `D_B⁻¹ r` (`block_apply`, auf der Karte dieselbe
  Operation in `gpu_vector.wgsl` mit `br` in den Parametern), λ_max per Lanczos auf
  `L⁻¹ A L⁻ᵀ`, Prolongator `(I − ω D_B⁻¹ A) P_t`. Für skalare Systeme (Wärme) ist alles wie
  vorher. Lasche: 3 Ebenen (87k > 3,5k > 246), Aufbau 0,9 s, 38 bis 76 Iterationen je
  Lösen, Gesamtlauf 168 s (master) → 58 s (direkter Solver 46 s). Auch sonst etwas besser:
  Seilrolle 76 → 60, Hex-Block 25 → 23, Tet10-Block 37 → 33 Iterationen.
- Offen: Lanczos-Vektoren auch im CG auf der Karte sammeln (Deflation dort), ein
  Kandidatensatz je Körper statt global, und größere Aggregate bei größerem Nahnullraum,
  damit die groben Ebenen nicht wachsen.

## Moden als eigener Ergebnistyp (2026-11-05, Thread "Add dedicated mode shape result type")

- Eigenwertschritte (`*FREQUENCY`, `*BUCKLE`, `*COMPLEX FREQUENCY`, auch zyklisch) legen ihre
  Moden immer in `Results::modes` ab: je Schritt ein `ModeSet` (Knotenliste einmal, je Mode
  Eigenwert, Frequenz bzw. Lastfaktor, Knotendurchmesser, Form als `f32`-Zeilen x, y, z und
  bei komplexen/zyklischen Moden der Imaginärteil). Normiert auf größte Verschiebung 1
  (mit Imaginärteil: größtes `|Re|² + |Im|²`), keine Spannungen, Dehnungen, Kräfte.
- Die `.frd`-Felder der Moden (DISP, STRESS, ...) wie CalculiX entstehen nur noch, wenn der
  Schritt `*NODE FILE`/`*EL FILE`/`*CONTACT FILE` anfordert (CLI, Fixture-Korpus
  unverändert). Spannungen der Moden werden sonst gar nicht berechnet.
- `OutputMap::apply` behandelt die Moden wie `DISP` (über `Results::mode_fields` und
  `set_mode_fields`), danach neu normiert. Der Schritt-Cache führt sie mit.
- Nodalis (`plx-solve`) lässt in Eigenwertschritten alle Feldausgaben weg, auch die aus dem
  Schritt davor übernommenen (`keep_modes_as_shapes`). `plx-results::from_solver` zeigt jede
  Mode als Inkrement mit dem Feld `MODE` (komplex zusätzlich `PMODE`, Betrag und Phase).
