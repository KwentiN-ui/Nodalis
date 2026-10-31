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

`Snapshot { model_fingerprint, state, average_force }` (M3 T16): `average_force` ist der
zeitgemittelte Kraftmaßstab der Residuenprüfung, den die Session von Step zu Step trägt;
ohne ihn konvergiert ein fortgesetzter Lauf nicht genau wie der Lauf von vorn. Der
Fingerprint deckt das Modell ohne Sets, Surfaces und Amplituden ab (ein fortsetzendes Modell
darf solche ergänzen, siehe `KONZEPT_CCX_ABLOESUNG.md` 3.5).

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
    gehaltenen B32-Endknoten folgen die Kantenmitten dem Knoten (ccx-Beobachtung).
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
- **Dokumentierte Abweichungen** (Fixture-`deviations`, `FIXTURES.md`):
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
- **Offen**: Membranen M3D4/M3D4R (weichen von jeder probierten Expansion und vom Q4 ab,
  nur an belasteten Knoten), die genaue
  BOX-Regel von ccx, die Knotenspannungen von PIPE (ccx extrapoliert anders als die lineare
  Querschnittsanpassung, bis 0,3 %),
  `*NODAL THICKNESS`, Druck auf Balken, U1 an Schalen-/Balkenknoten,
  zylindrischer `*TRANSFORM` an Schalenknoten, edge-tie-m3d4 (wegen M3D4).
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
  PrePoLix-Writers; `compare_dat` vergleicht Sets nach Namen).
- `plxs-verify`: Klasse B vergleicht nur noch das Step-Ende (auch bei gleicher
  Inkrementanzahl: die Inkremente nach dem Fließbeginn liegen über der Toleranz), die
  Inkrementanzahl nur als Größenordnung (Faktor 3). Skalierung je Feld über alle
  Komponenten, nicht je Komponente.
- Abnahmetest `solver/plxs-cli/tests/nonlinear_corpus.rs`: alle `nonlinear/*`-Fixtures
  grün oder vom Solver mit `not implemented yet` abgelehnt (SPRING1/GAPUNI/M3D4 anderer
  Threads); ein falsches Ergebnis fällt durch.

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

## Was M1 zu liefern hat (Abnahme)

- `solver_check::run` aus `plx-app` besteht mit `plxs` als Executable (Kragbalken mit Kraft
  und Druck, Selbsttest-Keywords).
- Alle Fixtures `static/*` (Stufe M1 in `FIXTURES.md`) sind grün:
  `cargo run -p plxs-verify --bin plxs-fixtures -- run --solver target/debug/plxs --area static`.
- Benchmark 176k-DOF-Block (Build-Doku) gegen ccx-PaStiX, 4 Threads, als Zahl im PR.
