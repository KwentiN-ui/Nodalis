# Fixture-Korpus des PrePoLix-Solvers

Stand M0 (2026-10-31). Der Korpus in `solver/fixtures/` ist die Feature-Matrix des Solvers:
jede Fixture ist eine Eingabe, die PrePoLix' eigener `.inp`-Writer schreibt (oder ein
Beispiel aus `testdata/`), mit dem Ergebnis, das CalculiX dafür liefert. Ein Feature gilt als
nachgebaut, wenn seine Fixtures mit `plxs` innerhalb der Toleranz liegen. Aus der Tabelle
unten leitet der Koordinator die Briefs der Stufen M1 bis M3 ab.

## Aufbau

```
solver/fixtures/<bereich>/<name>.inp          Eingabe (wie PrePoLix sie schreibt)
solver/fixtures/<bereich>/<name>.toml         Bereich, Toleranzklasse, Stufe, Features, Herkunft, Referenz
solver/fixtures/<bereich>/<name>.ref.frd.gz   .frd von CalculiX, gzip (flate2, pure Rust; das Konzept nannte zstd)
solver/fixtures/<bereich>/<name>.ref.dat      .dat von CalculiX, wenn die Eingabe History-Ausgaben hat
```

Herkunft: `scripts/solver_reference.sh` lässt die Writer-Tests von `plx-io` mit
`PLXS_FIXTURE_DIR` laufen (jeder Test, der eine Eingabe an CalculiX gibt, legt sie dort ab;
`run_ccx` in `crates/plx-io/src/inp/write/tests.rs`), importiert sie mit `plxs-fixtures import`
nach Bereichen und rechnet die Golden Files mit `plxs-fixtures reference` (ccx, ein Thread).
Golden Files im Repo stammen von ccx 2.21 (`apt calculix-ccx` im Container); die Fixtures
nutzen nichts, was sich zwischen 2.21 und 2.23 unterscheidet.

Ein neuer Writer-Test, der `run_ccx("name", ..)` ruft, wird beim nächsten Lauf des Skripts
automatisch zur Fixture `name`. Zwei Tests dürfen keinen Namen teilen (das Skript bricht ab).
Fixtures werden nie von Hand editiert; wer eine ändern will, ändert den Writer-Test.

## Toleranzklassen (`class` in der `.toml`)

| Klasse | Größen | Toleranz relativ zur Feldnorm (größter Betrag aller Komponenten des Felds) |
|---|---|---|
| A | linear: U, RF, NT, S/E, Eigenfrequenzen | 1e-5 (statt 1e-6 im Konzept: die ASCII-`.frd` hat sechs signifikante Stellen, mehr kann kein Solver treffen) |
| B | nichtlinear glatt: NLGEOM, Plastizität, Kriechen, Wärme transient, Strahlung, Gaps | 1e-3 am Step-Ende; Inkrementanzahl nur Größenordnung |
| C | Kontakt: U, CSTR, CF | 1e-2 am Step-Ende; Kontaktstatus je Knoten gleich |
| D | Struktur: `.frd`-Netz (Knoten, Elemente, Expansion), Inkremente, Feld- und Set-Namen, `.dat`-Blöcke | exakt, gilt für jede Fixture |
| E | Dynamik: Zeitverläufe | 1e-3 auf Normen, Spitzen innerhalb eines Inkrements |

Verglichen wird nicht Text, sondern was `plx_io::frd::read_frd_bytes` und
`plx_io::dat::parse_dat` liefern (`plxs_verify::compare_frd`, `compare_dat`): dieselben
Knoten-Ids und Elemente (expandierte Netze eingeschlossen), dieselben Inkremente (Step,
Nummer, Art, Wert), dieselben Felder, und jede rohe Komponente innerhalb der Toleranz.

Bezugsgröße ist die Feldnorm (Konzept 6.2), nicht der größte Betrag der einzelnen Komponente:
eine Komponente, die im ganzen Modell fast null ist (S13 in einer ebenen Scheibe, F3 bei
reiner Querlast), trägt nur Rundungsrauschen, und ccx' Extrapolation bei C3D15 weicht um
einige 1e-6 der Feldgröße von der exakten ab. Relativ zur Komponente wären das 1e-5 und mehr; relativ
zum Feld bleibt die Prüfung bei 1e-5 so streng wie gemeint (seit M2 T4).

Rundungsrauschen (`[noise]` in der `.toml`): Felder, die physikalisch null sind (etwa
Spannungen bei freier Wärmedehnung) und bei ccx nur Rundungsreste zeigen, bekommen einen
absoluten Rauschpegel je Feldname, z. B. `STRESS = 1e-6`. Die Skala einer Komponente ist
dann max(Feldnorm, Rauschpegel/Toleranz). Der Schlüssel ist die
einzige Ausnahme von „nie von Hand editieren“: `plxs-fixtures import` behält ihn.

## Bereiche und Stufen

Ein Bereich ist ein Verzeichnis und gehört einem Thread (Nummern aus dem Konzept, Abschnitt
5). Die Zuordnung trifft `plxs_verify::classify` nach den Keywords der Eingabe, der erste
Treffer gewinnt:

| Bereich | Stufe | Thread | Was die Fixtures brauchen |
|---|---|---|---|
| `static` | M1 | T1 Linalg, T2 Volumenelemente, T3 Static/Output/Session | C3D8, C3D20R, C3D4, C3D10 (auch C3D8I/C3D8R/C3D20/C3D6 in `elementtyp-*`), *BOUNDARY, *CLOAD, *DLOAD (P), *NODE/EL FILE, *NODE PRINT |
| `plane` | M2 | T4 | CPS/CPE/CAX, Dicke in der Section, Kantenlasten, Expansion in der `.frd` |
| `structural` | M2 | T5 | S3/S4/S8R, M3D*, B31/B32/B32R, T3D2, U1 (`*USER ELEMENT`), Beam Sections RECT/CIRC/PIPE/GENERAL, Expansion, Symmetrieebenen |
| `constraints` | M2 | T6 | *EQUATION, *RIGID BODY, *TIE, *TRANSFORM, *SPRING, *GAP, Re-Pivotierung überlappender MPCs, Vorspannung über Equations |
| `eigen` | M2 | T7 | *FREQUENCY (STORAGE, PERTURBATION), *BUCKLE, Eigenwerttabellen in der `.dat`, Buckling-Sets in der `.frd` |
| `nonlinear` | M2 | T8 | NLGEOM, *PLASTIC, *VISCO/*CREEP, automatische Inkrementierung, PEEQ/PE, `.sta` |
| `heat` | M2 | T9 | *HEAT TRANSFER stationär/transient, *CFLUX/*DFLUX/*FILM/*RADIATE, *COUPLED/*UNCOUPLED, *TEMPERATURE, *INITIAL CONDITIONS, Gap Conductance |
| `loads` | M2 | T10 | *AMPLITUDE (TOTAL TIME, SHIFTX/SHIFTY), P/GRAV/CENTRIF mit Amplituden, Verschiebungen mit Amplitude, quadratische Elemente, Step-Folgen (wandernde Lasten); Übernahme ohne OP=NEW in `plxs-input`-Tests |
| `contact` | M3 | T11 (N2S), T12 (S2S, Mortar) | *CONTACT PAIR, *SURFACE BEHAVIOR (HARD/LINEAR/TIED), *FRICTION, *CLEARANCE, *MODEL CHANGE, *CHANGE FRICTION, *CONTACT FILE/PRINT, schwache Federn |
| `dynamic` | M3 | T13 | *DYNAMIC (implizit, Rayleigh), *INITIAL CONDITIONS, TYPE=VELOCITY |
| `modal` | M3 | T14 | *MODAL DYNAMICS, *STEADY STATE DYNAMICS, *MODAL DAMPING, *COMPLEX FREQUENCY mit Coriolis (`.eig` aus *FREQUENCY, STORAGE=YES) |
| `cyclic` | M3 | T15 | *CYCLIC SYMMETRY MODEL, *SELECT CYCLIC SYMMETRY MODES, *SUBMODEL (`submodel` hat kein Golden File, weil es die `.frd` des Globalmodells braucht; der Thread baut sich den Zweischritt lokal) |
| `sensitivity` | ccx | – | *SENSITIVITY wurde für Phase 1 gestrichen; die Fixture prüft nur, dass der Parser sie liest |

Drei Fixtures (`modal/modal`, `dynamic/dynamik`, `dynamic/dynamik_gedaempft`) haben kein
Golden File im Repo, weil es mit Hunderten von Inkrementen 3 bis 7 MB groß wäre; der
zuständige Thread erzeugt es lokal mit `scripts/solver_reference.sh --name <name>` und checkt
es nicht ein. Der Korpus bleibt so unter 10 MB (Budget 20 MB).

## Benutzung

```sh
cargo test -p plxs-verify                     # Korpus prüft sich selbst (CI): jede Fixture
                                              # parst in die IR, jedes Golden File lässt sich
                                              # lesen, diese Tabelle ist aktuell
cargo build -p plxs-cli
cargo run -p plxs-verify --bin plxs-fixtures -- run --solver target/debug/plxs
cargo run -p plxs-verify --bin plxs-fixtures -- run --solver target/debug/plxs --area static
cargo run -p plxs-verify --bin plxs-fixtures -- run --solver target/debug/plxs --name kragbalken
cargo run -p plxs-verify --bin plxs-fixtures -- matrix   # diese Tabelle
scripts/solver_reference.sh                   # Korpus und Golden Files neu erzeugen (braucht ccx)
```

Ein Stufen-Thread schreibt in `solver/plxs-verify` einen Test, der `plxs_verify::run_corpus`
auf seinen Bereich anwendet und Bestehen verlangt, sobald seine Fixtures grün sind; so bleibt
die Parität in der CI. Bis dahin meldet `plxs-fixtures run` für jede Fixture `pass`, `FAIL`
(mit der ersten Abweichung), `ERROR` (Solver bricht ab, in M0 für alles außer `*NO ANALYSIS`:
`procedure not implemented yet`) oder `no reference`.

## Feature-Matrix

Erzeugt mit `plxs-fixtures matrix`; `cargo test -p plxs-verify` schlägt fehl, wenn die Tabelle
nicht mehr zum Korpus passt. Die Spalte Features lässt weg, was fast jede Fixture hat
(`*NODE`, `*ELEMENT`, `*STEP`, `*BOUNDARY`, `*CLOAD`, `*DLOAD`, `*NODE FILE`, ...).

| Fixture | Class | Stage | Reference | Features |
|---|---|---|---|---|
| `constraints/detected-bonded` | A | M2 | ccx 2.21 | C3D8, \*TIE, \*STATIC |
| `constraints/drehung` | A | M2 | ccx 2.21 | C3D8, \*RIGID BODY, \*STATIC |
| `constraints/each-node-system` | A | M2 | ccx 2.21 | C3D8, \*TRANSFORM, \*STATIC |
| `constraints/feder-flaeche-einzeln` | A | M2 | ccx 2.21 | C3D8, SPRING2, \*SPRING, \*EQUATION, \*STATIC |
| `constraints/feder-flaeche` | A | M2 | ccx 2.21 | C3D8, SPRING2, \*SPRING, \*EQUATION, \*STATIC |
| `constraints/federn-knoten` | A | M2 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*STATIC |
| `constraints/federn` | A | M2 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*STATIC |
| `constraints/gelockert` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*STATIC, \*NODE PRINT |
| `constraints/geteilt-false-false` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*TIE, \*STATIC, \*NODE PRINT |
| `constraints/geteilt-false-true` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*TIE, \*STATIC, \*NODE PRINT |
| `constraints/geteilt-true-false` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*TIE, \*STATIC, \*NODE PRINT |
| `constraints/geteilt-true-true` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*TIE, \*STATIC, \*NODE PRINT |
| `constraints/hinge-and-pin` | A | M2 | ccx 2.21 | C3D8, \*RIGID BODY, \*STATIC |
| `constraints/hinged-beam` | A | M2 | ccx 2.21 | C3D20R, \*RIGID BODY, \*STATIC |
| `constraints/moment` | A | M2 | ccx 2.21 | C3D8, \*RIGID BODY, \*STATIC |
| `constraints/referenzpunkt-2d` | A | M2 | ccx 2.21 | CPS8, \*RIGID BODY, \*STATIC |
| `constraints/region-bonded` | A | M2 | ccx 2.21 | C3D8, \*TIE, \*STATIC |
| `constraints/region-detected-gap` | A | M2 | ccx 2.21 | C3D8, \*TIE, \*STATIC |
| `constraints/region-no-separation-held` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*STATIC |
| `constraints/region-no-separation` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*STATIC |
| `constraints/region-tie` | A | M2 | ccx 2.21 | C3D8, \*RIGID BODY, \*TIE, \*STATIC |
| `constraints/remote-2d-deformable` | A | M2 | ccx 2.21 | CPS8, \*EQUATION, \*STATIC |
| `constraints/remote-2d` | A | M2 | ccx 2.21 | CPS8, \*RIGID BODY, \*STATIC |
| `constraints/remote-c3d20r-deformable` | A | M2 | ccx 2.21 | C3D20R, \*EQUATION, \*STATIC |
| `constraints/remote-c3d20r-rigid` | A | M2 | ccx 2.21 | C3D20R, \*RIGID BODY, \*STATIC |
| `constraints/remote-deformable` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*STATIC |
| `constraints/remote-system-deformable` | A | M2 | ccx 2.21 | C3D8, \*TRANSFORM, \*EQUATION, \*STATIC |
| `constraints/remote-system` | A | M2 | ccx 2.21 | C3D8, \*TRANSFORM, \*RIGID BODY, \*STATIC |
| `constraints/remote` | A | M2 | ccx 2.21 | C3D8, \*RIGID BODY, \*STATIC |
| `constraints/rigid-body-held` | A | M2 | ccx 2.21 | C3D8, \*RIGID BODY, \*TIE, \*STATIC |
| `constraints/schraube-ganz` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*STATIC, \*NODE PRINT |
| `constraints/symmetrie-2d-30` | A | M2 | ccx 2.21 | CPS8, \*TRANSFORM, \*STATIC |
| `constraints/symmetrie-solid-30` | A | M2 | ccx 2.21 | C3D8, \*TRANSFORM, \*STATIC |
| `constraints/tet-schraeg` | A | M2 | ccx 2.21 | C3D10, \*EQUATION, \*STATIC, \*NODE PRINT |
| `constraints/tet-vorspannung` | A | M2 | ccx 2.21 | C3D10, \*EQUATION, \*STATIC, \*NODE PRINT |
| `constraints/tie` | A | M2 | ccx 2.21 | C3D8, \*TIE, \*STATIC |
| `constraints/verkuerzung` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*STATIC, \*NODE PRINT |
| `constraints/verriegelt` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*STATIC, \*NODE PRINT |
| `constraints/verteilte-temperatur` | A | M2 | ccx 2.21 | C3D8, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*RIGID BODY, \*INITIAL CONDITIONS, \*STATIC, \*TEMPERATURE |
| `constraints/vorspannung` | A | M2 | ccx 2.21 | C3D8, \*EQUATION, \*STATIC, \*NODE PRINT |
| `contact/kontakt-2d` | C | M3 | ccx 2.21 | CPE8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC |
| `contact/kontakt-hard-m` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC |
| `contact/kontakt-hard-mm` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC |
| `contact/kontakt-hart` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC |
| `contact/kontakt-linear` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC |
| `contact/kontakt-mortar` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*CONTACT PAIR, \*STATIC |
| `contact/kontakt-tied-m` | C | M3 | ccx 2.21 | C3D8, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*FRICTION, \*CONTACT PAIR, \*STATIC |
| `contact/kontakt-tied-mm` | C | M3 | ccx 2.21 | C3D8, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*FRICTION, \*CONTACT PAIR, \*STATIC |
| `contact/kontakt-tied` | C | M3 | ccx 2.21 | C3D8, \*RIGID BODY, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*FRICTION, \*CONTACT PAIR, \*STATIC |
| `contact/region-conductance` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*GAP CONDUCTANCE, \*CONTACT PAIR, \*INITIAL CONDITIONS, \*COUPLED TEMPERATURE-DISPLACEMENT, \*CFLUX |
| `contact/region-contact-results` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE |
| `contact/region-friction-change` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*RIGID BODY, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*FRICTION, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE, \*CHANGE FRICTION |
| `contact/region-frictional` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*RIGID BODY, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*FRICTION, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE |
| `contact/region-frictionless-m` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE |
| `contact/region-frictionless-mm` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE |
| `contact/region-mortar` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE |
| `contact/region-offset` | C | M3 | ccx 2.21 | C3D8, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*CLEARANCE, \*STATIC, \*CONTACT FILE |
| `contact/region-rough` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*RIGID BODY, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*FRICTION, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE |
| `contact/region-switched-off` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE, \*MODEL CHANGE |
| `contact/region-switched-on` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE, \*MODEL CHANGE |
| `contact/region-tied` | C | M3 | ccx 2.21 | C3D8, \*RIGID BODY, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*FRICTION, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE |
| `contact/weak-springs-spalt-nlgeom` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE, NLGEOM |
| `contact/weak-springs-spalt` | C | M3 | ccx 2.21 | C3D8, SPRING1, \*SPRING, \*SURFACE INTERACTION, \*SURFACE BEHAVIOR, \*CONTACT PAIR, \*STATIC, \*CONTACT FILE |
| `cyclic/cyclic_sector` | A | M3 | ccx 2.21 | C3D8, \*TIE, \*CYCLIC SYMMETRY MODEL, \*FREQUENCY, \*SELECT CYCLIC SYMMETRY MODES |
| `cyclic/submodel` | A | M3 | none | \*SUBMODEL, C3D8, \*STATIC |
| `dynamic/anfangsdrehung` | E | M3 | ccx 2.21 | C3D20R, \*AMPLITUDE, \*INITIAL CONDITIONS, \*DYNAMIC |
| `dynamic/anfangsgeschwindigkeit` | E | M3 | ccx 2.21 | C3D20R, \*AMPLITUDE, \*INITIAL CONDITIONS, \*DYNAMIC |
| `dynamic/dynamik` | E | M3 | none | C3D20R, \*AMPLITUDE, \*DYNAMIC |
| `dynamic/dynamik_gedaempft` | E | M3 | none | C3D20R, \*AMPLITUDE, \*DYNAMIC, \*DAMPING |
| `eigen/beulen` | A | M2 | ccx 2.21 | C3D20R, \*BUCKLE |
| `eigen/beulform` | A | M2 | ccx 2.21 | C3D20R, \*BUCKLE |
| `eigen/deaktiviert` | A | M2 | ccx 2.21 | C3D20R, \*FREQUENCY |
| `eigen/eigenfrequenz` | A | M2 | ccx 2.21 | C3D20R, \*STATIC, \*FREQUENCY |
| `eigen/vorspannung-beulen` | A | M2 | ccx 2.21 | C3D20R, \*STATIC, \*BUCKLE |
| `heat/amplitude_temperature` | A | M2 | ccx 2.21 | C3D8, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*AMPLITUDE, \*HEAT TRANSFER, \*CFLUX |
| `heat/amplitudes_thermal` | B | M2 | ccx 2.21 | C3D8, \*PHYSICAL CONSTANTS, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*AMPLITUDE, \*HEAT TRANSFER, \*CFLUX, \*DFLUX, \*FILM, \*RADIATE |
| `heat/film-2d` | A | M2 | ccx 2.21 | CPS8, \*CONDUCTIVITY, \*HEAT TRANSFER, \*CFLUX, \*FILM |
| `heat/instationaer` | B | M2 | ccx 2.21 | C3D8, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*INITIAL CONDITIONS, \*HEAT TRANSFER, \*CFLUX |
| `heat/koerperwaerme` | A | M2 | ccx 2.21 | C3D8, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*HEAT TRANSFER, \*CFLUX, \*DFLUX |
| `heat/konvektion` | A | M2 | ccx 2.21 | C3D8, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*HEAT TRANSFER, \*CFLUX, \*FILM |
| `heat/strahlung` | B | M2 | ccx 2.21 | C3D8, \*PHYSICAL CONSTANTS, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*INITIAL CONDITIONS, \*HEAT TRANSFER, \*CFLUX, \*RADIATE |
| `heat/waermedehnung-entkoppelt` | A | M2 | ccx 2.21 | C3D8, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*INITIAL CONDITIONS, \*UNCOUPLED TEMPERATURE-DISPLACEMENT, \*CFLUX |
| `heat/waermedehnung` | A | M2 | ccx 2.21 | C3D8, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*INITIAL CONDITIONS, \*COUPLED TEMPERATURE-DISPLACEMENT, \*CFLUX |
| `heat/waermeleitung` | A | M2 | ccx 2.21 | C3D8, \*EXPANSION, \*CONDUCTIVITY, \*SPECIFIC HEAT, \*HEAT TRANSFER, \*CFLUX, \*DFLUX |
| `loads/amplitude_hold` | A | M2 | ccx 2.21 | C3D8, \*AMPLITUDE, \*STATIC |
| `loads/amplitude_loads` | A | M2 | ccx 2.21 | C3D8, \*AMPLITUDE, \*STATIC |
| `loads/amplitude_total_time` | A | M2 | ccx 2.21 | C3D8, \*AMPLITUDE, \*STATIC |
| `loads/centrifugal` | A | M2 | ccx 2.21 | C3D8, \*STATIC |
| `loads/gravity` | A | M2 | ccx 2.21 | C3D8, \*STATIC |
| `loads/gravity_c3d20r` | A | M2 | ccx 2.21 | C3D20R, \*STATIC |
| `loads/moving` | A | M2 | ccx 2.21 | C3D8, \*STATIC |
| `modal/modal` | E | M3 | none | C3D20R, \*AMPLITUDE, \*FREQUENCY, \*MODAL DYNAMICS, \*MODAL DAMPING, \*STEADY STATE DYNAMICS |
| `modal/rotor-perturbation-false` | A | M3 | ccx 2.21 | C3D8, \*STATIC, \*FREQUENCY, \*COMPLEX FREQUENCY |
| `modal/rotor-perturbation-true` | A | M3 | ccx 2.21 | C3D8, \*STATIC, \*FREQUENCY, \*COMPLEX FREQUENCY |
| `modal/rotor` | A | M3 | ccx 2.21 | C3D8, \*STATIC, \*FREQUENCY, \*COMPLEX FREQUENCY |
| `nonlinear/amplitude_force` | B | M2 | ccx 2.21 | C3D8, \*AMPLITUDE, \*STATIC, NLGEOM |
| `nonlinear/druck-nur-druck` | B | M2 | ccx 2.21 | C3D8, SPRING1, GAPUNI, \*PLASTIC, \*SPRING, \*GAP, \*STATIC |
| `nonlinear/druck-nur-einzeln` | B | M2 | ccx 2.21 | C3D8, SPRING1, GAPUNI, \*PLASTIC, \*SPRING, \*GAP, \*STATIC |
| `nonlinear/druck-nur-umgedreht-druck` | B | M2 | ccx 2.21 | C3D8, SPRING1, GAPUNI, \*PLASTIC, \*SPRING, \*GAP, \*STATIC |
| `nonlinear/druck-nur-umgedreht-zug` | B | M2 | ccx 2.21 | C3D8, SPRING1, GAPUNI, \*PLASTIC, \*SPRING, \*GAP, \*STATIC |
| `nonlinear/druck-nur-verteilt` | B | M2 | ccx 2.21 | C3D8, SPRING1, GAPUNI, \*PLASTIC, \*SPRING, \*GAP, \*STATIC |
| `nonlinear/druck-nur-zug` | B | M2 | ccx 2.21 | C3D8, SPRING1, GAPUNI, \*PLASTIC, \*SPRING, \*GAP, \*STATIC |
| `nonlinear/gap_stuetzen` | B | M2 | ccx 2.21 | C3D8, GAPUNI, \*GAP, \*STATIC, NLGEOM |
| `nonlinear/kriechen` | B | M2 | ccx 2.21 | C3D8, \*CREEP, \*STATIC, \*VISCO |
| `nonlinear/membran-vorgespannt` | B | M2 | ccx 2.21 | M3D4, \*MEMBRANE SECTION, \*STATIC, NLGEOM |
| `nonlinear/plastisch` | B | M2 | ccx 2.21 | C3D8, \*PLASTIC, \*STATIC |
| `plane/ebene-cax3` | A | M2 | ccx 2.21 | CAX3, \*STATIC |
| `plane/ebene-cax4` | A | M2 | ccx 2.21 | CAX4, \*STATIC |
| `plane/ebene-cax4r` | A | M2 | ccx 2.21 | CAX4R, \*STATIC |
| `plane/ebene-cax6` | A | M2 | ccx 2.21 | CAX6, \*STATIC |
| `plane/ebene-cax8` | A | M2 | ccx 2.21 | CAX8, \*STATIC |
| `plane/ebene-cax8r` | A | M2 | ccx 2.21 | CAX8R, \*STATIC |
| `plane/ebene-cpe3` | A | M2 | ccx 2.21 | CPE3, \*STATIC |
| `plane/ebene-cpe4` | A | M2 | ccx 2.21 | CPE4, \*STATIC |
| `plane/ebene-cpe4r` | A | M2 | ccx 2.21 | CPE4R, \*STATIC |
| `plane/ebene-cpe6` | A | M2 | ccx 2.21 | CPE6, \*STATIC |
| `plane/ebene-cpe8` | A | M2 | ccx 2.21 | CPE8, \*STATIC |
| `plane/ebene-cpe8r` | A | M2 | ccx 2.21 | CPE8R, \*STATIC |
| `plane/ebene-cps3` | A | M2 | ccx 2.21 | CPS3, \*STATIC |
| `plane/ebene-cps4` | A | M2 | ccx 2.21 | CPS4, \*STATIC |
| `plane/ebene-cps4r` | A | M2 | ccx 2.21 | CPS4R, \*STATIC |
| `plane/ebene-cps6` | A | M2 | ccx 2.21 | CPS6, \*STATIC |
| `plane/ebene-cps8` | A | M2 | ccx 2.21 | CPS8, \*STATIC |
| `plane/ebene-cps8r` | A | M2 | ccx 2.21 | CPS8R, \*STATIC |
| `plane/ebener-spannungszustand` | A | M2 | ccx 2.21 | CPS8, \*STATIC |
| `plane/ebener-verzerrungszustand` | A | M2 | ccx 2.21 | CPE8, \*STATIC |
| `plane/elementtyp-cax8r` | A | M2 | ccx 2.21 | CAX8R, \*STATIC |
| `plane/elementtyp-cpe8r` | A | M2 | ccx 2.21 | CPE8R, \*STATIC |
| `plane/rectangle-block` | A | M2 | ccx 2.21 | CPS8R, \*STATIC |
| `plane/rohr-innendruck` | A | M2 | ccx 2.21 | CAX8, \*STATIC |
| `plane/rohr-zug` | A | M2 | ccx 2.21 | CAX8, \*STATIC |
| `plane/symmetrie-2d-0` | A | M2 | ccx 2.21 | CPS8, \*STATIC |
| `sensitivity/sensitivitaet-frequenz` | A | ccx | ccx 2.21 | C3D8, \*DESIGN VARIABLES, \*FREQUENCY, \*SENSITIVITY, \*DESIGN RESPONSE |
| `static/amplitude_hold_ref` | A | M1 | ccx 2.21 | C3D8, \*STATIC |
| `static/amplitude_reference` | A | M1 | ccx 2.21 | C3D8, \*STATIC |
| `static/block` | A | M1 | ccx 2.21 | C3D20R, \*STATIC |
| `static/block_c3d20r` | A | M1 | ccx 2.21 | C3D20R, \*STATIC |
| `static/box-block` | A | M1 | ccx 2.21 | C3D20R, \*STATIC |
| `static/clamped-beam` | A | M1 | ccx 2.21 | C3D20R, \*STATIC |
| `static/druck-verteilt` | A | M1 | ccx 2.21 | C3D8, \*STATIC |
| `static/druck` | A | M1 | ccx 2.21 | C3D8, \*STATIC |
| `static/dynamik_statisch` | A | M1 | ccx 2.21 | C3D20R, \*STATIC |
| `static/elementtyp-c3d10t` | A | M1 | ccx 2.21 | C3D10T, \*STATIC |
| `static/elementtyp-c3d20` | A | M1 | ccx 2.21 | C3D20, \*STATIC |
| `static/elementtyp-c3d8i` | A | M1 | ccx 2.21 | C3D8I, \*STATIC |
| `static/elementtyp-c3d8r` | A | M1 | ccx 2.21 | C3D8R, \*STATIC |
| `static/feldausgabe` | A | M1 | ccx 2.21 | C3D8, \*STATIC |
| `static/kragbalken` | A | M1 | ccx 2.21 | C3D8, \*STATIC |
| `static/kragbalken_c3d20r` | A | M1 | ccx 2.21 | C3D20R, \*STATIC |
| `static/kragbalken_c3d8` | A | M1 | ccx 2.21 | C3D8, \*STATIC |
| `static/modal_statisch` | A | M1 | ccx 2.21 | C3D20R, \*STATIC |
| `static/remote-c3d20r-each-node` | A | M1 | ccx 2.21 | C3D20R, \*STATIC |
| `static/symmetrie-solid-0` | A | M1 | ccx 2.21 | C3D8, \*STATIC |
| `static/traktion` | A | M1 | ccx 2.21 | C3D8, \*STATIC |
| `static/wuerfel_c3d10` | A | M1 | ccx 2.21 | C3D10, \*STATIC |
| `structural/balken_b31` | A | M2 | ccx 2.21 | B31, \*BEAM SECTION, \*STATIC |
| `structural/balken_b32` | A | M2 | ccx 2.21 | B32, \*BEAM SECTION, \*STATIC |
| `structural/balken_gelenk` | A | M2 | ccx 2.21 | B31, \*BEAM SECTION, \*EQUATION, \*STATIC |
| `structural/balken_node_tie` | A | M2 | ccx 2.21 | B31, \*BEAM SECTION, \*STATIC |
| `structural/balken_starr` | A | M2 | ccx 2.21 | B32, \*BEAM SECTION, \*EQUATION, \*STATIC |
| `structural/edge-tie-m3d4` | A | M2 | ccx 2.21 | C3D8, M3D4, \*MEMBRANE SECTION, \*RIGID BODY, \*TIE, \*STATIC |
| `structural/edge-tie-s4` | A | M2 | ccx 2.21 | C3D8, S4, \*SHELL SECTION, \*RIGID BODY, \*TIE, \*STATIC |
| `structural/elementtyp-s8r` | A | M2 | ccx 2.21 | S8R, \*SHELL SECTION, \*STATIC |
| `structural/membran-m3d3` | A | M2 | ccx 2.21 | M3D3, \*MEMBRANE SECTION, \*STATIC |
| `structural/membran-m3d4` | A | M2 | ccx 2.21 | M3D4, \*MEMBRANE SECTION, \*STATIC |
| `structural/membran-m3d4r` | A | M2 | ccx 2.21 | M3D4R, \*MEMBRANE SECTION, \*STATIC |
| `structural/membran-s4r` | A | M2 | ccx 2.21 | M3D4R, \*MEMBRANE SECTION, \*STATIC |
| `structural/rohr_b32r` | A | M2 | ccx 2.21 | B32R, \*BEAM SECTION, \*STATIC |
| `structural/schale` | A | M2 | ccx 2.21 | S8, \*SHELL SECTION, \*STATIC |
| `structural/schalenkante` | A | M2 | ccx 2.21 | S8, \*SHELL SECTION, \*STATIC |
| `structural/stab_node_tie` | A | M2 | ccx 2.21 | T3D2, \*STATIC |
| `structural/stab_t3d2` | A | M2 | ccx 2.21 | T3D2, \*STATIC |
| `structural/symmetrie-0` | A | M2 | ccx 2.21 | S8, \*SHELL SECTION, \*STATIC |
| `structural/symmetrie-30` | A | M2 | ccx 2.21 | S8, \*TRANSFORM, \*SHELL SECTION, \*STATIC |
| `structural/symmetrie-frei` | A | M2 | ccx 2.21 | S8, \*SHELL SECTION, \*STATIC |
| `structural/symmetrie-voll` | A | M2 | ccx 2.21 | S8, \*SHELL SECTION, \*STATIC |
| `structural/u1-angle` | A | M2 | ccx 2.21 | \*USER ELEMENT, U1, \*BEAM SECTION, \*STATIC |
| `structural/u1-beam` | A | M2 | ccx 2.21 | \*USER ELEMENT, U1, \*BEAM SECTION, \*STATIC |
| `structural/u1-round-sketch` | A | M2 | ccx 2.21 | \*USER ELEMENT, U1, \*BEAM SECTION, \*STATIC |
| `structural/u1-sketch` | A | M2 | ccx 2.21 | \*USER ELEMENT, U1, \*BEAM SECTION, \*STATIC |
