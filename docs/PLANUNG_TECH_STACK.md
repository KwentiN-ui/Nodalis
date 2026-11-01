# nodalis – Analyse, Tech-Stack und Roadmap

Stand: 2026-10-08 · Grundlage: PrePoMax `master` von gitlab.com/MatejB/PrePoMax (Commit `fb96810`, 2026-10-06).
Deine lokale Checkout-Version kann davon leicht abweichen. Das nodalis-Repo enthielt zu Beginn nur ein `cargo init`-Gerüst.

---

## 1. PrePoMax im Überblick

### Umfang

| Projekt | Zweck | C#-Zeilen |
|---|---|---|
| `CaeGlobals` | Einheitensysteme, Formeln (NCalc), Selektions-Primitive, Tools | ~28k |
| `CaeMesh` | Knoten, Elemente, Sets, Surfaces, Geometrie-Topologie, Mesher-Setup (Netgen, Gmsh, Mmg, QuadWild, Transfinite) | ~59k |
| `CaeModel` | FE-Modell: Materialien, Sections, Constraints, Kontakt, Steps, BCs, Loads, Outputs + Ein/Ausgabe-Formate | ~44k |
| `CaeResults` | Ergebnis-Datenmodell, Feld- und History-Outputs, `.frd`/`.dat`-Reader | ~21k |
| `CaeJob` | Solver-Prozess starten und überwachen | ~2k |
| `vtkControl` | 3D-Viewport auf Basis ActiViz/VTK 5.8 (Actors, Farbskala, Animation, Schnittansicht, Widgets) | ~24k |
| `UserControls` | WinForms-Controls: Modellbaum, Property-Grid, Fortschrittsanzeige | ~29k |
| `PrePoMax` | GUI-Anwendung: `FrmMain` (~12k), `Controller` (~24k), ~158 Dialoge mit `View*`-Wrappern, Command-Klassen | ~153k |
| `APIs` | Eingebettete Python-API + MCP-Automationsserver | ~33k |

Insgesamt rund **400 000 Zeilen C#** (.NET Framework 4.8, WinForms). Ein vollständiger Nachbau ist also ein langfristiges Projekt; wir sollten gezielt den wertvollen Kern zuerst bauen.

### Architektur, die wir übernehmen sollten

1. **Schichtung** `Globals → Mesh → Model → Results`, Solver-Job separat, GUI ganz oben. Das passt eins zu eins auf einen Cargo-Workspace.
2. **Command-Pattern als Rückgrat.** Jede Benutzeraktion ist ein serialisierbares Command (`CAddLoad` usw.). Die History wird gespeichert, und *Edit → Regenerate* spielt die gesamte Sitzung von vorn ab (z. B. nach Geometrieänderung). Selektionen werden deshalb nicht als ID-Listen gespeichert, sondern als Rezept (`SelectionNodeMouse`, `SelectionNodeIds`, `SelectionNodeInvert`), das nach einem Neuvernetzen erneut aufgelöst wird. **Das ist die wichtigste Designentscheidung von PrePoMax und muss in nodalis von Anfang an drin sein.**
3. **Property-Wrapper** (`ViewCLoad` usw.): Jedes editierbare Objekt hat eine Ansichtsschicht mit Kategorie, Anzeigename, Einheit und Beschreibung. In Rust lässt sich das mit einem Trait plus Derive-Makro schlanker lösen.
4. **Eingabewerte als Formeln mit Einheit** (`EquationString`), nicht als nackte `f64`.

### Externe Werkzeuge und Bibliotheken

| Komponente | Wie PrePoMax sie nutzt | Plattform-Problem |
|---|---|---|
| **OpenCASCADE 8.0.1 + Netgen 5.3.1** | Eigenes Hilfsprogramm `NetGenMesher.exe`, per Kommandozeile aufgerufen (`STEP_ASSEMBLY_SPLIT_TO_COMPOUNDS`, `BREP_MESH`, `STL_MESH`, `BREP_DEFEATURE`, `BREP_SPLIT` …). Austausch über Dateien (`.brep`, `.vol`, `.vis` = Tessellierung für die Anzeige). | Quellcode des Helpers liegt **nicht** im Repo, nur die Windows-Binaries |
| **Gmsh 4.13** | C++/CLI-Wrapper `GmshCommon` + `GmshCaller.exe` (Hex-/Sweep-/Transfinite-Meshing, Three-Block-Mesher) | C++/CLI ist Windows-only; Gmsh selbst ist plattformunabhängig |
| **Mmg** (`mmg3d`, `mmgs`) | Remeshing per Kommandozeile | plattformunabhängig verfügbar |
| **QuadWild** | Quad-dominante Flächenvernetzung | experimentell, später |
| **lp_solve** | Intervall-Zuweisung für strukturierte Netze | später |
| **ActiViz/VTK 5.8** | gesamte 3D-Darstellung | .NET-Wrapper, Windows-only, uralt |
| **NCalc** | Formelauswertung | .NET |
| **CalculiX `ccx`** | Solver, extern | Linux nativ verfügbar |

### Dateiformate

- **Import Geometrie:** STEP, IGES, BREP (via OCC), STL, OBJ, 3MF
- **Import Netz/Modell:** `.inp` (CalculiX/Abaqus, inkl. Materialien), `.unv`, `.vol` (Netgen), `.mesh` (Mmg)
- **Export:** CalculiX-`.inp`, Abaqus-`.inp`, STL, 3MF, Gmsh, Mmg
- **Ergebnisse:** `.frd` (ASCII **und** Binär, auch gemischt), `.dat` (History), dazu OpenFOAM und Punktwolken
- **Projekt:** `.pmx` (Modell+Ergebnisse) und `.pmh` (History) – beide per .NET `BinaryFormatter` serialisiert. **Kompatibilität zu `.pmx` ist praktisch ausgeschlossen**; der Austausch mit PrePoMax läuft sinnvollerweise über `.inp`.

---

## 2. Tech-Stack-Vorschlag

### Cargo-Workspace (spiegelt die PrePoMax-Schichtung)

```
nodalis/
├─ crates/
│  ├─ plx-core      Einheiten, Formeln, Selektions-Rezepte, IDs, Fehler
│  ├─ plx-mesh      Knoten, Elementtypen, Sets, Surfaces, Skin-Extraktion, Geometrie-Topologie
│  ├─ plx-model     Materialien, Sections, Steps, BCs, Loads, Kontakt, Commands
│  ├─ plx-io        .inp lesen/schreiben, .frd/.dat lesen, .unv/.vol/.stl …
│  ├─ plx-results   Ergebnis-Datenmodell, abgeleitete Größen (von Mises, Hauptspannungen)
│  ├─ plx-mesher    Gmsh-Anbindung (FFI), später Netgen/OCC
│  ├─ plx-job       ccx starten, stdout/.sta/.cvg mitlesen
│  ├─ plx-render    wgpu-Renderer für FE-Netze
│  └─ plx-app       GUI (egui), Controller, Dialoge
└─ testdata/        kleine .inp/.frd-Beispiele
```

Die unteren Crates bleiben **frei von GUI-Abhängigkeiten** und sind damit einfach testbar (das fehlt PrePoMax komplett: kein Testprojekt, kein CI).

### GUI: **egui / eframe** (Empfehlung)

- Läuft identisch auf Linux (X11 und Wayland) und Windows, ein Binary, keine Laufzeit-Abhängigkeiten.
- Immediate Mode passt gut zu den vielen Eigenschafts-Dialogen: ein Property-Grid ist im Wesentlichen eine Schleife über Felder. Mit einem eigenen Derive-Makro (`#[derive(Properties)]` mit `#[prop(category, unit, desc)]`) bekommen wir die 158 `View*`-Wrapper fast geschenkt.
- `egui_dock` für andockbare Panels (Modellbaum, Viewport, Ausgabe), Baumansicht als eigenes Widget oder `egui_ltreeview`.
- Der 3D-Viewport wird per `egui_wgpu`-Paint-Callback direkt mit wgpu in die Oberfläche gerendert.

Alternativen:
- **Slint**: deklarativ, sehr gutes natives Aussehen, aber ein großes dynamisches Property-Grid und 3D-Integration sind mühsamer. Lizenz GPL/kommerziell (für ein GPL-Projekt okay).
- **Qt über cxx-qt**: am nächsten am „Desktop-CAE“-Look, aber schwerer Build (Qt auf Windows), viel C++-Glue. Für ein Hobbyprojekt zu viel Reibung.
- **iced**: elegant, aber weniger fertige Widgets für dichte Werkzeug-UIs.

### 3D-Rendering: **eigener Renderer auf wgpu + glam** (Empfehlung)

Es gibt in Rust kein VTK-Äquivalent, aber was ein FE-Viewer braucht, ist überschaubar:

- **Skin-Extraktion** auf der CPU: nur die äußeren Elementflächen rendern (Millionen Elemente bleiben so flüssig). Höhere Elemente (C3D10, C3D20, S8) werden für die Darstellung in lineare Teildreiecke zerlegt.
- **Feature-Kanten** und Netzkanten als eigene Linien-Passes.
- **Konturplots** per 1D-Farbtextur und Interpolation des Skalars im Fragment-Shader (korrekte Bänder, auch „banded“ wie in PrePoMax), Farbskala als egui-Overlay.
- **Verformte Darstellung und Animation**: Verschiebungen als zweiter Vertex-Buffer, Skalierung als Uniform, d. h. Animation kostet fast nichts.
- **Picking** über einen ID-Buffer (Offscreen-Pass, der Element-/Flächen-/Knoten-IDs schreibt); Rechteckselektion liest einen Bereich aus. Das ist robuster und schneller als Raycasting.
- **Schnittansicht** über Clip-Plane im Shader, Transparenz, Hide/Show pro Part.

Alternativen:
- **three-d** oder **kiss3d**: schnellerer Einstieg, aber bei Picking, Schnitt und großen Netzen stößt man an Grenzen.
- **Bevy**: Spiele-Engine mit ECS, zu viel Ballast, passt schlecht zur Werkzeug-UI.
- **VTK über FFI**: es gibt keine brauchbaren Rust-Bindings; der Build-Aufwand wäre größer als der Nutzen.

### CAD-Geometrie und Vernetzung: **Gmsh über die C-API** (Empfehlung für den Start)

PrePoMax trennt Geometriekern (OCC im NetGenMesher) und Mesher. Für den Anfang deckt **Gmsh allein** beides ab:

- Gmsh enthält OpenCASCADE und importiert STEP/IGES/BREP.
- Es liefert Topologie (Volumen, Flächen, Kanten, Punkte mit stabilen Tags), die wir für die Geometrie-Selektion im Viewport brauchen, und eine Tessellierung für die Anzeige.
- Es vernetzt Tet (1. und 2. Ordnung, inklusive Netgen-Optimierer), Hex/Transfinite, Extrusion; lokale Netzgrößen pro Fläche/Kante.
- Gmsh bietet ein offizielles SDK (`libgmsh.so` / `gmsh.dll` + `gmshc.h`) für Linux und Windows. Wir schreiben einen dünnen eigenen FFI-Layer über die stabile C-API (bestehende Crates wie `gmsh-sys` sind veraltet) und laden die Bibliothek zur Laufzeit (`libloading`), damit nodalis auch ohne Gmsh startet.
- Umgesetzt (Oktober 2026): Die offizielle Bibliothek kommt aus dem PyPI-Paket von Gmsh (`scripts/fetch_gmsh.py`, Linux und Windows, OCC/Netgen/TetGen eingebaut) und wird neben der Binary ausgeliefert. Lizenzen: Gmsh GPL-2.0-or-later mit OCC-Ausnahme, OCCT LGPL-2.1 – beides verträglich mit GPL-3.0. `opencascade-rs` ist für Import und Vernetzung nicht nötig und würde OCCT aus den Quellen bauen; es bleibt eine Option für spätere CAD-Operationen.
- Lizenz: GPL-2.0-or-later mit Ausnahme, passt zu einem GPL-3-Projekt.

Später, für PrePoMax-Parität bei CAD-Operationen (Defeaturing, Flächen splitten, Shell↔Solid, Solids verschmelzen):
- **Variante A:** eigenes kleines C++-Hilfsprogramm mit OCC + Netgen nach dem Vorbild von `NetGenMesher.exe`, per Kommandozeile und Dateien angebunden. Bewährtes Muster, isoliert Abstürze, aber C++-Build für zwei Plattformen.
- **Variante B:** `opencascade-rs` (Rust-Bindings über `cxx`). Direkter, aber unvollständig; fehlende Funktionen müssten wir selbst ergänzen.

Mmg (Remeshing) lässt sich wie in PrePoMax als externes Programm einbinden, sobald es gebraucht wird.

### CalculiX-Formate: **eigene Parser und Writer in Rust**

- `.inp`-Writer: Portierung von `CaeModel/FileInOut/Output/Calculix`. Das ist überwiegend Formatierungsarbeit und gut testbar (Snapshot-Tests gegen von PrePoMax erzeugte `.inp`-Dateien).
- `.inp`-Reader: handgeschriebener zeilenbasierter Parser (Keyword-Zeilen, Parameter, Datenzeilen, `*INCLUDE`). Kein Parser-Generator nötig.
- `.frd`-Reader: ASCII und Binär, mit `memmap2` und Streaming für große Dateien; Referenz ist `FrdFileReader.cs`, das auch gemischte Blöcke behandelt.
- `.dat`-Reader für History-Outputs.
- Solverlauf (`plx-job`): `std::process::Command`, stdout in einem Thread lesen, `.sta`/`.cvg` für die Konvergenzanzeige auswerten. `ccx` kommt unter Linux aus dem Paketmanager, unter Windows z. B. die Builds, die PrePoMax mitliefert. Pfad in den Einstellungen.

### Querschnitt

| Thema | Wahl | Bemerkung |
|---|---|---|
| Mathe | `glam` (Render), `nalgebra` (Tensoren, Hauptspannungen) | |
| Formeln | `evalexpr` | Ersatz für NCalc; Parameter wie in PrePoMax |
| Einheiten | eigenes Laufzeit-Einheitensystem | `uom` ist compile-time und passt nicht zu wählbaren Benutzer-Einheiten |
| Projektdatei | `serde` + versioniertes Format (z. B. MessagePack via `rmp-serde`, optional zstd-komprimiert) | Versionsfeld von Anfang an; History als Liste von Commands |
| Parallelität | `rayon` für Skin/Ergebnisse, Hintergrund-Threads + Kanäle für Meshing und Solver | |
| Fehler/Logs | `thiserror`, `anyhow` (App), `tracing` | |
| Dateidialoge | `rfd` | nativ auf beiden Plattformen |
| Tests/CI | `cargo test`, `insta` (Snapshots), GitHub Actions mit Linux- und Windows-Runner | |
| Python-API | später `pyo3` | erst wenn das Command-System steht |

### Lizenz

PrePoMax ist GPL-3.0. Sobald wir Code oder nicht-triviale Logik portieren (z. B. `.frd`-Reader, Transfinite-Mapping), ist nodalis ein abgeleitetes Werk und muss ebenfalls **GPL-3.0** sein. Das ist für ein FOSS-Hobbyprojekt unproblematisch und passt zu Gmsh und Netgen. Eine freizügigere Lizenz (MIT/Apache) ginge nur mit strikt eigenständiger Neuimplementierung.

---

## 3. Roadmap

Strategischer Vorschlag: **mit dem Postprozessor beginnen.** Ein `.frd`-Viewer ist in sich geschlossen, braucht kein CAD und ist unter Linux sofort nützlich (dort gibt es außer `cgx` und ParaView-Umwegen wenig Komfortables). Gleichzeitig entsteht dabei der Renderer, den der Präprozessor ohnehin braucht.

### M0 – Grundgerüst
- Cargo-Workspace mit den Crates oben, GPL-3-Lizenz, README.
- eframe-Fenster mit Dock-Layout (Baum links, Viewport Mitte, Ausgabe unten), leerer wgpu-Viewport mit Orbit/Pan/Zoom.
- CI auf Linux und Windows.

### M1 – Netz anzeigen
- `.inp`-Reader für Knoten, Elemente (C3D4/6/8/10/15/20, S3/4/6/8, B31/B32, T3D2), Sets und Surfaces.
- Skin-Extraktion, Rendering mit Netzkanten und Feature-Kanten, Parts im Modellbaum, Ein-/Ausblenden.
- Testdaten: ein paar kleine CalculiX-Beispiele.

### M2 – Ergebnis-Viewer (erstes echt nutzbares Release)
- `.frd`-Reader (ASCII + Binär), Auswahl von Step/Inkrement/Feld/Komponente.
- Konturplot mit Farbskala, verformte Darstellung mit Skalierung, Animation.
- Abgeleitete Größen (von Mises, Hauptspannungen), Min/Max-Anzeige, Knotenwert per Klick (Picking).
- `.dat`-History als einfache Tabelle.

### M3 – Einfaches Setup und Rechnen (geschlossener Kreis)
- **Command-System und Selektions-Rezepte** (Architekturkern, siehe 1.2).
- Elastisches Material, Solid Section, statischer Step, Fixed BC, Einzellast und Druck auf Knoten-/Flächensets, Field-Outputs.
- Selektion im Viewport (Knoten, Elementflächen, Rechteck).
- Property-Grid mit Einheiten und Formeln.
- `.inp` schreiben, `ccx` starten, Fortschritt anzeigen, Ergebnisse automatisch laden.
- Projektdatei speichern/laden, Undo über die History.

### M4 – Geometrie und Vernetzung
- STEP/IGES/BREP-Import über Gmsh, Anzeige mit Flächen- und Kantentopologie.
- Geometrie-Selektion (Flächen, Kanten, Volumen) und Übertragung auf Netz-Sets nach dem Vernetzen.
- Mesh-Setup: globale und lokale Netzgröße, Tet 1./2. Ordnung.
- *Regenerate*: Geometrie ersetzen, History neu abspielen.

### M5 und danach – Richtung PrePoMax-Parität
- Kontakt (Tie, Contact Pair, Surface Interaction), Constraints (Coupling, Rigid Body).
- Weitere Steps (Frequency, Buckle, Heat Transfer, Dynamic), weitere Lasten und Amplituden.
- Schalen und Balken inkl. Darstellung der Querschnitte, Hex-/Transfinite-Vernetzung.
- CAD-Operationen (Defeaturing, Split) über OCC-Helper oder `opencascade-rs`.
- Abaqus-Export, Python-API, ggf. MCP-Server wie in PrePoMax.

---

## 4. Offene Entscheidungen

| # | Frage | Empfehlung |
|---|---|---|
| 1 | Lizenz | GPL-3.0 (erlaubt das Portieren von PrePoMax-Logik) |
| 2 | GUI-Framework | egui/eframe |
| 3 | 3D-Rendering | eigener wgpu-Renderer |
| 4 | CAD/Mesher für den Start | Gmsh über die C-API |
| 5 | Reihenfolge | Postprozessor zuerst (M1/M2), dann Setup |
| 6 | GitHub-Repo und CI | Repo für nodalis anlegen und hier im Projekt verbinden, dann CI mit Linux+Windows |
| 7 | Mindest-Toolchain | aktuelles stabiles Rust, kein Nightly |
