# Modellprüfung: typische Abbruchursachen von CalculiX

Was nodalis vor dem Lauf prüft (`crates/plx-model/src/checks.rs`) und nach einem
fehlgeschlagenen Lauf aus der Ausgabe von CalculiX erkennt. Nachgestellt mit CalculiX 2.21
(Debian-Paket `calculix-ccx`, Spooles als Löser).

| Problem | Meldung von CalculiX 2.21 | Verhalten | Prüfung in nodalis |
|---|---|---|---|
| Teile nicht oder unvollständig gelagert (Starrkörperbewegung), z. B. Part ohne Tie | Spooles: keine, Lauf "erfolgreich" mit unbrauchbaren Werten; Pardiso/PaStiX: `zero pivot` | stiller Unsinn oder Abbruch | Rang der Lagerung je zusammenhängendem Teil (Ties verbinden, Kontakte nur lose), nur statische Steps |
| Stabwerk beweglich: Stab (Truss Section) aus mehreren Elementen, ebenes Fachwerk ohne Halt senkrecht zur Ebene | keine, Lauf "erfolgreich" mit beliebigen Verschiebungen an den freien Knoten (Pardiso: riesige Werte) | stiller Unsinn | Stäbe, Lager und Federn je Stabknoten müssen alle drei Richtungen aufspannen (Node Ties zählen als ein Knoten), nur statische Steps |
| Stabwerk an nur einem Knoten gelagert | keine | stiller Unsinn | Starrkörper-Prüfung; Stabknoten haben keine Rotationen, auch wenn das Netz die Linien als B32 führt |
| Teil nur über Kontakt gehalten | `zero pivot`, `too many cutbacks` | oft Abbruch | Warnung |
| Elemente ohne Section | `no material was assigned to element N` | Abbruch | Elemente je Part gegen Sections |
| Element in mehreren Sections (gleiches Part o. ä.) | keine Prüfung durch ccx, zwei Section-Definitionen je Element | Fehler im Check | Elemente je Section, die schon eine frühere Section belegt |
| Material ohne Elastizität | `no elastic constants were assigned` | Abbruch | Materialien, die eine Section nutzt |
| Frequency, Dynamic oder Steady State Dynamics Step ohne Dichte | `no density was assigned` | Abbruch | wie oben |
| Steady State Dynamics ohne vorherigen Frequency Step mit Storage=Yes | `*MODAL DYNAMIC`/`*STEADY STATE DYNAMICS`: `no eigenvalue file` bzw. Lesen von `.eig` schlägt fehl | Abbruch | Fehler: Modal-Steps brauchen einen aktiven Frequency Step mit gespeicherten Eigenformen davor |
| `*Boundary, op=New` in einem Modal-Step (Dynamic mit Modal superposition, Steady State Dynamics) | `in a modal dynamic step new SPCs are not allowed` | Abbruch | Writer lässt das Zurücksetzen der Lagerung in Modal-Steps weg (gleiche Lager erneut angeben ist erlaubt) |
| Wärmeübertragung ohne Wärmeleitfähigkeit | nur `WARNING ... no conductivity constants`, dann Absturz beim Lösen | Abbruch | Materialien in thermischen Steps |
| Instationäre Wärmeübertragung ohne Dichte, spezifische Wärme oder Anfangstemperatur | `no density was assigned`, `no specific heat was assigned`, `please define initial conditions for the temperature` | Abbruch | Materialien und Initial Conditions |
| Querkontraktionszahl >= 0,5 | `Poisson coefficient should be less than 0.5` | Abbruch beim Einlesen | E > 0, -1 < nu < 0,5 |
| Verzerrte/umgestülpte Elemente, 2D-Elemente im Uhrzeigersinn | `nonpositive jacobian determinant in element N` | Abbruch | Jacobi-Determinante an den Integrationspunkten (`plx-mesh/src/jacobian.rs`) |
| Anfangsinkrement > Step-Dauer | `initial increment size exceeds step size` | Abbruch | Step-Einstellungen |
| Gleiche Knoten in mehreren Randbedingungen mit verschiedenen Werten | keine | letzter Wert gilt | Warnung an der späteren Randbedingung |
| Rotationen (DOF 4-6) an Volumenelementen | keine | ignoriert | Warnung; an Balken und Schalen zählen sie für die Lagerung |
| Rotationen (DOF 4-6) in 2D-Modellen | `usermpc: mpc of type is unknown` | Abbruch (PR #59: auch Gleitkommafehler) | Writer schreibt nur DOF 1-2; Ausgabe wird erkannt |
| Freiheitsgrad in Randbedingung und abhängig in Gleichung | `dependent side of a MPC and a SPC` | Abbruch | Ausgabe wird erkannt |
| Randbedingung auf Tie-Slave-Knoten | `WARNING in gentiedmpc: DOF ... not active` | Tie dort aufgehoben, Lauf geht weiter | keine (meist harmlos) |
| Keine Konvergenz | `too many cutbacks`, `increment size smaller than minimum` | Abbruch | Ausgabe wird erkannt |
| Last nur auf festgehaltenen Knoten | keine | Last wirkungslos | Warnung |
| Statischer Step ohne Last | keine | Ergebnis null | Warnung; eine vorgegebene Temperatur (Defined Field) zählt als Last |
| Vorgegebene Temperatur (Defined Field) ohne Anfangstemperatur | `a *TEMPERATURE card is detected but no thermal *INITIAL CONDITIONS are given` | Abbruch beim Einlesen | Initial Conditions in Steps mit Defined Fields |
| Vorgegebene Temperatur ohne Wärmeausdehnung im Material | keine | Temperatur ohne Wirkung | Warnung |
| Anfangsgeschwindigkeit (Initial Condition Velocity) ohne Dynamic Step | keine | ignoriert | Warnung an der Anfangsbedingung; im Dynamic Step zählt sie als Last |
| Frequency Step ohne Lagerung | keine | sechs Starrkörpermoden nahe 0 Hz | keine (gewollt) |
| Balken-, Fachwerk- oder Schalenknoten in einem `*RIGID BODY` (Rigid Body Constraint, Displacement mit Rigid-Punkt) | 2.21: keine; 2.23 (mitgeliefert): `gen3dmpc: nodes belonging to beam elements must not be subject to a RIGID MPC` (Schalen ebenso), außer der Knoten ist ein Knot | Abbruch mit 2.23 | Writer koppelt diese Knoten über lineare `*EQUATION`s an Referenz- und Rotationsknoten (gleiche Ergebnisse wie `*RIGID BODY` in 2.21); Warnung nur bei NLGEOM (kleine Rotationen); Ausgabe wird erkannt |

Quellen: CalculiX-Handbuch (ccx 2.21), [PrePoMax-Forum: Known CalculiX limitations](https://prepomax.discourse.group/t/known-calculix-limitations/3050),
[PrePoMax-Forum: too many cutbacks](https://prepomax.discourse.group/t/error-too-many-cutbacks/2602),
[CalculiX-Forum: nonpositive jacobian](https://calculix.discourse.group/t/help-error-in-e-c3d-nonpositive-jacobian/1320),
[CalculiX-Forum: increment size smaller than minimum](https://calculix.discourse.group/t/error-increment-size-smaller-than-minimum/199).
