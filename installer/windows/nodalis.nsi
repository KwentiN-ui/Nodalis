; Windows-Installer für Nodalis, gebaut mit NSIS 3 (makensis, läuft auch unter Linux).
; Normalerweise ruft scripts/build_windows_installer.sh das Skript auf und übergibt:
;   VERSION  Programmversion, z. B. 0.1.0
;   STAGE    Ordner mit allen Dateien der Installation
;   OUTFILE  Pfad des fertigen Installers

Unicode true
SetCompressor /SOLID lzma

!macro REQUIRE NAME
  !ifndef ${NAME}
    !error "${NAME} fehlt: VERSION, STAGE und OUTFILE mit -D angeben, siehe scripts/build_windows_installer.sh"
  !endif
!macroend
!insertmacro REQUIRE VERSION
!insertmacro REQUIRE STAGE
!insertmacro REQUIRE OUTFILE

!define APP "nodalis"
!define APPNAME "Nodalis"
!define PROGID "nodalis.Projekt"
!define UNINSTALL_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP}"
!define SOURCE_URL "https://github.com/KwentiN-ui/Nodalis"

!include "MUI2.nsh"
!include "x64.nsh"
!include "FileFunc.nsh"
!include "Sections.nsh"

; 1 when Nodalis runs the installer to update itself (Help > Check for Updates): /UPDATE.
Var Update

Name "${APPNAME} ${VERSION}"
OutFile "${OUTFILE}"
InstallDir "$PROGRAMFILES64\${APP}"
InstallDirRegKey HKLM "Software\${APP}" "InstallDir"
RequestExecutionLevel admin
ManifestDPIAware true
BrandingText "${APPNAME} ${VERSION}"

VIProductVersion "${VERSION}.0"
VIAddVersionKey /LANG=1033 "ProductName" "${APPNAME}"
VIAddVersionKey /LANG=1033 "ProductVersion" "${VERSION}"
VIAddVersionKey /LANG=1033 "FileVersion" "${VERSION}"
VIAddVersionKey /LANG=1033 "FileDescription" "${APPNAME} Installer"
VIAddVersionKey /LANG=1033 "LegalCopyright" "GNU GPL 3.0 or later"

!define MUI_ICON "${STAGE}\nodalis.ico"
!define MUI_UNICON "${STAGE}\nodalis.ico"
!define MUI_ABORTWARNING
!define MUI_COMPONENTSPAGE_SMALLDESC

!define MUI_WELCOMEPAGE_TEXT "This wizard installs ${APPNAME} ${VERSION}, a pre- and postprocessor for CalculiX.$\r$\n$\r$\nThe CalculiX solver (ccx) is included, so analyses run right after installation. Another CalculiX can be set under Tools > Settings.$\r$\n$\r$\n$_CLICK"
; An update only shows the progress: the pages before and the finish page are skipped.
!define MUI_PAGE_CUSTOMFUNCTION_PRE SkipIfUpdate
!insertmacro MUI_PAGE_WELCOME
!define MUI_PAGE_CUSTOMFUNCTION_PRE SkipIfUpdate
!insertmacro MUI_PAGE_LICENSE "${STAGE}\LICENSE.txt"
!define MUI_PAGE_CUSTOMFUNCTION_PRE SkipIfUpdate
!insertmacro MUI_PAGE_COMPONENTS
!define MUI_PAGE_CUSTOMFUNCTION_PRE SkipIfUpdate
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!define MUI_PAGE_CUSTOMFUNCTION_PRE SkipIfUpdate
!define MUI_FINISHPAGE_RUN
!define MUI_FINISHPAGE_RUN_TEXT "Run ${APPNAME}"
!define MUI_FINISHPAGE_RUN_FUNCTION StartWithoutAdminRights
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES

!insertmacro MUI_LANGUAGE "English"

Function .onInit
  ${IfNot} ${RunningX64}
    MessageBox MB_OK|MB_ICONSTOP "${APPNAME} requires 64-bit Windows."
    Abort
  ${EndIf}
  SetRegView 64
  StrCpy $Update 0
  ${GetParameters} $0
  ClearErrors
  ${GetOptions} $0 "/UPDATE" $1
  ${IfNot} ${Errors}
    StrCpy $Update 1
    Call KeepChoices
    Call WaitForNodalis
  ${EndIf}
FunctionEnd

Function SkipIfUpdate
  ${If} $Update == 1
    Abort
  ${EndIf}
FunctionEnd

; Nodalis closes once it has started the update; nodalis.exe cannot be replaced before.
; Waits up to 30 seconds until it can be opened for writing.
Function WaitForNodalis
  StrCpy $1 0
  ${DoWhile} ${FileExists} "$INSTDIR\nodalis.exe"
    ClearErrors
    FileOpen $0 "$INSTDIR\nodalis.exe" a
    ${IfNot} ${Errors}
      FileClose $0
      ${Break}
    ${EndIf}
    IntOp $1 $1 + 1
    ${If} $1 > 60
      ${Break}
    ${EndIf}
    Sleep 500
  ${Loop}
FunctionEnd

; After an update Nodalis starts again, as the logged-in user like from the finish page.
Function .onInstSuccess
  ${If} $Update == 1
    Call StartWithoutAdminRights
  ${EndIf}
FunctionEnd

; The installer runs with administrator rights; nodalis itself should not, so it is started
; through Explorer, which runs it as the logged-in user.
Function StartWithoutAdminRights
  Exec '"$WINDIR\explorer.exe" "$INSTDIR\nodalis.exe"'
FunctionEnd

Section "${APPNAME} (required)" SecApp
  SectionIn RO
  SetShellVarContext all
  SetOutPath "$INSTDIR"
  File "${STAGE}\nodalis.exe"
  File "${STAGE}\gmsh-4.15.dll"
  File "${STAGE}\nodalis.ico"
  File "${STAGE}\LICENSE.txt"
  File "${STAGE}\THIRD-PARTY.txt"
  File "${STAGE}\SOURCE.txt"
  SetOutPath "$INSTDIR\licenses"
  File "${STAGE}\licenses\*.txt"
  ; The offline user guide that Help > User Guide opens.
  SetOutPath "$INSTDIR\doc"
  File "${STAGE}\doc\user-guide.html"
  ; CalculiX with its DLLs, used when no other CalculiX is set. An update replaces it whole.
  RMDir /r "$INSTDIR\ccx"
  SetOutPath "$INSTDIR\ccx"
  File /r "${STAGE}\ccx\*.*"
  SetOutPath "$INSTDIR"

  WriteUninstaller "$INSTDIR\uninstall.exe"
  WriteRegStr HKLM "Software\${APP}" "InstallDir" "$INSTDIR"
  WriteRegStr HKLM "${UNINSTALL_KEY}" "DisplayName" "${APPNAME}"
  WriteRegStr HKLM "${UNINSTALL_KEY}" "DisplayVersion" "${VERSION}"
  WriteRegStr HKLM "${UNINSTALL_KEY}" "DisplayIcon" "$INSTDIR\nodalis.ico"
  WriteRegStr HKLM "${UNINSTALL_KEY}" "Publisher" "${APPNAME}"
  WriteRegStr HKLM "${UNINSTALL_KEY}" "URLInfoAbout" "${SOURCE_URL}"
  WriteRegStr HKLM "${UNINSTALL_KEY}" "InstallLocation" "$INSTDIR"
  WriteRegStr HKLM "${UNINSTALL_KEY}" "UninstallString" '"$INSTDIR\uninstall.exe"'
  WriteRegStr HKLM "${UNINSTALL_KEY}" "QuietUninstallString" '"$INSTDIR\uninstall.exe" /S'
  WriteRegDWORD HKLM "${UNINSTALL_KEY}" "NoModify" 1
  WriteRegDWORD HKLM "${UNINSTALL_KEY}" "NoRepair" 1
  SectionGetSize ${SecApp} $0
  WriteRegDWORD HKLM "${UNINSTALL_KEY}" "EstimatedSize" $0

  ; "Open with" offers nodalis for the files it reads.
  WriteRegStr HKLM "Software\Classes\Applications\nodalis.exe" "FriendlyAppName" "${APPNAME}"
  WriteRegStr HKLM "Software\Classes\Applications\nodalis.exe\shell\open\command" "" '"$INSTDIR\nodalis.exe" "%1"'
  !macro SUPPORTED EXT
    WriteRegStr HKLM "Software\Classes\Applications\nodalis.exe\SupportedTypes" "${EXT}" ""
  !macroend
  !insertmacro SUPPORTED ".ndl"
  !insertmacro SUPPORTED ".inp"
  !insertmacro SUPPORTED ".frd"
  !insertmacro SUPPORTED ".step"
  !insertmacro SUPPORTED ".stp"
  !insertmacro SUPPORTED ".iges"
  !insertmacro SUPPORTED ".igs"
  !insertmacro SUPPORTED ".brep"
SectionEnd

Section "Start Menu Shortcut" SecStartMenu
  SetShellVarContext all
  CreateShortCut "$SMPROGRAMS\${APPNAME}.lnk" "$INSTDIR\nodalis.exe" "" "$INSTDIR\nodalis.ico"
SectionEnd

Section /o "Desktop Shortcut" SecDesktop
  SetShellVarContext all
  CreateShortCut "$DESKTOP\${APPNAME}.lnk" "$INSTDIR\nodalis.exe" "" "$INSTDIR\nodalis.ico"
SectionEnd

Section "Open .ndl projects with ${APPNAME}" SecAssociation
  WriteRegStr HKLM "Software\Classes\.ndl" "" "${PROGID}"
  WriteRegStr HKLM "Software\Classes\.ndl\OpenWithProgids" "${PROGID}" ""
  WriteRegStr HKLM "Software\Classes\${PROGID}" "" "${APPNAME} project"
  WriteRegStr HKLM "Software\Classes\${PROGID}\DefaultIcon" "" "$INSTDIR\nodalis.ico"
  WriteRegStr HKLM "Software\Classes\${PROGID}\shell\open\command" "" '"$INSTDIR\nodalis.exe" "%1"'
  ; SHCNE_ASSOCCHANGED: Explorer picks up the new icon and program right away.
  System::Call 'shell32::SHChangeNotify(i 0x08000000, i 0, p 0, p 0)'
SectionEnd

; An update keeps the shortcuts and the association as they were installed.
Function KeepChoices
  SetShellVarContext all
  ${IfNot} ${FileExists} "$SMPROGRAMS\${APPNAME}.lnk"
    !insertmacro UnselectSection ${SecStartMenu}
  ${EndIf}
  ${If} ${FileExists} "$DESKTOP\${APPNAME}.lnk"
    !insertmacro SelectSection ${SecDesktop}
  ${EndIf}
  ReadRegStr $0 HKLM "Software\Classes\.ndl" ""
  ${If} $0 != "${PROGID}"
    !insertmacro UnselectSection ${SecAssociation}
  ${EndIf}
FunctionEnd

!insertmacro MUI_FUNCTION_DESCRIPTION_BEGIN
  !insertmacro MUI_DESCRIPTION_TEXT ${SecApp} "The program with the Gmsh library for geometry import and meshing and the CalculiX solver."
  !insertmacro MUI_DESCRIPTION_TEXT ${SecStartMenu} "Start menu entry for all users."
  !insertmacro MUI_DESCRIPTION_TEXT ${SecDesktop} "Desktop icon for all users."
  !insertmacro MUI_DESCRIPTION_TEXT ${SecAssociation} "Double-clicking a .ndl file opens it in ${APPNAME}."
!insertmacro MUI_FUNCTION_DESCRIPTION_END

Function un.onInit
  SetRegView 64
FunctionEnd

Section "Uninstall"
  SetShellVarContext all
  Delete "$SMPROGRAMS\${APPNAME}.lnk"
  Delete "$DESKTOP\${APPNAME}.lnk"

  Delete "$INSTDIR\nodalis.exe"
  Delete "$INSTDIR\gmsh-4.15.dll"
  Delete "$INSTDIR\nodalis.ico"
  Delete "$INSTDIR\LICENSE.txt"
  Delete "$INSTDIR\THIRD-PARTY.txt"
  Delete "$INSTDIR\SOURCE.txt"
  RMDir /r "$INSTDIR\licenses"
  RMDir /r "$INSTDIR\doc"
  RMDir /r "$INSTDIR\ccx"
  ; The work directory nodalis creates next to itself when it may write there.
  RMDir /r "$INSTDIR\Temp"
  Delete "$INSTDIR\uninstall.exe"
  RMDir "$INSTDIR"

  ; Only remove the .ndl association if it still points to nodalis.
  ReadRegStr $0 HKLM "Software\Classes\.ndl" ""
  ${If} $0 == "${PROGID}"
    DeleteRegValue HKLM "Software\Classes\.ndl" ""
  ${EndIf}
  DeleteRegValue HKLM "Software\Classes\.ndl\OpenWithProgids" "${PROGID}"
  DeleteRegKey /ifempty HKLM "Software\Classes\.ndl\OpenWithProgids"
  DeleteRegKey /ifempty HKLM "Software\Classes\.ndl"
  DeleteRegKey HKLM "Software\Classes\${PROGID}"
  DeleteRegKey HKLM "Software\Classes\Applications\nodalis.exe"
  System::Call 'shell32::SHChangeNotify(i 0x08000000, i 0, p 0, p 0)'

  DeleteRegKey HKLM "${UNINSTALL_KEY}"
  DeleteRegKey HKLM "Software\${APP}"
  ; Settings in %APPDATA%\nodalis stay, like those of most programs.
SectionEnd
