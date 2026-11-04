#import "../../template.typ": *

== Animation <animation>

The animation button among the result controls of the ribbon opens the animation window.

#screenshot("animation.png", [Animation window])

#fields(
  [Type], [#ui("Scale factor") grows the displayed increment from undeformed to deformed; a mode
    swings between −1 and +1. #ui("Step increments") plays the increments of the step,
    #ui("Steps") the last increment of every step of the same kind, e.g. the positions of a
    moving load (@moving-load).],
  [Frames], [Number of frames of a scale factor animation (default 60).],
  [Frames per second], [Playback speed (default 30).],
  [Playback], [#ui("Once"), #ui("Loop") or #ui("Swing") (forth and back, default).],
  [Color scale], [#ui("Current frame") adapts the legend to every frame, #ui("All frames") keeps
    one range for the whole animation.],
)

The buttons step through the frames or play them; the slider selects a frame. Closing the window
shows the increment that was shown at the start, or the one you selected meanwhile.

The window can stay open while you change the increment: the #ui("Step, Increment") list and the
#ui("First"), #ui("Previous"), #ui("Next") and #ui("Last") tiles keep working. The
animation continues with the new increment (or the increments of its step) and keeps its
settings, so you can page through modes or steps without reopening the window.

=== Exporting the animation <animation-export>

The export button (disk symbol) at the right of the player row saves the animation as a file,
with the choices #ui("GIF …"), #ui("MP4 …") and #ui("MKV …").

#screenshot("animation-export.png", [Export menu of the animation window])

Nodalis then plays the animation once frame by frame and records the 3D view as it is shown,
with legend, labels and status block, at the size of the view on screen. The animation window is
hidden meanwhile; the status bar counts the frames and #key("Esc") cancels. The file follows the
settings of the window:

- #ui("Frames") and #ui("Frames per second") give the number of frames and the playing speed.
- #ui("Once") and #ui("Loop") record every frame once, #ui("Swing") records there and back, so a
  player that repeats the file swings smoothly. A GIF repeats endlessly unless the playback is
  #ui("Once"); videos are played as the video player is set.
- #ui("Color scale") is recorded as set: with #ui("Current frame") the legend changes from frame
  to frame.

GIF needs nothing else; its frame times are counted in hundredths of a second, so above 50 frames
per second it plays slower. MP4 and MKV are written by #link("https://ffmpeg.org")[FFmpeg], which
has to be installed: Nodalis looks for `ffmpeg` on the `PATH`, or uses the executable set under
#menu("Edit", "Settings ...") > #ui("Post-processing") > #ui("FFmpeg executable"). The video is
H.264 (or FFmpeg's default codec if it lacks H.264); its width and height are rounded down to even
numbers. The output pane reports the saved file or what went wrong.

== Sound of the mode shapes <sound>

For a frequency step, the sound button opens #ui("Sound of the Mode Shapes"). It plays the
eigenfrequencies as tones, so you can hear how the structure would ring.

#screenshot("sound.png", [Sound of the mode shapes])

- Tick the modes to hear them together; #ui("Level") sets the volume of each mode. A click on a
  mode number shows this mode swinging in the 3D view.
- #ui("Make audible"): #ui("Automatic") shifts all frequencies by whole octaves into the audible
  range, keeping their intervals. #ui("Octaves") applies a fixed shift, #ui("Original") none.
- #ui("Sound"): #ui("Sustained") tones, or #ui("Struck") tones that decay like a bell, higher modes
  faster (#ui("Decay time")).
- #ui("Show superposed while playing") overlays the ticked modes in the 3D view, each swinging at
  its own frequency ratio, with #ui("Speed") for the lowest mode.
- #ui("Save as WAV...") writes the sound to a file.
