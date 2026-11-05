#import "../../template.typ": *

== Animation <animation>

#ui("Animate") in the #ui("Step and Increment") group of the ribbon starts the animation that
fits the shown result and opens the animation window; a second click stops it. A mode shape
swings by its scale factor, a step with a moving load runs through its positions, a step of
several increments plays them one after another, and a single increment grows from undeformed to
deformed. #ui("Type") in the animation window switches to another type. For the modes of a
frequency step, #ui("Sound") appears right of #ui("Animate") (@sound).

#screenshot("results-animate-menu.png", [#ui("Animate") and, for a frequency step,
  #ui("Sound") in the ribbon])

#screenshot("animation.png", [Animation window])

#fields(
  [Type], [#ui("Scale factor") grows the displayed increment from undeformed to deformed; a mode
    swings between −1 and +1. #ui("Step") plays the increments of the shown step.
    #ui("All steps") plays the increments of all steps one after another, as the timeline
    (@results-timeline) shows them: the steps that run over time together, the modes of a
    frequency step apart. #ui("Positions") plays the positions of the moving load of the step
    (@moving-load), the last increment of each; it is offered only for such a step.],
  [Frames], [Number of frames of a scale factor animation (default 60).],
  [Frames per second], [Playback speed (default 30).],
  [Playback], [#ui("Once"), #ui("Loop") or #ui("Swing") (forth and back, default).],
  [Color scale], [#ui("Current frame") adapts the legend to every frame, #ui("All frames") keeps
    one range for the whole animation.],
)

The buttons step through the frames or play them; the slider selects a frame. Closing the window
shows the increment that was shown at the start, or the one you selected meanwhile.

The window can stay open while you change the increment: the #ui("Step") and #ui("Increment") lists and the
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

For a frequency step, #ui("Sound") right of #ui("Animate") in the ribbon opens #ui("Sound of the Mode Shapes"). It plays the
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
