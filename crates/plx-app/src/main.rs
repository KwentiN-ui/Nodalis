#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod amplitude_dialog;
mod analysis;
mod animation;
mod animation_export;
mod app;
mod arrows;
mod bolts;
mod cad_editor;
mod cad_selection;
mod campbell;
mod color_scale_dialog;
mod connections;
mod constraint_dialog;
mod contact_search;
mod contacts;
mod cross_section;
mod distribution_dialog;
mod exploded;
mod expose;
mod features;
mod field_output_dialog;
mod geometry_selection;
mod gizmo;
mod help;
mod history_output_dialog;
mod history_table;
mod hot_spot_dialog;
mod hot_spots;
mod icons;
mod ids;
mod lic;
mod links;
mod material_library;
mod material_set;
mod measure;
mod mesh_block;
mod mesh_tools;
mod meshing;
mod model;
mod model_properties;
mod model_space_sketch;
mod motion_preview;
mod node_icons;
mod node_picker;
mod numeric;
mod overlay;
mod parameter_set;
mod path_plot;
mod portable;
mod project;
mod properties;
mod query;
mod results;
mod screenshot;
mod section;
mod selection;
mod settings;
mod setup;
mod sketch;
mod solution_monitor;
mod solver_check;
mod sound;
mod status;
mod study;
mod style;
mod symbols;
mod table_export;
mod transformation_dialog;
mod tree;
mod tree_icons;
mod tree_rows;
mod undo;
mod update;
mod viewport;
mod virtual_table;
mod vtk_export_dialog;
mod xy_plot;

fn main() -> eframe::Result {
    if let Some(code) = meshing::worker_main() {
        std::process::exit(code);
    }
    env_logger::init();
    if let Ok(program) = std::env::current_exe() {
        meshing::set_worker(program);
    }
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Nodalis")
            .with_app_id("nodalis")
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([640.0, 400.0])
            .with_icon(window_icon()),
        renderer: eframe::Renderer::Wgpu,
        persist_window: true,
        persistence_path: portable::persistence_path(),
        ..Default::default()
    };
    eframe::run_native(
        "nodalis",
        options,
        Box::new(|cc| Ok(Box::new(project::ProjectApp::new(cc)?))),
    )
}

/// The program icon for the window and the task bar, exported from
/// `assets/icon/nodalis-icon.typ` by `scripts/make_icon.py`. Decoded once: the analysis
/// windows ask for it in every frame.
fn window_icon() -> std::sync::Arc<egui::IconData> {
    static ICON: std::sync::LazyLock<std::sync::Arc<egui::IconData>> =
        std::sync::LazyLock::new(|| std::sync::Arc::new(decode_window_icon()));
    ICON.clone()
}

fn decode_window_icon() -> egui::IconData {
    let decoder = png::Decoder::new(std::io::Cursor::new(include_bytes!(
        "../assets/icon/nodalis.png"
    )));
    let mut reader = decoder.read_info().expect("Icon is a valid PNG");
    let mut rgba = vec![0; reader.output_buffer_size().expect("Icon fits into memory")];
    let info = reader.next_frame(&mut rgba).expect("Icon is a valid PNG");
    assert_eq!(
        info.color_type,
        png::ColorType::Rgba,
        "Icon needs an alpha channel"
    );
    rgba.truncate(info.buffer_size());
    egui::IconData {
        rgba,
        width: info.width,
        height: info.height,
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn window_icon_decodes() {
        let icon = super::window_icon();
        assert_eq!((icon.width, icon.height), (256, 256));
        assert_eq!(icon.rgba.len(), 256 * 256 * 4);
    }

    /// The bundled Noto Sans subset and egui's fallback fonts lack emoji and many symbols,
    /// which then show up as empty boxes. The GUI sources stick to Latin-1 plus a few checked
    /// typographic signs.
    #[test]
    fn gui_sources_only_use_glyphs_of_the_default_font() {
        let sources = [
            ("analysis.rs", include_str!("analysis.rs")),
            ("animation.rs", include_str!("animation.rs")),
            ("app.rs", include_str!("app.rs")),
            ("contact_search.rs", include_str!("contact_search.rs")),
            ("contacts.rs", include_str!("contacts.rs")),
            (
                "field_output_dialog.rs",
                include_str!("field_output_dialog.rs"),
            ),
            ("gizmo.rs", include_str!("gizmo.rs")),
            (
                "history_output_dialog.rs",
                include_str!("history_output_dialog.rs"),
            ),
            ("history_table.rs", include_str!("history_table.rs")),
            ("icons.rs", include_str!("icons.rs")),
            ("links.rs", include_str!("links.rs")),
            ("material_library.rs", include_str!("material_library.rs")),
            ("main.rs", include_str!("main.rs")),
            ("model.rs", include_str!("model.rs")),
            ("model_properties.rs", include_str!("model_properties.rs")),
            (
                "model_space_sketch.rs",
                include_str!("model_space_sketch.rs"),
            ),
            ("overlay.rs", include_str!("overlay.rs")),
            ("project.rs", include_str!("project.rs")),
            ("project/frames.rs", include_str!("project/frames.rs")),
            ("project/minimap.rs", include_str!("project/minimap.rs")),
            ("project/numbers.rs", include_str!("project/numbers.rs")),
            ("properties.rs", include_str!("properties.rs")),
            ("results.rs", include_str!("results.rs")),
            ("selection.rs", include_str!("selection.rs")),
            ("settings.rs", include_str!("settings.rs")),
            ("setup.rs", include_str!("setup.rs")),
            ("solver_check.rs", include_str!("solver_check.rs")),
            ("status.rs", include_str!("status.rs")),
            ("style.rs", include_str!("style.rs")),
            ("update.rs", include_str!("update.rs")),
            ("tree.rs", include_str!("tree.rs")),
            ("tree_icons.rs", include_str!("tree_icons.rs")),
            ("tree_rows.rs", include_str!("tree_rows.rs")),
            ("viewport.rs", include_str!("viewport.rs")),
        ];
        for (file, text) in sources {
            for (line, content) in text.lines().enumerate() {
                for c in content.chars() {
                    let ok = (c as u32) < 0x100 || "…–".contains(c);
                    assert!(
                        ok,
                        "{file}:{}: character {c:?} is missing from the egui font",
                        line + 1
                    );
                }
            }
        }
    }
}
