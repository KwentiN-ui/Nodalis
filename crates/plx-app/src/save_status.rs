//! The line the status bars show while the project is saved on its worker thread: the project
//! window sets it, the status bars of the analysis windows read it.

fn id() -> egui::Id {
    egui::Id::new("nodalis save status")
}

/// Sets the line, `None` once nothing is being saved.
pub fn set(ctx: &egui::Context, status: Option<String>) {
    ctx.data_mut(|d| match status {
        Some(status) => {
            d.insert_temp(id(), status);
        }
        None => d.remove::<String>(id()),
    });
}

/// The line while saving.
pub fn get(ctx: &egui::Context) -> Option<String> {
    ctx.data(|d| d.get_temp::<String>(id()))
}
