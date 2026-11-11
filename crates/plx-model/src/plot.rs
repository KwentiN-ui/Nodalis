//! The plots of the Plot block of the project schematic: what kind of plot it draws and the
//! bins of a histogram.

use serde::{Deserialize, Serialize};

/// What a Plot block draws, which decides the inputs it has.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PlotKind {
    /// Y over X joined by lines; without X over the index of the values.
    #[default]
    Line,
    /// Y over X as points.
    Scatter,
    /// How many of the values fall into each of a number of equal bins.
    Histogram,
}

impl PlotKind {
    pub const ALL: [Self; 3] = [Self::Line, Self::Scatter, Self::Histogram];

    pub fn label(self) -> &'static str {
        match self {
            Self::Line => "Line",
            Self::Scatter => "Scatter",
            Self::Histogram => "Histogram",
        }
    }

    /// What it draws, for its tooltip.
    pub fn describe(self) -> &'static str {
        match self {
            Self::Line => "Y over X joined by lines; without X over the number of each value",
            Self::Scatter => "Y over X as points; without X over the number of each value",
            Self::Histogram => "how many of the values fall into each of the bins",
        }
    }

    /// The keys and labels of its inputs, in order.
    pub fn inputs(self) -> &'static [(&'static str, &'static str)] {
        match self {
            Self::Line | Self::Scatter => &[("x", "X"), ("y", "Y")],
            Self::Histogram => &[("values", "Values"), ("bins", "Bins")],
        }
    }
}

/// The bins of a histogram of `values`: `bins` equal bins from the smallest to the largest
/// value, each its lower and upper edge and how many values it holds. NaN is left out; equal
/// values fall into one bin of width zero.
pub fn histogram(values: &[f64], bins: usize) -> Vec<(f64, f64, usize)> {
    let finite: Vec<f64> = values.iter().copied().filter(|v| v.is_finite()).collect();
    let (Some(low), Some(high)) = (
        finite.iter().copied().reduce(f64::min),
        finite.iter().copied().reduce(f64::max),
    ) else {
        return Vec::new();
    };
    let bins = if high > low { bins.max(1) } else { 1 };
    let width = (high - low) / bins as f64;
    let mut counts = vec![0; bins];
    for value in finite {
        let at = if width > 0.0 {
            (((value - low) / width) as usize).min(bins - 1)
        } else {
            0
        };
        counts[at] += 1;
    }
    (counts.into_iter().enumerate())
        .map(|(i, count)| {
            let lower = low + width * i as f64;
            let upper = if i + 1 == bins { high } else { lower + width };
            (lower, upper, count)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_histogram_counts_the_values_of_each_bin() {
        let bins = histogram(&[0.0, 1.0, 1.5, 2.0, 4.0, f64::NAN], 4);
        assert_eq!(bins.len(), 4);
        assert_eq!(
            bins.iter().map(|b| b.2).collect::<Vec<_>>(),
            vec![1, 2, 1, 1]
        );
        assert_eq!((bins[0].0, bins[3].1), (0.0, 4.0));
        assert_eq!(histogram(&[3.0, 3.0], 5), vec![(3.0, 3.0, 2)]);
        assert!(histogram(&[], 5).is_empty());
    }

    #[test]
    fn the_kind_of_plot_decides_its_inputs() {
        assert_eq!(PlotKind::Line.inputs()[1].0, "y");
        assert_eq!(PlotKind::Histogram.inputs()[0].1, "Values");
    }
}
