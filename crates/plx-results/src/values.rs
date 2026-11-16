//! The values of a result component: in memory, or read from the results file when they are
//! wanted ([`LazyField`]), so that results need not fit into memory as a whole.
//!
//! A lazy field reads and converts all its components at once, the first time one of them
//! is wanted, and keeps them in a cache shared by all results, which drops the fields used
//! longest ago once it holds more than its budget ([`set_budget`]). What takes the values
//! ([`Values::get`]) holds them as long as it keeps them, whether the cache still does or
//! not.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

/// A conversion of every value, such as into the units shown.
pub type ValueMap = Arc<dyn Fn(f32) -> f32 + Send + Sync>;

/// Reads the values of all components of a field, in their order.
pub type Loader = Box<dyn Fn() -> Result<Vec<Vec<f32>>, String> + Send + Sync>;

/// The values of one component.
#[derive(Clone)]
pub struct Values(Repr);

#[derive(Clone)]
enum Repr {
    Owned(Arc<[f32]>),
    Lazy(Arc<LazyField>, usize),
}

impl Values {
    /// Component `index` of a lazy field.
    pub fn lazy(field: &Arc<LazyField>, index: usize) -> Self {
        Self(Repr::Lazy(Arc::clone(field), index))
    }

    /// The values; of a lazy field read if they are not in the cache, which blocks until
    /// they are.
    pub fn get(&self) -> Arc<[f32]> {
        match &self.0 {
            Repr::Owned(values) => Arc::clone(values),
            Repr::Lazy(field, index) => field.component(*index),
        }
    }

    /// Value `index`, if there is one; reads a lazy field.
    pub fn at(&self, index: usize) -> Option<f32> {
        self.get().get(index).copied()
    }

    /// The values as a vector of their own.
    pub fn to_vec(&self) -> Vec<f32> {
        self.get().to_vec()
    }

    /// The number of values, known without reading them.
    pub fn len(&self) -> usize {
        match &self.0 {
            Repr::Owned(values) => values.len(),
            Repr::Lazy(field, _) => field.len,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Whether the values are at hand without reading the file.
    pub fn is_ready(&self) -> bool {
        match &self.0 {
            Repr::Owned(_) => true,
            Repr::Lazy(field, _) => field.is_cached(),
        }
    }

    /// Whether the values are read from the file when wanted.
    pub fn is_lazy(&self) -> bool {
        matches!(self.0, Repr::Lazy(..))
    }

    /// Reads the values of a lazy field on a thread of its own, if they are not at hand,
    /// and calls the hook of [`on_loaded`] when they are.
    pub fn prefetch(&self) {
        if let Repr::Lazy(field, _) = &self.0 {
            field.prefetch();
        }
    }

    /// The values to change in place: those of a lazy field are read and kept in memory as
    /// values of their own from then on.
    pub fn make_mut(&mut self) -> &mut [f32] {
        let unique = match &mut self.0 {
            Repr::Owned(values) => Arc::get_mut(values).is_some(),
            Repr::Lazy(..) => false,
        };
        if !unique {
            let copy: Arc<[f32]> = self.to_vec().into();
            self.0 = Repr::Owned(copy);
        }
        match &mut self.0 {
            Repr::Owned(values) => Arc::get_mut(values).unwrap_or(&mut []),
            Repr::Lazy(..) => &mut [],
        }
    }

    /// Converts every value with `map`: values in memory now, those of a lazy field as they
    /// are read. All copies of a lazy component are converted.
    pub fn map(&mut self, map: impl Fn(f32) -> f32 + Send + Sync + 'static) {
        match &mut self.0 {
            Repr::Owned(values) => {
                *values = values.iter().map(|&v| map(v)).collect();
            }
            Repr::Lazy(field, index) => field.add_map(*index, Arc::new(map)),
        }
    }
}

impl From<Vec<f32>> for Values {
    fn from(values: Vec<f32>) -> Self {
        Self(Repr::Owned(values.into()))
    }
}

impl<const N: usize> From<[f32; N]> for Values {
    fn from(values: [f32; N]) -> Self {
        Vec::from(values).into()
    }
}

impl FromIterator<f32> for Values {
    fn from_iter<I: IntoIterator<Item = f32>>(iter: I) -> Self {
        iter.into_iter().collect::<Vec<f32>>().into()
    }
}

impl From<Arc<[f32]>> for Values {
    fn from(values: Arc<[f32]>) -> Self {
        Self(Repr::Owned(values))
    }
}

impl Default for Values {
    fn default() -> Self {
        Vec::new().into()
    }
}

impl PartialEq for Values {
    fn eq(&self, other: &Self) -> bool {
        self.len() == other.len() && *self.get() == *other.get()
    }
}

impl std::fmt::Debug for Values {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.0 {
            Repr::Owned(values) => f.debug_list().entries(values.iter()).finish(),
            Repr::Lazy(_, index) => write!(f, "Lazy({index} of {} values)", self.len()),
        }
    }
}

/// A field whose components are read from the results file when one of them is wanted.
pub struct LazyField {
    id: u64,
    /// Values of every component.
    len: usize,
    /// Number of components.
    count: usize,
    loader: Loader,
    maps: Mutex<Vec<Option<ValueMap>>>,
    /// Held while the field is read, so that it is read once when it is wanted twice.
    reading: Mutex<()>,
}

impl std::fmt::Debug for LazyField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LazyField")
            .field("len", &self.len)
            .field("count", &self.count)
            .finish_non_exhaustive()
    }
}

impl LazyField {
    /// A field of `count` components of `len` values each, read by `loader`.
    pub fn new(len: usize, count: usize, loader: Loader) -> Arc<Self> {
        static NEXT: AtomicU64 = AtomicU64::new(1);
        Arc::new(Self {
            id: NEXT.fetch_add(1, Ordering::Relaxed),
            len,
            count,
            loader,
            maps: Mutex::new(vec![None; count]),
            reading: Mutex::new(()),
        })
    }

    fn is_cached(&self) -> bool {
        cache().contains(self.id)
    }

    fn component(&self, index: usize) -> Arc<[f32]> {
        if let Some(values) = cache().get(self.id, index) {
            return values;
        }
        let _reading = self.reading.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(values) = cache().get(self.id, index) {
            return values;
        }
        let components = self.read();
        let values = components.get(index).cloned().unwrap_or_else(|| self.nan());
        cache().insert(self.id, components);
        values
    }

    /// The components read and converted; NaN if they cannot be read.
    fn read(&self) -> Vec<Arc<[f32]>> {
        let read = (self.loader)().and_then(|components| {
            if components.len() != self.count || components.iter().any(|c| c.len() != self.len) {
                Err("the results file does not match the results shown".to_owned())
            } else {
                Ok(components)
            }
        });
        let maps = self.maps.lock().unwrap_or_else(|e| e.into_inner()).clone();
        match read {
            Ok(components) => (components.into_iter().zip(maps))
                .map(|(mut values, map)| {
                    if let Some(map) = map {
                        values.iter_mut().for_each(|v| *v = map(*v));
                    }
                    values.into()
                })
                .collect(),
            Err(error) => {
                eprintln!("Results could not be read: {error}");
                (0..self.count).map(|_| self.nan()).collect()
            }
        }
    }

    fn nan(&self) -> Arc<[f32]> {
        vec![f32::NAN; self.len].into()
    }

    fn add_map(&self, index: usize, map: ValueMap) {
        let mut maps = self.maps.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(slot) = maps.get_mut(index) {
            *slot = Some(match slot.take() {
                Some(first) => Arc::new(move |v| map(first(v))),
                None => map,
            });
        }
        drop(maps);
        // Values read before are read again with the conversion.
        cache().remove(self.id);
    }

    fn prefetch(self: &Arc<Self>) {
        if self.is_cached() {
            return;
        }
        let queue = queue();
        let mut waiting = queue.waiting.lock().unwrap_or_else(|e| e.into_inner());
        if waiting.iter().any(|f| f.id == self.id) {
            return;
        }
        waiting.push_back(Arc::clone(self));
        queue.ready.notify_one();
    }
}

/// The fields [`Values::prefetch`] asked for, read in turn by [`READERS`] threads.
struct Queue {
    waiting: Mutex<std::collections::VecDeque<Arc<LazyField>>>,
    ready: std::sync::Condvar,
}

/// The threads that read the fields asked for ahead; each reads the chunks of a field in
/// parallel.
const READERS: usize = 2;

fn queue() -> &'static Queue {
    static QUEUE: OnceLock<Queue> = OnceLock::new();
    QUEUE.get_or_init(|| {
        for _ in 0..READERS {
            let _ = std::thread::Builder::new()
                .name("results reader".into())
                .spawn(read_ahead);
        }
        Queue {
            waiting: Mutex::new(Default::default()),
            ready: std::sync::Condvar::new(),
        }
    })
}

fn read_ahead() {
    let queue = queue();
    loop {
        let field = {
            let mut waiting = queue.waiting.lock().unwrap_or_else(|e| e.into_inner());
            loop {
                match waiting.pop_front() {
                    Some(field) => break field,
                    None => {
                        waiting = (queue.ready.wait(waiting)).unwrap_or_else(|e| e.into_inner());
                    }
                }
            }
        };
        // Wanted no more since: the results were closed.
        if Arc::strong_count(&field) == 1 {
            continue;
        }
        field.component(0);
        drop(field);
        loaded();
    }
}

/// Calls the hook of [`on_loaded`]: values read ahead are at hand.
pub fn loaded() {
    if let Some(hook) = HOOK.get() {
        hook();
    }
}

impl Drop for LazyField {
    fn drop(&mut self) {
        cache().remove(self.id);
    }
}

static HOOK: OnceLock<Box<dyn Fn() + Send + Sync>> = OnceLock::new();

/// Sets what is called when a field read by [`Values::prefetch`] is at hand, such as a
/// repaint; only the first call counts.
pub fn on_loaded(hook: impl Fn() + Send + Sync + 'static) {
    let _ = HOOK.set(Box::new(hook));
}

/// The default budget of the cache.
pub const DEFAULT_BUDGET: usize = 2 << 30;

/// Sets how many bytes of read fields the cache keeps at most; it keeps the field read last
/// even if it is larger.
pub fn set_budget(bytes: usize) {
    let mut cache = cache();
    cache.budget = bytes;
    cache.trim(0);
}

/// How many bytes of read fields the cache keeps at most ([`set_budget`]).
pub fn budget() -> usize {
    cache().budget
}

/// The bytes of read fields the cache holds.
pub fn cached_bytes() -> usize {
    cache().used
}

struct Entry {
    components: Vec<Arc<[f32]>>,
    bytes: usize,
    used: u64,
}

struct Cache {
    entries: HashMap<u64, Entry>,
    used: usize,
    budget: usize,
    clock: u64,
}

impl Cache {
    fn contains(&self, id: u64) -> bool {
        self.entries.contains_key(&id)
    }

    fn get(&mut self, id: u64, index: usize) -> Option<Arc<[f32]>> {
        self.clock += 1;
        let entry = self.entries.get_mut(&id)?;
        entry.used = self.clock;
        entry.components.get(index).cloned()
    }

    fn insert(&mut self, id: u64, components: Vec<Arc<[f32]>>) {
        self.remove(id);
        self.clock += 1;
        let bytes = components.iter().map(|c| c.len() * 4).sum();
        self.used += bytes;
        self.entries.insert(
            id,
            Entry {
                components,
                bytes,
                used: self.clock,
            },
        );
        self.trim(id);
    }

    fn remove(&mut self, id: u64) {
        if let Some(entry) = self.entries.remove(&id) {
            self.used -= entry.bytes;
        }
    }

    /// Drops the fields used longest ago, but `keep`, until the cache fits its budget.
    fn trim(&mut self, keep: u64) {
        while self.used > self.budget {
            let oldest = (self.entries.iter())
                .filter(|(id, _)| **id != keep)
                .min_by_key(|(_, e)| e.used)
                .map(|(id, _)| *id);
            match oldest {
                Some(id) => self.remove(id),
                None => break,
            }
        }
    }
}

fn cache() -> std::sync::MutexGuard<'static, Cache> {
    static CACHE: OnceLock<Mutex<Cache>> = OnceLock::new();
    (CACHE.get_or_init(|| {
        Mutex::new(Cache {
            entries: HashMap::new(),
            used: 0,
            budget: DEFAULT_BUDGET,
            clock: 0,
        })
    }))
    .lock()
    .unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;

    fn counted(reads: &Arc<AtomicUsize>, len: usize) -> Arc<LazyField> {
        let reads = Arc::clone(reads);
        LazyField::new(
            len,
            2,
            Box::new(move || {
                reads.fetch_add(1, Ordering::SeqCst);
                Ok(vec![vec![1.0; len], vec![2.0; len]])
            }),
        )
    }

    #[test]
    fn a_lazy_field_is_read_once_for_all_its_components() {
        let reads = Arc::new(AtomicUsize::new(0));
        let field = counted(&reads, 3);
        let (a, b) = (Values::lazy(&field, 0), Values::lazy(&field, 1));
        assert_eq!(a.len(), 3);
        assert_eq!(
            reads.load(Ordering::SeqCst),
            0,
            "not read before it is wanted"
        );
        assert!(!a.is_ready());
        assert_eq!(*a.get(), [1.0; 3]);
        assert_eq!(*b.get(), [2.0; 3]);
        assert!(b.is_ready());
        assert_eq!(reads.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn conversions_apply_to_lazy_values_as_they_are_read() {
        let reads = Arc::new(AtomicUsize::new(0));
        let field = counted(&reads, 2);
        let mut a = Values::lazy(&field, 0);
        assert_eq!(*a.get(), [1.0; 2]);
        a.map(|v| v * 10.0);
        a.map(|v| v + 1.0);
        assert_eq!(*a.get(), [11.0; 2]);
        assert_eq!(*Values::lazy(&field, 1).get(), [2.0; 2]);
        let mut owned = Values::from(vec![1.0, 2.0]);
        owned.map(|v| -v);
        assert_eq!(*owned.get(), [-1.0, -2.0]);
    }

    #[test]
    fn a_field_that_cannot_be_read_is_nan() {
        let field = LazyField::new(2, 1, Box::new(|| Err("gone".into())));
        assert!(Values::lazy(&field, 0).get().iter().all(|v| v.is_nan()));
        let wrong = LazyField::new(2, 1, Box::new(|| Ok(vec![vec![1.0]])));
        assert!(Values::lazy(&wrong, 0).get().iter().all(|v| v.is_nan()));
    }

    #[test]
    fn the_cache_drops_the_fields_used_longest_ago() {
        let mut cache = Cache {
            entries: HashMap::new(),
            used: 0,
            budget: 100,
            clock: 0,
        };
        let values = |n: usize| vec![Arc::<[f32]>::from(vec![0.0; n])];
        cache.insert(1, values(10));
        cache.insert(2, values(10));
        cache.get(1, 0);
        cache.insert(3, values(10));
        assert!(cache.contains(1) && !cache.contains(2) && cache.contains(3));
        // The field read last stays, larger than the budget or not.
        cache.insert(4, values(100));
        assert!(cache.contains(4) && cache.entries.len() == 1);
        assert_eq!(cache.used, 400);
    }
}
