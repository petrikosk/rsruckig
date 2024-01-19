use arrayvec::ArrayVec;

pub fn join(numbers: &[f64], high_precision: bool) -> String {
    if high_precision {
        numbers
            .iter()
            .map(|&num| format!("{:.16}", num))
            .collect::<Vec<String>>()
            .join(", ")
    } else {
        numbers
            .iter()
            .map(|&num| num.to_string())
            .collect::<Vec<String>>()
            .join(", ")
    }
}

#[inline]
pub fn integrate(t: f64, p0: f64, v0: f64, a0: f64, j: f64) -> (f64, f64, f64) {
    (
        p0 + t * (v0 + t * (a0 / 2.0 + t * j / 6.0)),
        v0 + t * (a0 + t * j / 2.0),
        a0 + t * j,
    )
}

// A utility enum to store either an array or a vector
pub enum DataArrayOrVec<T, const N: usize> {
    Stack(ArrayVec<T, N>),
    Heap(Vec<T>),
}

// Default to 1, if both const generic N and dofs is zero or None
impl<T: Copy, const N: usize> DataArrayOrVec<T, N> {
    pub(crate) fn new(dofs: Option<usize>, initial: T) -> Self {
        match dofs {
            Some(size) if N == 0 => DataArrayOrVec::Heap(vec![initial; size]),
            None if N == 0 => DataArrayOrVec::Heap(vec![initial; 1]),
            _ => {
                let mut array_vec = ArrayVec::new();
                for _ in 0..N {
                    array_vec.push(initial);
                }
                DataArrayOrVec::Stack(array_vec)
            }
        }
    }
}

impl<T: PartialEq, const N: usize> PartialEq for DataArrayOrVec<T, N> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (DataArrayOrVec::Stack(a), DataArrayOrVec::Stack(b)) => a == b,
            (DataArrayOrVec::Heap(a), DataArrayOrVec::Heap(b)) => a == b,
            _ => false,
        }
    }
}

impl<T: Copy + Default, const N: usize> Default for DataArrayOrVec<T, N> {
    fn default() -> Self {
        if N == 0 {
            // Default to a Heap with a default vector if N is zero
            DataArrayOrVec::Heap(Vec::new())
        } else {
            // Default to a Stack with a default ArrayVec if N is non-zero
            let mut array_vec = ArrayVec::<T, N>::new();
            for _ in 0..N {
                array_vec.push(T::default()); // Populate with default values of T
            }
            DataArrayOrVec::Stack(array_vec)
        }
    }
}
