use std::ops::{Deref, DerefMut, Index, IndexMut};

pub fn join<const DOF: usize>(numbers: &[f64], high_precision: bool) -> String {
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
#[derive(Debug)]
pub enum DataArrayOrVec<T, const N: usize>
    where T: std::fmt::Debug {
    Stack([T; N]),
    Heap(Vec<T>),
}

impl <T: Default + Clone + std::fmt::Debug, const N: usize> DataArrayOrVec<T, N> {
    pub fn new(dofs: Option<usize>, initial: T) -> Self {
        let size = dofs.unwrap_or(1);
        if N > 0 {
            let arr: [T; N] = std::array::from_fn(|_| initial.clone());
            DataArrayOrVec::Stack(arr)
        } else {
            DataArrayOrVec::Heap(vec![initial; size])
        }
    }

    pub fn get(&self, index: usize) -> Option<&T> {
        match self {
            DataArrayOrVec::Heap(v) => v.get(index),
            DataArrayOrVec::Stack(a) => a.get(index),
        }
    }

    pub fn iter(&self) -> Box<dyn Iterator<Item = &T> + '_> {
        match self {
            DataArrayOrVec::Heap(v) => Box::new(v.iter()),
            DataArrayOrVec::Stack(a) => Box::new(a.iter()),
        }
    }

    pub fn iter_mut(&mut self) -> Box<dyn Iterator<Item = &mut T> + '_> {
        match self {
            DataArrayOrVec::Heap(v) => Box::new(v.iter_mut()),
            DataArrayOrVec::Stack(a) => Box::new(a.iter_mut()),
        }
    }
}

impl<T: PartialEq + std::fmt::Debug, const N: usize> PartialEq for DataArrayOrVec<T, N> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (DataArrayOrVec::Stack(a), DataArrayOrVec::Stack(b)) => a == b,
            (DataArrayOrVec::Heap(a), DataArrayOrVec::Heap(b)) => a == b,
            _ => false,
        }
    }
}

impl<T: Clone + Default + std::fmt::Debug, const N: usize> Default for DataArrayOrVec<T, N> {
    fn default() -> Self {
        DataArrayOrVec::Heap(Vec::new())
    }
}

impl<T: Clone + Default + std::fmt::Debug, const N: usize> Index<usize> for DataArrayOrVec<T, N> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        match self {
            DataArrayOrVec::Heap(v) => &v[index],
            DataArrayOrVec::Stack(a) => &a[index],
        }
    }
}

impl<T: Clone + Default + std::fmt::Debug, const N: usize> IndexMut<usize> for DataArrayOrVec<T, N> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match self {
            DataArrayOrVec::Heap(v) => &mut v[index],
            DataArrayOrVec::Stack(a) => &mut a[index],
        }
    }
}

impl<T: Clone + Default + std::fmt::Debug, const N: usize> Clone for DataArrayOrVec<T, N> {
    fn clone(&self) -> Self {
        match self {
            DataArrayOrVec::Heap(vec) => DataArrayOrVec::Heap(vec.clone()),
            DataArrayOrVec::Stack(arr) => DataArrayOrVec::Stack(arr.clone()),
        }
    }
}

impl<T: Clone + Default + std::fmt::Debug, const N: usize> Deref for DataArrayOrVec<T, N> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        match self {
            DataArrayOrVec::Heap(vec) => vec,
            DataArrayOrVec::Stack(arr) => arr,
        }
    }
}

impl<T: Clone + Default + std::fmt::Debug, const N: usize> DerefMut for DataArrayOrVec<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            DataArrayOrVec::Heap(vec) => vec,
            DataArrayOrVec::Stack(arr) => arr,
        }
    }
}

// TODO: Needs further investigation before can be used
#[macro_export]
macro_rules! data_array_or_vec {
    ($($x:expr),+ $(,)?) => {
        {
            let temp: [_; $crate::count_exprs!($($x),*)] = [$($x),*];
            if temp.len() == N {
                $crate::DataArrayOrVec::Stack(temp)
            } else {
                $crate::DataArrayOrVec::Heap(temp.into())
            }
        }
    };
    ($x:expr; $n:expr) => {
        {
            if $n == N {
                $crate::DataArrayOrVec::Stack([$x; $n])
            } else {
                $crate::DataArrayOrVec::Heap(vec![$x; $n])
            }
        }
    };
}

// Secondary macros for calculating array sizes and generating array expressions.
#[macro_export]
macro_rules! count_exprs {
    ($x:expr) => (1usize);
    ($x:expr, $($xs:expr),* $(,)?) => (1usize + $crate::count_exprs!($($xs),*));
}

#[macro_export]
macro_rules! make_stack {
    ($($x:expr),* $(,)?) => {
        DataArrayOrVec::Stack([$($x),*])
    };
}