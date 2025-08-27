//! Utility functions and data structures for the Ruckig algorithm
//!
//! This module provides helper functions, common data structures,
//! and macros used throughout the Ruckig implementation.

use core::ops::{Deref, DerefMut, Index, IndexMut};

use crate::alloc::{vec, vec::Vec, boxed::Box, format, string::{String, ToString}};

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
/// A container that can store data either on the stack or the heap
///
/// This enum provides a unified interface for working with data stored either:
/// - On the stack with a fixed size (when N > 0)
/// - On the heap with a dynamic size (when N = 0)
///
/// This is a key component of rsruckig's flexible memory management, allowing
/// users to choose between compile-time (stack) or runtime (heap) allocation
/// for their degrees of freedom data.
///
/// # Type Parameters
///
/// * `T` - The type of the elements
/// * `N` - The size for stack allocation. If 0, heap allocation is used.
///
/// # Examples
///
/// Stack allocation (fixed size at compile time):
/// ```no_run
/// use rsruckig::prelude::*;
///
/// // Using the daov_stack! macro for stack allocation
/// let data: DataArrayOrVec<f64, 3> = daov_stack![0.0, 1.0, 2.0];
/// assert_eq!(data[1], 1.0);
/// ```
///
/// Heap allocation (dynamic size at runtime):
/// ```no_run
/// use rsruckig::prelude::*;
///
/// // Using the daov_heap! macro for heap allocation
/// let data: DataArrayOrVec<f64, 3> = daov_heap![0.0, 1.0, 2.0];
/// assert_eq!(data[1], 1.0);
/// ```
#[derive(Debug)]
pub enum DataArrayOrVec<T, const N: usize>
where
    T: core::fmt::Debug,
{
    /// Stack allocation with fixed size array
    Stack([T; N]),
    /// Heap allocation with dynamic vector
    Heap(Vec<T>),
}

impl<T: Default + Clone + core::fmt::Debug, const N: usize> DataArrayOrVec<T, N> {
    /// Create a new DataArrayOrVec with the specified number of elements
    ///
    /// This constructor chooses stack or heap allocation based on the template parameter N:
    /// - If N > 0, creates a stack-allocated array of size N, ignoring the dofs parameter
    /// - If N = 0, creates a heap-allocated vector with size specified by dofs
    ///
    /// # Arguments
    ///
    /// * `dofs` - The number of degrees of freedom (only used for heap allocation)
    /// * `initial` - The initial value to fill the container with
    ///
    /// # Returns
    ///
    /// A new DataArrayOrVec instance with all elements set to the initial value
    pub fn new(dofs: Option<usize>, initial: T) -> Self {
        let size = dofs.unwrap_or(1);
        if N > 0 {
            let arr: [T; N] = core::array::from_fn(|_| initial.clone());
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

impl<T: PartialEq + core::fmt::Debug, const N: usize> PartialEq for DataArrayOrVec<T, N> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (DataArrayOrVec::Stack(a), DataArrayOrVec::Stack(b)) => a == b,
            (DataArrayOrVec::Heap(a), DataArrayOrVec::Heap(b)) => a == b,
            _ => false,
        }
    }
}

impl<T: Clone + Default + core::fmt::Debug, const N: usize> Default for DataArrayOrVec<T, N> {
    fn default() -> Self {
        DataArrayOrVec::Heap(Vec::new())
    }
}

impl<T: Clone + Default + core::fmt::Debug, const N: usize> Index<usize> for DataArrayOrVec<T, N> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        match self {
            DataArrayOrVec::Heap(v) => &v[index],
            DataArrayOrVec::Stack(a) => &a[index],
        }
    }
}

impl<T: Clone + Default + core::fmt::Debug, const N: usize> IndexMut<usize>
    for DataArrayOrVec<T, N>
{
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match self {
            DataArrayOrVec::Heap(v) => &mut v[index],
            DataArrayOrVec::Stack(a) => &mut a[index],
        }
    }
}

impl<T: Clone + Default + core::fmt::Debug, const N: usize> Clone for DataArrayOrVec<T, N> {
    fn clone(&self) -> Self {
        match self {
            DataArrayOrVec::Heap(vec) => DataArrayOrVec::Heap(vec.clone()),
            DataArrayOrVec::Stack(arr) => DataArrayOrVec::Stack(arr.clone()),
        }
    }
}

impl<T: Clone + Default + core::fmt::Debug, const N: usize> Deref for DataArrayOrVec<T, N> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        match self {
            DataArrayOrVec::Heap(vec) => vec,
            DataArrayOrVec::Stack(arr) => arr,
        }
    }
}

impl<T: Clone + Default + core::fmt::Debug, const N: usize> DerefMut for DataArrayOrVec<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            DataArrayOrVec::Heap(vec) => vec,
            DataArrayOrVec::Stack(arr) => arr,
        }
    }
}

/// Helper macro for counting elements in a sequence.
/// This is used internally by the daov_stack and daov_heap macros.
#[macro_export]
macro_rules! count_exprs {
    () => (0);
    ($x:expr) => (1);
    ($x:expr, $($rest:expr),*) => (1 + count_exprs!($($rest),*));
}

/// Creates a stack-allocated `DataArrayOrVec` instance with fixed-size array storage.
///
/// This macro simplifies the creation of stack-allocated vectors for kinematic data.
/// It uses the `DataArrayOrVec::Stack` variant internally, which stores data in a fixed-size array.
///
/// # Examples
///
/// ```
/// use rsruckig::prelude::*;
///
/// // Create a stack-allocated array with 3 elements
/// let positions: DataArrayOrVec<f64, 3> = daov_stack![0.0, 1.0, 2.0];
///
/// // Access elements via indexing
/// assert_eq!(positions[0], 0.0);
/// assert_eq!(positions[1], 1.0);
/// assert_eq!(positions[2], 2.0);
/// ```
///
/// You can also create an array with repeated values:
///
/// ```
/// use rsruckig::prelude::*;
///
/// // Create a stack-allocated array with 5 zeros
/// let zeros: DataArrayOrVec<f64, 5> = daov_stack![0.0; 5];
///
/// assert_eq!(zeros[0], 0.0);
/// assert_eq!(zeros[4], 0.0);
/// ```
#[macro_export]
macro_rules! daov_stack {
    ($($x:expr),+ $(,)?) => {
        {
            let temp = [$($x),*];
            rsruckig::prelude::DataArrayOrVec::Stack(temp)
        }
    };
    ($x:expr; $n:expr) => {
        {
            rsruckig::prelude::DataArrayOrVec::Stack([$x; $n])
        }
    };
}

/// Creates a heap-allocated `DataArrayOrVec` instance with dynamic Vec storage.
///
/// This macro simplifies the creation of heap-allocated vectors for kinematic data.
/// It uses the `DataArrayOrVec::Heap` variant internally, which stores data in a Vec.
/// This is particularly useful when the number of degrees of freedom is only known at runtime.
///
/// # Examples
///
/// ```
/// use rsruckig::prelude::*;
///
/// // Create a heap-allocated vector with 3 elements
/// let velocities: DataArrayOrVec<f64, 3> = daov_heap![0.0, 1.0, 2.0];
///
/// // Access elements via indexing
/// assert_eq!(velocities[0], 0.0);
/// assert_eq!(velocities[1], 1.0);
/// assert_eq!(velocities[2], 2.0);
/// ```
///
/// You can also create a vector with repeated values:
///
/// ```
/// use rsruckig::prelude::*;
///
/// // Create a heap-allocated vector with 5 zeros
/// let zeros: DataArrayOrVec<f64, 5> = daov_heap![0.0; 5];
///
/// assert_eq!(zeros[0], 0.0);
/// assert_eq!(zeros[4], 0.0);
/// ```
#[macro_export]
macro_rules! daov_heap {
    ($($x:expr),+ $(,)?) => {
        {
            let temp = [$($x),*];
            rsruckig::prelude::DataArrayOrVec::Heap(temp.into())
        }
    };
    ($x:expr; $n:expr) => {
        {
            rsruckig::prelude::DataArrayOrVec::Heap(vec![$x; $n])
        }
    };
}
