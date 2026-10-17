//! Fast Rubik's cube solver: Thistlethwaite's four nested subgroups, with
//! every phase compiled into lookup tables of optimal algorithms.
#![allow(clippy::needless_range_loop)]
mod algs;
mod beam;
mod chain;
pub mod coord;
mod merge;
pub mod solver;
mod tables;
