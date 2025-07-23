//! DHT22 Sensor Driver for Embedded Rust
//!
//! This crate provides a platform-agnostic driver for the DHT22 (AM2302) temperature
//! and humidity sensor, built on top of the [`embedded-hal`] traits.
//!
//! # Features
//! - Blocking synchronous API using `embedded-hal` traits
//! - Designed for `no_std` environments
//! - Optional logging support via `defmt`
//!
//! # Dependencies
//! This driver depends on the following `embedded-hal` traits:
//! - [`InputPin`] and [`OutputPin`] for GPIO access
//! - [`DelayNs`] for accurate timing
//!
//! # Optional Features
//! - `defmt`: Implements `defmt::Format` for logging support
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//! [`InputPin`]: embedded_hal::digital::InputPin
//! [`OutputPin`]: embedded_hal::digital::OutputPin
//! [`DelayNs`]: embedded_hal::delay::DelayNs

#![cfg_attr(not(test), no_std)]

pub mod dht22;
pub mod error;

pub use dht22::{Dht22, Reading};
pub use error::DhtError;
