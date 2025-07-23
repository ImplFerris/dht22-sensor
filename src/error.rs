/// Possible errors from the DHT22 driver.
#[derive(Debug, PartialEq, Eq)]
pub enum DhtError<E> {
    /// Timed out waiting for a pin state change.
    Timeout,
    /// Checksum did not match the received data.
    ChecksumMismatch,
    /// Error from the GPIO pin (input/output).
    PinError(E),
}

impl<E> From<E> for DhtError<E> {
    fn from(value: E) -> Self {
        Self::PinError(value)
    }
}
