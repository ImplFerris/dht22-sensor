use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};

use crate::error::DhtError;

/// Maximum time to wait (in microseconds) for the pin to change state.
///
/// Used to detect timeouts when waiting for the DHT22 to respond.
const TIMEOUT_US: u8 = 100;

/// Driver for the DHT22 temperature and humidity sensor.
pub struct Dht22<PIN, D> {
    pin: PIN,
    delay: D,
}

/// Reading returned by the DHT22 sensor.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// Temperature in degrees Celsius.
    pub temperature: f32,
    /// Relative humidity in percent.
    pub relative_humidity: f32,
}

impl<PIN, DELAY, E> Dht22<PIN, DELAY>
where
    PIN: InputPin<Error = E> + OutputPin<Error = E>,
    DELAY: DelayNs,
{
    /// Creates a new instance of the DHT22 driver.
    ///
    /// # Arguments
    ///
    /// * `pin` - The GPIO pin connected to the DHT22 data line. Must support both input and output.
    /// * `delay` - A delay provider implementing the `DelayNs` trait.
    pub fn new(pin: PIN, delay: DELAY) -> Self {
        Dht22 { pin, delay }
    }

    /// Reads a temperature and humidity measurement from the DHT22 sensor.
    ///
    /// This method performs the complete DHT22 communication sequence:
    /// sending a start signal, waiting for the sensor's response,
    /// reading 5 bytes, validating the checksum, and decoding the result.
    ///
    /// # Returns
    ///
    /// * `Ok(Reading)` if the read is successful and the checksum is valid.
    /// * `Err(DhtError)` if a communication or checksum error occurs.
    pub fn read(&mut self) -> Result<Reading, DhtError<E>> {
        self.start()?;

        let mut data = [0; 4];

        for b in data.iter_mut() {
            *b = self.read_byte()?;
        }

        let checksum = self.read_byte()?;
        if data.iter().fold(0u8, |sum, v| sum.wrapping_add(*v)) != checksum {
            Err(DhtError::ChecksumMismatch)
        } else {
            Ok(self.parse_data(data))
        }
    }

    /// Converts the 4-byte data into a `Reading` struct.
    fn parse_data(&self, data: [u8; 4]) -> Reading {
        let [hum_hi, hum_lo, temp_hi, temp_lo] = data;

        let joined_humidity = u16::from_be_bytes([hum_hi, hum_lo]);
        let relative_humidity = joined_humidity as f32 / 10.0;

        let is_temp_negative = (temp_hi >> 7) != 0;
        let temp_hi = temp_hi & 0b0111_1111;
        let joined_temp = u16::from_be_bytes([temp_hi, temp_lo]);
        let mut temperature = joined_temp as f32 / 10.0;
        if is_temp_negative {
            temperature = -temperature;
        }

        Reading {
            temperature,
            relative_humidity,
        }
    }

    /// Sends the start signal to the DHT22 and waits for its response.
    ///
    /// This includes pulling the line low for at least 1 ms,
    /// then high, followed by waiting for the sensor's 80us low
    /// and 80us high response.
    fn start(&mut self) -> Result<(), DhtError<E>> {
        // MCU sends start request
        self.pin.set_low()?;
        self.delay.delay_ms(1);
        self.pin.set_high()?;
        self.delay.delay_us(40);

        // Waiting for DHT22 Response
        self.wait_for_low()?; // 80us
        self.wait_for_high()?; // 80us
        Ok(())
    }

    /// Reads one byte (8 bits) from the sensor.
    ///
    /// # Returns
    ///
    /// * `Ok(u8)` with the read byte
    /// * `Err(DhtError)` on communication failure
    fn read_byte(&mut self) -> Result<u8, DhtError<E>> {
        let mut byte: u8 = 0;

        for i in 0..8 {
            let bit_mask = 1 << (7 - i);
            if self.read_bit()? {
                byte |= bit_mask;
            }
        }

        Ok(byte)
    }

    /// Reads a single bit from the sensor.
    ///
    /// The bit is determined by the duration of the high signal
    /// after the DHT22 pulls the line low.
    fn read_bit(&mut self) -> Result<bool, DhtError<E>> {
        // Wait for DHT pulls line low
        self.wait_for_low()?; // ~50us

        // Step 2: DHT pulls line high
        self.wait_for_high()?;

        // Step 3: Delay ~35us, then sample pin
        self.delay.delay_us(35);

        // If it is still High, then the bit value is 1
        let bit_is_one = self.pin.is_high()?;
        self.wait_for_low()?;

        Ok(bit_is_one)
    }

    /// Waits until the data line goes high or times out.
    fn wait_for_high(&mut self) -> Result<(), DhtError<E>> {
        Self::wait_for_state(&mut self.delay, || self.pin.is_high())
    }

    /// Waits until the data line goes low or times out.
    fn wait_for_low(&mut self) -> Result<(), DhtError<E>> {
        Self::wait_for_state(&mut self.delay, || self.pin.is_low())
    }

    /// Generic wait loop that checks a pin condition until true or timeout.
    ///
    /// # Arguments
    ///
    /// * `delay` - Delay provider
    /// * `condition` - Closure that returns true when the expected condition is met
    ///
    /// # Errors
    ///
    /// Returns `DhtError::Timeout` if the timeout is exceeded
    fn wait_for_state<F>(delay: &mut DELAY, mut condition: F) -> Result<(), DhtError<E>>
    where
        F: FnMut() -> Result<bool, E>,
    {
        for _ in 0..TIMEOUT_US {
            if condition()? {
                return Ok(());
            }
            delay.delay_us(1);
        }
        Err(DhtError::Timeout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::delay::CheckedDelay;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    // use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::delay::Transaction as DelayTx;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTx,
    };

    fn start_sequence() -> Vec<PinTx> {
        vec![
            PinTx::set(PinState::High), // Initial High
            // MCU initiates communication by pulling the data line low, then releasing it (pulling it high)
            PinTx::set(PinState::Low),
            PinTx::set(PinState::High),
            // Sensor responds
            PinTx::get(PinState::Low),
            PinTx::get(PinState::High),
        ]
    }

    // Helper to encode one byte into 8 bits (MSB first)
    fn encode_byte(byte: u8) -> Vec<PinTx> {
        (0..8)
            .flat_map(|i| {
                // Extract bit (MSB first: bit 7 to bit 0)
                let bit = (byte >> (7 - i)) & 1;
                vec![
                    PinTx::get(PinState::Low),  // wait_for_low
                    PinTx::get(PinState::High), // wait_for_high
                    PinTx::get(if bit == 1 {
                        // sample
                        PinState::High
                    } else {
                        PinState::Low
                    }),
                    PinTx::get(PinState::Low), // end of bit
                ]
            })
            .collect()
    }

    #[test]
    fn test_start_sequence() {
        let mut expect = vec![];
        expect.extend_from_slice(&start_sequence());

        let mut pin = PinMock::new(&expect);
        pin.set_high().unwrap();

        let delay_transactions = vec![DelayTx::delay_ms(1), DelayTx::delay_us(40)];
        let mut delay = CheckedDelay::new(&delay_transactions);

        let mut dht = Dht22::new(pin.clone(), &mut delay);
        dht.start().unwrap();

        pin.done();
        delay.done();
    }

    #[test]
    fn test_wait_for_state() {
        let mut expect = vec![];

        expect.extend_from_slice(&[
            // pin setting high
            PinTx::set(PinState::High),
            // wait_for_high
            PinTx::get(PinState::Low), // Triggers Delay 1us
            PinTx::get(PinState::Low), // Triggers Delay 1us
            PinTx::get(PinState::High),
            // wait_for_low
            PinTx::get(PinState::Low),
        ]);

        let mut pin = PinMock::new(&expect);
        pin.set_high().unwrap();

        let delay_transactions = vec![DelayTx::delay_us(1), DelayTx::delay_us(1)];
        let mut delay = CheckedDelay::new(&delay_transactions);

        let mut dht = Dht22::new(pin.clone(), &mut delay);
        dht.wait_for_high().unwrap();
        dht.wait_for_low().unwrap();

        pin.done();
        delay.done();
    }

    #[test]
    fn test_read_bit_one() {
        let mut pin = PinMock::new(&[
            // wait_for_low
            PinTx::get(PinState::Low), // Mimicks DHT pulling low to signal start of data bit
            // wait_for_high
            PinTx::get(PinState::High), // Then pulls high - duration determines bit value
            // delay_us(35) -> handled in delay
            // Sample pin after delay
            PinTx::get(PinState::High), // is it still High? (High -> 1)
            // Final wait_for_low
            PinTx::get(PinState::Low), // End of bit
        ]);

        let delay_transactions = vec![
            // wait_for_low
            DelayTx::delay_us(35),
        ];
        let mut delay = CheckedDelay::new(&delay_transactions);

        let mut dht = Dht22::new(pin.clone(), &mut delay);

        let bit = dht.read_bit().unwrap();
        assert!(bit);

        pin.done();
        delay.done();
    }

    #[test]
    fn test_read_bit_zero() {
        let mut pin = PinMock::new(&[
            // wait_for_low
            PinTx::get(PinState::High), // To trigger Delay of 1 us, we keep it High first
            PinTx::get(PinState::Low),
            // wait_for_high
            PinTx::get(PinState::Low), // To trigger Delay of 1 us, we keep it Low first
            PinTx::get(PinState::High), // now high
            // sample bit after delay (35us)
            PinTx::get(PinState::Low), // We will set it Low to indicate bit value is "0"
            // final wait_for_low
            PinTx::get(PinState::High), // To trigger Delay of 1 us, we keep it High first
            PinTx::get(PinState::Low),  // now low
        ]);

        let delay_transactions = vec![
            DelayTx::delay_us(1),  // after 1st pin high during wait_for_low
            DelayTx::delay_us(1),  // after 1st pin low during wait_for_high
            DelayTx::delay_us(35), // sampling delay
            DelayTx::delay_us(1),  // after 1st high in final wait_for_low
        ];
        let mut delay = CheckedDelay::new(&delay_transactions);

        let mut dht = Dht22::new(pin.clone(), &mut delay);

        let bit = dht.read_bit().unwrap();
        assert!(!bit);

        pin.done();
        delay.done();
    }

    #[test]
    fn test_read_timeout() {
        let pin_expects: Vec<PinTx> = (0..100).map(|_| PinTx::get(PinState::High)).collect();
        let mut pin = PinMock::new(&pin_expects);

        let delay_expects: Vec<DelayTx> = (0..100).map(|_| DelayTx::delay_us(1)).collect();

        let mut delay = CheckedDelay::new(&delay_expects);

        let mut dht = Dht22::new(pin.clone(), &mut delay);

        assert_eq!(dht.read_bit().unwrap_err(), DhtError::Timeout);

        pin.done();
        delay.done();
    }

    #[test]
    fn test_parse_data_positive_temp() {
        let mut pin = PinMock::new(&[]);

        let dht = Dht22::new(pin.clone(), NoopDelay);
        // Humidity: 55.5% -> [0x02, 0x2B] => 555
        // Temperature: 24.6C -> [0x00, 0xF6] => 246
        let data = [0x02, 0x2B, 0x00, 0xF6];

        let reading = dht.parse_data(data);

        assert_eq!(
            reading,
            Reading {
                relative_humidity: 55.5,
                temperature: 24.6,
            }
        );
        pin.done();
    }

    #[test]
    fn test_parse_data_negative_temp() {
        let mut pin = PinMock::new(&[]);

        let dht = Dht22::new(pin.clone(), NoopDelay);

        // Humidity: 40.0% -> [0x01, 0x90] => 400
        // Temperature: -1.0C -> [0x80, 0x0A]
        // Bit 7 of temp_hi is 1 => negative
        // Clear sign bit: 0x80 & 0x7F = 0x00, so [0x00, 0x0A] = 10 => 1.0 then negated
        let data = [0x01, 0x90, 0x80, 0x0A];

        let reading = dht.parse_data(data);

        assert_eq!(
            reading,
            Reading {
                relative_humidity: 40.0,
                temperature: -1.0,
            }
        );
        pin.done();
    }

    #[test]
    fn test_read_byte() {
        let pin_states = encode_byte(0b10111010);

        let mut pin = PinMock::new(&pin_states);
        let delay_expects = vec![DelayTx::delay_us(35); 8];
        let mut delay = CheckedDelay::new(&delay_expects);

        let mut dht = Dht22::new(pin.clone(), &mut delay);
        let byte = dht.read_byte().unwrap();
        assert_eq!(byte, 0b10111010);

        pin.done();
        delay.done();
    }

    #[test]
    fn test_read_valid() {
        // Data to simulate: [0x01, 0x90, 0x00, 0xF6], checksum = 0x87

        // Start sequence
        let mut pin_states = start_sequence();

        let data_bytes = [0x01, 0x90, 0x00, 0xF6];
        let checksum = 0x87;

        for byte in data_bytes.iter().chain(std::iter::once(&checksum)) {
            pin_states.extend(encode_byte(*byte));
        }

        let mut pin = PinMock::new(&pin_states);
        pin.set_high().unwrap();

        // Delays: start = 1ms + 40us
        let mut delay_transactions = vec![DelayTx::delay_ms(1), DelayTx::delay_us(40)];
        // Delay for data bit transfer: 40 bits * 35us delay
        delay_transactions.extend(std::iter::repeat_n(DelayTx::delay_us(35), 40));

        let mut delay = CheckedDelay::new(&delay_transactions);

        let mut dht = Dht22::new(pin.clone(), &mut delay);
        let reading = dht.read().unwrap();

        assert_eq!(
            reading,
            Reading {
                relative_humidity: 40.0,
                temperature: 24.6,
            }
        );

        pin.done();
        delay.done();
    }

    #[test]
    fn test_read_invalid() {
        // Data to simulate: [0x01, 0x90, 0x00, 0xF6], checksum = 0x87

        // Start sequence
        let mut pin_states = start_sequence();

        let data_bytes = [0x01, 0x90, 0x00, 0xF6];
        let checksum = 0x81; // Wrong checksum value

        for byte in data_bytes.iter().chain(std::iter::once(&checksum)) {
            pin_states.extend(encode_byte(*byte));
        }

        let mut pin = PinMock::new(&pin_states);
        pin.set_high().unwrap();

        // Delays: start = 1ms + 40us
        let mut delay_transactions = vec![DelayTx::delay_ms(1), DelayTx::delay_us(40)];
        // Delay for data bit transfer: 40 bits * 35us delay
        delay_transactions.extend(std::iter::repeat_n(DelayTx::delay_us(35), 40));

        let mut delay = CheckedDelay::new(&delay_transactions);

        let mut dht = Dht22::new(pin.clone(), &mut delay);
        assert_eq!(dht.read().unwrap_err(), DhtError::ChecksumMismatch);

        pin.done();
        delay.done();
    }
}
