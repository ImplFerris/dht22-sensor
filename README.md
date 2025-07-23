# DHT22 Sensor Driver

A platform-agnostic driver for the DHT22 (AM2302) temperature and humidity sensor using [`embedded-hal`] traits.  
Designed for use in `no_std` embedded environments.

---

## Features

- Supports blocking reads from the DHT22 sensor.
- Compatible with any platform that implements [`embedded-hal`] traits.
- `no_std` support.
- Tested using `embedded-hal-mock`.

---

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
# dht22-sensor = "0.1" 
dht22-sensor = { git = "https://github.com/implferris/dht22-sensor" }
```

### Example
```rust
use dht22_sensor::Dht22;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::delay::DelayNs;

// `pin` must support switching between input/output modes.
// `delay` should implement DelayNs for microsecond delays.

let mut dht = Dht22::new(pin, delay);
match dht.read() {
    Ok(reading) => {
        info!("Temperature: {:.1} °C", reading.temperature);
        info!("Humidity: {:.1} %", reading.humidity);
    }
    Err(e) => {
        error!("DHT22 read error: {:?}", e);
    }
}
```

## License
MIT
 