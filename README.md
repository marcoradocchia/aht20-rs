<div align="center">
  <h1 align="center">aht20-rs</h1>

  <!-- ![Crates.io downloads](https://img.shields.io/crates/d/aht20-rs?label=crates.io%20downloads&logo=rust) -->
  <!-- ![Crates.io version](https://img.shields.io/crates/v/aht20-rs?color=%23d8a657&logo=rust) -->
  <!-- TODO: uncomment on crates.io publishing -->
</div>

[Embedded-hal](https://crates.io/crates/embedded-hal) [AHT20](https://learn.adafruit.com/adafruit-aht20)
**temperature** and **humidity** sensor rust driver.

# Example
Small usage example of this driver below:
```rust
// Construct a new `Aht20` instance and initialize the sensor.
let aht20 = Aht20::new(i2c, delay)?;
// Perform temperature and humidity measurement.
let measure = aht20.measure()?;

// Print human-readable measurement values: this values are lazily evaluated on
// function calls.
println!(
    "Humidity: {}%, Temperature: {}°C",
    measure.humidity(),
    measure.temperature(),
);
// One may want to retrieve only one of the two values: only required value's relative 
// calculations are performed.
println!(
    "I just need temperature value, don't waste compute time! Temperature: {}°C",
    measure.temperature(),
);

// WHERE NEEDED: perform a soft reset.
aht20.reset()?;
```
