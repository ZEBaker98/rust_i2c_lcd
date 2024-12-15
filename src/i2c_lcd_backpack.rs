use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

pub struct Mcp23008<I2C> {
    address: u8,
    i2c: I2C,
    direction_cache: u8,
    output_cache: u8,
}

impl<I2C: I2c> Mcp23008<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Mcp23008 {
            address,
            i2c,
            direction_cache: 0,
            output_cache: 0,
        }
    }

    pub fn register_read(&mut self, register: usize) -> Result<u8, I2C::Error> {
        let mut read: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[register.try_into().unwrap()], &mut read)?;
        Ok(read[0])
    }

    pub fn register_write(&mut self, register: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[register, value])
    }

    pub fn direction_get(&mut self) -> Result<u8, I2C::Error> {
        self.direction_cache = self.register_read(0x00)?;
        Ok(self.direction_cache)
    }

    pub fn direction_set(&mut self, value: u8) -> Result<(), I2C::Error> {
        self.direction_cache = value;
        self.register_write(0x00, self.direction_cache)
    }

    pub fn direction_set_masked(&mut self, value: u8, mask: u8) -> Result<(), I2C::Error> {
        self.direction_cache &= !mask;
        self.direction_cache |= value & mask;
        self.register_write(0x00, self.direction_cache)
    }

    pub fn io_get(&mut self) -> Result<u8, I2C::Error> {
        self.register_read(0x09)
    }

    pub fn io_set(&mut self, value: u8) -> Result<(), I2C::Error> {
        self.output_cache = value & !self.direction_cache;
        self.register_write(0x0A, self.output_cache)
    }

    pub fn io_set_masked(&mut self, value: u8, mask: u8) -> Result<(), I2C::Error> {
        self.output_cache &= !mask;
        self.output_cache |= value & mask & !self.direction_cache;
        self.register_write(0x0A, self.output_cache)
    }
}

pub struct LCDBackpack<I2C, D> {
    mcp23008: Mcp23008<I2C>,
    delay: D,
}

impl<I2C: I2c, D: DelayNs> LCDBackpack<I2C, D> {
    pub fn new(mut mcp23008: Mcp23008<I2C>, delay: D) -> Self {
        mcp23008.direction_set(0x00).unwrap();
        mcp23008.io_set(0x80).unwrap();
        LCDBackpack { mcp23008, delay }
    }
}

impl<I2C: I2c, D: DelayNs> lcd::Hardware for LCDBackpack<I2C, D> {
    fn rs(&mut self, bit: bool) {
        self.mcp23008
            .io_set_masked((bit as u8) << 1, 0x1 << 1)
            .unwrap();
    }

    fn enable(&mut self, bit: bool) {
        self.mcp23008
            .io_set_masked((bit as u8) << 2, 0x1 << 2)
            .unwrap();
    }

    fn data(&mut self, data: u8) {
        self.mcp23008.io_set_masked(data << 3, 0xF << 3).unwrap();
    }
}

impl<I2C: I2c, D: DelayNs> lcd::Delay for LCDBackpack<I2C, D> {
    fn delay_us(&mut self, us: u32) {
        self.delay.delay_us(us);
    }
}
