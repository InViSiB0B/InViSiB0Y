pub struct MMU 
{
// ROM (cartridge)
rom: Vec<u8>,
rom_bank: usize,

// RAM
vram: [u8; 0x2000],
wram: [u8; 0x2000],
oam: [u8; 0xA0],
hram: [u8; 0x7F],

// Cartridge RAM
cart_ram: Vec<u8>,
ram_bank: usize,
ram_enabled: bool,

// I/O Registers
io: [u8; 0x80],

// Interrupt Enable Register
ie: u8,

// MBC type
mbc_type: MBCType,
}

impl MMU
{
    pub fn new(rom_data: Vec<u8>) -> Self
    {
        let mbc_type = determine_mbc_type(&rom_data);
        let cart_ram_size = determine_ram_size(&rom_data);

        Self
        {
            rom: rom_data,
            rom_bank: 1, // Bank 0 is fixed, bank 1 is switchable by defaulte
            vram: [0; 0x2000],
            wram: [0; 0x2000],
            oam: [0; 0xA0],
            hram: [0; 0x7F],
            cart_ram: vec![0; cart_ram_size],
            ram_bank: 0,
            ram_enabled: false,
            io: [0; 0x80],
            ie: 0, 
            mbc_type,
        }
    }

    pub fn read_byte(&self, addr: u16) -> u8
    {
        match addr
        {
            // ROM bank 0 (fixed)
            0x0000..=0x3FFF => self.rom[addr as usize],

            //ROM bank 1+ (switchable)
            0x4000..=0x7FFF => {
            let bank_offset = self.rom_bank * 0x4000;
            self.rom[bank_offset + (addr as usize - 0x4000)]
            },
            // VRAM
            0x8000..=0x9FFF => self.vram[(addr - 0x8000) as usize],

            // External RAM
            0xA000..=0xBFFF => {
                if self.ram_enabled
                {
                    let bank_offset = self.ram_bank * 0x2000;
                    self.cart_ram[bank_offset + (addr as usize - 0xA000)]
                }
                else
                {
                    0xFF // Return 0xFF if RAM is not enabled
                }
            },

            // WRAM
            0xC000..=0xDFFF => self.wram[(addr - 0xC000) as usize],

            // Echo RAM (mirror of 0xC000-0xDDFF)
            0xE000..=0xFDFF => self.wram[(addr - 0xE000) as usize],

            // OAM
            0xFE00..=0xFE9F => self.oam[(addr - 0xFE00) as usize],

            // Not usable
            0xFEA0..=0xFEFF => 0xFF,

            // I/O Registers
            0xFF00..=0xFF7F => self.io[(addr - 0xFF00) as usize],

            // HRAM
            0xFF80..=0xFFFE => self.hram[(addr - 0xFF80) as usize],

            // Interrupt Enable register
            0xFFFF => self.ie
        }
    }

    pub fn write_byte (&mut self, addr: u16, value: u8)
    {
        match addr 
        {

            // MBC control registers
            0x0000..=0x7FFF => self.handle_mbc_write(addr, value),

            // VRAM
            0x8000..=0x9FFF => self.vram[(addr - 0x8000) as usize] = value,

            // External RAM
            0xA000..=0xBFFF => {
                if self.ram_enabled
                {
                    let bank_offset = self.ram_bank * 0x2000;
                    self.cart_ram[bank_offset + (addr as usize - 0xA000)] = value;
                }
            },

            // WRAM
            0xC000..=0xDFFF => self.wram[(addr - 0xC000) as usize] = value,

            // Echo RAM (mirror of 0xC000-0xDDFF)
            0xE000..=0xFDFF => self.wram[(addr - 0xE000) as usize] = value,

            // OAM
            0xFE00..=0xFE9F => self.oam[(addr - 0xFE00) as usize] = value,

            // Not usable
            0xFEA0..=0xFEFF => {},

            // I/O registers
            0xFF00..=0xFF7F => {
                self.io[(addr - 0xFF00) as usize] = value;
                self.handle_io_write(addr, value);
            },

            // HRAM
            0xFF80..=0xFFFE => self.hram[(addr - 0xFF80) as usize] = value,

            // Interrupt Enable register
            0xFFFF => self.ie = value,
        }
    }

    fn handle_mbc_write(&mut self, addr: u16, value: u8)
    {
        match self.mbc_type
        {
            MBCType::None => {}, // No banking, writes to ROM are ignored

            MBCType::MBC1 => {
                match addr 
                {
                    // RAM enable
                    0x0000..=0x1FFF => self.ram_enabled = (value & 0x0F) == 0x0A,

                    // ROM bank number (lower 5 bits)
                    0x2000..=0x3FFF=> {
                        let mut bank = value & 0x1F;
                        if bank == 0 { bank = 1; } // Bank 0 is treated as bank 1
                        self.rom_bank = (self.rom_bank & 0x60) | (bank as usize);
                    },
                    // ROM/RAM bank number (upper 2 bits)
                    0x4000..=0x5FFF => {
                        let bits = (value & 0x03) as usize;
                        // Mode select determines whether these bits affect ROM or RAM bank
                        if self.io[0x50] & 0x01 == 0
                        {
                            // ROM banking mode
                            self.rom_bank = (self.rom_bank & 0x1F) | (bits << 5);
                        }
                        else
                        {
                            // RAM banking mode
                            self.ram_bank = bits;
                        }
                    },
                    // ROM/RAM mode select
                    0x6000..=0x7FFF => {
                        self.io[0x50] = value & 0x01; // Store mode in an I/O register
                    },

                    _=>{},
                }
            },
            // Implement other MBX types here
            _=>{}, 
        }   

    }

    fn handle_io_write(&mut self, addr: u16, value: u8)
    {
        match addr
        {
            // Handle special I/O registers
            0xFF01=> {
                // Serial transfer data
                self.io[0x01] = value;
            },

            0xFF02=> {
                // Serial transfer control
                self.io[0x02] = value;

                // If bit 7 is set, a transfer is requested
                if value == 0x81 
                {
                    let output_char = self.io[0x01] as char;
                    println!("Serial transfer: {}", output_char);
                    
                    // Reset the transfer flag after handling
                    self.io[0x02] = 0; // Reset completely
                }
            },
            // Timer registers
            0xFF04 => self.io[0x04] = 0, // DIV is reset on any write
            0xFF05 => self.io[0x05] = value, // TIMA
            0xFF06 => self.io[0x06] = value, // TMA
            0xFF07 => self.io[0x07] = value, // TAC

            // Add others here
            _=>{},
        }
    }

    pub fn check_serial_output(&mut self) -> Option<char>
    {
        // Check if a serial transfer has been initiated (0xFF02 == 0x81)
        if self.io[0x02] == 0x81
        {
            let output_char = self.io[0x01] as char;
            self.io[0x02] = 0; // Reset the transfer flag
            Some(output_char)
        }
        else {
            {
                None
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum MBCType
{
    None,
    MBC1,
    MBC2,
    MBC3,
    MBC5,
}

fn determine_mbc_type(rom: &[u8]) -> MBCType
{
    match rom[0x147]
    {
        0x00 => MBCType::None,
        0x01..=0x03 => MBCType::MBC1,
        0x05..=0x06 => MBCType::MBC2,
        0x0F..=0x13 => MBCType::MBC3,
        0x19..=0x1E => MBCType::MBC5,
        _=> MBCType::None, // Default to no MBC
    }
}

fn determine_ram_size(rom: &[u8]) -> usize
{
    match rom[0x149]
    {
        0x00 => 0,
        0x01 => 2 * 1024,
        0x02 => 8 * 1024,
        0x03 => 32 * 1024,
        0x04 => 128 * 1024,
        0x05 => 64 * 1024,
        _=> 0,
    }
}
