#[derive(Debug, Default)]
pub struct Cpu 
{
    pub a: u8,
    pub f: u8, // Flags register
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub h: u8,
    pub l: u8,
    pub sp: u16, // Stack Pointer
    pub pc: u16, // Program Counter
}

impl Cpu 
{
    pub fn new() -> Self 
    {

        Self 
        {
            a: 0,
            f: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            h: 0,
            l: 0,
            sp: 0xFFFE, // Stack pointer starts at 0xFFFE
            pc: 0x100,  // Program counter starts at 0x100 (after boot ROM)
        }
    }

    pub fn fetch(&mut self, memory: &[u8]) -> u8 
    {
        let opcode = memory[self.pc as usize]; 
        self.pc += 1; // Move to the next instruction
        opcode
    }

    pub fn execute(&mut self, opcode: u8) 
    {
        match opcode 
        {
            0x00 => {} // NOP (No Operation)
            0x3E => { // LD A, n (Load immediate value into A)
                self.a = opcode; // Normally, we'd fetch the next byte
            }
            _ => panic!("Unknown opcode: {:#04x}", opcode),
        }
    }

    pub fn step(&mut self, memory: &[u8]) 
    {
        let opcode = self.fetch(memory);
        self.execute(opcode);
    }
}

