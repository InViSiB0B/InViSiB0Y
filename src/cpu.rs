#[derive(Debug, Default)]
#[allow(dead_code)]
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
        println!("Fetching opcode {:#04x} at PC={:#06x}", opcode, self.pc);
        self.pc += 1; // Move to the next instruction
        opcode
    }

    pub fn execute(&mut self, opcode: u8, memory: &mut [u8]) 
    {
        match opcode 
        {
            0x00 => {
                println!("Executing NOP at PCC={:#06x}", self.pc);
            } // NOP (No Operation)

            0x3E => { // LD A, n (Load immediate value into A)
                println!("Executing LD A, n at PC={:#06x}", self.pc);
                self.a = memory[self.pc as usize];
                self.pc += 1; // Move past the immediate byte
            }
            0xC3 => { // JP nn (Jump to immediate 16-bit address)
                println!("Executing JP nn at PC={:#06x}", self.pc);
                let low = memory[self.pc as usize] as u16;  // Convert u8 to u16
                let high = memory[(self.pc + 1) as usize] as u16; // Convert u8 to u16
                self.pc = (high << 8) | low; // Set PC to the new address
            }
            0xF3 => { // DI (Disable Interrupts)
                println!("Executing DI at PC={:#06x}", self.pc);
                // Normally, we'd update the CPU's interrupt flag, but for now, we just log it
            }
            0x31 => { // LD SP, nn (Load immediate 16-bit value into SP)
                let low = memory[(self.pc) as usize] as u16;
                let high = memory[(self.pc + 1) as usize] as u16;
                self.sp = (high << 8) | low;
                println!("Executing LD SP, nn at PC={:#06x}, SP set to {:#06x}", self.pc, self.sp);
                self.pc += 2; // Advance past the 16-bit immediate operand
            }
            0xEA => { // LD (nn), A (Store A at address nn)
                let low = memory[(self.pc) as usize] as u16;
                let high = memory[(self.pc + 1) as usize] as u16;
                let addr = (high << 8) | low; // Construct 16-bit address
                memory[addr as usize] = self.a; // Store A at address nn
                println!(
                    "Executing LD (nn), A at PC={:#06x}, storing A={:#04x} at address {:#06x}",
                    self.pc, self.a, addr
                );
                self.pc += 2; // Advance past 16-bit operand
            }
            0xE0 => {
                let address = 0xFF00 + self.fetch(memory) as u16;
                memory[address as usize] = self.a;
                println!("Executing LDH (n), A at PC={:#06x}, storing A={:#04x} at {:#06x}", self.pc, self.a, address);
            }
            0x21 => { // LD HL, nn
                let lo = self.fetch(memory); // Low byte
                let hi = self.fetch(memory); // High byte
                self.h = hi; // Store in H
                self.l = lo; // Store in L
                println!("Executing LD HL, nn at PC={:#06x}, HL set to {:#04x}{:02x}", self.pc, self.h, self.l);
            }
            0xCD => { // CALL nn
                let lo = self.fetch(memory); // Low byte of address
                let hi = self.fetch(memory); // High byte of address
                let address = ((hi as u16) << 8) | (lo as u16); // Combine into a 16-bit address
            
                // Push current PC onto the stack (decrement SP and store the PC)
                self.sp = self.sp.wrapping_sub(2); // Decrement SP
                memory[self.sp as usize] = (self.pc >> 8) as u8; // Store high byte of PC
                memory[(self.sp + 1) as usize] = (self.pc & 0xFF) as u8; // Store low byte of PC
            
                // Set the PC to the new address (jump to the subroutine)
                self.pc = address;
            
                println!("Executing CALL nn at PC={:#06x}, calling address {:#06x}", self.pc, address);
            }
            0x7D => {
                self.a = self.l; // Load L into A
                println!("Executing LD A, L at PC={:#06x}", self.pc);
            }
            0x7C => {
                self.a = self.h; // Load H into A
                println!("Executing LD A, H at PC={:#06x}", self.pc);
            }
            0x18 => {
                let displacement = memory[self.pc as usize + 1] as i8; // Read the signed displacement (next byte)
                self.pc = self.pc.wrapping_add(2) + displacement as u16; // Jump to the new address
                println!("Executing JR r8 at PC={:#06x}, Jumping to {:#06x}", self.pc, self.pc);
            }
            0xf5 => {
                // Combine A (8 bits) and F (8 bits) into a 16-bit value for AF
                let af_value = ((self.a as u16) << 8) | (self.f as u16); // Shift A (8 bits) to the left by 8 positions, then OR with F (8 bits)
            
                // Decrement SP by 2 (2 bytes to push onto stack)
                self.sp = self.sp.wrapping_sub(2);
            
                // Store AF in memory (first A, then F)
                memory[self.sp as usize] = (af_value >> 8) as u8; // Push A (high byte)
                memory[(self.sp + 1) as usize] = (af_value & 0xFF) as u8; // Push F (low byte)
            
                println!("Executing PUSH AF at PC={:#06x}, Pushing A={:#02x} and F={:#02x} to stack", self.pc, self.a, self.f);
            }
            0xc5 => {
                // Combine B and C into the BC register pair
                let bc_value = ((self.b as u16) << 8) | (self.c as u16);
            
                // Decrement SP by 2 (to make space for two bytes on the stack)
                self.sp = self.sp.wrapping_sub(2);
            
                // Store BC in memory (first B, then C)
                memory[self.sp as usize] = (bc_value >> 8) as u8; // Push B (high byte)
                memory[(self.sp + 1) as usize] = (bc_value & 0xFF) as u8; // Push C (low byte)
            
                println!("Executing PUSH BC at PC={:#06x}, Pushing B={:#02x} and C={:#02x} to stack", self.pc, self.b, self.c);
            }
            0x44 => {
                // LD B, B is a no-op for the B register
                println!("Executing LD B, B at PC={:#06x}, B remains unchanged", self.pc);
            }
            0x4D => {
                // LD C, L copies L into C
                self.c = self.l;
                println!("Executing LD C, L at PC={:#06x}, C = {}", self.pc, self.c);
            }
            _ => {
                panic!("Unknown opcode: {:#04x} at PC={:#06x}", opcode,self.pc);
            }
        }
    }

    pub fn step(&mut self, memory: &mut [u8]) 
    {
        let opcode = self.fetch(memory);
        println!("Fetching opcode {:#04x} at PC={:#06x}", opcode, self.pc);
        self.execute(opcode, memory);
    }
}

