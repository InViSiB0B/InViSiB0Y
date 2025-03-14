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
                self.pc = self.pc.wrapping_add(1); // Increment PC past the displacement byte
                self.pc = self.pc.wrapping_add(displacement as u16); // Add the displacement to the PC
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
            0x03 => { // INC BC (Increment BC)
                // Combine B and C into a 16-bit value
                let mut bc = ((self.b as u16) << 8) | (self.c as u16);
                
                // Increment BC, using wrapping_add to handle overflow
                bc = bc.wrapping_add(1);
                
                // Split back into separate registers
                self.b = (bc >> 8) as u8;
                self.c = (bc & 0xFF) as u8;
                
                println!("Executing INC BC at PC={:#06x}, BC now {:#06x}", self.pc, bc);
            }
            0x79 => { // LD A, C (Load C into A)
                self.a = self.c;
                println!("Executing LD A, C at PC={:#06x}, A now {:#04x}", self.pc, self.a);
            }
            0xc1 => { // POP BC (Pop BC from stack)
                // Read from stack memory at current SP location
                let c = memory[self.sp as usize]; // Low byte
                let b = memory[(self.sp + 1) as usize]; // High byte
                
                // Update BC registers
                self.c = c;
                self.b = b;
                
                // Increment SP by 2 bytes
                self.sp = self.sp.wrapping_add(2);
                
                println!("Executing POP BC at PC={:#06x}, BC now {:#02x}{:02x}", self.pc, self.b, self.c);
            }
            0xc9 => { // RET (Return from subroutine)
                // Read return address from stack
                let low = memory[self.sp as usize]; // Lower byte of address
                let high = memory[(self.sp + 1) as usize]; // Higher byte of address
                
                // Combine into 16-bit address
                let address = ((high as u16) << 8) | (low as u16);
                
                // Set PC to the return address
                self.pc = address;
                
                // Increment SP by 2 bytes
                self.sp = self.sp.wrapping_add(2);
                
                println!("Executing RET at PC={:#06x}, returning to address {:#06x}", self.pc, address);
            }
            0x7E => { // LD A, (HL) (Load value at address in HL into A)
                // Combine H and L into a 16-bit address
                let hl_addr = ((self.h as u16) << 8) | (self.l as u16);
                
                // Read from memory at the address in HL
                self.a = memory[hl_addr as usize];
                
                println!("Executing LD A, (HL) at PC={:#06x}, loading A={:#04x} from address {:#06x}", 
                         self.pc, self.a, hl_addr);
            }
            0x0C => { // INC C (Increment C)
                // Save original value for flag checking
                let original = self.c;
                
                // Increment C with wrapping
                self.c = self.c.wrapping_add(1);
                
                // Update flags:
                // Z flag - Set if result is zero
                if self.c == 0 {
                    self.f = self.f | 0x80; // Set bit 7 (Z flag)
                } else {
                    self.f = self.f & 0x7F; // Reset bit 7 (Z flag)
                }
                
                // N flag - Reset (bit 6)
                self.f = self.f & 0xBF; // Reset bit 6 (N flag)
                
                // H flag - Set if carry from bit 3 to bit 4
                if (original & 0x0F) == 0x0F {
                    self.f = self.f | 0x20; // Set bit 5 (H flag)
                } else {
                    self.f = self.f & 0xDF; // Reset bit 5 (H flag)
                }
                
                // C flag - Not affected
                
                println!("Executing INC C at PC={:#06x}, C now {:#04x}, flags={:#04x}", 
                         self.pc, self.c, self.f);
            }
            0x60 => { // LD H, B (Load B into H)
                self.h = self.b;
                println!("Executing LD H, B at PC={:#06x}, H now {:#04x}", self.pc, self.h);
            }
            0x06 => { // LD B, n (Load immediate value into B)
                println!("Executing LD B, n at PC={:#06x}", self.pc);
                self.b = memory[self.pc as usize]; // Load the immediate value into B
                self.pc += 1; // Move past the immediate byte
            }
            0x66 => { // LD H, (HL) (Load value at address HL into H)
                // Combine H and L into a 16-bit address (HL)
                let hl_addr = ((self.h as u16) << 8) | (self.l as u16);
                
                // Load value from memory at address HL into H
                self.h = memory[hl_addr as usize];
                
                println!("Executing LD H, (HL) at PC={:#06x}, loading H={:#04x} from address {:#06x}", 
                         self.pc, self.h, hl_addr);
            }
            0x3C => { // INC A (Increment A by 1)
                self.a = self.a.wrapping_add(1); // Use wrapping_add to avoid overflow issues
                self.pc = self.pc.wrapping_add(1); // Increment PC to move to the next instruction
                println!("Executing INC A at PC={:#06x}, A={:#04x}", self.pc, self.a);
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

