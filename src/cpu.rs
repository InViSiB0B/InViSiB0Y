use crate::mmu::MMU;

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
    pub ime: bool, // Interrupt Master Enable
    pub debug_mode: bool, // Debug mode
    pub halted: bool // CPU is halted
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
            ime: false, // Interrupts are disabled by default
            debug_mode: false,  // Initialize to false by default
            halted: false // Initialize to false by default
        }
    }

    pub fn fetch(&mut self, mmu: &MMU) -> u8 
    {
        let opcode = mmu.read_byte(self.pc);
        println!("Fetching opcode {:#04x} at PC={:#06x}", opcode, self.pc);
        println!("Fetch: Reading byte at PC={:#06x}, value={:#04x}", self.pc, opcode);
        self.pc = self.pc.wrapping_add(1);  // Move to the next instruction
        opcode
    }

    pub fn push(&mut self, mmu: &mut MMU, value: u8) 
    {
        self.sp = self.sp.wrapping_sub(1); // Decrement SP before writing
        mmu.write_byte(self.sp, value);
    }

    pub fn pop(&mut self, mmu: &mut MMU) -> u8 
    {
        let value = mmu.read_byte(self.sp);
        self.sp = self.sp.wrapping_add(1); // Increment SP after reading
        value
    }

    pub fn execute(&mut self, opcode: u8, mmu: &mut MMU) -> u8
    {
        match opcode 
        {
            0x00 => { // NOP (No Operation)
                println!("Executing NOP at PCC={:#06x}", self.pc);
                4 // NOP takes 4 cycles
            }
            0x01 => { // LD BC, nn

                self.c = mmu.read_byte(self.pc);
                self.b = mmu.read_byte(self.pc + 1);
                self.pc += 2;
                println!("Executing LD BC, {:#04x}{:#04x} at PC={:#06x}", self.b, self.c, self.pc);
                12 // LD BC, nn takes 12 cycles
            }
            0x02 => { // LD (BC), A
                let addr = (self.b as u16) << 8 | self.c as u16;
                mmu.write_byte(addr, self.a);
                println!("Executing LD (BC), A at PC={:#06x}", self.pc);
                8 // LD (BC), A takes 8 cycles
            }
            0x03 => { // INC BC
                let bc = ((self.b as u16) << 8 | self.c as u16).wrapping_add(1);
                self.b = (bc >> 8) as u8;
                self.c = bc as u8;
                println!("Executing INC BC at PC={:#06x}", self.pc);
                8 // INC BC, takes 8 cycles
            }
            0x04 => { // INC B
                self.b = self.b.wrapping_add(1);
                println!("Executing INC B at PC={:#06x}", self.pc);
                4 // INC B takes 4 cycles
            }
            0x05 => { // DEC B
                self.b = self.b.wrapping_sub(1);
                println!("Executing DEC B at PC={:#06x}", self.pc);
                4 // DEC B takes 4 cycles
            }
            0x06 => { // LD B, n
                self.b = mmu.read_byte(self.pc);
                self.pc += 1;
                println!("Executing LD B, {:#04x} at PC={:#06x}", self.b, self.pc);
                8 // LD B, n takes 8 cycles
            }
            0x07 => { // RLCA
                let carry = self.a >> 7;
                self.f = carry << 4;
                self.a = (self.a << 1) | carry;
                println!("Executing RLCA at PC={:#06x}", self.pc);
                4 // RCLA takes 4 cycles
            }
            0x08 => { // LD (nn), SP
                // Read 16-bit address from next two bytes (little-endian)
                let low = mmu.read_byte(self.pc);
                let high = mmu.read_byte(self.pc + 1);
                let addr = (high as u16) << 8 | low as u16;
                
                // Write SP to memory (little-endian)
                mmu.write_byte(addr, self.sp as u8); // Low byte of SP
                mmu.write_byte(addr + 1, (self.sp >> 8) as u8); // High byte of SP
                
                // Increment PC by 2 (for the two address bytes)
                self.pc += 2;
                
                println!("Executing LD ({:#06x}), SP at PC={:#06x}", addr, self.pc);
                20 // LD (nn), SP takes 20 cycles
            }
            0x09 => { // ADD HL, BC
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add((self.b as u16) << 8 | self.c as u16);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing ADD HL, BC at PC={:#06x}", self.pc);
                8 // ADD HL, BC takes 8 cycles
            }
            0x0A => { // LD A, (BC)
                let addr = (self.b as u16) << 8 | self.c as u16;
                self.a = mmu.read_byte(addr);
                println!("Executing LD A, (BC) at PC={:#06x}", self.pc);
                8 // LD A, (BC) takes 8 cycles
            }
            0x0B => { // DEC BC
                let bc = ((self.b as u16) << 8 | self.c as u16).wrapping_sub(1);
                self.b = (bc >> 8) as u8;
                self.c = bc as u8;
                println!("Executing DEC BC at PC={:#06x}", self.pc);
                8 // DEC B takes 8 cycles
            }
            0x0C => { // INC C
                self.c = self.c.wrapping_add(1);
                println!("Executing INC C at PC={:#06x}", self.pc);
                4 // INC C takes 4 cycles
            }
            0x0D => { // DEC C
                self.c = self.c.wrapping_sub(1);
                println!("Executing DEC C at PC={:#06x}", self.pc);
                4 // DEC C takes 4 cycles
            }
            0x0E => { // LD C, n
                self.c = mmu.read_byte(self.pc);
                self.pc += 1;
                println!("Executing LD C, {:#04x} at PC={:#06x}", self.c, self.pc);
                8 // LD C, n takes 8 cycles
            }
            0x0F => { // RRCA
                let carry = self.a & 1;
                self.f = carry << 4;
                self.a = (self.a >> 1) | (carry << 7);
                println!("Executing RRCA at PC={:#06x}", self.pc);
                4 // RRCA takes 4 cycles
            }
            0x10 => { // STOP
                self.pc = self.pc.wrapping_add(1); // Skip the next byte

                // Enter low power mode
                self.halted = true; // CPU is halted
                println!("Executing STOP at PC={:#06x}", self.pc);
                4 // STOP takes 4 cycles
            }
            0x11 => { // LD DE, nn
                self.e = mmu.read_byte(self.pc);
                self.d = mmu.read_byte(self.pc + 1);
                self.pc += 2;
                println!("Executing LD DE, {:#04x}{:#04x} at PC={:#06x}", self.d, self.e, self.pc);
                12 // LD DE, nn takes 12 cycles
            }
            0x12 => { // LD (DE), A
                let addr = (self.d as u16) << 8 | self.e as u16;
                mmu.write_byte(addr, self.a); // Write A to the address in DE
                println!("Executing LD (DE), A at PC={:#06x}", self.pc);
                8 // LD (DE), A takes 8 cycles
            }
            0x13 => { // INC DE
                let de = ((self.d as u16) << 8 | self.e as u16).wrapping_add(1);
                self.d = (de >> 8) as u8;
                self.e = de as u8;
                println!("Executing INC DE at PC={:#06x}", self.pc);
                8 // INC DE takes 8 cycles
            }
            0x14 => { // INC D
                self.d = self.d.wrapping_add(1);
                println!("Executing INC D at PC={:#06x}", self.pc);
                4 // INC D takes 4 cycles
            }
            0x15 => { // DEC D
                self.d = self.d.wrapping_sub(1);
                println!("Executing DEC D at PC={:#06x}", self.pc);
                4 // DEC D takes 4 cycles
            }
            0x16 => { // LD D, n
                self.d = mmu.read_byte(self.pc);
                self.pc += 1;
                println!("Executing LD D, {:#04x} at PC={:#06x}", self.d, self.pc);
                8 // LD D, n takes 8 cycles
            }
            0x17 => { // RLA
                let carry = self.a >> 7;
                self.a = (self.a << 1) | (self.f >> 4);
                self.f = carry << 4;
                println!("Executing RLA at PC={:#06x}", self.pc);
                4 // RLA takes 4 cycles
            }
            0x18 => { // JR n
                let offset = mmu.read_byte(self.pc) as i8;
                self.pc = self.pc.wrapping_add(1).wrapping_add(offset as u16);
                println!("Executing JR {:#04x} at PC={:#06x}", offset, self.pc);
                12 // JR n takes 12 cycles
            }
            0x19 => { // ADD HL, DE
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add((self.d as u16) << 8 | self.e as u16);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing ADD HL, DE at PC={:#06x}", self.pc);
                8 // ADD HL, DE takes 8 cycles
            }
            0x1A => { // LD A, (DE)
                let addr = (self.d as u16) << 8 | self.e as u16;
                self.a = mmu.read_byte(addr);
                println!("Executing LD A, (DE) at PC={:#06x}", self.pc);
                8 // LD A, (DE) takes 8 cycles
            }
            0x1B => { // DEC DE
                let de = ((self.d as u16) << 8 | self.e as u16).wrapping_sub(1);
                self.d = (de >> 8) as u8;
                self.e = de as u8;
                println!("Executing DEC DE at PC={:#06x}", self.pc);
                8 // DEC DE takes 8 cycles
            }
            0x1C => { // INC E
                self.e = self.e.wrapping_add(1);
                println!("Executing INC E at PC={:#06x}", self.pc);
                4 // INC E takes 4 cycles
            }
            0x1D => { // DEC E
                self.e = self.e.wrapping_sub(1);
                println!("Executing DEC E at PC={:#06x}", self.pc);
                4 // DEC E takes 4 cycles
            }
            0x1E => { // LD E, n
                self.e = mmu.read_byte(self.pc);
                self.pc += 1;
                println!("Executing LD E, {:#04x} at PC={:#06x}", self.e, self.pc);
                8 // LD E, n takes 8 cycles
            }
            0x1F => { // RRA
                // Get bit 0 of A (will become carry flag)
                let bit0 = self.a & 0x01;
                
                // Get current carry flag (bit 4 of F)
                let old_carry = (self.f & 0x10) >> 4;
                
                // Rotate right through carry
                self.a = (self.a >> 1) | (old_carry << 7);
                
                // Update flags - Z is reset, N is reset, H is reset, C gets bit 0 of original A
                self.f = 0;  // Clear all flags
                if bit0 != 0 { self.f |= 0x10; }  // Set C flag if bit 0 was set
    
                println!("Executing RRA at PC={:#06x}", self.pc);
                4 // LD E, n takes 4 cycles
            }
            0x20 => { // JR NZ, n
                let offset = mmu.read_byte(self.pc) as i8;
                self.pc = self.pc.wrapping_add(1); // Increment PC to skip the offset byte
                
                // Zero flag is bit 7 of register F
                if self.f & 0x80 == 0 
                {
                    // Condition is true (NZ - Not Zero), perform the jump
                    self.pc = self.pc.wrapping_add(offset as u16);
                    println!("Executing JR NZ, {:#04x} at PC={:#06x}", offset, self.pc);
                    12 // 12 cycles when branch is taken
                } 
                else 
                {
                    // Condition is false (Z - Zero), don't jump
                    println!("Executing JR NZ, {:#04x} at PC={:#06x} (not taken)", offset, self.pc);
                    8  // 8 cycles when branch is not taken
                }
            }
            0x21 => { // LD HL, nn
                self.l = mmu.read_byte(self.pc);
                self.h = mmu.read_byte(self.pc + 1);
                self.pc += 2;
                println!("Executing LD HL, {:#04x}{:#04x} at PC={:#06x}", self.h, self.l, self.pc);
                12 // LD DL, nn takes 12 cycles
            }
            0x22 => { // LD (HL+), A
                let addr = (self.h as u16) << 8 | self.l as u16;
                mmu.write_byte(addr, self.a); // Write A to the address in HL
                
                // Increment HL
                let hl = addr.wrapping_add(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                
                println!("Executing LD (HL+), A at PC={:#06x}", self.pc);
                8 // LD (HL+), A takes 8 cycles
            }
            0x23 => { // INC HL
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing INC HL at PC={:#06x}", self.pc);
                8 // INC HL takes 8 cycles
            }
            0x24 => { // INC H
                self.h = self.h.wrapping_add(1);
                println!("Executing INC H at PC={:#06x}", self.pc);
                4 // INC H takes 4 cycles
            }
            0x25 => { // DEC H
                self.h = self.h.wrapping_sub(1);
                println!("Executing DEC H at PC={:#06x}", self.pc);
                4 // DEC H takes 4 cycles
            }
            0x26 => { // LD H, n
                self.h = mmu.read_byte(self.pc);
                self.pc += 1;
                println!("Executing LD H, {:#04x} at PC={:#06x}", self.h, self.pc);
                8 // LD H, n takes 8 cycles
            }
            0x27 => { // DAA
                // Decimal Adjust Accumulator after addition/subtraction
                let mut adjust = 0;
                let carry_flag = self.f & 0x10 != 0;
                let half_carry = self.f & 0x20 != 0;
                let subtract = self.f & 0x40 != 0;

                if subtract 
                {
                    // After subtraction
                    if carry_flag { adjust |= 0x60; }
                    if half_carry { adjust |= 0x06; }
                } 
                else 
                {
                    // After addition
                    if carry_flag || (self.a > 0x99) { adjust |= 0x60; }
                    if half_carry || ((self.a & 0x0F) > 0x09) { adjust |= 0x06; }
                }

                // Apply adjustment
                if subtract {self.a = self.a.wrapping_sub(adjust);} 
                else {self.a = self.a.wrapping_add(adjust);}

                // Update flags
                self.f &= 0x40; // Preserve N flag, reset H flag
                if adjust >= 0x60 { self.f |= 0x10; } // Set C flag if needed
                if self.a == 0 { self.f |= 0x80; } // Set Z flag if result is zero
                println!("Executing DAA at PC={:#06x}", self.pc);
                4 // DAA takes 4 cycles
            }
            0x28 => { // JR Z, n
                let offset = mmu.read_byte(self.pc) as i8;
                self.pc = self.pc.wrapping_add(1); // Increment PC to skip the offset byte
                
                // Zero flag is bit 7 of register F
                if self.f & 0x80 != 0 
                {
                    // Condition is true (Z - Zero), perform the jump
                    self.pc = self.pc.wrapping_add(offset as u16);
                    println!("Executing JR Z, {:#04x} at PC={:#06x}", offset, self.pc);
                    12 // 12 cycles when branch is taken
                } 
                else 
                {
                    // Condition is false (NZ - Not Zero), don't jump
                    println!("Executing JR Z, {:#04x} at PC={:#06x} (not taken)", offset, self.pc);
                    8  // 8 cycles when branch is not taken
                }
            }
            0x29 => { // ADD HL, HL
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add((self.h as u16) << 8 | self.l as u16);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing ADD HL, HL at PC={:#06x}", self.pc);
                8 // ADD HL, HL takes 8 cycles
            }
            0x2A => { // LD A, (HL+)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a = mmu.read_byte(addr);
                let hl = addr.wrapping_add(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing LD A, (HL+) at PC={:#06x}", self.pc);
                8 // LD A, (HL+) takes 8 cycles
            }
            0x2B => { // DEC HL
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_sub(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing DEC HL at PC={:#06x}", self.pc);
                8 // DEC HL takes 8 cycles
            }
            0x2C => { // INC L
                self.l = self.l.wrapping_add(1);
                println!("Executing INC L at PC={:#06x}", self.pc);
                4 // INC L takes 4 cycles
            }
            0x2D => { // DEC L
                self.l = self.l.wrapping_sub(1);
                println!("Executing DEC L at PC={:#06x}", self.pc);
                4 // DEC L takes 4 cycles
            }
            0x2E => { // LD L, n
                self.l = mmu.read_byte(self.pc);
                self.pc += 1;
                println!("Executing LD L, {:#04x} at PC={:#06x}", self.l, self.pc);
                8 // LD L,n takes 8 cycles
            }
            0x2F => { // CPL
                self.a = !self.a;
                self.f |= 0x60;
                println!("Executing CPL at PC={:#06x}", self.pc);
                4 // CPL takes 4 cycles
            }
            0x30 => { // JR NC, n
                let offset = mmu.read_byte(self.pc) as i8;
                self.pc = self.pc.wrapping_add(1);
                
                // Carry flag is bit 4 (0x10)
                let carry_set = (self.f & 0x10) != 0;
                println!("JR NC: F={:#04x}, carry_set={}, offset={}", self.f, carry_set, offset);
                
                if !carry_set 
                {
                    self.pc = self.pc.wrapping_add(offset as u16);
                    println!("  Jump taken to {:#06x}", self.pc);
                    12 // 12 cycles when branch is taken
                } 
                else 
                {
                    println!("  Jump not taken, continuing at {:#06x}", self.pc);
                    8  // 8 cycles when branch is not taken
                }
            }
            0x31 => { // LD SP, nn
                 // Get low byte first (little endian)
                let low = mmu.read_byte(self.pc);
                let high = mmu.read_byte(self.pc + 1);
                self.sp = ((high as u16) << 8) | (low as u16);
                self.pc += 2;
                println!("Executing LD SP, {:#04x}{:#04x} at PC={:#06x}", self.sp >> 8, self.sp, self.pc);
                12 // LD SP, nn takes 12 cycles
            }
            0x32 => { // LD (HL-), A
                let addr = (self.h as u16) << 8 | self.l as u16;
                mmu.write_byte(addr, self.a); // Write A to the address in HL

                // Decrement HL
                let hl = addr.wrapping_sub(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;

                println!("Executing LD (HL-), A at PC={:#06x}", self.pc);
                8 // LD (HL-), A
            }
            0x33 => { // INC SP
                self.sp = self.sp.wrapping_add(1);
                println!("Executing INC SP at PC={:#06x}", self.pc);
                8 // INC SP takes 8 cycles
            }
            0x34 => { // INC (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let value = mmu.read_byte(addr).wrapping_add(1);
                mmu.write_byte(addr, value);
                println!("Executing INC (HL) at PC={:#06x}", self.pc);
                12 // INC (HL) takes 12 cycles
            }
            0x35 => { // DEC (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let value = mmu.read_byte(self.pc).wrapping_sub(1);
                mmu.write_byte(addr, value);
                println!("Executing DEC (HL) at PC={:#06x}", self.pc);
                12 // DEC (HL) takes 12 cycles
            }
            0x36 => { // LD (HL), n
                let addr = (self.h as u16) << 8 | self.l as u16;
                let value = mmu.read_byte(self.pc);
                mmu.write_byte(addr, value);
                self.pc += 1;
                println!("Executing LD (HL), {:#04x} at PC={:#06x}", mmu.read_byte(addr), self.pc);
                12 // LD (HL), n takes 12 cycles
            }
            0x37 => { // SCF
                self.f = 0x10;
                println!("Executing SCF at PC={:#06x}", self.pc);
                4 // SCF takes 4 cycles
            }
            0x38 => { // JR C, n
                let offset = mmu.read_byte(self.pc) as i8;
                self.pc = self.pc.wrapping_add(1); // Increment PC to skip the offset byte
                
                // Carry flag is bit 4 of register F
                if self.f & 0x10 != 0 {
                    // Condition is true (C - Carry), perform the jump
                    self.pc = self.pc.wrapping_add(offset as u16);
                    println!("Executing JR C, {:#04x} at PC={:#06x}", offset, self.pc);
                    12 // 12 cycles when branch is taken
                } else {
                    // Condition is false (NC - No Carry), don't jump
                    println!("Executing JR C, {:#04x} at PC={:#06x} (not taken)", offset, self.pc);
                    8  // 8 cycles when branch is not taken
                }
            }
            0x39 => { // ADD HL, SP
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add(self.sp);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing ADD HL, SP at PC={:#06x}", self.pc);
                8 // ADD HL, SP takes 8 cycles
            }
            0x3A => { // LD A, (HL-)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a = mmu.read_byte(addr);
                let hl = addr.wrapping_sub(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing LD A, (HL-) at PC={:#06x}", self.pc);
                8 // LD A, (HL-) takes 8 cycles
            }
            0x3B => { // DEC SP
                self.sp = self.sp.wrapping_sub(1);
                println!("Executing DEC SP at PC={:#06x}", self.pc);
                8 // DEC SP takes 8 cycles
            }
            0x3C => { // INC A
                self.a = self.a.wrapping_add(1);
                println!("Executing INC A at PC={:#06x}", self.pc);
                4 // INC A takes 4 cycles
            }
            0x3D => { // DEC A
                self.a = self.a.wrapping_sub(1);
                println!("Executing DEC A at PC={:#06x}", self.pc);
                4 // DEC A takes 4 cycles
            }
            0x3E => { // LD A, n
                self.a = mmu.read_byte(self.pc);
                self.pc += 1;
                println!("Executing LD A, {:#04x} at PC={:#06x}", self.a, self.pc);
                8 //  LD A, n takes 8 cycles
            }
            0x3F => { // CCF
                self.f ^= 0x10;
                println!("Executing CCF at PC={:#06x}", self.pc);
                4 // CCF takes 4 cycles
            }
            0x40 => { // LD B, B
                println!("Executing LD B, B at PC={:#06x}", self.pc);
                4 // LD B, B takes 4 cycles
            }
            0x41 => { // LD B, C
                self.b = self.c;
                println!("Executing LD B, C at PC={:#06x}", self.pc);
                4 // LD B, C takes 4 cycles
            }
            0x42 => { // LD B, D
                self.b = self.d;
                println!("Executing LD B, D at PC={:#06x}", self.pc);
                4 // LD B, D takes 4 cycles
            }
            0x43 => { // LD B, E
                self.b = self.e;
                println!("Executing LD B, E at PC={:#06x}", self.pc);
                4 // LD B, E takes 4 cycles
            }
            0x44 => { // LD B, H
                self.b = self.h;
                println!("Executing LD B, H at PC={:#06x}", self.pc);
                4 // LD B, H takes 4 cycles
            }
            0x45 => { // LD B, L
                self.b = self.l;
                println!("Executing LD B, L at PC={:#06x}", self.pc);
                4 // LD B, L takes 4 cycles
            }
            0x46 => { // LD B, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.b = mmu.read_byte(addr);
                println!("Executing LD B, (HL) at PC={:#06x}", self.pc);
                8 // LD B, (HL) takes 8 cycles
            }
            0x47 => { // LD B, A
                self.b = self.a;
                println!("Executing LD B, A at PC={:#06x}", self.pc);
                4 // LD B, A takes 4 cycles
            }
            0x48 => { // LD C, B
                self.c = self.b;
                println!("Executing LD C, B at PC={:#06x}", self.pc);
                4 // LD C, B takes 4 cycles
            }
            0x49 => { // LD C, C
                println!("Executing LD C, C at PC={:#06x}", self.pc);
                4 // LD C, C takes 4 cycles
            }
            0x4A => { // LD C, D
                self.c = self.d;
                println!("Executing LD C, D at PC={:#06x}", self.pc);
                4 // LD C, D takes 4 cycles
            }
            0x4B => { // LD C, E
                self.c = self.e;
                println!("Executing LD C, E at PC={:#06x}", self.pc);
                4 // LD C, E takes 4 cycles
            }
            0x4C => { // LD C, H
                self.c = self.h;
                println!("Executing LD C, H at PC={:#06x}", self.pc);
                4 // LD C, H takes 4 cycles
            }
            0x4D => { // LD C, L
                self.c = self.l;
                println!("Executing LD C, L at PC={:#06x}", self.pc);
                4 // LD C, L takes 4 cycles
            }
            0x4E => { // LD C, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.c = mmu.read_byte(addr);
                println!("Executing LD C, (HL) at PC={:#06x}", self.pc);
                8 // LD C, (HL) takes 8 cycles
            }
            0x4F => { // LD C, A
                self.c = self.a;
                println!("Executing LD C, A at PC={:#06x}", self.pc);
                4 // LD C, A takes 4 cycles
            }
            0x50 => { // LD D, B
                self.d = self.b;
                println!("Executing LD D, B at PC={:#06x}", self.pc);
                4 // LD C, A takes 4 cycles
            }
            0x51 => { // LD D, C
                self.d = self.c;
                println!("Executing LD D, C at PC={:#06x}", self.pc);
                4 // LD D, C takes 4 cycles
            }
            0x52 => { // LD D, D
                println!("Executing LD D, D at PC={:#06x}", self.pc);
                4 // LD D, D takes 4 cycles

            }
            0x53 => { // LD D, E
                self.d = self.e;
                println!("Executing LD D, E at PC={:#06x}", self.pc);
                4 // LD D, E takes 4 cycles
            }
            0x54 => { // LD D, H
                self.d = self.h;
                println!("Executing LD D, H at PC={:#06x}", self.pc);
                4 // LD D, H takes 4 cycles
            }
            0x55 => { // LD D, L
                self.d = self.l;
                println!("Executing LD D, L at PC={:#06x}", self.pc);
                4 // LD D, L takes 4 cycles
            }
            0x56 => { // LD D, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.d = mmu.read_byte(addr);
                println!("Executing LD D, (HL) at PC={:#06x}", self.pc);
                8 // LD D, (HL) takes 8 cycles
            }
            0x57 => { // LD D, A
                self.d = self.a;
                println!("Executing LD D, A at PC={:#06x}", self.pc);
                4 // LD D, A takes 4 cycles
            }
            0x58 => { // LD E, B
                self.e = self.b;
                println!("Executing LD E, B at PC={:#06x}", self.pc);
                4 // LD E, B takes 4 cycles
            }  
            0x59 => { // LD E, C
                self.e = self.c;
                println!("Executing LD E, C at PC={:#06x}", self.pc);
                4 // LD E, C takes 4 cycles
            }
            0x5A => { // LD E, D
                self.e = self.d;
                println!("Executing LD E, D at PC={:#06x}", self.pc);
                4 // LD E, D takes 4 cycles
            }
            0x5B => { // LD E, E
                println!("Executing LD E, E at PC={:#06x}", self.pc);
                4 // LD E, E takes 4 cycles
            }
            0x5C => { // LD E, H
                self.e = self.h;
                println!("Executing LD E, H at PC={:#06x}", self.pc);
                4 // LD E, H takes 4 cycles
            }
            0x5D => { // LD E, L
                self.e = self.l;
                println!("Executing LD E, L at PC={:#06x}", self.pc);
                4 // LD E, L takes 4 cycles
            }
            0x5E => { // LD E, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.e = mmu.read_byte(addr);
                println!("Executing LD E, (HL) at PC={:#06x}", self.pc);
                8 // LD E, (HL) takes 8 cycles
            }
            0x5F => { // LD E, A
                self.e = self.a;
                println!("Executing LD E, A at PC={:#06x}", self.pc);
                4 // LD E, A takes 4 cycles
            }
            0x60 => { // LD H, B
                self.h = self.b;
                println!("Executing LD H, B at PC={:#06x}", self.pc);
                4 // LD H, B takes 4 cycles
            }
            0x61 => { // LD H, C
                self.h = self.c;
                println!("Executing LD H, C at PC={:#06x}", self.pc);
                4 // LD H, C takes 4 cycles
            }
            0x62 => { // LD H, D
                self.h = self.d;
                println!("Executing LD H, D at PC={:#06x}", self.pc);
                4 // LD H, D takes 4 cycles
            }
            0x63 => { // LD H, E
                self.h = self.e;
                println!("Executing LD H, E at PC={:#06x}", self.pc);
                4 // LD H, E takes 4 cycles
            }
            0x64 => { // LD H, H
                println!("Executing LD H, H at PC={:#06x}", self.pc);
                4 // LD H, H takes 4 cycles
            }
            0x65 => { // LD H, L
                self.h = self.l;
                println!("Executing LD H, L at PC={:#06x}", self.pc);
                4 // LD H, L takes 4 cycles
            }
            0x66 => { // LD H, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.h = mmu.read_byte(addr);
                println!("Executing LD H, (HL) at PC={:#06x}", self.pc);
                8 // LD H, (HL) takes 8 cycles
            }
            0x67 => { // LD H, A
                self.h = self.a;
                println!("Executing LD H, A at PC={:#06x}", self.pc);
                4 // LD H, A takes 4 cycles
            } 
            0x68 => { // LD L, B
                self.l = self.b;
                println!("Executing LD L, B at PC={:#06x}", self.pc);
                4 // LD L, B takes 4 cycles
            }
            0x69 => { // LD L, C
                self.l = self.c;
                println!("Executing LD L, C at PC={:#06x}", self.pc);
                4 // LD L, C takes 4 cycles
            }
            0x6A => { // LD L, D
                self.l = self.d;
                println!("Executing LD L, D at PC={:#06x}", self.pc);
                4 // LD L, D takes 4 cycles
            }
            0x6B => { // LD L, E
                self.l = self.e;
                println!("Executing LD L, E at PC={:#06x}", self.pc);
                4 // LD L, E takes 4 cycles
            }
            0x6C => { // LD L, H
                self.l = self.h;
                println!("Executing LD L, H at PC={:#06x}", self.pc);
                4 // LD L, H takes 4 cycles
            }
            0x6D => { // LD L, L
                println!("Executing LD L, L at PC={:#06x}", self.pc);
                4 // LD L. L takes 4 cycles
            }
            0x6E => { // LD L, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.l = mmu.read_byte(addr);
                println!("Executing LD L, (HL) at PC={:#06x}", self.pc);
                8 // LD L, (HL) takes 8 cycles
            }
            0x6F => { // LD L, A
                self.l = self.a;
                println!("Executing LD L, A at PC={:#06x}", self.pc);
                4 // LD L, A takes 4 cycles
            }
            0x70 => { // LD (HL), B
                let addr = (self.h as u16) << 8 | self.l as u16;
                mmu.write_byte(addr, self.b);
                println!("Executing LD (HL), B at PC={:#06x}", self.pc);
                8 // LD (HL), B takes 8 cycles
            }
            0x71 => { // LD (HL), C
                let addr = (self.h as u16) << 8 | self.l as u16;
                mmu.write_byte(addr, self.c);
                println!("Executing LD (HL), C at PC={:#06x}", self.pc);
                8 // LD (HL), C takes 8 cycles
            }
            0x72 => { // LD (HL), D
                let addr = (self.h as u16) << 8 | self.l as u16;
                mmu.write_byte(addr, self.d);
                println!("Executing LD (HL), D at PC={:#06x}", self.pc);
                8 // LD (HL), D takes 8 cycles
            }
            0x73 => { // LD (HL), E
                let addr = (self.h as u16) << 8 | self.l as u16;
                mmu.write_byte(addr, self.e);
                println!("Executing LD (HL), E at PC={:#06x}", self.pc);
                8 // LD (HL), E takes 8 cycles
            }
            0x74 => { // LD (HL), H
                let addr = (self.h as u16) << 8 | self.l as u16;
                mmu.write_byte(addr, self.h);
                println!("Executing LD (HL), H at PC={:#06x}", self.pc);
                8 // LD (HL), H takes 8 cycles
            }
            0x75 => { // LD (HL), L
                let addr = (self.h as u16) << 8 | self.l as u16;
                mmu.write_byte(addr, self.l);
                println!("Executing LD (HL), L at PC={:#06x}", self.pc);
                8 // LD (HL), L takes 8 cycles
            }
            0x76 => { // HALT
                println!("Executing HALT at PC={:#06x}", self.pc);
                // Actually halt the CPU
                self.halted = true;
                4 // HALT takes 4 cycles
            }
            0x77 => { // LD (HL), A
                let addr = (self.h as u16) << 8 | self.l as u16;
                mmu.write_byte(addr, self.a);
                println!("Executing LD (HL), A at PC={:#06x}", self.pc);
                8 // LD (HL), A takes 8 cycles
            }
            0x78 => { // LD A, B
                self.a = self.b;
                println!("Executing LD A, B at PC={:#06x}", self.pc);
                4 // LD A, B takes 4 cycles
            }
            0x79 => { // LD A, C
                self.a = self.c;
                println!("Executing LD A, C at PC={:#06x}", self.pc);
                4 // LD A, C takes 4 cycles
            }
            0x7A => { // LD A, D
                self.a = self.d;
                println!("Executing LD A, D at PC={:#06x}", self.pc);
                4 // LD A, D takes 4 cycles
            }
            0x7B => { // LD A, E
                self.a = self.e;
                println!("Executing LD A, E at PC={:#06x}", self.pc);
                4 // LD A, E takes 4 cycles
            }
            0x7C => { // LD A, H
                self.a = self.h;
                println!("Executing LD A, H at PC={:#06x}", self.pc);
                4 // LD A, H takes 4 cycles
            }
            0x7D => { // LD A, L
                self.a = self.l;
                println!("Executing LD A, L at PC={:#06x}", self.pc);
                4 // LD A, L takes 4 cycles
            }
            0x7E => { // LD A, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a = mmu.read_byte(addr);
                println!("Executing LD A, (HL) at PC={:#06x}", self.pc);
                8 // LD A, (HL) takes 8 cycles
            }
            0x7F => { // LD A, A
                println!("Executing LD A, A at PC={:#06x}", self.pc);
                4 // LD A, A takes 4 cycles
            }
            0x80 => { // ADD A, B
                let a = self.a;
                let b = self.b;
                
                self.a = a.wrapping_add(b);
                
                self.f = 0; // Clear all flags
                if self.a == 0 { self.f |= 0x80; } // Z flag
                if (a & 0x0F) + (b & 0x0F) > 0x0F { self.f |= 0x20; } // H flag
                if (a as u16) + (b as u16) > 0xFF { self.f |= 0x10; } // C flag
                println!("Executing ADD A, B at PC={:#06x}", self.pc);
                4 // ADD A, B takes 4 cycles
            }
            0x81 => { // ADD A, C
                let a = self.a;
                let c = self.c;
                
                self.a = a.wrapping_add(c);
                
                self.f = 0; // Clear all flags
                if self.a == 0 { self.f |= 0x80; } // Z flag
                if (a & 0x0F) + (c & 0x0F) > 0x0F { self.f |= 0x20; } // H flag
                if (a as u16) + (c as u16) > 0xFF { self.f |= 0x10; } // C flag
                println!("Executing ADD A, C at PC={:#06x}", self.pc);
                4 // ADD A, C takes 4 cycles
            }
            0x82 => { // ADD A, D
                let a = self.a;
                let d = self.d;
                
                self.a = a.wrapping_add(d);
                
                self.f = 0; // Clear all flags
                if self.a == 0 { self.f |= 0x80; } // Z flag
                if (a & 0x0F) + (d & 0x0F) > 0x0F { self.f |= 0x20; } // H flag
                if (a as u16) + (d as u16) > 0xFF { self.f |= 0x10; } // C flag
                println!("Executing ADD A, D at PC={:#06x}", self.pc);
                4 // ADD A, D takes 4 cycles
            }
            0x83 => { // ADD A, E
                let a = self.a;
                let e = self.e;
                
                self.a = a.wrapping_add(e);
                
                self.f = 0; // Clear all flags
                if self.a == 0 { self.f |= 0x80; } // Z flag
                if (a & 0x0F) + (e & 0x0F) > 0x0F { self.f |= 0x20; } // H flag
                if (a as u16) + (e as u16) > 0xFF { self.f |= 0x10; } // C flag
                println!("Executing ADD A, E at PC={:#06x}", self.pc);
                4 // ADD A, E takes 4 cycles
            }
            0x84 => { // ADD A, H
                let a = self.a;
                let h = self.h;
                
                self.a = a.wrapping_add(h);
                
                self.f = 0; // Clear all flags
                if self.a == 0 { self.f |= 0x80; } // Z flag
                if (a & 0x0F) + (h & 0x0F) > 0x0F { self.f |= 0x20; } // H flag
                if (a as u16) + (h as u16) > 0xFF { self.f |= 0x10; } // C flag
                println!("Executing ADD A, H at PC={:#06x}", self.pc);
                4 // ADD A, H takes 4 cycles
            }
            0x85 => { // ADD A, L
                let a = self.a;
                let l = self.l;
                
                self.a = a.wrapping_add(l);
                
                self.f = 0; // Clear all flags
                if self.a == 0 { self.f |= 0x80; } // Z flag
                if (a & 0x0F) + (l & 0x0F) > 0x0F { self.f |= 0x20; } // H flag
                if (a as u16) + (l as u16) > 0xFF { self.f |= 0x10; } // C flag
                println!("Executing ADD A, L at PC={:#06x}", self.pc);
                4 // ADD A, L takes 4 cycles
            }
            0x86 => { // ADD A, (HL)
                let a = self.a;
                let addr = (self.h as u16) << 8 | self.l as u16;
                let value = mmu.read_byte(addr);
                
                self.a = a.wrapping_add(value);
                
                self.f = 0; // Clear all flags
                if self.a == 0 { self.f |= 0x80; } // Z flag
                if (a & 0x0F) + (value & 0x0F) > 0x0F { self.f |= 0x20; } // H flag
                if (a as u16) + (value as u16) > 0xFF { self.f |= 0x10; } // C flag
                println!("Executing ADD A, (HL) at PC={:#06x}", self.pc);
                8 // ADD A, (HL) takes 8 cycles
            }
            0x87 => { // ADD A, A
                let a = self.a;
    
                self.a = a.wrapping_add(a);
                
                self.f = 0; // Clear all flags
                if self.a == 0 { self.f |= 0x80; } // Z flag
                if (a & 0x0F) + (a & 0x0F) > 0x0F { self.f |= 0x20; } // H flag
                if (a as u16) + (a as u16) > 0xFF { self.f |= 0x10; } // C flag;
                println!("Executing ADD A, A at PC={:#06x}", self.pc);
                4 // ADD A, A takes 4 cycles
            }
            0x88 => { // ADC A, B
                let a = self.a;
                let b = self.b;
                let carry = (self.f & 0x10) >> 4; // Get current carry flag
                
                // Perform addition with carry
                let result = a.wrapping_add(b).wrapping_add(carry);
                
                // Set flags
                self.f = 0; // Clear all flags
                if result == 0 { self.f |= 0x80; } // Z flag
                
                // H flag - carry from bit 3 to bit 4
                if (a & 0x0F) + (b & 0x0F) + carry > 0x0F { self.f |= 0x20;}
                
                // C flag - carry from bit 7
                if (a as u16) + (b as u16) + (carry as u16) > 0xFF { self.f |= 0x10;}
                
                self.a = result;
                println!("Executing ADC A, B at PC={:#06x}", self.pc);
                4 // ADD A, C takes 4 cycles
            }
            0x89 => { // ADC A, C
                let a = self.a;
                let c = self.c;
                let carry = (self.f & 0x10) >> 4;
                
                let result = a.wrapping_add(c).wrapping_add(carry);
                
                self.f = 0;
                if result == 0 { self.f |= 0x80; }
                if (a & 0x0F) + (c & 0x0F) + carry > 0x0F { self.f |= 0x20; }
                if (a as u16) + (c as u16) + (carry as u16) > 0xFF { self.f |= 0x10; }
                
                self.a = result;
                println!("Executing ADC A, C at PC={:#06x}", self.pc);
                4 // ADC A, C takes 4 cycles
            }
            0x8A => { // ADC A, D
                let a = self.a;
                let d = self.d;
                let carry = (self.f & 0x10) >> 4;
                
                let result = a.wrapping_add(d).wrapping_add(carry);
                
                self.f = 0;
                if result == 0 { self.f |= 0x80; }
                if (a & 0x0F) + (d & 0x0F) + carry > 0x0F { self.f |= 0x20; }
                if (a as u16) + (d as u16) + (carry as u16) > 0xFF { self.f |= 0x10; }
                
                self.a = result;
                println!("Executing ADC A, D at PC={:#06x}", self.pc);
                4 // ADC A, D takes 4 cycles
            }
            0x8B => { // ADC A, E
                let a = self.a;
                let e = self.e;
                let carry = (self.f & 0x10) >> 4;
                
                let result = a.wrapping_add(e).wrapping_add(carry);
                
                self.f = 0;
                if result == 0 { self.f |= 0x80; }
                if (a & 0x0F) + (e & 0x0F) + carry > 0x0F { self.f |= 0x20; }
                if (a as u16) + (e as u16) + (carry as u16) > 0xFF { self.f |= 0x10; }
                
                self.a = result;
                println!("Executing ADC A, E at PC={:#06x}", self.pc);
                4 // ADC A, E takes 4 cycles
            }  
            0x8C => { // ADC A, H
                let a = self.a;
                let h = self.h;
                let carry = (self.f & 0x10) >> 4;
                
                let result = a.wrapping_add(h).wrapping_add(carry);
                
                self.f = 0;
                if result == 0 { self.f |= 0x80; }
                if (a & 0x0F) + (h & 0x0F) + carry > 0x0F { self.f |= 0x20; }
                if (a as u16) + (h as u16) + (carry as u16) > 0xFF { self.f |= 0x10; }
                
                self.a = result;
                println!("Executing ADC A, H at PC={:#06x}", self.pc);
                4 // ADC A, H takes 4 cycles
            }
            0x8D => { // ADC A, L
                let a = self.a;
                let l = self.l;
                let carry = (self.f & 0x10) >> 4;
                
                let result = a.wrapping_add(l).wrapping_add(carry);
                
                self.f = 0;
                if result == 0 { self.f |= 0x80; }
                if (a & 0x0F) + (l & 0x0F) + carry > 0x0F { self.f |= 0x20; }
                if (a as u16) + (l as u16) + (carry as u16) > 0xFF { self.f |= 0x10; }
                
                self.a = result;
                println!("Executing ADC A, L at PC={:#06x}", self.pc);
                4 // ADC A, L takes 4 cycles
            }
            0x8E => { // ADC A, (HL)
                let a = self.a;
                let addr = (self.h as u16) << 8 | self.l as u16;
                let value = mmu.read_byte(addr);
                let carry = (self.f & 0x10) >> 4;
                
                let result = a.wrapping_add(value).wrapping_add(carry);
                
                self.f = 0;
                if result == 0 { self.f |= 0x80; }
                if (a & 0x0F) + (value & 0x0F) + carry > 0x0F { self.f |= 0x20; }
                if (a as u16) + (value as u16) + (carry as u16) > 0xFF { self.f |= 0x10; }
                
                self.a = result;
                println!("Executing ADC A, (HL) at PC={:#06x}", self.pc);
                8 // ADC A, (HL) takes 8 cycles
            }
            0x8F => { // ADC A, A
                let a = self.a;
                let carry = (self.f & 0x10) >> 4;
                
                let result = a.wrapping_add(a).wrapping_add(carry);
                
                self.f = 0;
                if result == 0 { self.f |= 0x80; }
                if (a & 0x0F) + (a & 0x0F) + carry > 0x0F { self.f |= 0x20; }
                if (a as u16) + (a as u16) + (carry as u16) > 0xFF { self.f |= 0x10; }
                
                self.a = result;
                println!("Executing ADC A, A at PC={:#06x}", self.pc);
                4 // ADC A, A takes 4 cycles
            }
            0x90 => { // SUB B
                let a = self.a;
                let b = self.b;
                
                // Perform subtraction
                self.a = a.wrapping_sub(b);
                
                // Set flags
                self.f = 0x40;  // Set N flag (bit 6)
                
                // Set Z flag if result is zero
                if self.a == 0 {self.f |= 0x80;}
                
                // Set H flag if borrow from bit 4
                if (a & 0x0F) < (b & 0x0F) { self.f |= 0x20; }
                
                // Set C flag if borrow needed
                if a < b { self.f |= 0x10; }
                println!("Executing SUB B at PC={:#06x}", self.pc);
                4 // SUB B takes 4 cycles
            }
            0x91 => { // SUB C
                let a = self.a;
                let c = self.c;
                
                self.a = a.wrapping_sub(c);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < (c & 0x0F) { self.f |= 0x20; }
                if a < c { self.f |= 0x10; }
                println!("Executing SUB C at PC={:#06x}", self.pc);
                4 // SUB C takes 4 cycles
            }
            0x92 => { // SUB D
                let a = self.a;
                let d = self.d;
                
                self.a = a.wrapping_sub(d);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < (d & 0x0F) { self.f |= 0x20; }
                if a < d { self.f |= 0x10; }
                println!("Executing SUB D at PC={:#06x}", self.pc);
                4 // SUB D takes 4 cycles
            }
            0x93 => { // SUB E
                let a = self.a;
                let e = self.e;
                
                self.a = a.wrapping_sub(e);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < (e & 0x0F) { self.f |= 0x20; }
                if a < e { self.f |= 0x10; }
                println!("Executing SUB E at PC={:#06x}", self.pc);
                4 // SUB E takes 4 cycles
            }
            0x94 => { // SUB H
                let a = self.a;
                let h = self.h;
                
                self.a = a.wrapping_sub(h);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < (h & 0x0F) { self.f |= 0x20; }
                if a < h { self.f |= 0x10; }
                println!("Executing SUB H at PC={:#06x}", self.pc);
                4 // SUB E takes 4 cycles
            }
            0x95 => { // SUB L
                let a = self.a;
                let l = self.l;
                
                self.a = a.wrapping_sub(l);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < (l & 0x0F) { self.f |= 0x20; }
                if a < l { self.f |= 0x10; }
                println!("Executing SUB L at PC={:#06x}", self.pc);
                4 // SUB L takes 4 cycles
            }
            0x96 => { // SUB (HL)
                let a = self.a;
                let addr = (self.h as u16) << 8 | self.l as u16;
                let value = mmu.read_byte(addr);
                
                self.a = a.wrapping_sub(value);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < (value & 0x0F) { self.f |= 0x20; }
                if a < value { self.f |= 0x10; }
                println!("Executing SUB (HL) at PC={:#06x}", self.pc);
                8 // SUB (HL) takes 8 cycles
            }
            0x97 => { // SUB A
                let _a = self.a;
    
                self.a = 0; // A - A is always 0
                
                self.f = 0x40 | 0x80; // Set N and Z flags
                
                // No half carry or carry when subtracting a value from itself
                
                println!("Executing SUB A at PC={:#06x}", self.pc);
                4 // SUB A tales 4 cycles
            }
            0x98 => { // SBC A, B
                let a = self.a;
                let b = self.b;
                let carry = (self.f & 0x10) >> 4;
                
                // Calculate total to subtract (b + carry)
                let total_sub = b.wrapping_add(carry);
                
                // Perform subtraction
                self.a = a.wrapping_sub(total_sub);
                
                // Set flags
                self.f = 0x40; // Set N flag
                
                if self.a == 0 { self.f |= 0x80; } // Z flag
                
                // H flag - borrow from bit 4
                if (a & 0x0F) < ((b & 0x0F) + carry) { self.f |= 0x20; }
                
                // C flag - if borrow needed
                if (a as u16) < (b as u16) + (carry as u16) { self.f |= 0x10; }
                println!("Executing SBC A, B at PC={:#06x}", self.pc);
                4 // SBC A, B takes 4 cycles
            }
            0x99 => { // SBC A, C
                let a = self.a;
                let c = self.c;
                let carry = (self.f & 0x10) >> 4;
                
                let total_sub = c.wrapping_add(carry);
                
                self.a = a.wrapping_sub(total_sub);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < ((c & 0x0F) + carry) { self.f |= 0x20; }
                if (a as u16) < (c as u16) + (carry as u16) { self.f |= 0x10; }
                println!("Executing SBC A, C at PC={:#06x}", self.pc);
                4 // SBC A, C takes 4 cycles
            }
            0x9A => { // SBC A, D
                let a = self.a;
                let d = self.d;
                let carry = (self.f & 0x10) >> 4;
                
                let total_sub = d.wrapping_add(carry);
                
                self.a = a.wrapping_sub(total_sub);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < ((d & 0x0F) + carry) { self.f |= 0x20; }
                if (a as u16) < (d as u16) + (carry as u16) { self.f |= 0x10; }
                println!("Executing SBC A, D at PC={:#06x}", self.pc);
                4 // SBC A, D takes 4 cycles
            }
            0x9B => { // SBC A, E
                let a = self.a;
                let e = self.e;
                let carry = (self.f & 0x10) >> 4;
                
                let total_sub = e.wrapping_add(carry);
                
                self.a = a.wrapping_sub(total_sub);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < ((e & 0x0F) + carry) { self.f |= 0x20; }
                if (a as u16) < (e as u16) + (carry as u16) { self.f |= 0x10; }
                println!("Executing SBC A, E at PC={:#06x}", self.pc);
                4 // SBC A, E takes 4 cycles
            }
            0x9C => { // SBC A, H
                let a = self.a;
                let h = self.h;
                let carry = (self.f & 0x10) >> 4;
                
                let total_sub = h.wrapping_add(carry);
                
                self.a = a.wrapping_sub(total_sub);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < ((h & 0x0F) + carry) { self.f |= 0x20; }
                if (a as u16) < (h as u16) + (carry as u16) { self.f |= 0x10; }
                println!("Executing SBC A, H at PC={:#06x}", self.pc);
                4 // SBC A, H takes 4 cycles
            }
            0x9D => { // SBC A, L
                let a = self.a;
                let l = self.l;
                let carry = (self.f & 0x10) >> 4;
                
                let total_sub = l.wrapping_add(carry);
                
                self.a = a.wrapping_sub(total_sub);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < ((l & 0x0F) + carry) { self.f |= 0x20; }
                if (a as u16) < (l as u16) + (carry as u16) { self.f |= 0x10; }
                println!("Executing SBC A, L at PC={:#06x}", self.pc);
                4 // SBC A, L takes 4 cycles
            }
            0x9E => { // SBC A, (HL)
                let a = self.a;
                let addr = (self.h as u16) << 8 | self.l as u16;
                let value = mmu.read_byte(addr);
                let carry = (self.f & 0x10) >> 4;
                
                let total_sub = value.wrapping_add(carry);
                
                self.a = a.wrapping_sub(total_sub);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < ((value & 0x0F) + carry) { self.f |= 0x20; }
                if (a as u16) < (value as u16) + (carry as u16) { self.f |= 0x10; }
                println!("Executing SBC A, (HL) at PC={:#06x}", self.pc);
                8 // SBC A, (HL) takes 8 cycles
            }
            0x9F => { // SBC A, A
                let a = self.a;
                let carry = (self.f & 0x10) >> 4;
                
                // A - A - carry
                self.a = a.wrapping_sub(a).wrapping_sub(carry);
                
                self.f = 0x40; // Set N flag
                if self.a == 0 { self.f |= 0x80; } // Set Z flag if result is zero
                if carry == 1 
                { 
                    self.f |= 0x20; // Set H flag if there was a carry
                    self.f |= 0x10; // Set C flag if there was a carry
                }
                println!("Executing SBC A, A at PC={:#06x}", self.pc);
                4 // SBC A, A takes 4 cycles
            }
            0xA0 => { // AND B
                self.a &= self.b;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND B at PC={:#06x}", self.pc);
                4 // AND B takes 4 cycles
            }
            0xA1 => { // AND C
                self.a &= self.c;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND C at PC={:#06x}", self.pc);
                4 // AND C takes 4 cycles
            }
            0xA2 => { // AND D
                self.a &= self.d;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND D at PC={:#06x}", self.pc);
                4 // AND D takes 4 cycles
            }
            0xA3 => { // AND E
                self.a &= self.e;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND E at PC={:#06x}", self.pc);
                4 // AND E takes 4 cycles
            }
            0xA4 => { // AND H
                self.a &= self.h;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND H at PC={:#06x}", self.pc);
                4 // AND H takes 4 cycles
            }
            0xA5 => { // AND L
                self.a &= self.l;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND L at PC={:#06x}", self.pc);
                4 // AND L takes 4 cycles
            }
            0xA6 => { // AND (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a &= mmu.read_byte(addr);
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND (HL) at PC={:#06x}", self.pc);
                8 // AND (HL) takes 8 cycles
            }
            0xA7 => { // AND A
                self.a &= self.a;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND A at PC={:#06x}", self.pc);
                4 // AND A takes 4 cycles
            }
            0xA8 => { // XOR B
                self.a ^= self.b;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR B at PC={:#06x}", self.pc);
                4 // XOR B takes 4 cycles
            }
            0xA9 => { // XOR C
                self.a ^= self.c;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR C at PC={:#06x}", self.pc);
                4 // XOR C takes 4 cycles
            }
            0xAA => { // XOR D
                self.a ^= self.d;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR D at PC={:#06x}", self.pc);
                4 // XOR D takes 4 cycles
            }
            0xAB => { // XOR E
                self.a ^= self.e;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR E at PC={:#06x}", self.pc);
                4 // XOR E takes 4 cycles
            }
            0xAC => { // XOR H
                self.a ^= self.h;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR H at PC={:#06x}", self.pc);
                4 // XOR H takes 4 cycles
            }
            0xAD => { // XOR L
                self.a ^= self.l;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR L at PC={:#06x}", self.pc);
                4 // XOR L takes 4 cycles
            }
            0xAE => { // XOR (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a ^= mmu.read_byte(addr);
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR (HL) at PC={:#06x}", self.pc);
                8 // XOR (HL) takes 8 cycles
            }
            0xAF => { // XOR A
                self.a ^= self.a;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR A at PC={:#06x}", self.pc);
                4 // XOR A takes 4 cycles
            }
            0xB0 => { // OR B
                self.a |= self.b;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR B at PC={:#06x}", self.pc);
                4 // OR B takes 4 cycles
            }
            0xB1 => { // OR C
                self.a |= self.c;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR C at PC={:#06x}", self.pc);
                4 // OR C takes 4 cycles
            }
            0xB2 => { // OR D
                self.a |= self.d;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR D at PC={:#06x}", self.pc);
                4 // OR D takes 4 cycles
            }
            0xB3 => { // OR E
                self.a |= self.e;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR E at PC={:#06x}", self.pc);
                4 // OR E takes 4 cycles
            }
            0xB4 => { // OR H
                self.a |= self.h;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR H at PC={:#06x}", self.pc);
                4 // OR H takes 4 cycles
            }
            0xB5 => { // OR L
                self.a |= self.l;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR L at PC={:#06x}", self.pc);
                4 // OR L takes 4 cycles
            }
            0xB6 => { // OR (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a |= mmu.read_byte(addr);
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR (HL) at PC={:#06x}", self.pc);
                8 // OR (HL) takes 8 cycles
            }
            0xB7 => { // OR A
                self.a |= self.a;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR A at PC={:#06x}", self.pc);
                4 // OR A takes 4 cycles
            }
            0xB8 => { // CP B
                let (result, carry) = self.a.overflowing_sub(self.b);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP B at PC={:#06x}", self.pc);
                4 // CP B takes 4 cycles
            }
            0xB9 => { // CP C
                let (result, carry) = self.a.overflowing_sub(self.c);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP C at PC={:#06x}", self.pc);
                4 // CP C takes 4 cycles
            }
            0xBA => { // CP D
                let (result, carry) = self.a.overflowing_sub(self.d);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP D at PC={:#06x}", self.pc);
                4 // CP D takes 4 cycles
            }
            0xBB => { // CP E
                let (result, carry) = self.a.overflowing_sub(self.e);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP E at PC={:#06x}", self.pc);
                4 // CP E takes 4 cycles
            }
            0xBC => { // CP H
                let (result, carry) = self.a.overflowing_sub(self.h);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP H at PC={:#06x}", self.pc);
                4 // CP H takes 4 cycles
            }
            0xBD => { // CP L
                let (result, carry) = self.a.overflowing_sub(self.l);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP L at PC={:#06x}", self.pc);
                4 // CP L takes 4 cycles
            }
            0xBE => { // CP (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let (result, carry) = self.a.overflowing_sub(mmu.read_byte(addr));
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP (HL) at PC={:#06x}", self.pc);
                8 // CP (HL) takes 8 cycles
            }
            0xBF => { // CP A
                let (result, carry) = self.a.overflowing_sub(self.a);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP A at PC={:#06x}", self.pc);
                4 // CP A takes 4 cycles
            }
            0xC0 => { // RET NZ
                if self.f & 0x80 == 0 
                {
                    // Not zero, so return
                    let lo = self.pop(mmu);
                    let hi = self.pop(mmu);
                    self.pc = (hi as u16) << 8 | lo as u16;
                    
                    println!("Executing RET NZ at PC={:#06x} (taken)", self.pc);
                    20 // 20 cycles when return is taken
                } 
                else 
                {
                    println!("Executing RET NZ at PC={:#06x} (not taken)", self.pc);
                    8  // 8 cycles when return is not taken
                }
            }
            0xC1 => { // POP BC
                self.c = self.pop(mmu);
                self.b = self.pop(mmu);
                println!("Executing POP BC at PC={:#06x}", self.pc);
                12 // POP BC takes 12 cycles
            }
            0xC2 => { // JP NZ, nn
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                
                if self.f & 0x80 == 0 
                {
                    // Not zero, so jump
                    self.pc = (hi as u16) << 8 | lo as u16;
                    println!("Executing JP NZ, nn at PC={:#06x} (taken)", self.pc);
                    16 // 16 cycles when jump is taken
                } 
                else 
                {
                    println!("Executing JP NZ, nn at PC={:#06x} (not taken)", self.pc);
                    12 // 12 cycles when jump is not taken
                }
            }
            0xC3 => { // JP nn
                 // Get low byte first (little endian)
                 let low = mmu.read_byte(self.pc);
                 let high = mmu.read_byte(self.pc + 1);
                 self.pc = ((high as u16) << 8) | (low as u16);
                 println!("Executing JP nn at PC={:#06x}", self.pc);
                 16 // JP nn takes 16 cycles
            }
            0xC4 => { // CALL NZ, nn
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                
                if self.f & 0x80 == 0 
                {
                    // Not zero, so call
                    self.push(mmu, (self.pc >> 8) as u8); // Push high byte of PC
                    self.push(mmu, self.pc as u8);        // Push low byte of PC
                    self.pc = (hi as u16) << 8 | lo as u16;
                    println!("Executing CALL NZ, nn at PC={:#06x} (taken)", self.pc);
                    24 // 24 cycles when call is taken
                } 
                else 
                {
                    println!("Executing CALL NZ, nn at PC={:#06x} (not taken)", self.pc);
                    12 // 12 cycles when call is not taken
                }
            }
            0xC5 => { // PUSH BC
                self.push(mmu, self.b);
                self.push(mmu, self.c);
                println!("Executing PUSH BC at PC={:#06x}", self.pc);
                16 // PUSH BC takes 16 cycles
            }
            0xC6 => { // ADD A, n
                let a = self.a;
                let n = self.fetch(mmu);
                
                self.a = a.wrapping_add(n);
                
                self.f = 0; // Clear all flags
                if self.a == 0 { self.f |= 0x80; } // Z flag
                if (a & 0x0F) + (n & 0x0F) > 0x0F { self.f |= 0x20; } // H flag
                if (a as u16) + (n as u16) > 0xFF { self.f |= 0x10; } // C flag
                println!("Executing ADD A, n at PC={:#06x}", self.pc);
                8 // ADD A, n takes 8 cycles
            }
            0xC7 => { // RST 00H
                self.push(mmu, (self.pc >> 8) as u8);
                self.push(mmu, self.pc as u8);
                self.pc = 0x00;
                println!("Executing RST 00H at PC={:#06x}", self.pc);
                16 // RST 00H takes 16 cycles
            }
            0xC8 => { // RET Z
                if self.f & 0x80 != 0 
                {
                    // Zero flag is set, so return
                    let lo = self.pop(mmu);
                    let hi = self.pop(mmu);
                    self.pc = (hi as u16) << 8 | lo as u16;
                    
                    println!("Executing RET Z at PC={:#06x} (taken)", self.pc);
                    20 // 20 cycles when return is taken
                } 
                else 
                {
                    println!("Executing RET Z at PC={:#06x} (not taken)", self.pc);
                    8  // 8 cycles when return is not taken
                }
            }
            0xC9 => { // RET
                let lo = self.pop(mmu);
                let hi = self.pop(mmu);
                self.pc = (hi as u16) << 8 | lo as u16;
                println!("Executing RET at PC={:#06x}", self.pc);
                16 // RET takes 16 cycles
            }
            0xCA => { // JP Z, nn
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                
                if self.f & 0x80 != 0 
                {
                    // Zero flag is set, so jump
                    self.pc = (hi as u16) << 8 | lo as u16;
                    println!("Executing JP Z, nn at PC={:#06x} (taken)", self.pc);
                    16 // 16 cycles when jump is taken
                } 
                else 
                {
                    println!("Executing JP Z, nn at PC={:#06x} (not taken)", self.pc);
                    12 // 12 cycles when jump is not taken
                }
            }
            0xCB => { // Prefix CB
                // Fetch the next byte after CB using fetch()
                let cb_opcode = self.fetch(mmu);  // This will get the byte and increment PC
                
                if self.debug_mode { println!("Executing CB opcode {:#04x} at PC={:#06x}", cb_opcode, self.pc - 1); }
                
                // Execute the CB-prefixed instruction
                let cb_cycles = self.execute_cb(cb_opcode, mmu);
                
                // The total cycles is 4 (for CB prefix) plus whatever cycles the CB instruction takes
                4 + cb_cycles // Return the combined cycle count
            }
            0xCC => { // CALL Z, nn
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                
                if self.f & 0x80 != 0 
                {
                    // Zero flag is set, so call
                    self.push(mmu, (self.pc >> 8) as u8); // Push high byte of PC
                    self.push(mmu, self.pc as u8);        // Push low byte of PC
                    self.pc = (hi as u16) << 8 | lo as u16;
                    println!("Executing CALL Z, nn at PC={:#06x} (taken)", self.pc);
                    24 // 24 cycles when call is taken
                } 
                else 
                {
                    println!("Executing CALL Z, nn at PC={:#06x} (not taken)", self.pc);
                    12 // 12 cycles when call is not taken
                }
            }
            0xCD => { // CALL nn
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                self.push(mmu, (self.pc >> 8) as u8);
                self.push(mmu, self.pc as u8);
                self.pc = (hi as u16) << 8 | lo as u16;
                println!("Executing CALL nn at PC={:#06x}", self.pc);
                24 // CALL nn takes 24 cycles
            }
            0xCE => { // ADC A, n
                let a = self.a;
                let n = self.fetch(mmu);
                let carry = (self.f & 0x10) >> 4;
                
                let result = a.wrapping_add(n).wrapping_add(carry);
                
                self.f = 0;
                if result == 0 { self.f |= 0x80; }
                if (a & 0x0F) + (n & 0x0F) + carry > 0x0F { self.f |= 0x20; }
                if (a as u16) + (n as u16) + (carry as u16) > 0xFF { self.f |= 0x10; }
                
                self.a = result;
                println!("Executing ADC A, n at PC={:#06x}", self.pc);
                8 // ADC A, n takes 8 cycles
            }
            0xCF => { // RST 08H
                self.push(mmu, (self.pc >> 8) as u8);
                self.push(mmu, self.pc as u8);
                self.pc = 0x08;
                println!("Executing RST 08H at PC={:#06x}", self.pc);
                16 // RST 08H takes 16 cycles
            }
            0xD0 => { // RET NC
                if self.f & 0x10 == 0 
                {
                    // Carry flag is not set, so return
                    let lo = self.pop(mmu);
                    let hi = self.pop(mmu);
                    self.pc = (hi as u16) << 8 | lo as u16;
                    
                    println!("Executing RET NC at PC={:#06x} (taken)", self.pc);
                    20 // 20 cycles when return is taken
                } 
                else 
                {
                    println!("Executing RET NC at PC={:#06x} (not taken)", self.pc);
                    8  // 8 cycles when return is not taken
                }
            }
            0xD1 => { // POP DE
                self.e = self.pop(mmu);
                self.d = self.pop(mmu);
                println!("Executing POP DE at PC={:#06x}", self.pc);
                12 // POP DE takes 12 cycles
            }
            0xD2 => { // JP NC, nn
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                
                if self.f & 0x10 == 0 
                {
                    // Carry flag is not set, so jump
                    self.pc = (hi as u16) << 8 | lo as u16;
                    println!("Executing JP NC, nn at PC={:#06x} (taken)", self.pc);
                    16 // 16 cycles when jump is taken
                } 
                else 
                {
                    println!("Executing JP NC, nn at PC={:#06x} (not taken)", self.pc);
                    12 // 12 cycles when jump is not taken
                }
            }
            0xD4 => { // CALL NC, nn
                    let lo = self.fetch(mmu);
                    let hi = self.fetch(mmu);
                    
                    if self.f & 0x10 == 0 
                    {
                        // Carry flag is not set, so call
                        self.push(mmu, (self.pc >> 8) as u8); // Push high byte of PC
                        self.push(mmu, self.pc as u8);        // Push low byte of PC
                        self.pc = (hi as u16) << 8 | lo as u16;
                        println!("Executing CALL NC, nn at PC={:#06x} (taken)", self.pc);
                        24 // 24 cycles when call is taken
                    } 
                    else 
                    {
                        println!("Executing CALL NC, nn at PC={:#06x} (not taken)", self.pc);
                        12 // 12 cycles when call is not taken
                    }
            }
            0xD5 => { // PUSH DE
                self.push(mmu, self.d);
                self.push(mmu, self.e);
                println!("Executing PUSH DE at PC={:#06x}", self.pc);
                16 // PUSH DE takes 16 cycles
            }
            0xD6 => { // SUB n
                let n = self.fetch(mmu);
                let a = self.a;
                
                // Perform subtraction
                self.a = a.wrapping_sub(n);
                
                // Set flags
                self.f = 0x40;  // Set N flag (bit 6)
                
                // Set Z flag if result is zero
                if self.a == 0 { self.f |= 0x80; }
                
                // Set H flag if borrow from bit 4
                if (a & 0x0F) < (n & 0x0F) { self.f |= 0x20; }
                
                // Set C flag if borrow needed
                if a < n 
                { 
                    self.f |= 0x10; 
                    println!("Setting carry flag because {} < {}", a, n);
                }

                if a < 0x06 
                {
                    println!("CRITICAL: A={:#04x} < n={:#04x}, this should set carry!", a, n);
                    println!("         After SUB: A={:#04x}, F={:#04x}", self.a, self.f);
                }
                
                println!("SUB n: A={:#04x}, n={:#04x}, result={:#04x}, a<n={}, half_borrow={}, F={:#04x}", 
                        a, n, self.a, a < n, (a & 0x0F) < (n & 0x0F), self.f);

                8 // SUB N takes 8 cycles
            }
            0xD7 => { // RST 10H
                self.push(mmu, (self.pc >> 8) as u8);
                self.push(mmu, self.pc as u8);
                self.pc = 0x10;
                println!("Executing RST 10H at PC={:#06x}", self.pc);
                16 // RST 19H takes 16 cycles
            }
            0xD8 => { // RET C
                if self.f & 0x10 != 0 
                {
                    // Carry flag is set, so return
                    let lo = self.pop(mmu);
                    let hi = self.pop(mmu);
                    self.pc = (hi as u16) << 8 | lo as u16;
                    
                    println!("Executing RET C at PC={:#06x} (taken)", self.pc);
                    20 // 20 cycles when return is taken
                } 
                else 
                {
                    println!("Executing RET C at PC={:#06x} (not taken)", self.pc);
                    8  // 8 cycles when return is not taken
                }
            }
            0xD9 => { // RETI
                let lo = self.pop(mmu);
                let hi = self.pop(mmu);
                self.pc = (hi as u16) << 8 | lo as u16;
                println!("Executing RETI at PC={:#06x}", self.pc);
                16 // RETI takes 16 cycles
            }
            0xDA => { // JP C, nn
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                
                if self.f & 0x10 != 0 
                {
                    // Carry flag is set, so jump
                    self.pc = (hi as u16) << 8 | lo as u16;
                    println!("Executing JP C, nn at PC={:#06x} (taken)", self.pc);
                    16 // 16 cycles when jump is taken
                } 
                else 
                {
                    println!("Executing JP C, nn at PC={:#06x} (not taken)", self.pc);
                    12 // 12 cycles when jump is not taken
                }
            }
            0xDC => { // CALL C, nn
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                
                if self.f & 0x10 != 0 
                {
                    // Carry flag is set, so call
                    self.push(mmu, (self.pc >> 8) as u8); // Push high byte of PC
                    self.push(mmu, self.pc as u8);        // Push low byte of PC
                    self.pc = (hi as u16) << 8 | lo as u16;
                    println!("Executing CALL C, nn at PC={:#06x} (taken)", self.pc);
                    24 // 24 cycles when call is taken
                } 
                else 
                {
                    println!("Executing CALL C, nn at PC={:#06x} (not taken)", self.pc);
                    12 // 12 cycles when call is not taken
                }
            }
            0xDE => { // SBC A, n
                let a = self.a;
                let n = self.fetch(mmu);
                let carry = (self.f & 0x10) >> 4;
                
                let total_sub = n.wrapping_add(carry);
                
                self.a = a.wrapping_sub(total_sub);
                
                self.f = 0x40;
                if self.a == 0 { self.f |= 0x80; }
                if (a & 0x0F) < ((n & 0x0F) + carry) { self.f |= 0x20; }
                if (a as u16) < (n as u16) + (carry as u16) { self.f |= 0x10; }
                println!("Executing SBC A, n at PC={:#06x}", self.pc);
                8 // SBC A, n takes 8 cycles
            }
            0xDF => { // RST 18H
                self.push(mmu, (self.pc >> 8) as u8);
                self.push(mmu, self.pc as u8);
                self.pc = 0x18;
                println!("Executing RST 18H at PC={:#06x}", self.pc);
                16 // RST 18H takes 16  cycles
            }
            0xE0 => { // LDH (n), A
                let n = self.fetch(mmu);
                mmu.write_byte(0xFF00 + n as u16, self.a);
                println!("Executing LDH (n), A at PC={:#06x}", self.pc);
                12 // LDH (n), A takes 12 cycles
            }
            0xE1 => { // POP HL
                self.l = self.pop(mmu);
                self.h = self.pop(mmu);
                println!("Executing POP HL at PC={:#06x}", self.pc);
                12 // POP HL takes 12 cycles
            }
            0xE2 => { // LD (C), A
                mmu.write_byte(0xFF00 + self.c as u16, self.a);
                println!("Executing LD (C), A at PC={:#06x}", self.pc);
                8 // LD (C), A takes 8 cycles
            }
            0xE5 => { // PUSH HL
                self.push(mmu, self.h);
                self.push(mmu, self.l);
                println!("Executing PUSH HL at PC={:#06x}", self.pc);
                16 // PUSH HL takes 16 cycles
            }
            0xE6 => { // AND n
                let n = self.fetch(mmu);
                self.a &= n;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND n at PC={:#06x}", self.pc);
                8 // AND n takes 8 cycles
            }
            0xE7 => { // RST 20H
                self.push(mmu, (self.pc >> 8) as u8);
                self.push(mmu, self.pc as u8);
                self.pc = 0x20;
                println!("Executing RST 20H at PC={:#06x}", self.pc);
                16 // RST 20H takes 16 cycles
            }
            0xE8 => { // ADD SP, d
                let n = self.fetch(mmu) as i8 as i16 as u16;
                let result = self.sp.wrapping_add(n);
                self.f = if (self.sp & 0xFF) + (n & 0xFF) > 0xFF { 0x10 } else { 0 } | if (self.sp & 0x0F) + (n & 0x0F) > 0x0F { 0x20 } else { 0 };
                self.sp = result;
                println!("Executing ADD SP, d at PC={:#06x}", self.pc);
                16 // ADD SP, d takes 16 cycles
            }
            0xE9 => { // JP (HL)
                self.pc = (self.h as u16) << 8 | self.l as u16;
                println!("Executing JP (HL) at PC={:#06x}", self.pc);
                4 // JP (HL) takes 4 cycles
            }
            0xEA => { // LD (nn), A
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                let addr = (hi as u16) << 8 | lo as u16;
                mmu.write_byte(addr, self.a);
                println!("Executing LD (nn), A at PC={:#06x}", self.pc);
                16 // LD (nn), A takes 16 cycles
            }
            0xEE => { // XOR n
                let n = self.fetch(mmu);
                self.a ^= n;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR n at PC={:#06x}", self.pc);
                8 // XOR n takes 8 cycles
            }
            0xEF => { // RST 28H
                self.push(mmu, (self.pc >> 8) as u8);
                self.push(mmu, self.pc as u8);
                self.pc = 0x28;
                println!("Executing RST 28H at PC={:#06x}", self.pc);
                16 // RST 28H takes 16 cycles
            }
            0xF0 => { // LDH A, (n)
                let n = self.fetch(mmu);
                self.a = mmu.read_byte(0xFF00 + n as u16);
                println!("Executing LDH A, (n) at PC={:#06x}", self.pc);
                12 // LDH A, (n) takes 16 cycles
            }
            0xF1 => { // POP AF
                self.f = self.pop(mmu);
                self.a = self.pop(mmu);
                println!("Executing POP AF at PC={:#06x}", self.pc);
                12 // POP AF takes 12 cycles
            }
            0xF2 => { // LD A, (C)
                self.a = mmu.read_byte(0xFF00 + self.c as u16);
                println!("Executing LD A, (C) at PC={:#06x}", self.pc);
                8 // LD A, (C) takes 8 cycles
            }
            0xF3 => { // DI
                self.ime = false;
                println!("Executing DI at PC={:#06x}", self.pc);
                4 // DI takes 4 cycles
            }
            0xF5 => { // PUSH AF
                self.push(mmu, self.a);
                self.push(mmu, self.f);
                println!("Executing PUSH AF at PC={:#06x}", self.pc);
                16 // PUSH AF takes 16 cycles
            }
            0xF6 => { // OR n
                let n = self.fetch(mmu);
                self.a |= n;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR n at PC={:#06x}", self.pc);
                8 // OR n takes 8 cycles
            }
            0xF7 => { // RST 30H
                self.push(mmu, (self.pc >> 8) as u8);
                self.push(mmu, self.pc as u8);
                self.pc = 0x30;
                println!("Executing RST 30H at PC={:#06x}", self.pc);
                16 // RST 30H takes 8 cycles
            }
            0xF8 => { // LD HL, SP+n
                let n = self.fetch(mmu) as i8 as i16 as u16;
                let result = self.sp.wrapping_add(n);
                self.f = if (self.sp & 0xFF) + (n & 0xFF) > 0xFF { 0x10 } else { 0 } | if (self.sp & 0x0F) + (n & 0x0F) > 0x0F { 0x20 } else { 0 };
                self.l = result as u8;
                self.h = (result >> 8) as u8;
                println!("Executing LD HL, SP+d at PC={:#06x}", self.pc);
                12 // LD HL, SP+n takes 12 cycles

            }
            0xF9 => { // LD SP, HL
                self.sp = (self.h as u16) << 8 | self.l as u16;
                println!("Executing LD SP, HL at PC={:#06x}", self.pc);
                8 // LD SP, HL takes 9 cycles
            }
            0xFA => { // LD A, (nn)
                let lo = self.fetch(mmu);
                let hi = self.fetch(mmu);
                let addr = (hi as u16) << 8 | lo as u16;
                self.a = mmu.read_byte(addr);
                println!("Executing LD A, (nn) at PC={:#06x}", self.pc);
                16 // LD A, (nn) takes 16 cycles
            }
            0xFB => { // EI
                self.ime = true;
                println!("Executing EI at PC={:#06x}", self.pc);
                4 // EI takes 4 cycles
            }
            0xFE => { // CP n
                let n = self.fetch(mmu);
    
                // Check if borrow needed (for carry flag)
                let borrow = self.a < n;
                
                // Calculate result without changing A
                let result = self.a.wrapping_sub(n);
                
                // Set flags
                self.f = 0;
                if result == 0 { self.f |= 0x80; }  // Z flag
                self.f |= 0x40;                      // N flag always set for subtraction
                if borrow { self.f |= 0x10; }        // C flag if borrow needed
                
                println!("Executing CP n at PC={:#06x}", self.pc);
                8 // CP n takes 8 cycles
            }
            0xFF => { // RST 38H
                self.push(mmu, (self.pc >> 8) as u8);
                self.push(mmu, self.pc as u8);
                self.pc = 0x38;
                println!("Executing RST 38H at PC={:#06x}", self.pc);
                16 // RST 38H takes 16 cycles
            }
            _ => {
                panic!("Unknown opcode: {:#04x} at PC={:#06x}", opcode,self.pc);
            }
        }
    }

    pub fn execute_cb(&mut self, cb_opcode: u8, mmu: &mut MMU) -> u8 
    {
        println!("Executing CB opcode: {:#04x}, PC: {:#06x}", cb_opcode, self.pc);
        let cycles = match cb_opcode 
        {
            // RLC - Rotate Left Circular
            0x00..=0x07 => {
                let reg = cb_opcode & 0x07;
                self.rlc_r(reg, mmu)
            },
            
            // RRC - Rotate Right Circular
            0x08..=0x0F => {
                let reg = cb_opcode & 0x07;
                self.rrc_r(reg, mmu)
            },
            
            // RL - Rotate Left through Carry
            0x10..=0x17 => {
                let reg = cb_opcode & 0x07;
                self.rl_r(reg, mmu)
            },
            
            // RR - Rotate Right through Carry
            0x18..=0x1F => {
                let reg = cb_opcode & 0x07;
                self.rr_r(reg, mmu)
            },
            
            // SLA - Shift Left Arithmetic
            0x20..=0x27 => {
                let reg = cb_opcode & 0x07;
                self.sla_r(reg, mmu)
            },
            
            // SRA - Shift Right Arithmetic
            0x28..=0x2F => {
                let reg = cb_opcode & 0x07;
                self.sra_r(reg, mmu)
            },
            
            // SWAP - Swap nibbles
            0x30..=0x37 => {
                let reg = cb_opcode & 0x07;
                self.swap_r(reg, mmu)
            },
            
            // SRL - Shift Right Logical
            0x38..=0x3F => {
                let reg = cb_opcode & 0x07;
                self.srl_r(reg, mmu)
            },
            
            // BIT - Test bit
            0x40..=0x7F => {
                let bit = (cb_opcode - 0x40) >> 3;
                let reg = cb_opcode & 0x07;
                self.bit_r(bit, reg, mmu)
            },
            
            // RES - Reset bit
            0x80..=0xBF => {
                let bit = (cb_opcode - 0x80) >> 3;
                let reg = cb_opcode & 0x07;
                self.res_r(bit, reg, mmu)
            },
            
            // SET - Set bit
            0xC0..=0xFF => {
                let bit = (cb_opcode - 0xC0) >> 3;
                let reg = cb_opcode & 0x07;
                self.set_r(bit, reg, mmu)
            },
        };
        cycles
    }

    fn get_register(&mut self, reg_code: u8, mmu: &MMU) -> u8 {
        match reg_code {
            0 => self.b,
            1 => self.c,
            2 => self.d,
            3 => self.e,
            4 => self.h,
            5 => self.l,
            6 => {
                let address = ((self.h as u16) << 8) | (self.l as u16);
                mmu.read_byte(address)
            },
            7 => self.a,
            _ => panic!("Invalid register code"),
        }
    }
    
    fn set_register(&mut self, reg_code: u8, value: u8, mmu: &mut MMU) 
    {
        match reg_code 
        {
            0 => self.b = value,
            1 => self.c = value,
            2 => self.d = value,
            3 => self.e = value,
            4 => self.h = value,
            5 => self.l = value,
            6 => {
                let address = ((self.h as u16) << 8) | (self.l as u16);
                mmu.write_byte(address, value);
            },
            7 => self.a = value,
            _ => panic!("Invalid register code"),
        }
    }  

    // Rotate Left with Carry
    fn rlc_r(&mut self, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let carry = (value & 0x80) >> 7;  // Bit 7 becomes carry
        let result = (value << 1) | carry;  // Rotate left, with bit 7 becoming bit 0
        
        self.set_register(reg_code, result, mmu);
        
        // Set flags
        self.f = 0;  // Clear all flags
        if result == 0 { self.f |= 0x80; }  // Z flag
        if carry == 1 { self.f |= 0x10; }   // C flag
        
        if reg_code == 6 { 16 } else { 8 }  // (HL) takes 16 cycles, registers take 8
    }

    // Rotate Right with Carry
    fn rrc_r(&mut self, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, &mmu);
        let carry = value & 0x01;  // Bit 0 becomes carry
        let result = (value >> 1) | (carry << 7);  // Rotate right, with bit 0 becoming bit 7
        
        self.set_register(reg_code, result, mmu);
        
        // Set flags
        self.f = 0;  // Clear all flags
        if result == 0 { self.f |= 0x80; }  // Z flag
        if carry == 1 { self.f |= 0x10; }   // C flag
        
        if reg_code == 6 { 16 } else { 8 }
    }

    // Rotate Left through Carry
    fn rl_r(&mut self, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let old_carry = (self.f & 0x10) >> 4;  // Get current carry flag
        let new_carry = (value & 0x80) >> 7;   // Bit 7 becomes new carry
        let result = (value << 1) | old_carry;  // Rotate left through carry
        
        self.set_register(reg_code, result, mmu);
        
        // Set flags
        self.f = 0;  // Clear all flags
        if result == 0 { self.f |= 0x80; }    // Z flag
        if new_carry == 1 { self.f |= 0x10; }  // C flag
        
        if reg_code == 6 { 16 } else { 8 }
    }

    // Rotate Right through Carry
    fn rr_r(&mut self, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let old_carry = (self.f & 0x10) >> 4;  // Get current carry flag
        let new_carry = value & 0x01;          // Bit 0 becomes new carry
        let result = (value >> 1) | (old_carry << 7);  // Rotate right through carry
        
        self.set_register(reg_code, result, mmu);
        
        // Set flags
        self.f = 0;  // Clear all flags
        if result == 0 { self.f |= 0x80; }    // Z flag
        if new_carry == 1 { self.f |= 0x10; }  // C flag
        
        if reg_code == 6 { 16 } else { 8 }
    }

    // Shift Left Arithmetic
    fn sla_r(&mut self, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let carry = (value & 0x80) >> 7;  // Bit 7 becomes carry
        let result = value << 1;          // Shift left, bit 0 becomes 0
        
        self.set_register(reg_code, result, mmu);
        
        // Set flags
        self.f = 0;  // Clear all flags
        if result == 0 { self.f |= 0x80; }  // Z flag
        if carry == 1 { self.f |= 0x10; }   // C flag
        
        if reg_code == 6 { 16 } else { 8 }
    }

    // Shift Right Arithmetic (preserves sign bit)
    fn sra_r(&mut self, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let carry = value & 0x01;        // Bit 0 becomes carry
        let bit7 = value & 0x80;         // Preserve bit 7 (sign bit)
        let result = (value >> 1) | bit7;  // Shift right, keeping bit 7 the same
        
        self.set_register(reg_code, result, mmu);
        
        // Set flags
        self.f = 0;  // Clear all flags
        if result == 0 { self.f |= 0x80; }  // Z flag
        if carry == 1 { self.f |= 0x10; }   // C flag
        
        if reg_code == 6 { 16 } else { 8 }
    }

    // Swap upper and lower nibbles
    fn swap_r(&mut self, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let result = ((value & 0x0F) << 4) | ((value & 0xF0) >> 4);  // Swap nibbles
        
        self.set_register(reg_code, result, mmu);
        
        // Set flags
        self.f = 0;  // Clear all flags
        if result == 0 { self.f |= 0x80; }  // Z flag
        
        if reg_code == 6 { 16 } else { 8 }
    }

    // Shift Right Logical
    fn srl_r(&mut self, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let carry = value & 0x01;  // Bit 0 becomes carry
        let result = value >> 1;   // Shift right, bit 7 becomes 0
        
        self.set_register(reg_code, result, mmu);
        
        // Set flags
        self.f = 0;  // Clear all flags
        if result == 0 { self.f |= 0x80; }  // Z flag
        if carry == 1 { self.f |= 0x10; }   // C flag
        
        if reg_code == 6 { 16 } else { 8 }
    }

    // Test bit
    fn bit_r(&mut self, bit: u8, reg_code: u8, mmu: &MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let is_bit_set = (value & (1 << bit)) != 0;
    
        // Set flags
        self.f &= 0x10;          // Preserve C flag, reset others
        self.f |= 0x20;          // Set H flag
        if !is_bit_set {
            self.f |= 0x80;      // Set Z flag if bit is 0
        }
    
        if reg_code == 6 { 12 } else { 8 }
    }

    // Reset (clear) bit
    fn res_r(&mut self, bit: u8, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let result = value & !(1 << bit);  // Clear the specified bit
    
        self.set_register(reg_code, result, mmu);
    
        // No flags affected
    
        if reg_code == 6 { 16 } else { 8 }
    }

    // Set bit
    fn set_r(&mut self, bit: u8, reg_code: u8, mmu: &mut MMU) -> u8 
    {
        let value = self.get_register(reg_code, mmu);
        let result = value | (1 << bit);  // Set the specified bit
    
        self.set_register(reg_code, result, mmu);
    
        // No flags affected
    
        if reg_code == 6 { 16 } else { 8 }
    }
    pub fn step(&mut self, mmu: &mut MMU) -> (bool, u8) 
    {
        if self.halted {
            return (false, 0);
        }
        
        let opcode = self.fetch(mmu);
        let cycles = self.execute(opcode, mmu);
        (true, cycles)
    }
}

