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
        }
    }

    pub fn fetch(&mut self, memory: &[u8]) -> u8 
    {
        let opcode = memory[self.pc as usize]; 
        println!("Fetching opcode {:#04x} at PC={:#06x}", opcode, self.pc);
        self.pc += 1; // Move to the next instruction
        opcode
    }

    pub fn push(&mut self, memory: &mut [u8], value: u8) {
        self.sp = self.sp.wrapping_sub(1); // Decrement SP before writing
        memory[self.sp as usize] = value;
    }

    pub fn pop(&mut self, memory: &[u8]) -> u8 {
        let value = memory[self.sp as usize];
        self.sp = self.sp.wrapping_add(1); // Increment SP after reading
        value
    }

    pub fn execute(&mut self, opcode: u8, memory: &mut [u8]) 
    {
        match opcode 
        {
            0x00 => { // NOP (No Operation)
                println!("Executing NOP at PCC={:#06x}", self.pc);
            }
            0x01 => { // LD BC, nn

                self.c = memory[self.pc as usize];
                self.b = memory[self.pc as usize + 1];
                self.pc += 2;
                println!("Executing LD BC, {:#04x}{:#04x} at PC={:#06x}", self.b, self.c, self.pc);
            }
            0x02 => { // LD (BC), A
                let addr = (self.b as u16) << 8 | self.c as u16;
                memory[addr as usize] = self.a;
                println!("Executing LD (BC), A at PC={:#06x}", self.pc);
            }
            0x03 => { // INC BC
                let bc = ((self.b as u16) << 8 | self.c as u16).wrapping_add(1);
                self.b = (bc >> 8) as u8;
                self.c = bc as u8;
                println!("Executing INC BC at PC={:#06x}", self.pc);
            }
            0x04 => { // INC B
                self.b = self.b.wrapping_add(1);
                println!("Executing INC B at PC={:#06x}", self.pc);
            }
            0x05 => { // DEC B
                self.b = self.b.wrapping_sub(1);
                println!("Executing DEC B at PC={:#06x}", self.pc);
            }
            0x06 => { // LD B, n
                self.b = memory[self.pc as usize];
                self.pc += 1;
                println!("Executing LD B, {:#04x} at PC={:#06x}", self.b, self.pc);
            }
            0x07 => { // RLCA
                let carry = self.a >> 7;
                self.f = carry << 4;
                self.a = (self.a << 1) | carry;
                println!("Executing RLCA at PC={:#06x}", self.pc);
            }
            0x08 => { // LD (nn), SP
                let addr = (memory[self.pc as usize + 1] as u16) << 8 | memory[self.pc as usize] as u16;
                memory[addr as usize] = self.sp as u8;
                memory[(addr + 1) as usize] = (self.sp >> 8) as u8;
                self.pc += 2;
                println!("Executing LD ({:#06x}), SP at PC={:#06x}", addr, self.pc);
            }
            0x09 => { // ADD HL, BC
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add((self.b as u16) << 8 | self.c as u16);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing ADD HL, BC at PC={:#06x}", self.pc);
            }
            0x0A => { // LD A, (BC)
                let addr = (self.b as u16) << 8 | self.c as u16;
                self.a = memory[addr as usize];
                println!("Executing LD A, (BC) at PC={:#06x}", self.pc);
            }
            0x0B => { // DEC BC
                let bc = ((self.b as u16) << 8 | self.c as u16).wrapping_sub(1);
                self.b = (bc >> 8) as u8;
                self.c = bc as u8;
                println!("Executing DEC BC at PC={:#06x}", self.pc);
            }
            0x0C => { // INC C
                self.c = self.c.wrapping_add(1);
                println!("Executing INC C at PC={:#06x}", self.pc);
            }
            0x0D => { // DEC C
                self.c = self.c.wrapping_sub(1);
                println!("Executing DEC C at PC={:#06x}", self.pc);
            }
            0x0E => { // LD C, n
                self.c = memory[self.pc as usize];
                self.pc += 1;
                println!("Executing LD C, {:#04x} at PC={:#06x}", self.c, self.pc);
            }
            0x0F => { // RRCA
                let carry = self.a & 1;
                self.f = carry << 4;
                self.a = (self.a >> 1) | (carry << 7);
                println!("Executing RRCA at PC={:#06x}", self.pc);
            }
            0x10 => { // STOP
                println!("Executing STOP at PC={:#06x}", self.pc);
            }
            0x11 => { // LD DE, nn
                self.e = memory[self.pc as usize];
                self.d = memory[self.pc as usize + 1];
                self.pc += 2;
                println!("Executing LD DE, {:#04x}{:#04x} at PC={:#06x}", self.d, self.e, self.pc);
            }
            0x12 => { // LD (DE), A
                let addr = (self.d as u16) << 8 | self.e as u16;
                memory[addr as usize] = self.a;
                println!("Executing LD (DE), A at PC={:#06x}", self.pc);
            }
            0x13 => { // INC DE
                let de = ((self.d as u16) << 8 | self.e as u16).wrapping_add(1);
                self.d = (de >> 8) as u8;
                self.e = de as u8;
                println!("Executing INC DE at PC={:#06x}", self.pc);
            }
            0x14 => { // INC D
                self.d = self.d.wrapping_add(1);
                println!("Executing INC D at PC={:#06x}", self.pc);
            }
            0x15 => { // DEC D
                self.d = self.d.wrapping_sub(1);
                println!("Executing DEC D at PC={:#06x}", self.pc);
            }
            0x16 => { // LD D, n
                self.d = memory[self.pc as usize];
                self.pc += 1;
                println!("Executing LD D, {:#04x} at PC={:#06x}", self.d, self.pc);
            }
            0x17 => { // RLA
                let carry = self.a >> 7;
                self.a = (self.a << 1) | (self.f >> 4);
                self.f = carry << 4;
                println!("Executing RLA at PC={:#06x}", self.pc);
            }
            0x18 => { // JR n
                let offset = memory[self.pc as usize] as i8;
                self.pc = self.pc.wrapping_add(1).wrapping_add(offset as u16);
                println!("Executing JR {:#04x} at PC={:#06x}", offset, self.pc);
            }
            0x19 => { // ADD HL, DE
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add((self.d as u16) << 8 | self.e as u16);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing ADD HL, DE at PC={:#06x}", self.pc);
            }
            0x1A => { // LD A, (DE)
                let addr = (self.d as u16) << 8 | self.e as u16;
                self.a = memory[addr as usize];
                println!("Executing LD A, (DE) at PC={:#06x}", self.pc);
            }
            0x1B => { // DEC DE
                let de = ((self.d as u16) << 8 | self.e as u16).wrapping_sub(1);
                self.d = (de >> 8) as u8;
                self.e = de as u8;
                println!("Executing DEC DE at PC={:#06x}", self.pc);
            }
            0x1C => { // INC E
                self.e = self.e.wrapping_add(1);
                println!("Executing INC E at PC={:#06x}", self.pc);
            }
            0x1D => { // DEC E
                self.e = self.e.wrapping_sub(1);
                println!("Executing DEC E at PC={:#06x}", self.pc);
            }
            0x1E => { // LD E, n
                self.e = memory[self.pc as usize];
                self.pc += 1;
                println!("Executing LD E, {:#04x} at PC={:#06x}", self.e, self.pc);
            }
            0x1F => { // RRA
                let carry = self.a & 1;
                self.a = (self.a >> 1) | (self.f << 7);
                self.f = carry << 4;
                println!("Executing RRA at PC={:#06x}", self.pc);
            }
            0x20 => { // JR NZ, n
                let offset = memory[self.pc as usize] as i8;
                if self.f & 0x80 == 0 {
                    self.pc = self.pc.wrapping_add(1).wrapping_add(offset as u16);
                } else {
                    self.pc += 1;
                }
                println!("Executing JR NZ, {:#04x} at PC={:#06x}", offset, self.pc);
            }
            0x21 => { // LD HL, nn
                self.l = memory[self.pc as usize];
                self.h = memory[self.pc as usize + 1];
                self.pc += 2;
                println!("Executing LD HL, {:#04x}{:#04x} at PC={:#06x}", self.h, self.l, self.pc);
            }
            0x22 => { // LD (HL+), A
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = self.a;
                let hl = addr.wrapping_add(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing LD (HL+), A at PC={:#06x}", self.pc);
            }
            0x23 => { // INC HL
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing INC HL at PC={:#06x}", self.pc);
            }
            0x24 => { // INC H
                self.h = self.h.wrapping_add(1);
                println!("Executing INC H at PC={:#06x}", self.pc);
            }
            0x25 => { // DEC H
                self.h = self.h.wrapping_sub(1);
                println!("Executing DEC H at PC={:#06x}", self.pc);
            }
            0x26 => { // LD H, n
                self.h = memory[self.pc as usize];
                self.pc += 1;
                println!("Executing LD H, {:#04x} at PC={:#06x}", self.h, self.pc);
            }
            0x27 => { // DAA
                println!("Executing DAA at PC={:#06x}", self.pc);
            }
            0x28 => { // JR Z, n
                let offset = memory[self.pc as usize] as i8;
                if self.f & 0x80 != 0 {
                    self.pc = self.pc.wrapping_add(1).wrapping_add(offset as u16);
                } else {
                    self.pc += 1;
                }
                println!("Executing JR Z, {:#04x} at PC={:#06x}", offset, self.pc);
            }
            0x29 => { // ADD HL, HL
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add((self.h as u16) << 8 | self.l as u16);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing ADD HL, HL at PC={:#06x}", self.pc);
            }
            0x2A => { // LD A, (HL+)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a = memory[addr as usize];
                let hl = addr.wrapping_add(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing LD A, (HL+) at PC={:#06x}", self.pc);
            }
            0x2B => { // DEC HL
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_sub(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing DEC HL at PC={:#06x}", self.pc);
            }
            0x2C => { // INC L
                self.l = self.l.wrapping_add(1);
                println!("Executing INC L at PC={:#06x}", self.pc);
            }
            0x2D => { // DEC L
                self.l = self.l.wrapping_sub(1);
                println!("Executing DEC L at PC={:#06x}", self.pc);
            }
            0x2E => { // LD L, n
                self.l = memory[self.pc as usize];
                self.pc += 1;
                println!("Executing LD L, {:#04x} at PC={:#06x}", self.l, self.pc);
            }
            0x2F => { // CPL
                self.a = !self.a;
                self.f |= 0x60;
                println!("Executing CPL at PC={:#06x}", self.pc);
            }
            0x30 => { // JR NC, n
                let offset = memory[self.pc as usize] as i8;
                if self.f & 0x10 == 0 {
                    self.pc = self.pc.wrapping_add(1).wrapping_add(offset as u16);
                } else {
                    self.pc += 1;
                }
                println!("Executing JR NC, {:#04x} at PC={:#06x}", offset, self.pc);
            }
            0x31 => { // LD SP, nn
                self.sp = (memory[self.pc as usize + 1] as u16) << 8 | memory[self.pc as usize] as u16;
                self.pc += 2;
                println!("Executing LD SP, {:#04x}{:#04x} at PC={:#06x}", self.sp >> 8, self.sp, self.pc);
            }
            0x32 => { // LD (HL-), A
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = self.a;
                let hl = addr.wrapping_sub(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing LD (HL-), A at PC={:#06x}", self.pc);
            }
            0x33 => { // INC SP
                self.sp = self.sp.wrapping_add(1);
                println!("Executing INC SP at PC={:#06x}", self.pc);
            }
            0x34 => { // INC (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let value = memory[addr as usize].wrapping_add(1);
                memory[addr as usize] = value;
                println!("Executing INC (HL) at PC={:#06x}", self.pc);
            }
            0x35 => { // DEC (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let value = memory[addr as usize].wrapping_sub(1);
                memory[addr as usize] = value;
                println!("Executing DEC (HL) at PC={:#06x}", self.pc);
            }
            0x36 => { // LD (HL), n
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = memory[self.pc as usize];
                self.pc += 1;
                println!("Executing LD (HL), {:#04x} at PC={:#06x}", memory[addr as usize], self.pc);
            }
            0x37 => { // SCF
                self.f = 0x10;
                println!("Executing SCF at PC={:#06x}", self.pc);
            }
            0x38 => { // JR C, n
                let offset = memory[self.pc as usize] as i8;
                if self.f & 0x10 != 0 {
                    self.pc = self.pc.wrapping_add(1).wrapping_add(offset as u16);
                } else {
                    self.pc += 1;
                }
                println!("Executing JR C, {:#04x} at PC={:#06x}", offset, self.pc);
            }
            0x39 => { // ADD HL, SP
                let hl = ((self.h as u16) << 8 | self.l as u16).wrapping_add(self.sp);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing ADD HL, SP at PC={:#06x}", self.pc);
            }
            0x3A => { // LD A, (HL-)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a = memory[addr as usize];
                let hl = addr.wrapping_sub(1);
                self.h = (hl >> 8) as u8;
                self.l = hl as u8;
                println!("Executing LD A, (HL-) at PC={:#06x}", self.pc);
            }
            0x3B => { // DEC SP
                self.sp = self.sp.wrapping_sub(1);
                println!("Executing DEC SP at PC={:#06x}", self.pc);
            }
            0x3C => { // INC A
                self.a = self.a.wrapping_add(1);
                println!("Executing INC A at PC={:#06x}", self.pc);
            }
            0x3D => { // DEC A
                self.a = self.a.wrapping_sub(1);
                println!("Executing DEC A at PC={:#06x}", self.pc);
            }
            0x3E => { // LD A, n
                self.a = memory[self.pc as usize];
                self.pc += 1;
                println!("Executing LD A, {:#04x} at PC={:#06x}", self.a, self.pc);
            }
            0x3F => { // CCF
                self.f ^= 0x10;
                println!("Executing CCF at PC={:#06x}", self.pc);
            }
            0x40 => { // LD B, B
                println!("Executing LD B, B at PC={:#06x}", self.pc);
            }
            0x41 => { // LD B, C
                self.b = self.c;
                println!("Executing LD B, C at PC={:#06x}", self.pc);
            }
            0x42 => { // LD B, D
                self.b = self.d;
                println!("Executing LD B, D at PC={:#06x}", self.pc);
            }
            0x43 => { // LD B, E
                self.b = self.e;
                println!("Executing LD B, E at PC={:#06x}", self.pc);
            }
            0x44 => { // LD B, H
                self.b = self.h;
                println!("Executing LD B, H at PC={:#06x}", self.pc);
            }
            0x45 => { // LD B, L
                self.b = self.l;
                println!("Executing LD B, L at PC={:#06x}", self.pc);
            }
            0x46 => { // LD B, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.b = memory[addr as usize];
                println!("Executing LD B, (HL) at PC={:#06x}", self.pc);
            }
            0x47 => { // LD B, A
                self.b = self.a;
                println!("Executing LD B, A at PC={:#06x}", self.pc);
            }
            0x48 => { // LD C, B
                self.c = self.b;
                println!("Executing LD C, B at PC={:#06x}", self.pc);
            }
            0x49 => { // LD C, C
                println!("Executing LD C, C at PC={:#06x}", self.pc);
            }
            0x4A => { // LD C, D
                self.c = self.d;
                println!("Executing LD C, D at PC={:#06x}", self.pc);
            }
            0x4B => { // LD C, E
                self.c = self.e;
                println!("Executing LD C, E at PC={:#06x}", self.pc);
            }
            0x4C => { // LD C, H
                self.c = self.h;
                println!("Executing LD C, H at PC={:#06x}", self.pc);
            }
            0x4D => { // LD C, L
                self.c = self.l;
                println!("Executing LD C, L at PC={:#06x}", self.pc);
            }
            0x4E => { // LD C, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.c = memory[addr as usize];
                println!("Executing LD C, (HL) at PC={:#06x}", self.pc);
            }
            0x4F => { // LD C, A
                self.c = self.a;
                println!("Executing LD C, A at PC={:#06x}", self.pc);
            }
            0x50 => { // LD D, B
                self.d = self.b;
                println!("Executing LD D, B at PC={:#06x}", self.pc);
            }
            0x51 => { // LD D, C
                self.d = self.c;
                println!("Executing LD D, C at PC={:#06x}", self.pc);
            }
            0x52 => { // LD D, D
                println!("Executing LD D, D at PC={:#06x}", self.pc);
            }
            0x53 => { // LD D, E
                self.d = self.e;
                println!("Executing LD D, E at PC={:#06x}", self.pc);
            }
            0x54 => { // LD D, H
                self.d = self.h;
                println!("Executing LD D, H at PC={:#06x}", self.pc);
            }
            0x55 => { // LD D, L
                self.d = self.l;
                println!("Executing LD D, L at PC={:#06x}", self.pc);
            }
            0x56 => { // LD D, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.d = memory[addr as usize];
                println!("Executing LD D, (HL) at PC={:#06x}", self.pc);
            }
            0x57 => { // LD D, A
                self.d = self.a;
                println!("Executing LD D, A at PC={:#06x}", self.pc);
            }
            0x58 => { // LD E, B
                self.e = self.b;
                println!("Executing LD E, B at PC={:#06x}", self.pc);
            }  
            0x59 => { // LD E, C
                self.e = self.c;
                println!("Executing LD E, C at PC={:#06x}", self.pc);
            }
            0x5A => { // LD E, D
                self.e = self.d;
                println!("Executing LD E, D at PC={:#06x}", self.pc);
            }
            0x5B => { // LD E, E
                println!("Executing LD E, E at PC={:#06x}", self.pc);
            }
            0x5C => { // LD E, H
                self.e = self.h;
                println!("Executing LD E, H at PC={:#06x}", self.pc);
            }
            0x5D => { // LD E, L
                self.e = self.l;
                println!("Executing LD E, L at PC={:#06x}", self.pc);
            }
            0x5E => { // LD E, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.e = memory[addr as usize];
                println!("Executing LD E, (HL) at PC={:#06x}", self.pc);
            }
            0x5F => { // LD E, A
                self.e = self.a;
                println!("Executing LD E, A at PC={:#06x}", self.pc);
            }
            0x60 => { // LD H, B
                self.h = self.b;
                println!("Executing LD H, B at PC={:#06x}", self.pc);
            }
            0x61 => { // LD H, C
                self.h = self.c;
                println!("Executing LD H, C at PC={:#06x}", self.pc);
            }
            0x62 => { // LD H, D
                self.h = self.d;
                println!("Executing LD H, D at PC={:#06x}", self.pc);
            }
            0x63 => { // LD H, E
                self.h = self.e;
                println!("Executing LD H, E at PC={:#06x}", self.pc);
            }
            0x64 => { // LD H, H
                println!("Executing LD H, H at PC={:#06x}", self.pc);
            }
            0x65 => { // LD H, L
                self.h = self.l;
                println!("Executing LD H, L at PC={:#06x}", self.pc);
            }
            0x66 => { // LD H, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.h = memory[addr as usize];
                println!("Executing LD H, (HL) at PC={:#06x}", self.pc);
            }
            0x67 => { // LD H, A
                self.h = self.a;
                println!("Executing LD H, A at PC={:#06x}", self.pc);
            } 
            0x68 => { // LD L, B
                self.l = self.b;
                println!("Executing LD L, B at PC={:#06x}", self.pc);
            }
            0x69 => { // LD L, C
                self.l = self.c;
                println!("Executing LD L, C at PC={:#06x}", self.pc);
            }
            0x6A => { // LD L, D
                self.l = self.d;
                println!("Executing LD L, D at PC={:#06x}", self.pc);
            }
            0x6B => { // LD L, E
                self.l = self.e;
                println!("Executing LD L, E at PC={:#06x}", self.pc);
            }
            0x6C => { // LD L, H
                self.l = self.h;
                println!("Executing LD L, H at PC={:#06x}", self.pc);
            }
            0x6D => { // LD L, L
                println!("Executing LD L, L at PC={:#06x}", self.pc);
            }
            0x6E => { // LD L, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.l = memory[addr as usize];
                println!("Executing LD L, (HL) at PC={:#06x}", self.pc);
            }
            0x6F => { // LD L, A
                self.l = self.a;
                println!("Executing LD L, A at PC={:#06x}", self.pc);
            }
            0x70 => { // LD (HL), B
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = self.b;
                println!("Executing LD (HL), B at PC={:#06x}", self.pc);
            }
            0x71 => { // LD (HL), C
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = self.c;
                println!("Executing LD (HL), C at PC={:#06x}", self.pc);
            }
            0x72 => { // LD (HL), D
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = self.d;
                println!("Executing LD (HL), D at PC={:#06x}", self.pc);
            }
            0x73 => { // LD (HL), E
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = self.e;
                println!("Executing LD (HL), E at PC={:#06x}", self.pc);
            }
            0x74 => { // LD (HL), H
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = self.h;
                println!("Executing LD (HL), H at PC={:#06x}", self.pc);
            }
            0x75 => { // LD (HL), L
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = self.l;
                println!("Executing LD (HL), L at PC={:#06x}", self.pc);
            }
            0x76 => { // HALT
                println!("Executing HALT at PC={:#06x}", self.pc);
            }
            0x77 => { // LD (HL), A
                let addr = (self.h as u16) << 8 | self.l as u16;
                memory[addr as usize] = self.a;
                println!("Executing LD (HL), A at PC={:#06x}", self.pc);
            }
            0x78 => { // LD A, B
                self.a = self.b;
                println!("Executing LD A, B at PC={:#06x}", self.pc);
            }
            0x79 => { // LD A, C
                self.a = self.c;
                println!("Executing LD A, C at PC={:#06x}", self.pc);
            }
            0x7A => { // LD A, D
                self.a = self.d;
                println!("Executing LD A, D at PC={:#06x}", self.pc);
            }
            0x7B => { // LD A, E
                self.a = self.e;
                println!("Executing LD A, E at PC={:#06x}", self.pc);
            }
            0x7C => { // LD A, H
                self.a = self.h;
                println!("Executing LD A, H at PC={:#06x}", self.pc);
            }
            0x7D => { // LD A, L
                self.a = self.l;
                println!("Executing LD A, L at PC={:#06x}", self.pc);
            }
            0x7E => { // LD A, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a = memory[addr as usize];
                println!("Executing LD A, (HL) at PC={:#06x}", self.pc);
            }
            0x7F => { // LD A, A
                println!("Executing LD A, A at PC={:#06x}", self.pc);
            }
            0x80 => { // ADD A, B
                let (result, carry) = self.a.overflowing_add(self.b);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADD A, B at PC={:#06x}", self.pc);
            }
            0x81 => { // ADD A, C
                let (result, carry) = self.a.overflowing_add(self.c);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADD A, C at PC={:#06x}", self.pc);
            }
            0x82 => { // ADD A, D
                let (result, carry) = self.a.overflowing_add(self.d);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADD A, D at PC={:#06x}", self.pc);
            }
            0x83 => { // ADD A, E
                let (result, carry) = self.a.overflowing_add(self.e);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADD A, E at PC={:#06x}", self.pc);
            }
            0x84 => { // ADD A, H
                let (result, carry) = self.a.overflowing_add(self.h);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADD A, H at PC={:#06x}", self.pc);
            }
            0x85 => { // ADD A, L
                let (result, carry) = self.a.overflowing_add(self.l);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADD A, L at PC={:#06x}", self.pc);
            }
            0x86 => { // ADD A, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let (result, carry) = self.a.overflowing_add(memory[addr as usize]);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADD A, (HL) at PC={:#06x}", self.pc);
            }
            0x87 => { // ADD A, A
                let (result, carry) = self.a.overflowing_add(self.a);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADD A, A at PC={:#06x}", self.pc);
            }
            0x88 => { // ADC A, B
                let (result, carry) = self.a.overflowing_add(self.b.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADC A, B at PC={:#06x}", self.pc);
            }
            0x89 => { // ADC A, C
                let (result, carry) = self.a.overflowing_add(self.c.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADC A, C at PC={:#06x}", self.pc);
            }
            0x8A => { // ADC A, D
                let (result, carry) = self.a.overflowing_add(self.d.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADC A, D at PC={:#06x}", self.pc);
            }
            0x8B => { // ADC A, E
                let (result, carry) = self.a.overflowing_add(self.e.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADC A, E at PC={:#06x}", self.pc);
            }  
            0x8C => { // ADC A, H
                let (result, carry) = self.a.overflowing_add(self.h.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADC A, H at PC={:#06x}", self.pc);
            }
            0x8D => { // ADC A, L
                let (result, carry) = self.a.overflowing_add(self.l.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADC A, L at PC={:#06x}", self.pc);
            }
            0x8E => { // ADC A, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let (result, carry) = self.a.overflowing_add(memory[addr as usize].wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADC A, (HL) at PC={:#06x}", self.pc);
            }
            0x8F => { // ADC A, A
                let (result, carry) = self.a.overflowing_add(self.a.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADC A, A at PC={:#06x}", self.pc);
            }
            0x90 => { // SUB B
                let (result, carry) = self.a.overflowing_sub(self.b);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SUB B at PC={:#06x}", self.pc);
            }
            0x91 => { // SUB C
                let (result, carry) = self.a.overflowing_sub(self.c);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SUB C at PC={:#06x}", self.pc);
            }
            0x92 => { // SUB D
                let (result, carry) = self.a.overflowing_sub(self.d);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SUB D at PC={:#06x}", self.pc);
            }
            0x93 => { // SUB E
                let (result, carry) = self.a.overflowing_sub(self.e);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SUB E at PC={:#06x}", self.pc);
            }
            0x94 => { // SUB H
                let (result, carry) = self.a.overflowing_sub(self.h);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SUB H at PC={:#06x}", self.pc);
            }
            0x95 => { // SUB L
                let (result, carry) = self.a.overflowing_sub(self.l);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SUB L at PC={:#06x}", self.pc);
            }
            0x96 => { // SUB (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let (result, carry) = self.a.overflowing_sub(memory[addr as usize]);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SUB (HL) at PC={:#06x}", self.pc);
            }
            0x97 => { // SUB A
                let (result, carry) = self.a.overflowing_sub(self.a);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SUB A at PC={:#06x}", self.pc);
            }
            0x98 => { // SBC A, B
                let (result, carry) = self.a.overflowing_sub(self.b.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SBC A, B at PC={:#06x}", self.pc);
            }
            0x99 => { // SBC A, C
                let (result, carry) = self.a.overflowing_sub(self.c.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SBC A, C at PC={:#06x}", self.pc);
            }
            0x9A => { // SBC A, D
                let (result, carry) = self.a.overflowing_sub(self.d.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SBC A, D at PC={:#06x}", self.pc);
            }
            0x9B => { // SBC A, E
                let (result, carry) = self.a.overflowing_sub(self.e.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SBC A, E at PC={:#06x}", self.pc);
            }
            0x9C => { // SBC A, H
                let (result, carry) = self.a.overflowing_sub(self.h.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SBC A, H at PC={:#06x}", self.pc);
            }
            0x9D => { // SBC A, L
                let (result, carry) = self.a.overflowing_sub(self.l.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SBC A, L at PC={:#06x}", self.pc);
            }
            0x9E => { // SBC A, (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let (result, carry) = self.a.overflowing_sub(memory[addr as usize].wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SBC A, (HL) at PC={:#06x}", self.pc);
            }
            0x9F => { // SBC A, A
                let (result, carry) = self.a.overflowing_sub(self.a.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SBC A, A at PC={:#06x}", self.pc);
            }
            0xA0 => { // AND B
                self.a &= self.b;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND B at PC={:#06x}", self.pc);
            }
            0xA1 => { // AND C
                self.a &= self.c;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND C at PC={:#06x}", self.pc);
            }
            0xA2 => { // AND D
                self.a &= self.d;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND D at PC={:#06x}", self.pc);
            }
            0xA3 => { // AND E
                self.a &= self.e;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND E at PC={:#06x}", self.pc);
            }
            0xA4 => { // AND H
                self.a &= self.h;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND H at PC={:#06x}", self.pc);
            }
            0xA5 => { // AND L
                self.a &= self.l;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND L at PC={:#06x}", self.pc);
            }
            0xA6 => { // AND (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a &= memory[addr as usize];
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND (HL) at PC={:#06x}", self.pc);
            }
            0xA7 => { // AND A
                self.a &= self.a;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND A at PC={:#06x}", self.pc);
            }
            0xA8 => { // XOR B
                self.a ^= self.b;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR B at PC={:#06x}", self.pc);
            }
            0xA9 => { // XOR C
                self.a ^= self.c;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR C at PC={:#06x}", self.pc);
            }
            0xAA => { // XOR D
                self.a ^= self.d;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR D at PC={:#06x}", self.pc);
            }
            0xAB => { // XOR E
                self.a ^= self.e;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR E at PC={:#06x}", self.pc);
            }
            0xAC => { // XOR H
                self.a ^= self.h;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR H at PC={:#06x}", self.pc);
            }
            0xAD => { // XOR L
                self.a ^= self.l;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR L at PC={:#06x}", self.pc);
            }
            0xAE => { // XOR (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a ^= memory[addr as usize];
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR (HL) at PC={:#06x}", self.pc);
            }
            0xAF => { // XOR A
                self.a ^= self.a;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR A at PC={:#06x}", self.pc);
            }
            0xB0 => { // OR B
                self.a |= self.b;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR B at PC={:#06x}", self.pc);
            }
            0xB1 => { // OR C
                self.a |= self.c;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR C at PC={:#06x}", self.pc);
            }
            0xB2 => { // OR D
                self.a |= self.d;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR D at PC={:#06x}", self.pc);
            }
            0xB3 => { // OR E
                self.a |= self.e;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR E at PC={:#06x}", self.pc);
            }
            0xB4 => { // OR H
                self.a |= self.h;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR H at PC={:#06x}", self.pc);
            }
            0xB5 => { // OR L
                self.a |= self.l;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR L at PC={:#06x}", self.pc);
            }
            0xB6 => { // OR (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                self.a |= memory[addr as usize];
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR (HL) at PC={:#06x}", self.pc);
            }
            0xB7 => { // OR A
                self.a |= self.a;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR A at PC={:#06x}", self.pc);
            }
            0xB8 => { // CP B
                let (result, carry) = self.a.overflowing_sub(self.b);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP B at PC={:#06x}", self.pc);
            }
            0xB9 => { // CP C
                let (result, carry) = self.a.overflowing_sub(self.c);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP C at PC={:#06x}", self.pc);
            }
            0xBA => { // CP D
                let (result, carry) = self.a.overflowing_sub(self.d);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP D at PC={:#06x}", self.pc);
            }
            0xBB => { // CP E
                let (result, carry) = self.a.overflowing_sub(self.e);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP E at PC={:#06x}", self.pc);
            }
            0xBC => { // CP H
                let (result, carry) = self.a.overflowing_sub(self.h);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP H at PC={:#06x}", self.pc);
            }
            0xBD => { // CP L
                let (result, carry) = self.a.overflowing_sub(self.l);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP L at PC={:#06x}", self.pc);
            }
            0xBE => { // CP (HL)
                let addr = (self.h as u16) << 8 | self.l as u16;
                let (result, carry) = self.a.overflowing_sub(memory[addr as usize]);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP (HL) at PC={:#06x}", self.pc);
            }
            0xBF => { // CP A
                let (result, carry) = self.a.overflowing_sub(self.a);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP A at PC={:#06x}", self.pc);
            }
            0xC0 => { // RET NZ
                if self.f & 0x80 == 0 {
                    let lo = self.pop(memory);
                    let hi = self.pop(memory);
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing RET NZ at PC={:#06x}", self.pc);
            }
            0xC1 => { // POP BC
                self.c = self.pop(memory);
                self.b = self.pop(memory);
                println!("Executing POP BC at PC={:#06x}", self.pc);
            }
            0xC2 => { // JP NZ, nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                if self.f & 0x80 == 0 {
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing JP NZ, nn at PC={:#06x}", self.pc);
            }
            0xC3 => { // JP nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                self.pc = (hi as u16) << 8 | lo as u16;
                println!("Executing JP nn at PC={:#06x}", self.pc);
            }
            0xC4 => { // CALL NZ, nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                if self.f & 0x80 == 0 {
                    self.push(memory, (self.pc >> 8) as u8);
                    self.push(memory, self.pc as u8);
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing CALL NZ, nn at PC={:#06x}", self.pc);
            }
            0xC5 => { // PUSH BC
                self.push(memory, self.b);
                self.push(memory, self.c);
                println!("Executing PUSH BC at PC={:#06x}", self.pc);
            }
            0xC6 => { // ADD A, n
                let n = self.fetch(memory);
                let (result, carry) = self.a.overflowing_add(n);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADD A, n at PC={:#06x}", self.pc);
            }
            0xC7 => { // RST 00H
                self.push(memory, (self.pc >> 8) as u8);
                self.push(memory, self.pc as u8);
                self.pc = 0x00;
                println!("Executing RST 00H at PC={:#06x}", self.pc);
            }
            0xC8 => { // RET Z
                if self.f & 0x80 != 0 {
                    let lo = self.pop(memory);
                    let hi = self.pop(memory);
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing RET Z at PC={:#06x}", self.pc);
            }
            0xC9 => { // RET
                let lo = self.pop(memory);
                let hi = self.pop(memory);
                self.pc = (hi as u16) << 8 | lo as u16;
                println!("Executing RET at PC={:#06x}", self.pc);
            }
            0xCA => { // JP Z, nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                if self.f & 0x80 != 0 {
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing JP Z, nn at PC={:#06x}", self.pc);
            }
        
              // 0xCB prefix, kind of a weird situation, TODO
              
            0xCC => { // CALL Z, nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                if self.f & 0x80 != 0 {
                    self.push(memory, (self.pc >> 8) as u8);
                    self.push(memory, self.pc as u8);
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing CALL Z, nn at PC={:#06x}", self.pc);
            }
            0xCD => { // CALL nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                self.push(memory, (self.pc >> 8) as u8);
                self.push(memory, self.pc as u8);
                self.pc = (hi as u16) << 8 | lo as u16;
                println!("Executing CALL nn at PC={:#06x}", self.pc);
            }
            0xCE => { // ADC A, n
                let n = self.fetch(memory);
                let (result, carry) = self.a.overflowing_add(n.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | if carry { 0x10 } else { 0 };
                println!("Executing ADC A, n at PC={:#06x}", self.pc);
            }
            0xCF => { // RST 08H
                self.push(memory, (self.pc >> 8) as u8);
                self.push(memory, self.pc as u8);
                self.pc = 0x08;
                println!("Executing RST 08H at PC={:#06x}", self.pc);
            }
            0xD0 => { // RET NC
                if self.f & 0x10 == 0 {
                    let lo = self.pop(memory);
                    let hi = self.pop(memory);
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing RET NC at PC={:#06x}", self.pc);
            }
            0xD1 => { // POP DE
                self.e = self.pop(memory);
                self.d = self.pop(memory);
                println!("Executing POP DE at PC={:#06x}", self.pc);
            }
            0xD2 => { // JP NC, nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                if self.f & 0x10 == 0 {
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing JP NC, nn at PC={:#06x}", self.pc);
            }
            0xD4 => { // CALL NC, nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                if self.f & 0x10 == 0 {
                    self.push(memory, (self.pc >> 8) as u8);
                    self.push(memory, self.pc as u8);
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing CALL NC, nn at PC={:#06x}", self.pc);
            }
            0xD5 => { // PUSH DE
                self.push(memory, self.d);
                self.push(memory, self.e);
                println!("Executing PUSH DE at PC={:#06x}", self.pc);
            }
            0xD6 => { // SUB n
                let n = self.fetch(memory);
                let (result, carry) = self.a.overflowing_sub(n);
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SUB n at PC={:#06x}", self.pc);
            }
            0xD7 => { // RST 10H
                self.push(memory, (self.pc >> 8) as u8);
                self.push(memory, self.pc as u8);
                self.pc = 0x10;
                println!("Executing RST 10H at PC={:#06x}", self.pc);
            }
            0xD8 => { // RET C
                if self.f & 0x10 != 0 {
                    let lo = self.pop(memory);
                    let hi = self.pop(memory);
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing RET C at PC={:#06x}", self.pc);
            }
            0xD9 => { // RETI
                let lo = self.pop(memory);
                let hi = self.pop(memory);
                self.pc = (hi as u16) << 8 | lo as u16;
                println!("Executing RETI at PC={:#06x}", self.pc);
            }
            0xDA => { // JP C, nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                if self.f & 0x10 != 0 {
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing JP C, nn at PC={:#06x}", self.pc);
            }
            0xDC => { // CALL C, nn
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                if self.f & 0x10 != 0 {
                    self.push(memory, (self.pc >> 8) as u8);
                    self.push(memory, self.pc as u8);
                    self.pc = (hi as u16) << 8 | lo as u16;
                }
                println!("Executing CALL C, nn at PC={:#06x}", self.pc);
            }
            0xDE => { // SBC A, n
                let n = self.fetch(memory);
                let (result, carry) = self.a.overflowing_sub(n.wrapping_add(self.f >> 4));
                self.a = result;
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing SBC A, n at PC={:#06x}", self.pc);
            }
            0xDF => { // RST 18H
                self.push(memory, (self.pc >> 8) as u8);
                self.push(memory, self.pc as u8);
                self.pc = 0x18;
                println!("Executing RST 18H at PC={:#06x}", self.pc);
            }
            0xE0 => { // LDH (n), A
                let n = self.fetch(memory);
                memory[0xFF00 + n as usize] = self.a;
                println!("Executing LDH (n), A at PC={:#06x}", self.pc);
            }
            0xE1 => { // POP HL
                self.l = self.pop(memory);
                self.h = self.pop(memory);
                println!("Executing POP HL at PC={:#06x}", self.pc);
            }
            0xE2 => { // LD (C), A
                memory[0xFF00 + self.c as usize] = self.a;
                println!("Executing LD (C), A at PC={:#06x}", self.pc);
            }
            0xE5 => { // PUSH HL
                self.push(memory, self.h);
                self.push(memory, self.l);
                println!("Executing PUSH HL at PC={:#06x}", self.pc);
            }
            0xE6 => { // AND n
                let n = self.fetch(memory);
                self.a &= n;
                self.f = if self.a == 0 { 0x80 } else { 0 } | 0x20;
                println!("Executing AND n at PC={:#06x}", self.pc);
            }
            0xE7 => { // RST 20H
                self.push(memory, (self.pc >> 8) as u8);
                self.push(memory, self.pc as u8);
                self.pc = 0x20;
                println!("Executing RST 20H at PC={:#06x}", self.pc);
            }
            0xE8 => { // ADD SP, d
                let n = self.fetch(memory) as i8 as i16 as u16;
                let result = self.sp.wrapping_add(n);
                self.f = if (self.sp & 0xFF) + (n & 0xFF) > 0xFF { 0x10 } else { 0 } | if (self.sp & 0x0F) + (n & 0x0F) > 0x0F { 0x20 } else { 0 };
                self.sp = result;
                println!("Executing ADD SP, d at PC={:#06x}", self.pc);
            }
            0xE9 => { // JP (HL)
                self.pc = (self.h as u16) << 8 | self.l as u16;
                println!("Executing JP (HL) at PC={:#06x}", self.pc);
            }
            0xEA => { // LD (nn), A
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                let addr = (hi as u16) << 8 | lo as u16;
                memory[addr as usize] = self.a;
                println!("Executing LD (nn), A at PC={:#06x}", self.pc);
            }
            0xEE => { // XOR n
                let n = self.fetch(memory);
                self.a ^= n;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing XOR n at PC={:#06x}", self.pc);
            }
            0xEF => { // RST 28H
                self.push(memory, (self.pc >> 8) as u8);
                self.push(memory, self.pc as u8);
                self.pc = 0x28;
                println!("Executing RST 28H at PC={:#06x}", self.pc);
            }
            0xF0 => { // LDH A, (n)
                let n = self.fetch(memory);
                self.a = memory[0xFF00 + n as usize];
                println!("Executing LDH A, (n) at PC={:#06x}", self.pc);
            }
            0xF1 => { // POP AF
                self.f = self.pop(memory);
                self.a = self.pop(memory);
                println!("Executing POP AF at PC={:#06x}", self.pc);
            }
            0xF2 => { // LD A, (C)
                self.a = memory[0xFF00 + self.c as usize];
                println!("Executing LD A, (C) at PC={:#06x}", self.pc);
            }
            0xF3 => { // DI
                self.ime = false;
                println!("Executing DI at PC={:#06x}", self.pc);
            }
            0xF5 => { // PUSH AF
                self.push(memory, self.a);
                self.push(memory, self.f);
                println!("Executing PUSH AF at PC={:#06x}", self.pc);
            }
            0xF6 => { // OR n
                let n = self.fetch(memory);
                self.a |= n;
                self.f = if self.a == 0 { 0x80 } else { 0 };
                println!("Executing OR n at PC={:#06x}", self.pc);
            }
            0xF7 => { // RST 30H
                self.push(memory, (self.pc >> 8) as u8);
                self.push(memory, self.pc as u8);
                self.pc = 0x30;
                println!("Executing RST 30H at PC={:#06x}", self.pc);
            }
            0xF8 => { // LD HL, SP+d
                let n = self.fetch(memory) as i8 as i16 as u16;
                let result = self.sp.wrapping_add(n);
                self.f = if (self.sp & 0xFF) + (n & 0xFF) > 0xFF { 0x10 } else { 0 } | if (self.sp & 0x0F) + (n & 0x0F) > 0x0F { 0x20 } else { 0 };
                self.l = result as u8;
                self.h = (result >> 8) as u8;
                println!("Executing LD HL, SP+d at PC={:#06x}", self.pc);
            }
            0xF9 => { // LD SP, HL
                self.sp = (self.h as u16) << 8 | self.l as u16;
                println!("Executing LD SP, HL at PC={:#06x}", self.pc);
            }
            0xFA => { // LD A, (nn)
                let lo = self.fetch(memory);
                let hi = self.fetch(memory);
                let addr = (hi as u16) << 8 | lo as u16;
                self.a = memory[addr as usize];
                println!("Executing LD A, (nn) at PC={:#06x}", self.pc);
            }
            0xFB => { // EI
                self.ime = true;
                println!("Executing EI at PC={:#06x}", self.pc);
            }
            0xFE => { // CP n
                let n = self.fetch(memory);
                let (result, carry) = self.a.overflowing_sub(n);
                self.f = if result == 0 { 0x80 } else { 0 } | 0x40 | if carry { 0x10 } else { 0 };
                println!("Executing CP n at PC={:#06x}", self.pc);
            }
            0xFF => { // RST 38H
                self.push(memory, (self.pc >> 8) as u8);
                self.push(memory, self.pc as u8);
                self.pc = 0x38;
                println!("Executing RST 38H at PC={:#06x}", self.pc);
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

