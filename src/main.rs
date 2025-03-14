use std::fs::File;
use std::io::Read;

mod cpu;

use cpu::Cpu;

fn main()
{
    let mut cpu = Cpu::new();
    let mut memory = [0u8; 0x10000];

    // Load the ROM
    let mut file = File::open("C:\\Projects\\InViSiB0Y\\gb-test-roms-master\\cpu_instrs\\cpu_instrs.gb").expect("Failed to open test ROM");
    file.read_exact(&mut memory[..])
        .expect("Failed to load ROM into memory");

    // Print the first few bytes of the ROM to confirm it's loaded
    println!("First 16 bytes of ROM: {:?}", &memory[0..16]);

    // Run the CPU through the steps
    loop 
    {
        cpu.step(&mut memory);
    }
}