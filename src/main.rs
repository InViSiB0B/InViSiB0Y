use std::fs::File;
use std::io::Read;
use std::time::Instant;
mod cpu;
use cpu::Cpu;

fn main() {
    let mut cpu = Cpu::new();
    let mut memory = [0u8; 0x10000];
    
    // Load the ROM
    let mut file = File::open("C:\\Projects\\InViSiB0Y\\gb-test-roms-master\\cpu_instrs\\cpu_instrs.gb")
        .expect("Failed to open test ROM");
    
    file.read_exact(&mut memory[..])
        .expect("Failed to load ROM into memory");
    
    // Print the first few bytes of the ROM to confirm it's loaded
    println!("First 16 bytes of ROM: {:?}", &memory[0..16]);
    
    // Set up a buffer for collecting serial output
    let mut serial_output = String::new();
    
    // Track start time for informational purposes only
    let start_time = Instant::now();
    let mut instructions_executed = 0;
    
    println!("Starting CPU execution - will run until completion...");
    
    // Infinite loop - let the ROM run its tests completely
    loop {
        instructions_executed += 1;
        
        // Report progress periodically
        if instructions_executed % 10000000 == 0 {  // Every 10 million instructions
            println!("Executed {} million instructions, time elapsed: {:?}", 
                    instructions_executed / 1000000, start_time.elapsed());
        }
        
        if !cpu.step(&mut memory) {
            println!("CPU halted or error detected");
            break;
        }
        
        // Check for writes to the serial port (0xFF01 = data, 0xFF02 = control)
        if memory[0xFF02] == 0x81 {
            let output_char = memory[0xFF01] as char;
            print!("{}", output_char);  // Print immediately to console
            serial_output.push(output_char);  // Add to output buffer
            memory[0xFF02] = 0;  // Reset transfer flag
        }
    }
    
    // After the loop, dump CPU state and results
    println!("\nExecution completed in {:?} with {} instructions executed", 
            start_time.elapsed(), instructions_executed);
    
    println!("Final CPU state: PC={:#06x}, SP={:#06x}", cpu.pc, cpu.sp);
    println!("Registers: A={:#04x} F={:#04x} BC={:#04x} DE={:#04x} HL={:#04x}",
        cpu.a, cpu.f,
        ((cpu.b as u16) << 8) | cpu.c as u16,
        ((cpu.d as u16) << 8) | cpu.e as u16,
        ((cpu.h as u16) << 8) | cpu.l as u16);
    
    if serial_output.is_empty() {
        println!("No serial output detected from the test ROM.");
    } else {
        println!("Complete serial output: {}", serial_output);
    }
}