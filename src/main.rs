use std::fs::File;
use std::io::Read;
use std::time::Instant;
mod cpu;
use cpu::Cpu;

fn main() 
{
    // Initialize CPU and memory
    let mut cpu = Cpu::new();
    let mut memory = [0u8; 0x10000];
    
    // Load the ROM
    let mut file = File::open("C:\\Projects\\InViSiB0Y\\gb-test-roms-master\\cpu_instrs\\cpu_instrs.gb")
        .expect("Failed to open test ROM");
    
    // Read as much of the file as possible into memory
    match file.read(&mut memory[..]) 
    {
        Ok(n) => println!("Loaded {} bytes of ROM", n),
        Err(e) => println!("Error loading ROM: {}", e),
    }
    
    // Print the first few bytes of the ROM to confirm it's loaded
    println!("First 16 bytes of ROM: {:?}", &memory[0..16]);
    
    // Set up a buffer for collecting serial output
    let mut serial_output = String::new();
    
    // Initialize performance tracking
    let start_time = Instant::now();
    let mut instructions_executed: u64 = 0; // Use u64 to avoid overflow
    
    println!("Starting CPU execution - running until completion...");

    if instructions_executed % 1_000_000 == 0 {  // Check less frequently
        println!("Current PC: {:#06x}, Opcode: {:#04x}", cpu.pc, memory[cpu.pc as usize]);
        println!("Registers: A={:#04x} F={:#04x} BC={:#04x} DE={:#04x} HL={:#04x}",
            cpu.a, cpu.f,
            ((cpu.b as u16) << 8) | cpu.c as u16,
            ((cpu.d as u16) << 8) | cpu.e as u16,
            ((cpu.h as u16) << 8) | cpu.l as u16);
    }
    
    // Run the emulation
    loop 
    {
        // Execute one CPU instruction
        if !cpu.step(&mut memory) 
        {
            println!("CPU halted or error detected");
            break;
        }
        
        instructions_executed = instructions_executed.wrapping_add(1); // Use wrapping_add to avoid panic
        
        // Report progress periodically
        if instructions_executed % 10_000_000 == 0 
        {  // Every 10 million instructions
            println!("Executed {} million instructions, time elapsed: {:?}", 
                    instructions_executed / 1_000_000, start_time.elapsed());
        }
        
        // Check for writes to the serial port (0xFF01 = data, 0xFF02 = control)
        if memory[0xFF02] == 0x81 
        {
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
    
    if serial_output.is_empty() 
    {
        println!("No serial output detected from the test ROM.");
    } 
    else 
    {
        println!("Complete serial output: {}", serial_output);
    }
}